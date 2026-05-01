#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#ifdef CONFIG_DEBUG_AUDIO_PCM_TCP
#include "lwip/sockets.h"
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include "event_state.h"
#include "mqtt_entity.h"
#include "report_decoder.h"
#include "siri_audio.h"
#include "siri_ble.h"

// Forward-declared — ESP-IDF ships the implementation in libbt.a but doesn't
// expose the header via include path.
void ble_store_config_init(void);

static const char *TAG = "main";

#define TICK_PERIOD_MS 50

// MQTT topic constants. Switch / Number / Button entities use the
// mqtt_entity helper, which generates topics from each entity's
// unique_id — so they don't appear here.
#define TOPIC_EVENT "siri_remote/event"
#define TOPIC_TOUCH_RAW "siri_remote/touch_raw"
#define TOPIC_CONN "siri_remote/connection"
#define TOPIC_REMOTE_STATUS "siri_remote/remote_status"
#define TOPIC_BATTERY "siri_remote/battery"
#define TOPIC_CHARGING "siri_remote/charging"
#define TOPIC_DISCOVERY_EVENT "homeassistant/event/siri_remote/config"
#define TOPIC_DISCOVERY_BATTERY "homeassistant/sensor/siri_remote_battery/config"
#define TOPIC_DISCOVERY_CHARGING "homeassistant/sensor/siri_remote_charging/config"

// HA-side device identifier — every entity attaches to this so they all
// group as one device in the HA UI.
#define HA_DEVICE_ID "siri_remote_bridge"

// State.
// Set once in mqtt_start() before any concurrent reader exists; the volatile
// keeps the compiler honest about the cross-task visibility nonetheless.
static esp_mqtt_client_handle_t volatile s_mqtt;
static event_state_t *s_es;
static SemaphoreHandle_t s_es_lock;
static volatile bool s_mqtt_connected;
static volatile bool s_remote_connected;  // tracks BLE-side state
                                          // for the MQTT-reconnect
                                          // republish of remote_status
static esp_timer_handle_t s_tick_timer;
static mqtt_entity_t *s_raw_stream_entity;
// Cached copy of s_raw_stream_entity's value, set from the MQTT-task
// on_change callback and read on the BLE notify hot path (~50 Hz). Avoids
// chasing the entity list inside on_notify_cb. 32-bit aligned bool on
// Xtensa is atomic — no lock needed.
static volatile bool s_raw_stream_enabled;
static mqtt_entity_t *s_repair_entity;
// Phase 3B.8 runtime tunables. NVS-persisted via the helper; on_change
// callbacks push live updates into event_state / siri_ble.
static mqtt_entity_t *s_swipe_y_pri_entity;
static mqtt_entity_t *s_swipe_dist_entity;
static mqtt_entity_t *s_dbl_ms_entity;
static mqtt_entity_t *s_hold_ms_entity;
static mqtt_entity_t *s_battery_low_entity;
static mqtt_entity_t *s_ble_lat_entity;
// On first-bond, Apple's HID flushes the press/release notifies that were
// buffered during the bond window (the TV+VolUp pairing-combo the user
// just held) right after the button CCCD subscribe lands. Without
// suppression they surface as phantom click events in HA. We squash button
// notifies for a short window after a fresh-bond on_connected; the window
// is keyed on idle_ms==0, which is unique to siri_ble's DISCOVERING→
// CONNECTED transition (bonded reconnects always pass a non-zero idle).
#define PAIRING_FLUSH_SUPPRESS_MS 1500
static uint32_t s_suppress_buttons_until_ms;
// Activity tracking for the optional idle-disconnect path. Compiled out
// entirely in the default always-connected mode (CONFIG_IDLE_DISCONNECT_MS=0).
#if CONFIG_IDLE_DISCONNECT_MS > 0
static volatile uint32_t s_last_activity_ms;
#define MARK_ACTIVITY()                                                                            \
    do {                                                                                           \
        s_last_activity_ms = now_ms();                                                             \
    } while (0)
#define MARK_INACTIVE()                                                                            \
    do {                                                                                           \
        s_last_activity_ms = 0;                                                                    \
    } while (0)
#else
#define MARK_ACTIVITY() ((void)0)
#define MARK_INACTIVE() ((void)0)
#endif

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// --- MQTT publish helpers ---

static void mqtt_publish(const char *topic, const char *payload, int len, int qos, bool retain)
{
    if (!s_mqtt_connected) {
        return;
    }
    esp_mqtt_client_publish(s_mqtt, topic, payload, len, qos, retain ? 1 : 0);
}

static void publish_ha_discovery_event(void)
{
    // Single retained config message that tells HA to auto-create an Event entity.
    static const char PAYLOAD[] = "{"
                                  "\"name\":\"Siri Remote\","
                                  "\"unique_id\":\"siri_remote_events\","
                                  "\"state_topic\":\"" TOPIC_EVENT "\","
                                  "\"event_types\":["
                                  "\"click\",\"double_click\",\"hold_start\",\"hold_end\","
                                  "\"swipe_up\",\"swipe_down\",\"swipe_left\",\"swipe_right\"],"
                                  "\"device\":{"
                                  "\"identifiers\":[\"siri_remote_bridge\"],"
                                  "\"name\":\"Siri Remote Bridge\","
                                  "\"manufacturer\":\"Apple\","
                                  "\"model\":\"Siri Remote (3rd gen)\""
                                  "},"
                                  "\"availability_topic\":\"" TOPIC_CONN "\","
                                  "\"payload_available\":\"online\","
                                  "\"payload_not_available\":\"offline\""
                                  "}";
    mqtt_publish(TOPIC_DISCOVERY_EVENT, PAYLOAD, sizeof(PAYLOAD) - 1, 1, /*retain*/ true);
}

// Battery + charging are continuous-state sensors. Their availability tracks
// BOTH the bridge's MQTT connection and the remote's BLE connection — when the
// remote sleeps, HA marks them unavailable so a stale value isn't displayed.
#define AVAILABILITY_BRIDGE_AND_REMOTE                                                             \
    "\"availability\":["                                                                           \
    "{\"topic\":\"" TOPIC_CONN "\","                                                               \
    "\"payload_available\":\"online\","                                                            \
    "\"payload_not_available\":\"offline\"},"                                                      \
    "{\"topic\":\"" TOPIC_REMOTE_STATUS "\","                                                      \
    "\"payload_available\":\"connected\","                                                         \
    "\"payload_not_available\":\"disconnected\"}"                                                  \
    "],"                                                                                           \
    "\"availability_mode\":\"all\""

static void publish_ha_discovery_battery(void)
{
    static const char PAYLOAD[] =
        "{"
        "\"name\":\"Battery\","
        "\"unique_id\":\"siri_remote_battery\","
        "\"state_topic\":\"" TOPIC_BATTERY "\","
        "\"value_template\":\"{{ value_json.level }}\","
        "\"device_class\":\"battery\","
        "\"unit_of_measurement\":\"%\","
        "\"device\":{\"identifiers\":[\"siri_remote_bridge\"]}," AVAILABILITY_BRIDGE_AND_REMOTE "}";
    mqtt_publish(TOPIC_DISCOVERY_BATTERY, PAYLOAD, sizeof(PAYLOAD) - 1, 1, /*retain*/ true);
}

static void publish_ha_discovery_charging(void)
{
    static const char PAYLOAD[] =
        "{"
        "\"name\":\"Charging\","
        "\"unique_id\":\"siri_remote_charging\","
        "\"state_topic\":\"" TOPIC_CHARGING "\","
        "\"icon\":\"mdi:battery-charging\","
        "\"device\":{\"identifiers\":[\"siri_remote_bridge\"]}," AVAILABILITY_BRIDGE_AND_REMOTE "}";
    mqtt_publish(TOPIC_DISCOVERY_CHARGING, PAYLOAD, sizeof(PAYLOAD) - 1, 1, /*retain*/ true);
}

static void publish_remote_status(bool connected)
{
    mqtt_publish(TOPIC_REMOTE_STATUS, connected ? "connected" : "disconnected", 0, 1,
                 /*retain*/ true);
}

// Latest battery + charging bytes cached so we can re-publish them on MQTT
// (re)connect. Necessary because on cold boot the BLE bonded reconnect
// often completes (and runs the battery/charging initial read) before
// Wi-Fi + MQTT finish associating — the publish goes nowhere, and the
// remote only emits *change* notifications afterwards, so HA is stuck
// without a value until the level happens to shift.
// 0xFF = unknown / not yet observed.
static uint8_t s_last_battery_level = 0xFF;
static uint8_t s_last_charging_byte = 0xFF;

static void publish_battery(uint8_t level)
{
    s_last_battery_level = level;
    char buf[64];
    int32_t threshold = mqtt_entity_get_value(s_battery_low_entity);
    bool low = threshold > 0 && level <= threshold;
    int n = snprintf(buf, sizeof(buf), "{\"level\":%u,\"low\":%s}", (unsigned)level,
                     low ? "true" : "false");
    if (n > 0) {
        ESP_LOGI(TAG, "battery=%u%%%s", (unsigned)level, low ? " (LOW)" : "");
        mqtt_publish(TOPIC_BATTERY, buf, n, 1, /*retain*/ true);
    }
}

static void publish_charging(uint8_t state_byte)
{
    s_last_charging_byte = state_byte;
    const char *s = siri_decode_charging_state(state_byte);
    ESP_LOGI(TAG, "charging state=0x%02x => %s", state_byte, s);
    mqtt_publish(TOPIC_CHARGING, s, 0, 1, /*retain*/ true);
}

static void publish_touch_raw(const siri_touch_frame_t *f)
{
    char buf[96];
    int n;
    if (f->finger_down) {
        n = snprintf(buf, sizeof(buf),
                     "{\"x\":%" PRId32 ",\"y\":%" PRId32 ",\"p\":%u,\"down\":true}", f->x, f->y,
                     (unsigned)f->pressure);
    } else {
        n = snprintf(buf, sizeof(buf), "{\"down\":false}");
    }
    if (n > 0) {
        mqtt_publish(TOPIC_TOUCH_RAW, buf, n, 0, /*retain*/ false);
    }
}

// --- event_state emit callback ---

static void emit_cb(const event_state_event_t *evt, void *user)
{
    (void)user;
    const char *event_type = event_state_action_name(evt->action);
    if (event_type == NULL) {
        return;
    }

    char buf[128];
    int n = 0;
    switch (evt->action) {
    case EVT_CLICK:
    case EVT_HOLD_END: {
        const char *btn = siri_button_name(evt->button);
        n = snprintf(buf, sizeof(buf),
                     "{\"event_type\":\"%s\",\"button\":\"%s\",\"duration_ms\":%" PRIu32 "}",
                     event_type, btn ? btn : "unknown", evt->duration_ms);
        break;
    }
    case EVT_DOUBLE_CLICK:
    case EVT_HOLD_START: {
        const char *btn = siri_button_name(evt->button);
        n = snprintf(buf, sizeof(buf), "{\"event_type\":\"%s\",\"button\":\"%s\"}", event_type,
                     btn ? btn : "unknown");
        break;
    }
    case EVT_SWIPE_UP:
    case EVT_SWIPE_DOWN:
    case EVT_SWIPE_LEFT:
    case EVT_SWIPE_RIGHT:
        n = snprintf(buf, sizeof(buf), "{\"event_type\":\"%s\",\"distance\":%" PRId32 "}",
                     event_type, evt->distance);
        break;
    }
    if (n > 0) {
        ESP_LOGI(TAG, "emit %.*s", n, buf);
        mqtt_publish(TOPIC_EVENT, buf, n, 0, /*retain*/ false);
    }
}

#ifdef CONFIG_DEBUG_AUDIO_PCM_TCP
// --- Phase 5.A.3 TCP PCM sink (development validation only) ---
//
// On Mic-button-driven audio session start: open a TCP connection to the
// configured host:port. For each decoded Opus frame: send the raw int16
// PCM samples. On session end: close. Listener side runs:
//
//   nc -l <PORT> | ffplay -f s16le -ar 16000 -ac 1 -
//
// All errors (failed connect, send failure, host unreachable) are logged
// once and the socket is dropped — we don't want to interrupt the BLE
// path or the decoder over a debug channel. Streaming resumes on the
// next session.

static volatile int s_pcm_socket = -1;

static void pcm_tcp_open_socket(void *user)
{
    (void)user;
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGW(TAG, "pcm_tcp: socket() rc=%d errno=%d", sock, errno);
        return;
    }
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_AUDIO_PCM_TCP_PORT),
    };
    if (inet_aton(CONFIG_AUDIO_PCM_TCP_HOST, &addr.sin_addr) == 0) {
        ESP_LOGW(TAG, "pcm_tcp: inet_aton('%s') failed", CONFIG_AUDIO_PCM_TCP_HOST);
        close(sock);
        return;
    }
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGW(TAG, "pcm_tcp: connect %s:%d errno=%d (run `nc -l %d` on the host)",
                 CONFIG_AUDIO_PCM_TCP_HOST, CONFIG_AUDIO_PCM_TCP_PORT, errno,
                 CONFIG_AUDIO_PCM_TCP_PORT);
        close(sock);
        return;
    }
    ESP_LOGI(TAG, "pcm_tcp: streaming to %s:%d", CONFIG_AUDIO_PCM_TCP_HOST,
             CONFIG_AUDIO_PCM_TCP_PORT);
    s_pcm_socket = sock;
}

static void pcm_tcp_close_socket(void *user)
{
    (void)user;
    int sock = s_pcm_socket;
    s_pcm_socket = -1;
    if (sock >= 0) {
        close(sock);
        ESP_LOGI(TAG, "pcm_tcp: stream closed");
    }
}

static void pcm_tcp_send_frame(const int16_t *samples, size_t count, void *user)
{
    (void)user;
    int sock = s_pcm_socket;
    if (sock < 0)
        return;
    ssize_t n = send(sock, samples, count * sizeof(int16_t), 0);
    if (n < 0) {
        ESP_LOGW(TAG, "pcm_tcp: send errno=%d; closing", errno);
        s_pcm_socket = -1;
        close(sock);
    }
}
#endif  // CONFIG_DEBUG_AUDIO_PCM_TCP

// --- siri_ble callbacks ---

static void on_notify_cb(uint16_t attr_handle, const uint8_t *data, size_t len, void *user)
{
    (void)user;
    MARK_ACTIVITY();
    if (attr_handle == SIRI_HANDLE_BUTTON) {
        if (now_ms() < s_suppress_buttons_until_ms) {
            ESP_LOGI(TAG, "suppressing buffered pairing-combo button notify");
            return;
        }
        uint16_t btns = siri_decode_button_bytes(data, len);

#ifdef CONFIG_VOICE_ENABLED
        // Detect Mic-button transitions to drive audio session lifecycle.
        // gen-3 doesn't use the gen-1 sentinels — the Mic bit on the
        // standard button bitmap *is* the session marker.
        static uint16_t s_prev_btns;
        bool mic_was = (s_prev_btns & SIRI_BTN_MIC) != 0;
        bool mic_now = (btns & SIRI_BTN_MIC) != 0;
        if (mic_now && !mic_was) {
            siri_audio_session_start();
        } else if (!mic_now && mic_was) {
            siri_audio_session_end();
        }
        s_prev_btns = btns;
#endif

        xSemaphoreTake(s_es_lock, portMAX_DELAY);
        event_state_feed_buttons(s_es, btns, now_ms());
        xSemaphoreGive(s_es_lock);
#ifdef CONFIG_VOICE_ENABLED
    } else if (attr_handle == SIRI_HANDLE_AUDIO) {
        siri_audio_dispatch_packet(data, len);
#endif
    } else if (attr_handle == SIRI_HANDLE_BATTERY) {
        if (len >= 1) {
            publish_battery(data[0]);
        }
    } else if (attr_handle == SIRI_HANDLE_CHARGING) {
        if (len >= 1) {
            publish_charging(data[0]);
        }
    } else if (attr_handle == SIRI_HANDLE_TOUCH) {
        siri_touch_frame_t frame;
        if (siri_decode_touch_frame(data, len, &frame)) {
#ifdef CONFIG_DEBUG_TOUCH_FRAMES
            // Diagnostic dump: raw bytes + parsed view. Used to study how the
            // circular touchpad encodes positions at its edges (small swipes
            // near top/bottom misclassify direction).
            ESP_LOGI(TAG,
                     "touch raw=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x"
                     " | x=%" PRId32 " y=%" PRId32 " p=%u down=%d ctr=%" PRIu32,
                     data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                     data[8], data[9], data[10], frame.x, frame.y, (unsigned)frame.pressure,
                     (int)frame.finger_down, frame.remote_counter);
#endif
            xSemaphoreTake(s_es_lock, portMAX_DELAY);
            event_state_feed_touch(s_es, &frame, now_ms());
            xSemaphoreGive(s_es_lock);
            if (s_raw_stream_enabled) {
                publish_touch_raw(&frame);
            }
        }
    }
    // 0x0035 (audio) is Phase 5.
}

static void on_connected_cb(uint32_t idle_ms, void *user)
{
    (void)user;
    ESP_LOGI(TAG, "remote connected (idle %" PRIu32 " ms)", idle_ms);
    s_remote_connected = true;
    publish_remote_status(true);
    MARK_ACTIVITY();  // start the idle clock (no-op unless CONFIG_IDLE_DISCONNECT_MS>0)
    if (idle_ms == 0) {
        // Fresh bond from siri_ble's DISCOVERING→CONNECTED transition.
        // Suppress buffered HID flushes for the next 1.5 s.
        s_suppress_buttons_until_ms = now_ms() + PAIRING_FLUSH_SUPPRESS_MS;
    }
}

static void on_disconnected_cb(void *user)
{
    (void)user;
    s_remote_connected = false;
    publish_remote_status(false);
    MARK_INACTIVE();  // halt the idle check until next connection
    xSemaphoreTake(s_es_lock, portMAX_DELAY);
    event_state_reset(s_es, now_ms());
    xSemaphoreGive(s_es_lock);
}

static void repair_button_pressed(int32_t value, void *user)
{
    (void)value;
    (void)user;
    ESP_LOGI(TAG, "HA repair button pressed");
    siri_ble_repair();
}

static void on_raw_stream_changed(int32_t value, void *user)
{
    (void)user;
    s_raw_stream_enabled = (value != 0);
}

// --- Phase 3B.8 Number-entity on_change callbacks ---
//
// event_state setters are called under s_es_lock for ordering with the
// feed/tick functions. siri_ble_set_slave_latency handles its own state
// internally. battery_low has no setter — publish_battery reads the entity
// value directly when it next publishes.

static void on_swipe_y_pri_changed(int32_t value, void *user)
{
    (void)user;
    xSemaphoreTake(s_es_lock, portMAX_DELAY);
    event_state_set_swipe_y_priority(s_es, value);
    xSemaphoreGive(s_es_lock);
    ESP_LOGI(TAG, "swipe_y_priority -> %" PRId32, value);
}

static void on_swipe_dist_changed(int32_t value, void *user)
{
    (void)user;
    xSemaphoreTake(s_es_lock, portMAX_DELAY);
    event_state_set_swipe_min_distance(s_es, value);
    xSemaphoreGive(s_es_lock);
    ESP_LOGI(TAG, "swipe_min_distance -> %" PRId32, value);
}

static void on_dbl_ms_changed(int32_t value, void *user)
{
    (void)user;
    xSemaphoreTake(s_es_lock, portMAX_DELAY);
    event_state_set_double_click_max_ms(s_es, (uint32_t)value);
    xSemaphoreGive(s_es_lock);
    ESP_LOGI(TAG, "double_click_max_ms -> %" PRId32, value);
}

static void on_hold_ms_changed(int32_t value, void *user)
{
    (void)user;
    xSemaphoreTake(s_es_lock, portMAX_DELAY);
    event_state_set_hold_min_ms(s_es, (uint32_t)value);
    xSemaphoreGive(s_es_lock);
    ESP_LOGI(TAG, "hold_min_ms -> %" PRId32, value);
}

static void on_ble_lat_changed(int32_t value, void *user)
{
    (void)user;
    siri_ble_set_slave_latency((uint16_t)value);
}

// --- FreeRTOS tick for event_state ---

static void tick_timer_cb(void *arg)
{
    (void)arg;
    uint32_t now = now_ms();
    xSemaphoreTake(s_es_lock, portMAX_DELAY);
    event_state_tick(s_es, now);
    xSemaphoreGive(s_es_lock);

#if CONFIG_IDLE_DISCONNECT_MS > 0
    uint32_t last = s_last_activity_ms;
    if (last != 0 && (now - last) > CONFIG_IDLE_DISCONNECT_MS) {
        s_last_activity_ms = 0;  // prevent repeated terminate calls
        siri_ble_idle_disconnect();
    }
#endif
}

// --- Wi-Fi ---

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "wifi: connecting");
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "wifi: disconnected, retrying");
            esp_wifi_connect();
            break;
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ip_evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "wifi: got IP " IPSTR, IP2STR(&ip_evt->ip_info.ip));
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// --- MQTT ---

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id, void *data)
{
    (void)args;
    (void)base;
    esp_mqtt_event_handle_t evt = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "mqtt: connected");
        s_mqtt_connected = true;
        esp_mqtt_client_publish(s_mqtt, TOPIC_CONN, "online", 0, 1, /*retain*/ 1);
        publish_ha_discovery_event();
        publish_ha_discovery_battery();
        publish_ha_discovery_charging();
        // Republish remote_status reflecting the *current* BLE state. If
        // BLE is already connected (cold-boot bonded reconnect that beat
        // MQTT to the punch, or MQTT-only reconnect mid-session), we
        // must NOT clobber it with "disconnected" — the battery and
        // charging entities use availability_mode=all and treat
        // remote_status=disconnected as "make me unavailable", which
        // silently drops every state publish until BLE itself bounces.
        publish_remote_status(s_remote_connected);
        // Helper-managed entities (raw-stream Switch, future Buttons/Numbers)
        // (re)publish their discovery + state and (re)subscribe to commands.
        mqtt_entity_on_mqtt_connected(s_mqtt);
        // If BLE already delivered battery / charging values before MQTT
        // came up (cold-boot bonded reconnect path), republish the cached
        // values now that the broker is listening. Without this HA waits
        // for the next change notification, which on a stable battery may
        // never come.
        if (s_last_battery_level != 0xFF) {
            publish_battery(s_last_battery_level);
        }
        if (s_last_charging_byte != 0xFF) {
            publish_charging(s_last_charging_byte);
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "mqtt: disconnected");
        s_mqtt_connected = false;
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "mqtt: subscribed (msg_id=%d)", evt->msg_id);
        break;
    case MQTT_EVENT_DATA:
        (void)mqtt_entity_dispatch_data(evt->topic, evt->topic_len, evt->data, evt->data_len);
        break;
    default:
        break;
    }
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
        .credentials.username = CONFIG_MQTT_USERNAME,
        .credentials.authentication.password = CONFIG_MQTT_PASSWORD,
        .session.last_will.topic = TOPIC_CONN,
        .session.last_will.msg = "offline",
        .session.last_will.msg_len = 7,
        .session.last_will.qos = 1,
        .session.last_will.retain = 1,
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(
        esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt));
}

// --- NimBLE task glue ---

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// --- app_main ---

void app_main(void)
{
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_raw_stream_entity = mqtt_entity_create(&(mqtt_entity_config_t){
        .kind = MQTT_ENTITY_SWITCH,
        .unique_id = "siri_remote_raw_stream",
        .display_name = "Raw Touch Stream",
        .icon = "mdi:gesture-tap-hold",
        .device_id = HA_DEVICE_ID,
        .availability_topic = TOPIC_CONN,
        .nvs_key = "raw_stream",
        .default_value = 0,
        .on_change = on_raw_stream_changed,
    });
    assert(s_raw_stream_entity != NULL);
    s_raw_stream_enabled = (mqtt_entity_get_value(s_raw_stream_entity) != 0);
    ESP_LOGI(TAG, "raw_stream initial state: %s", s_raw_stream_enabled ? "ON" : "OFF");

    s_repair_entity = mqtt_entity_create(&(mqtt_entity_config_t){
        .kind = MQTT_ENTITY_BUTTON,
        .unique_id = "siri_remote_repair",
        .display_name = "Re-pair Remote",
        .icon = "mdi:bluetooth-refresh",
        .device_id = HA_DEVICE_ID,
        .availability_topic = TOPIC_CONN,
        .on_change = repair_button_pressed,
    });
    assert(s_repair_entity != NULL);

    // Phase 3B.8 runtime-tunable Number entities. Created BEFORE event_state
    // so we can build the initial config from each entity's loaded value
    // (which is the NVS value if persisted, else the Kconfig default
    // supplied as default_value).
    s_swipe_y_pri_entity = mqtt_entity_create(&(mqtt_entity_config_t){
        .kind = MQTT_ENTITY_NUMBER,
        .unique_id = "siri_remote_swipe_y_priority",
        .display_name = "Swipe Y Priority",
        .icon = "mdi:gesture-swipe-vertical",
        .device_id = HA_DEVICE_ID,
        .availability_topic = TOPIC_CONN,
        .min_value = 0,
        .max_value = 200,
        .step_value = 5,
        .nvs_key = "swipe_y_pri",
        .default_value = CONFIG_EVENT_SWIPE_Y_PRIORITY,
        .on_change = on_swipe_y_pri_changed,
    });
    assert(s_swipe_y_pri_entity != NULL);

    s_swipe_dist_entity = mqtt_entity_create(&(mqtt_entity_config_t){
        .kind = MQTT_ENTITY_NUMBER,
        .unique_id = "siri_remote_swipe_min_distance",
        .display_name = "Swipe Min Distance",
        .icon = "mdi:gesture-swipe",
        .device_id = HA_DEVICE_ID,
        .availability_topic = TOPIC_CONN,
        .min_value = 0,
        .max_value = 500,
        .step_value = 10,
        .nvs_key = "swipe_dist",
        .default_value = CONFIG_EVENT_SWIPE_MIN_DISTANCE,
        .on_change = on_swipe_dist_changed,
    });
    assert(s_swipe_dist_entity != NULL);

    s_dbl_ms_entity = mqtt_entity_create(&(mqtt_entity_config_t){
        .kind = MQTT_ENTITY_NUMBER,
        .unique_id = "siri_remote_double_window_ms",
        .display_name = "Double-Click Window",
        .icon = "mdi:cursor-default-click",
        .device_id = HA_DEVICE_ID,
        .availability_topic = TOPIC_CONN,
        .min_value = 0,
        .max_value = 2000,
        .step_value = 50,
        .unit_of_measurement = "ms",
        .nvs_key = "dbl_ms",
        .default_value = CONFIG_EVENT_DOUBLE_WINDOW_MS,
        .on_change = on_dbl_ms_changed,
    });
    assert(s_dbl_ms_entity != NULL);

    s_hold_ms_entity = mqtt_entity_create(&(mqtt_entity_config_t){
        .kind = MQTT_ENTITY_NUMBER,
        .unique_id = "siri_remote_hold_threshold_ms",
        .display_name = "Hold Threshold",
        .icon = "mdi:gesture-tap-hold",
        .device_id = HA_DEVICE_ID,
        .availability_topic = TOPIC_CONN,
        .min_value = 0,
        .max_value = 10000,
        .step_value = 100,
        .unit_of_measurement = "ms",
        .nvs_key = "hold_ms",
        .default_value = CONFIG_EVENT_HOLD_THRESHOLD_MS,
        .on_change = on_hold_ms_changed,
    });
    assert(s_hold_ms_entity != NULL);

    s_battery_low_entity = mqtt_entity_create(&(mqtt_entity_config_t){
        .kind = MQTT_ENTITY_NUMBER,
        .unique_id = "siri_remote_battery_low_pct",
        .display_name = "Battery Low Threshold",
        .icon = "mdi:battery-alert",
        .device_id = HA_DEVICE_ID,
        .availability_topic = TOPIC_CONN,
        .min_value = 0,
        .max_value = 100,
        .step_value = 5,
        .unit_of_measurement = "%",
        .nvs_key = "bat_low",
        .default_value = CONFIG_BATTERY_LOW_THRESHOLD_PCT,
        .on_change = NULL,  // publish_battery reads via mqtt_entity_get_value
    });
    assert(s_battery_low_entity != NULL);

    s_ble_lat_entity = mqtt_entity_create(&(mqtt_entity_config_t){
        .kind = MQTT_ENTITY_NUMBER,
        .unique_id = "siri_remote_ble_slave_latency",
        .display_name = "BLE Slave Latency",
        .icon = "mdi:bluetooth-settings",
        .device_id = HA_DEVICE_ID,
        .availability_topic = TOPIC_CONN,
        .min_value = 0,
        .max_value = 500,
        .step_value = 20,
        .nvs_key = "ble_lat",
        .default_value = CONFIG_BLE_SLAVE_LATENCY_DEFAULT,
        .on_change = on_ble_lat_changed,
    });
    assert(s_ble_lat_entity != NULL);

    s_es_lock = xSemaphoreCreateMutex();
    assert(s_es_lock != NULL);

    event_state_config_t es_cfg = {
        .double_click_max_ms = (uint32_t)mqtt_entity_get_value(s_dbl_ms_entity),
        .hold_min_ms = (uint32_t)mqtt_entity_get_value(s_hold_ms_entity),
        .swipe_min_distance = mqtt_entity_get_value(s_swipe_dist_entity),
        .swipe_y_priority_threshold = mqtt_entity_get_value(s_swipe_y_pri_entity),
    };
    s_es = event_state_create(&es_cfg, emit_cb, NULL);
    assert(s_es != NULL);

    // Push the persisted BLE slave-latency into siri_ble *before* siri_ble_start
    // so the first conn-param update (post-setup) uses the right value.
    siri_ble_set_slave_latency((uint16_t)mqtt_entity_get_value(s_ble_lat_entity));

#ifdef CONFIG_VOICE_ENABLED
    // Phase 5.A.2 voice pipeline. Allocates OpusDecoder + decode task on
    // CPU1 + FreeRTOS queue. Must come after PSRAM init (handled implicitly
    // by ESP-IDF startup before app_main) and before siri_ble_start so the
    // dispatcher is ready when the first audio packet arrives.
    siri_audio_config_t audio_cfg = {
#ifdef CONFIG_DEBUG_AUDIO_PCM_TCP
        .on_pcm = pcm_tcp_send_frame,
        .on_session_start = pcm_tcp_open_socket,
        .on_session_end = pcm_tcp_close_socket,
#else
        .on_pcm = NULL,  // Phase 5.B will swap this for ESPHome voice_assistant
        .on_session_start = NULL,
        .on_session_end = NULL,
#endif
        .user = NULL,
    };
    ESP_ERROR_CHECK(siri_audio_start(&audio_cfg));
#endif

    esp_timer_create_args_t ta = {
        .callback = tick_timer_cb,
        .name = "event_state_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_tick_timer, TICK_PERIOD_MS * 1000));

    wifi_start();
    mqtt_start();

    ESP_ERROR_CHECK(nimble_port_init());
    ble_store_config_init();

    siri_ble_config_t ble_cfg = {
        .on_notify = on_notify_cb,
        .on_connected = on_connected_cb,
        .on_disconnected = on_disconnected_cb,
        .user = NULL,
    };
    ESP_ERROR_CHECK(siri_ble_start(&ble_cfg));
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "booted");
}
