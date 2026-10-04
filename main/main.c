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

#include "button_pulse.h"
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
// Must match the device name HA registers, because the pulse payload's
// "device" field is what a blueprint compares against
// device_attr(<device>, 'name'). Defined once and used by both the discovery
// payload and the event payload so the two cannot drift.
#define HA_DEVICE_NAME "Siri Remote Bridge"

// State.
// Set once in mqtt_start() before any concurrent reader exists; the volatile
// keeps the compiler honest about the cross-task visibility nonetheless.
static esp_mqtt_client_handle_t volatile s_mqtt;
static button_pulse_t *s_bp;
static SemaphoreHandle_t s_pulse_lock;
static volatile bool s_mqtt_connected;
static volatile bool s_remote_connected;  // tracks BLE-side state
                                          // for the MQTT-reconnect
                                          // republish of remote_status
static esp_timer_handle_t s_tick_timer;
static mqtt_entity_t *s_repair_entity;

// Pulses are emitted from inside button_pulse_{feed_buttons,tick}, which run
// under s_pulse_lock. Publishing there would call esp_mqtt_client_publish —
// which takes the MQTT client mutex and can block on the outbox — while
// holding our lock, on the high-priority esp_timer task, against the NimBLE
// notify path. That was tolerable at a few events per interaction; it is not
// at 10 Hz. So the emit callback only stages here, and flush_pulses()
// publishes once the lock is released. Sized for every button held at once.
#define PULSE_STAGE_MAX 13
typedef struct {
    siri_button_bit_t button;
    uint32_t repeat;
    uint32_t held_ms;
} staged_pulse_t;
static staged_pulse_t s_pulse_stage[PULSE_STAGE_MAX];
static size_t s_pulse_staged;
// On first-bond, Apple's HID flushes the press/release notifies that were
// buffered during the bond window (the TV+VolUp pairing-combo the user
// just held) right after the button CCCD subscribe lands. Without
// suppression they surface as phantom click events in HA. We squash button
// notifies for a short window after a fresh-bond on_connected; the window
// is keyed on idle_ms==0, which is unique to siri_ble's DISCOVERING→
// CONNECTED transition (bonded reconnects always pass a non-zero idle).
#define PAIRING_FLUSH_SUPPRESS_MS 1500
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
    // Single retained config message that tells HA to auto-create an Event
    // entity. event_type carries the button name, so HA generates one device
    // trigger per button; repeat/held_ms ride along as state attributes.
    static const char PAYLOAD[] = "{"
                                  "\"name\":\"Siri Remote\","
                                  "\"unique_id\":\"siri_remote_events\","
                                  "\"state_topic\":\"" TOPIC_EVENT "\","
                                  "\"event_types\":["
                                  "\"tv\",\"volume_up\",\"volume_down\",\"select\","
                                  "\"power\",\"mic\",\"back\",\"mute\","
                                  "\"play_pause\",\"up\",\"right\",\"down\",\"left\"],"
                                  "\"device\":{"
                                  "\"identifiers\":[\"" HA_DEVICE_ID "\"],"
                                  "\"name\":\"" HA_DEVICE_NAME "\","
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
    // Was a runtime Number entity; the pulse refactor removed the Number
    // platform, so the threshold is compile-time. HA can of course do this
    // thresholding template-side instead.
    bool low =
        (CONFIG_BATTERY_LOW_THRESHOLD_PCT > 0) && (level <= CONFIG_BATTERY_LOW_THRESHOLD_PCT);
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

// --- button_pulse emit callback ---
//
// Runs inside button_pulse_{feed_buttons,tick} with s_pulse_lock held, so it
// must not touch MQTT. Stage only; flush_pulses() does the publishing.

static void emit_cb(const button_pulse_event_t *evt, void *user)
{
    (void)user;
    if (s_pulse_staged >= PULSE_STAGE_MAX) {
        return;  // unreachable: at most one pulse per button per call
    }
    s_pulse_stage[s_pulse_staged++] = (staged_pulse_t){
        .button = evt->button,
        .repeat = evt->repeat,
        .held_ms = evt->held_ms,
    };
}

// Drain whatever the last feed/tick staged and publish it. Must be called
// with s_pulse_lock NOT held.
static void flush_pulses(void)
{
    staged_pulse_t batch[PULSE_STAGE_MAX];
    size_t n;

    xSemaphoreTake(s_pulse_lock, portMAX_DELAY);
    n = s_pulse_staged;
    if (n > 0) {
        memcpy(batch, s_pulse_stage, n * sizeof(batch[0]));
        s_pulse_staged = 0;
    }
    xSemaphoreGive(s_pulse_lock);

    for (size_t i = 0; i < n; i++) {
        const char *btn = siri_button_name(batch[i].button);
        if (btn == NULL) {
            continue;
        }
        char buf[192];
        int len = snprintf(buf, sizeof(buf),
                           "{\"event_type\":\"%s\",\"device\":\"" HA_DEVICE_NAME "\","
                           "\"button\":\"%s\",\"repeat\":%" PRIu32 ",\"held_ms\":%" PRIu32 "}",
                           btn, btn, batch[i].repeat, batch[i].held_ms);
        if (len <= 0) {
            continue;
        }
        // The press is worth seeing at INFO; repeats arrive ~10x/sec and
        // would drown the UART, so they sit at DEBUG.
        if (batch[i].repeat == 0) {
            ESP_LOGI(TAG, "pulse button=%s press", btn);
        } else {
            ESP_LOGD(TAG, "pulse button=%s repeat=%" PRIu32 " held=%" PRIu32 "ms", btn,
                     batch[i].repeat, batch[i].held_ms);
        }
        // QoS 0: nothing infers release from the absence of a specific
        // packet, so a dropped pulse costs exactly one missed increment.
        mqtt_publish(TOPIC_EVENT, buf, len, 0, /*retain*/ false);
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

        xSemaphoreTake(s_pulse_lock, portMAX_DELAY);
        button_pulse_feed_buttons(s_bp, btns, now_ms());
        xSemaphoreGive(s_pulse_lock);
        flush_pulses();
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
        // Touch/gesture support is deliberately dropped — swipes are not a
        // feature of this firmware and nothing downstream consumes a frame.
        // We stay subscribed to the touch CCCD rather than removing its
        // SETUP_STEPS[] entry, because altering the setup chain risks the
        // always-connected wake behaviour for no real gain.
#ifdef CONFIG_DEBUG_TOUCH_FRAMES
        siri_touch_frame_t frame;
        if (siri_decode_touch_frame(data, len, &frame)) {
            // Diagnostic dump: raw bytes + parsed view, kept so the pad can
            // still be studied during protocol work.
            ESP_LOGI(TAG,
                     "touch raw=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x"
                     " | x=%" PRId32 " y=%" PRId32 " p=%u down=%d ctr=%" PRIu32,
                     data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                     data[8], data[9], data[10], frame.x, frame.y, (unsigned)frame.pressure,
                     (int)frame.finger_down, frame.remote_counter);
        }
#endif
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
    }
}

static void on_disconnected_cb(void *user)
{
    (void)user;
    s_remote_connected = false;
    publish_remote_status(false);
    MARK_INACTIVE();  // halt the idle check until next connection
    xSemaphoreTake(s_pulse_lock, portMAX_DELAY);
    button_pulse_reset(s_bp);
    s_pulse_staged = 0;  // drop anything staged but not yet published
    xSemaphoreGive(s_pulse_lock);
}

static void repair_button_pressed(int32_t value, void *user)
{
    (void)value;
    (void)user;
    ESP_LOGI(TAG, "HA repair button pressed");
    siri_ble_repair();
}

// --- FreeRTOS tick for button_pulse ---
//
// Repeats exist only because of this timer: the remote sends HID reports on
// change, not while a button is held.

static void tick_timer_cb(void *arg)
{
    (void)arg;
    uint32_t now = now_ms();
    xSemaphoreTake(s_pulse_lock, portMAX_DELAY);
    button_pulse_tick(s_bp, now);
    xSemaphoreGive(s_pulse_lock);
    flush_pulses();

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

    s_pulse_lock = xSemaphoreCreateMutex();
    assert(s_pulse_lock != NULL);

    button_pulse_config_t pulse_cfg = {
        .repeat_interval_ms = CONFIG_PULSE_REPEAT_INTERVAL_MS,
    };
    s_bp = button_pulse_create(&pulse_cfg, emit_cb, NULL);
    assert(s_bp != NULL);
    ESP_LOGI(TAG, "button_pulse: repeat_interval=%d ms (0 = press only)",
             CONFIG_PULSE_REPEAT_INTERVAL_MS);

    // Push the configured BLE slave-latency into siri_ble *before*
    // siri_ble_start so the first conn-param update (post-setup) uses it.
    // Compile-time since the pulse refactor removed the Number platform.
    siri_ble_set_slave_latency((uint16_t)CONFIG_BLE_SLAVE_LATENCY_DEFAULT);

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
        .name = "button_pulse_tick",
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
        // siri_ble owns the post-pairing blackout now — it covers every
        // notification, not just buttons, and keeps both build paths identical.
        .pairing_flush_suppress_ms = PAIRING_FLUSH_SUPPRESS_MS,
    };
    ESP_ERROR_CHECK(siri_ble_start(&ble_cfg));
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "booted");
}
