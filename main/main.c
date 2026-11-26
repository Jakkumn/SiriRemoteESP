#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
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

#include "button_pulse.h"
#include "mqtt_entity.h"
#include "report_decoder.h"
#include "siri_audio.h"
#ifdef CONFIG_DEBUG_AUDIO_PCM_TCP
#include "siri_audio_pcm_tcp.h"
#endif
#include "siri_ble.h"

static const char *TAG = "main";

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
// at 10 Hz. So the emit callback only stages here, and flush_pulses() hands
// the batch to the publisher task once the lock is released. Sized for every
// button held at once.
#define PULSE_STAGE_MAX 13
typedef struct {
    siri_button_bit_t button;
    uint32_t repeat;
    uint32_t held_ms;
} staged_pulse_t;
static staged_pulse_t s_pulse_stage[PULSE_STAGE_MAX];
static size_t s_pulse_staged;
// On first-bond, Apple's HID flushes the press/release notifies that were
// buffered during the bond window (the pairing combo the user just held)
// right after the button CCCD subscribe lands. Without suppression they
// surface as phantom presses in HA. siri_ble drops all notifies for this
// long after a fresh bond's setup chain; we only supply the duration.
#define PAIRING_FLUSH_SUPPRESS_MS 1500

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Arm the one-shot tick for exactly when button_pulse next has a repeat due,
// or leave it stopped if nothing is held. Caller holds s_pulse_lock. now is
// truncated to the millisecond, so firing `due` ms after it lands at or after
// the due instant — never a hair early, which would skip a whole interval.
static void rearm_tick_locked(uint32_t now)
{
    (void)esp_timer_stop(s_tick_timer);  // ESP_ERR_INVALID_STATE if idle: fine
    uint32_t due = button_pulse_next_due_ms(s_bp, now);
    if (due != BUTTON_PULSE_NONE_DUE) {
        (void)esp_timer_start_once(s_tick_timer, (uint64_t)due * 1000);
    }
}

// --- MQTT publish helpers ---
//
// esp_mqtt_client_publish sends synchronously in the caller's task and can
// block for the network timeout (~10 s) when Wi-Fi is struggling. Runtime
// publishes originate on the NimBLE host task (battery, charging, remote
// status, button-driven pulses) and the esp_timer task (repeat pulses) —
// a stall on the first stops BLE notify processing, and a stall on the
// second delays every esp_timer callback in the system, NimBLE's callouts
// included. So those callers enqueue, and a low-priority publisher task does
// the blocking send. A full queue drops: a pulse costs one missed increment,
// and retained state is republished on the next MQTT connect.
//
// The MQTT event handler's own discovery / LWT publishes stay direct
// (mqtt_publish_now): they already run on the MQTT task, and their payloads
// are larger than a queue slot.

#define PUB_PAYLOAD_MAX 192  // largest runtime payload is a pulse event
#define PUB_QUEUE_DEPTH 16
typedef struct {
    const char *topic;  // always a TOPIC_* literal, so the pointer outlives the item
    uint16_t len;
    uint8_t qos;
    bool retain;
    char payload[PUB_PAYLOAD_MAX];
} pub_msg_t;
static QueueHandle_t s_pub_queue;
static uint32_t s_pub_dropped;  // log-rate-limit only; a lost increment is harmless

static void mqtt_publish_now(const char *topic, const char *payload, int len, int qos, bool retain)
{
    if (!s_mqtt_connected) {
        return;
    }
    esp_mqtt_client_publish(s_mqtt, topic, payload, len, qos, retain ? 1 : 0);
}

// Non-blocking. len == 0 means NUL-terminated, matching esp_mqtt_client_publish.
static void mqtt_publish(const char *topic, const char *payload, int len, int qos, bool retain)
{
    if (!s_mqtt_connected) {
        return;  // same as before the queue: offline publishes go nowhere
    }
    size_t n = len > 0 ? (size_t)len : strlen(payload);
    if (n > PUB_PAYLOAD_MAX) {
        ESP_LOGE(TAG, "publish to %s dropped: %u B exceeds queue slot", topic, (unsigned)n);
        return;
    }
    pub_msg_t msg = {.topic = topic, .len = (uint16_t)n, .qos = (uint8_t)qos, .retain = retain};
    memcpy(msg.payload, payload, n);
    if (xQueueSend(s_pub_queue, &msg, 0) != pdTRUE) {
        if ((++s_pub_dropped & 31) == 1) {
            ESP_LOGW(TAG, "publish queue full — dropped %" PRIu32 " message(s) so far",
                     s_pub_dropped);
        }
    }
}

static void publisher_task(void *arg)
{
    (void)arg;
    pub_msg_t msg;
    for (;;) {
        if (xQueueReceive(s_pub_queue, &msg, portMAX_DELAY) == pdTRUE) {
            mqtt_publish_now(msg.topic, msg.payload, msg.len, msg.qos, msg.retain);
        }
    }
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
    mqtt_publish_now(TOPIC_DISCOVERY_EVENT, PAYLOAD, sizeof(PAYLOAD) - 1, 1, /*retain*/ true);
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
        "\"state_class\":\"measurement\","  // HA long-term statistics
        "\"entity_category\":\"diagnostic\","
        "\"unit_of_measurement\":\"%\","
        "\"device\":{\"identifiers\":[\"siri_remote_bridge\"]}," AVAILABILITY_BRIDGE_AND_REMOTE "}";
    mqtt_publish_now(TOPIC_DISCOVERY_BATTERY, PAYLOAD, sizeof(PAYLOAD) - 1, 1, /*retain*/ true);
}

static void publish_ha_discovery_charging(void)
{
    static const char PAYLOAD[] =
        "{"
        "\"name\":\"Charging\","
        "\"unique_id\":\"siri_remote_charging\","
        "\"state_topic\":\"" TOPIC_CHARGING "\","
        "\"icon\":\"mdi:battery-charging\","
        "\"entity_category\":\"diagnostic\","
        "\"device\":{\"identifiers\":[\"siri_remote_bridge\"]}," AVAILABILITY_BRIDGE_AND_REMOTE "}";
    mqtt_publish_now(TOPIC_DISCOVERY_CHARGING, PAYLOAD, sizeof(PAYLOAD) - 1, 1, /*retain*/ true);
}

// Retained state topics (remote_status, battery, charging) go out at QoS 0.
// esp-mqtt keeps QoS 1 messages in its outbox across a reconnect and resends
// them after it, so a "disconnected" queued just before a Wi-Fi blip could
// land *after* the fresh state MQTT_EVENT_CONNECTED republishes and stay
// retained for days, holding battery/charging unavailable. QoS 1 bought
// nothing here: every connect republishes all three from current state.
#define STATE_QOS 0

static void publish_remote_status(bool connected)
{
    mqtt_publish(TOPIC_REMOTE_STATUS, connected ? "connected" : "disconnected", 0, STATE_QOS,
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
        mqtt_publish(TOPIC_BATTERY, buf, n, STATE_QOS, /*retain*/ true);
    }
}

static void publish_charging(uint8_t state_byte)
{
    s_last_charging_byte = state_byte;
    const char *s = siri_decode_charging_state(state_byte);
    ESP_LOGI(TAG, "charging state=0x%02x => %s", state_byte, s);
    mqtt_publish(TOPIC_CHARGING, s, 0, STATE_QOS, /*retain*/ true);
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

// Drain whatever the last feed/tick staged and queue it for the publisher
// task. Must be called with s_pulse_lock NOT held.
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
// Development-only PCM sink; see siri_audio_pcm_tcp.h.
static void pcm_tcp_open_cb(void *user)
{
    (void)user;
    siri_audio_pcm_tcp_open(CONFIG_AUDIO_PCM_TCP_HOST, CONFIG_AUDIO_PCM_TCP_PORT);
}

static void pcm_tcp_close_cb(void *user)
{
    (void)user;
    siri_audio_pcm_tcp_close();
}

static void pcm_tcp_send_cb(const int16_t *samples, size_t count, void *user)
{
    (void)user;
    siri_audio_pcm_tcp_send(samples, count);
}
#endif  // CONFIG_DEBUG_AUDIO_PCM_TCP

// --- siri_ble callbacks ---

#ifdef CONFIG_VOICE_ENABLED
// Last button bitmap, for Mic edge detection. NimBLE host task only (notify
// and disconnect callbacks).
static uint16_t s_prev_btns;
#endif

static void on_notify_cb(uint16_t attr_handle, const uint8_t *data, size_t len, void *user)
{
    (void)user;
    if (attr_handle == SIRI_HANDLE_BUTTON) {
        uint16_t btns = siri_decode_button_bytes(data, len);

#ifdef CONFIG_VOICE_ENABLED
        // Detect Mic-button transitions to drive audio session lifecycle.
        // gen-3 doesn't use the gen-1 sentinels — the Mic bit on the
        // standard button bitmap *is* the session marker.
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
        uint32_t now = now_ms();  // under the lock: a tick can't slip in between
        button_pulse_feed_buttons(s_bp, btns, now);
        rearm_tick_locked(now);
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
}

static void on_disconnected_cb(void *user)
{
    (void)user;
#ifdef CONFIG_VOICE_ENABLED
    // No release report will arrive for a Mic held when the link dropped: end
    // the session it would have ended, and forget the bitmap so a Mic held at
    // reconnect is seen as a fresh press rather than matching the stale bit.
    if ((s_prev_btns & SIRI_BTN_MIC) != 0) {
        siri_audio_session_end();
    }
    s_prev_btns = 0;
#endif
    s_remote_connected = false;
    publish_remote_status(false);
    xSemaphoreTake(s_pulse_lock, portMAX_DELAY);
    button_pulse_reset(s_bp);
    rearm_tick_locked(now_ms());  // nothing held any more: stops the tick
    s_pulse_staged = 0;           // drop anything staged but not yet published
    xSemaphoreGive(s_pulse_lock);
}

static void repair_button_pressed(int32_t value, void *user)
{
    (void)value;
    (void)user;
    ESP_LOGI(TAG, "HA repair button pressed");
    siri_ble_repair();
}

// --- Tick for button_pulse ---
//
// Repeats exist only because of this timer: the remote sends HID reports on
// change, not while a button is held. It is a one-shot re-armed for the next
// due repeat (rearm_tick_locked), so it fires exactly on the repeat interval
// while a button is held and not at all otherwise. It used to be a 50 ms
// periodic, which quantised repeats to the tick (75 ms became 100, 100 ms
// occasionally 150) and woke the CPU 20x/s with nothing held.

static void tick_timer_cb(void *arg)
{
    (void)arg;
    xSemaphoreTake(s_pulse_lock, portMAX_DELAY);
    uint32_t now = now_ms();  // under the lock, so no feed lands in between
    button_pulse_tick(s_bp, now);
    rearm_tick_locked(now);
    xSemaphoreGive(s_pulse_lock);
    flush_pulses();
}

// --- Wi-Fi ---
//
// Reconnects back off: 1 s doubling to 30 s, reset on getting an IP. Retrying
// immediately on every STA_DISCONNECTED turns an AP outage or a bad password
// into back-to-back full-channel scans, and the radio is shared with the
// always-connected BLE link that this firmware treats as load-bearing.
#define WIFI_RETRY_MIN_MS 1000
#define WIFI_RETRY_MAX_MS 30000
static esp_timer_handle_t s_wifi_retry_timer;
static uint32_t s_wifi_retry_ms = WIFI_RETRY_MIN_MS;  // event-loop task only

static void wifi_retry_cb(void *arg)
{
    (void)arg;
    esp_wifi_connect();
}

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
            ESP_LOGW(TAG, "wifi: disconnected, retrying in %lu ms", (unsigned long)s_wifi_retry_ms);
            (void)esp_timer_stop(s_wifi_retry_timer);  // INVALID_STATE if idle: fine
            (void)esp_timer_start_once(s_wifi_retry_timer, (uint64_t)s_wifi_retry_ms * 1000);
            s_wifi_retry_ms =
                s_wifi_retry_ms >= WIFI_RETRY_MAX_MS / 2 ? WIFI_RETRY_MAX_MS : s_wifi_retry_ms * 2;
            break;
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ip_evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "wifi: got IP " IPSTR, IP2STR(&ip_evt->ip_info.ip));
        s_wifi_retry_ms = WIFI_RETRY_MIN_MS;
    }
}

static void wifi_start(void)
{
    const esp_timer_create_args_t retry_args = {
        .callback = wifi_retry_cb,
        .name = "wifi_retry",
    };
    ESP_ERROR_CHECK(esp_timer_create(&retry_args, &s_wifi_retry_timer));

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
        (void)mqtt_entity_dispatch_data(evt->topic, evt->topic_len, evt->data, evt->data_len,
                                        evt->retain);
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

// --- app_main ---

void app_main(void)
{
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

    // Before anything that can publish: BLE callbacks, the tick timer, MQTT.
    s_pub_queue = xQueueCreate(PUB_QUEUE_DEPTH, sizeof(pub_msg_t));
    assert(s_pub_queue != NULL);
    BaseType_t ok = xTaskCreate(publisher_task, "mqtt_pub", 4096, NULL, tskIDLE_PRIORITY + 2, NULL);
    assert(ok == pdPASS);
    (void)ok;

    button_pulse_config_t pulse_cfg = {
        .repeat_interval_ms = CONFIG_PULSE_REPEAT_INTERVAL_MS,
    };
    s_bp = button_pulse_create(&pulse_cfg, emit_cb, NULL);
    assert(s_bp != NULL);
    ESP_LOGI(TAG, "button_pulse: repeat_interval=%d ms (0 = press only)",
             CONFIG_PULSE_REPEAT_INTERVAL_MS);

#ifdef CONFIG_VOICE_ENABLED
    // Phase 5.A.2 voice pipeline. Allocates OpusDecoder + decode task on
    // CPU1 + FreeRTOS queue. Must come after PSRAM init (handled implicitly
    // by ESP-IDF startup before app_main) and before siri_ble_start so the
    // dispatcher is ready when the first audio packet arrives.
    siri_audio_config_t audio_cfg = {
#ifdef CONFIG_DEBUG_AUDIO_PCM_TCP
        .on_pcm = pcm_tcp_send_cb,
        .on_session_start = pcm_tcp_open_cb,
        .on_session_end = pcm_tcp_close_cb,
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
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_tick_timer));  // armed on demand

    wifi_start();
    mqtt_start();

    siri_ble_config_t ble_cfg = {
        .on_notify = on_notify_cb,
        .on_connected = on_connected_cb,
        .on_disconnected = on_disconnected_cb,
        .user = NULL,
        // siri_ble owns the post-pairing blackout now — it covers every
        // notification, not just buttons, and keeps both build paths identical.
        .pairing_flush_suppress_ms = PAIRING_FLUSH_SUPPRESS_MS,
        .idle_disconnect_ms = CONFIG_IDLE_DISCONNECT_MS,
        .slave_latency = (uint16_t)CONFIG_BLE_SLAVE_LATENCY_DEFAULT,
    };
    // Brings up NimBLE and starts its host task.
    ESP_ERROR_CHECK(siri_ble_start(&ble_cfg));

    ESP_LOGI(TAG, "booted");
}
