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

#include "event_state.h"
#include "report_decoder.h"
#include "siri_ble.h"

// Forward-declared — ESP-IDF ships the implementation in libbt.a but doesn't
// expose the header via include path.
void ble_store_config_init(void);

static const char *TAG = "main";

#define NVS_NS        "siri_bridge"
#define NVS_KEY_RAW   "raw_stream"
#define TICK_PERIOD_MS 50

// MQTT topic constants.
#define TOPIC_EVENT                  "siri_remote/event"
#define TOPIC_TOUCH_RAW              "siri_remote/touch_raw"
#define TOPIC_CONN                   "siri_remote/connection"
#define TOPIC_REMOTE_STATUS          "siri_remote/remote_status"
#define TOPIC_BATTERY                "siri_remote/battery"
#define TOPIC_CHARGING               "siri_remote/charging"
#define TOPIC_CMD_RAW_STREAM         "siri_remote/cmd/raw_stream"
#define TOPIC_STATE_RAW_STREAM       "siri_remote/state/raw_stream"
#define TOPIC_DISCOVERY_EVENT        "homeassistant/event/siri_remote/config"
#define TOPIC_DISCOVERY_SWITCH       "homeassistant/switch/siri_remote_raw_stream/config"
#define TOPIC_DISCOVERY_BATTERY      "homeassistant/sensor/siri_remote_battery/config"
#define TOPIC_DISCOVERY_CHARGING     "homeassistant/sensor/siri_remote_charging/config"

// State.
static esp_mqtt_client_handle_t s_mqtt;
static event_state_t           *s_es;
static SemaphoreHandle_t        s_es_lock;
static volatile bool            s_mqtt_connected;
static volatile bool            s_raw_stream_enabled;
static esp_timer_handle_t       s_tick_timer;
// Activity tracking for idle-disconnect. Updated on every BLE notification
// from the remote. 0 = no remote activity yet (post-disconnect or pre-first-event).
static volatile uint32_t        s_last_activity_ms;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// --- NVS for raw-stream toggle ---

static bool nvs_load_raw_stream_enabled(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t v = 0;
    esp_err_t err = nvs_get_u8(h, NVS_KEY_RAW, &v);
    nvs_close(h);
    return err == ESP_OK && v != 0;
}

static void nvs_save_raw_stream_enabled(bool on)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open for raw_stream save failed");
        return;
    }
    (void)nvs_set_u8(h, NVS_KEY_RAW, on ? 1 : 0);
    (void)nvs_commit(h);
    nvs_close(h);
}

// --- MQTT publish helpers ---

static void mqtt_publish(const char *topic, const char *payload, int len, int qos, bool retain)
{
    if (!s_mqtt_connected) {
        return;
    }
    esp_mqtt_client_publish(s_mqtt, topic, payload, len, qos, retain ? 1 : 0);
}

static void publish_raw_stream_state(void)
{
    const char *s = s_raw_stream_enabled ? "on" : "off";
    mqtt_publish(TOPIC_STATE_RAW_STREAM, s, 0, 1, /*retain*/ true);
}

static void publish_ha_discovery_event(void)
{
    // Single retained config message that tells HA to auto-create an Event entity.
    static const char PAYLOAD[] =
        "{"
        "\"name\":\"Siri Remote\","
        "\"unique_id\":\"siri_remote_events\","
        "\"state_topic\":\"" TOPIC_EVENT "\","
        "\"event_types\":["
        "\"click\",\"double_click\",\"hold_start\",\"hold_end\","
        "\"swipe_up\",\"swipe_down\",\"swipe_left\",\"swipe_right\","
        "\"pickup\"],"
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
#define AVAILABILITY_BRIDGE_AND_REMOTE \
    "\"availability\":[" \
        "{\"topic\":\"" TOPIC_CONN "\"," \
            "\"payload_available\":\"online\"," \
            "\"payload_not_available\":\"offline\"}," \
        "{\"topic\":\"" TOPIC_REMOTE_STATUS "\"," \
            "\"payload_available\":\"connected\"," \
            "\"payload_not_available\":\"disconnected\"}" \
    "]," \
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
        "\"device\":{\"identifiers\":[\"siri_remote_bridge\"]},"
        AVAILABILITY_BRIDGE_AND_REMOTE
        "}";
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
        "\"device\":{\"identifiers\":[\"siri_remote_bridge\"]},"
        AVAILABILITY_BRIDGE_AND_REMOTE
        "}";
    mqtt_publish(TOPIC_DISCOVERY_CHARGING, PAYLOAD, sizeof(PAYLOAD) - 1, 1, /*retain*/ true);
}

static void publish_remote_status(bool connected)
{
    mqtt_publish(TOPIC_REMOTE_STATUS,
                 connected ? "connected" : "disconnected",
                 0, 1, /*retain*/ true);
}

static void publish_battery(uint8_t level)
{
    char buf[64];
    bool low = CONFIG_BATTERY_LOW_THRESHOLD_PCT > 0 &&
               level <= CONFIG_BATTERY_LOW_THRESHOLD_PCT;
    int n = snprintf(buf, sizeof(buf),
                     "{\"level\":%u,\"low\":%s}",
                     (unsigned)level, low ? "true" : "false");
    if (n > 0) {
        ESP_LOGI(TAG, "battery=%u%%%s", (unsigned)level, low ? " (LOW)" : "");
        mqtt_publish(TOPIC_BATTERY, buf, n, 1, /*retain*/ true);
    }
}

static void publish_charging(uint8_t state_byte)
{
    // BLE-standard 0x2A1A "Battery Power State" is a single byte with four
    // 2-bit fields. Bits 4..5 = charging field, 2..3 = discharging field.
    // Value 3 in either means "yes that state is active".
    uint8_t discharging = (state_byte >> 2) & 0x03;
    uint8_t charging    = (state_byte >> 4) & 0x03;
    const char *s;
    if (charging == 3) {
        s = "charging";
    } else if (discharging == 3) {
        s = "discharging";
    } else {
        s = "plugged_in";
    }
    ESP_LOGI(TAG, "charging state=0x%02x => %s", state_byte, s);
    mqtt_publish(TOPIC_CHARGING, s, 0, 1, /*retain*/ true);
}

static void publish_ha_discovery_switch(void)
{
    static const char PAYLOAD[] =
        "{"
        "\"name\":\"Raw Touch Stream\","
        "\"unique_id\":\"siri_remote_raw_stream\","
        "\"command_topic\":\"" TOPIC_CMD_RAW_STREAM "\","
        "\"state_topic\":\"" TOPIC_STATE_RAW_STREAM "\","
        "\"payload_on\":\"on\","
        "\"payload_off\":\"off\","
        "\"state_on\":\"on\","
        "\"state_off\":\"off\","
        "\"device\":{\"identifiers\":[\"siri_remote_bridge\"]},"
        "\"availability_topic\":\"" TOPIC_CONN "\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "\"entity_category\":\"config\","
        "\"icon\":\"mdi:gesture-tap-hold\""
        "}";
    mqtt_publish(TOPIC_DISCOVERY_SWITCH, PAYLOAD, sizeof(PAYLOAD) - 1, 1, /*retain*/ true);
}

static void publish_touch_raw(const siri_touch_frame_t *f)
{
    char buf[96];
    int n;
    if (f->finger_down) {
        n = snprintf(buf, sizeof(buf),
                     "{\"x\":%" PRId32 ",\"y\":%" PRId32 ",\"p\":%u,\"down\":true}",
                     f->x, f->y, (unsigned)f->pressure);
    } else {
        n = snprintf(buf, sizeof(buf), "{\"down\":false}");
    }
    if (n > 0) {
        mqtt_publish(TOPIC_TOUCH_RAW, buf, n, 0, /*retain*/ false);
    }
}

// --- event_state emit callback ---

static const char *action_to_event_type(event_action_t a)
{
    switch (a) {
    case EVT_CLICK:        return "click";
    case EVT_DOUBLE_CLICK: return "double_click";
    case EVT_HOLD_START:   return "hold_start";
    case EVT_HOLD_END:     return "hold_end";
    case EVT_SWIPE_UP:     return "swipe_up";
    case EVT_SWIPE_DOWN:   return "swipe_down";
    case EVT_SWIPE_LEFT:   return "swipe_left";
    case EVT_SWIPE_RIGHT:  return "swipe_right";
    case EVT_PICKUP:       return "pickup";
    }
    return NULL;
}

static void emit_cb(const event_state_event_t *evt, void *user)
{
    (void)user;
    const char *event_type = action_to_event_type(evt->action);
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
        n = snprintf(buf, sizeof(buf),
                     "{\"event_type\":\"%s\",\"button\":\"%s\"}",
                     event_type, btn ? btn : "unknown");
        break;
    }
    case EVT_SWIPE_UP:
    case EVT_SWIPE_DOWN:
    case EVT_SWIPE_LEFT:
    case EVT_SWIPE_RIGHT:
        n = snprintf(buf, sizeof(buf),
                     "{\"event_type\":\"%s\",\"distance\":%" PRId32 "}",
                     event_type, evt->distance);
        break;
    case EVT_PICKUP:
        // wake_press_likely=true means the user almost certainly pressed a
        // button to wake the remote — but on cold-boot reconnects, Apple
        // drops the wake-press notify, so the specific button identity may
        // never arrive on 0x0039. HA automations should treat pickup as the
        // generic "user activated remote" signal (see README).
        n = snprintf(buf, sizeof(buf),
                     "{\"event_type\":\"%s\",\"idle_duration_ms\":%" PRIu32 ",\"wake_press_likely\":true}",
                     event_type, evt->idle_duration_ms);
        break;
    }
    if (n > 0) {
        ESP_LOGI(TAG, "emit %.*s", n, buf);
        mqtt_publish(TOPIC_EVENT, buf, n, 0, /*retain*/ false);
    }
}

// --- siri_ble callbacks ---

static void on_notify_cb(uint16_t attr_handle, const uint8_t *data, size_t len, void *user)
{
    (void)user;
    s_last_activity_ms = now_ms();
    if (attr_handle == 0x0039) {
        uint16_t btns = siri_decode_button_bytes(data, len);
        xSemaphoreTake(s_es_lock, portMAX_DELAY);
        event_state_feed_buttons(s_es, btns, now_ms());
        xSemaphoreGive(s_es_lock);
    } else if (attr_handle == 0x002E) {
        if (len >= 1) {
            publish_battery(data[0]);
        }
    } else if (attr_handle == 0x0031) {
        if (len >= 1) {
            publish_charging(data[0]);
        }
    } else if (attr_handle == 0x003D) {
        siri_touch_frame_t frame;
        if (siri_decode_touch_frame(data, len, &frame)) {
#ifdef CONFIG_DEBUG_TOUCH_FRAMES
            // Diagnostic dump: raw bytes + parsed view. Used to study how the
            // circular touchpad encodes positions at its edges (small swipes
            // near top/bottom misclassify direction).
            ESP_LOGI(TAG,
                     "touch raw=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x"
                     " | x=%" PRId32 " y=%" PRId32 " p=%u down=%d ctr=%" PRIu32,
                     data[0], data[1], data[2], data[3], data[4], data[5],
                     data[6], data[7], data[8], data[9], data[10],
                     frame.x, frame.y, (unsigned)frame.pressure,
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
    publish_remote_status(true);
    s_last_activity_ms = now_ms();  // start the idle clock
    xSemaphoreTake(s_es_lock, portMAX_DELAY);
    event_state_feed_connect(s_es, idle_ms, now_ms());
    xSemaphoreGive(s_es_lock);
}

static void on_disconnected_cb(void *user)
{
    (void)user;
    publish_remote_status(false);
    s_last_activity_ms = 0;  // halt the idle check until next connection
    xSemaphoreTake(s_es_lock, portMAX_DELAY);
    event_state_reset(s_es, now_ms());
    xSemaphoreGive(s_es_lock);
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
    (void)arg; (void)data;
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
        default: break;
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

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid,     CONFIG_WIFI_SSID,     sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// --- MQTT ---

static void handle_raw_stream_cmd(const char *data, int len)
{
    bool on;
    if (len == 2 && memcmp(data, "on", 2) == 0) {
        on = true;
    } else if (len == 3 && memcmp(data, "off", 3) == 0) {
        on = false;
    } else {
        ESP_LOGW(TAG, "mqtt: unknown raw_stream command (%d bytes)", len);
        return;
    }
    if (on == s_raw_stream_enabled) {
        return;
    }
    s_raw_stream_enabled = on;
    nvs_save_raw_stream_enabled(on);
    publish_raw_stream_state();
    ESP_LOGI(TAG, "raw_stream %s", on ? "ON" : "OFF");
}

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id, void *data)
{
    (void)args; (void)base;
    esp_mqtt_event_handle_t evt = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "mqtt: connected");
        s_mqtt_connected = true;
        esp_mqtt_client_publish(s_mqtt, TOPIC_CONN, "online", 0, 1, /*retain*/ 1);
        publish_ha_discovery_event();
        publish_ha_discovery_switch();
        publish_ha_discovery_battery();
        publish_ha_discovery_charging();
        publish_raw_stream_state();
        // Default remote status to disconnected on MQTT (re)connect — BLE
        // callbacks will flip it to "connected" once encryption succeeds.
        publish_remote_status(false);
        esp_mqtt_client_subscribe(s_mqtt, TOPIC_CMD_RAW_STREAM, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "mqtt: disconnected");
        s_mqtt_connected = false;
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "mqtt: subscribed (msg_id=%d)", evt->msg_id);
        break;
    case MQTT_EVENT_DATA: {
        // Diagnostic: log every received topic + payload until 3A.2 raw-stream
        // Switch is verified working, then drop back to a single match path.
        char topic_buf[80];
        int tl = (evt->topic != NULL && evt->topic_len > 0)
                     ? (evt->topic_len < (int)sizeof(topic_buf) - 1
                            ? evt->topic_len
                            : (int)sizeof(topic_buf) - 1)
                     : 0;
        if (tl > 0) {
            memcpy(topic_buf, evt->topic, tl);
        }
        topic_buf[tl] = '\0';
        char data_buf[32];
        int dl = evt->data_len < (int)sizeof(data_buf) - 1
                     ? evt->data_len
                     : (int)sizeof(data_buf) - 1;
        if (dl > 0) {
            memcpy(data_buf, evt->data, dl);
        }
        data_buf[dl] = '\0';
        ESP_LOGI(TAG, "mqtt RX: topic='%s' (len=%d) data='%s' (len=%d)",
                 topic_buf, evt->topic_len, data_buf, evt->data_len);

        if (evt->topic != NULL &&
            evt->topic_len == (int)strlen(TOPIC_CMD_RAW_STREAM) &&
            memcmp(evt->topic, TOPIC_CMD_RAW_STREAM, evt->topic_len) == 0) {
            handle_raw_stream_cmd(evt->data, evt->data_len);
        }
        break;
    }
    default:
        break;
    }
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri             = CONFIG_MQTT_BROKER_URI,
        .credentials.username           = CONFIG_MQTT_USERNAME,
        .credentials.authentication.password = CONFIG_MQTT_PASSWORD,
        .session.last_will.topic        = TOPIC_CONN,
        .session.last_will.msg          = "offline",
        .session.last_will.msg_len      = 7,
        .session.last_will.qos          = 1,
        .session.last_will.retain       = 1,
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
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

    s_raw_stream_enabled = nvs_load_raw_stream_enabled();
    ESP_LOGI(TAG, "raw_stream initial state: %s", s_raw_stream_enabled ? "ON" : "OFF");

    s_es_lock = xSemaphoreCreateMutex();
    assert(s_es_lock != NULL);

    event_state_config_t es_cfg = {
        .double_click_max_ms         = CONFIG_EVENT_DOUBLE_WINDOW_MS,
        .hold_min_ms                 = CONFIG_EVENT_HOLD_THRESHOLD_MS,
        .swipe_min_distance          = CONFIG_EVENT_SWIPE_MIN_DISTANCE,
        .swipe_y_priority_threshold  = CONFIG_EVENT_SWIPE_Y_PRIORITY,
        .pickup_idle_threshold_ms    = CONFIG_EVENT_PICKUP_IDLE_THRESHOLD_MS,
    };
    s_es = event_state_create(&es_cfg, emit_cb, NULL);
    assert(s_es != NULL);

    esp_timer_create_args_t ta = {
        .callback = tick_timer_cb,
        .name     = "event_state_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_tick_timer, TICK_PERIOD_MS * 1000));

    wifi_start();
    mqtt_start();

    ESP_ERROR_CHECK(nimble_port_init());
    ble_store_config_init();

    siri_ble_config_t ble_cfg = {
        .on_notify       = on_notify_cb,
        .on_connected    = on_connected_cb,
        .on_disconnected = on_disconnected_cb,
        .user            = NULL,
    };
    ESP_ERROR_CHECK(siri_ble_start(&ble_cfg));
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "booted");
}
