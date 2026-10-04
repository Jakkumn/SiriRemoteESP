// Minimal Siri Remote BLE central — no Wi-Fi, no MQTT, no button_pulse.
// Just connect, run setup, decode + log button presses to UART.
//
// Purpose: isolate whether the cold-boot / long-idle wake-press loss
// happens at the BLE layer (in which case this minimal build will also
// lose it) or whether it's caused by something in the higher-layer code
// of the production firmware (Wi-Fi/BT coexistence, MQTT processing
// stealing CPU, pulse-emitter lock contention, etc.).
//
// Build:  source $IDF_PATH/export.sh && idf.py build flash monitor
// Pair:   on first boot with empty NVS, hold remote's home+vol-up for
//         5 seconds to force pairing mode. Bridge + remote will JustWorks
//         pair; bond is persisted to NVS.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "wake_min";

// Hardcoded MAC of the developer's gen-3 Siri Remote.
// NimBLE stores addresses LSB-first.
static const uint8_t TARGET_MAC[6] = {0xd0, 0x7e, 0x9a, 0xea, 0xc3, 0xe0};

// gen-3 GATT handles (verified empirically via peer_disc_all in main project).
#define BUTTON_CCCD_HANDLE   0x003A
#define TOUCH_CCCD_HANDLE    0x003E
#define BATTERY_CCCD_HANDLE  0x002F
#define CHARGING_CCCD_HANDLE 0x0032
#define MAGIC_HANDLE         0x004D
static const uint8_t MAGIC_VALUE[2]   = {0xF0, 0x00};
static const uint8_t ENABLE_NOTIFY[2] = {0x01, 0x00};

// Setup chain serialized via callback, mirroring the main project's flow.
// CCCDs first, then magic unlock. `delay_after_ms` lets us insert a wait
// between steps — used to test whether subsequent GATT operations are
// interrupting Apple's wake-press flush after a CCCD subscribe.
typedef struct {
    uint16_t       handle;
    const uint8_t *value;
    size_t         len;
    const char    *label;
    uint32_t       delay_after_ms;
} setup_step_t;

// CONNECTED-LOW-POWER MODE: standard setup chain (button + touch + battery
// + charging CCCDs + magic unlock). After the chain completes we request a
// connection parameter update to the remote's PPCP (latency=80, intervals
// 7.5-15ms) so the remote can deep-sleep between events while we stay
// connected. IDLE_DISCONNECT_MS=0 below disables the auto-disconnect, so
// the link is permanent and wake-press never crosses the disconnect/
// reconnect boundary that's been losing it.
static const setup_step_t SETUP_STEPS[] = {
    {BUTTON_CCCD_HANDLE,   ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "button CCCD",   0},
    {TOUCH_CCCD_HANDLE,    ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "touch CCCD",    0},
    {BATTERY_CCCD_HANDLE,  ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "battery CCCD",  0},
    {CHARGING_CCCD_HANDLE, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "charging CCCD", 0},
    {MAGIC_HANDLE,         MAGIC_VALUE,   sizeof(MAGIC_VALUE),   "magic unlock",  0},
};
#define SETUP_STEP_COUNT (sizeof(SETUP_STEPS) / sizeof(SETUP_STEPS[0]))

static uint8_t  s_own_addr_type;
static uint16_t s_setup_done_conn = 0xFFFF;
static uint16_t s_pending_setup_conn = 0xFFFF;
static uint16_t s_setup_step_conn;
static size_t   s_setup_step_idx;
static uint32_t s_connect_ms;
static uint32_t s_last_activity_ms;
static uint16_t s_active_conn = 0xFFFF;
static esp_timer_handle_t s_setup_step_timer;

// CONNECTED-LOW-POWER MODE: never disconnect. The remote stays asleep
// between conn events (slave latency 80 means it can skip 80 events ≈
// 1.2 s of effective deep-sleep at 15 ms intervals) but the link itself
// is alive, so a button press always lands as a HID notify within ~15 ms
// of the press. Set to a non-zero value to re-enable the disconnect-then-
// wait behaviour for testing.
#define IDLE_DISCONNECT_MS 0

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

#define WAKE_LOG(fmt, ...) \
    ESP_LOGI(TAG, "t=%lums " fmt, \
             (unsigned long)(now_ms() - s_connect_ms), ##__VA_ARGS__)

// Decode a Siri Remote gen-3 button bitmap (2 bytes, LE) into a name.
// Returns NULL if no buttons or multi-button chord.
static const char *decode_button(const uint8_t *data, size_t len)
{
    if (len < 2) return NULL;
    uint16_t bm = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    if (bm == 0) return NULL;
    // Single-bit detection: if more than one bit set, return "chord"
    if ((bm & (bm - 1)) != 0) return "chord";
    switch (bm) {
    case 0x0001: return "tv";
    case 0x0002: return "volume_up";
    case 0x0004: return "volume_down";
    case 0x0008: return "select";
    case 0x0010: return "power";
    case 0x0020: return "mic";
    case 0x0040: return "back";
    case 0x0080: return "mute";
    case 0x0100: return "play_pause";
    case 0x0200: return "up";
    case 0x0400: return "right";
    case 0x0800: return "down";
    case 0x1000: return "left";
    default:     return "unknown";
    }
}

static int on_setup_step_done(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg);

// Tunable conn params for the latency-sweep experiment. Constraint from BLE
// spec: supervision_timeout > 2 * conn_interval_max * (1 + slave_latency).
// At itvl_max=12 (15 ms) that's:
//   latency  80 → timeout >  2.43 s   (PPCP baseline; remote's advertised pref)
//   latency 200 → timeout >  6.03 s
//   latency 400 → timeout > 12.03 s
// Apple's peripheral may reject latencies far above its PPCP — if it does,
// CONN_UPDATE fires with non-zero status and we keep whatever was negotiated
// at connect.
#define LP_ITVL_MIN     6     // 7.5 ms
#define LP_ITVL_MAX     12    // 15 ms
#define LP_LATENCY      400   // try 80, 200, 400
#define LP_SUP_TIMEOUT  1500  // 15 s — covers up to latency=400 with margin

static void request_low_power_conn_params(uint16_t conn_handle)
{
    const struct ble_gap_upd_params p = {
        .itvl_min            = LP_ITVL_MIN,
        .itvl_max            = LP_ITVL_MAX,
        .latency             = LP_LATENCY,
        .supervision_timeout = LP_SUP_TIMEOUT,
        .min_ce_len          = 0,
        .max_ce_len          = 0,
    };
    int rc = ble_gap_update_params(conn_handle, &p);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_update_params rc=%d", rc);
    } else {
        WAKE_LOG("requested low-power conn params (%u.%u-%u.%ums, lat=%d, sup=%dms)",
                 (LP_ITVL_MIN * 125) / 100, (LP_ITVL_MIN * 125) % 100,
                 (LP_ITVL_MAX * 125) / 100, (LP_ITVL_MAX * 125) % 100,
                 LP_LATENCY, LP_SUP_TIMEOUT * 10);
    }
}

static void issue_setup_step(size_t idx)
{
    if (idx >= SETUP_STEP_COUNT) {
        WAKE_LOG("setup chain complete");
        request_low_power_conn_params(s_setup_step_conn);
        return;
    }
    const setup_step_t *step = &SETUP_STEPS[idx];
    int rc = ble_gattc_write_flat(s_setup_step_conn, step->handle,
                                  step->value, step->len,
                                  on_setup_step_done, (void *)step);
    if (rc != 0) {
        ESP_LOGE(TAG, "%s queue failed rc=%d", step->label, rc);
        s_setup_step_idx++;
        issue_setup_step(s_setup_step_idx);
    }
}

static void setup_step_resume_cb(void *arg)
{
    WAKE_LOG("delay expired, continuing setup chain");
    issue_setup_step(s_setup_step_idx);
}

static int on_setup_step_done(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg)
{
    const setup_step_t *step = arg;
    if (error->status != 0) {
        ESP_LOGE(TAG, "%s failed status=0x%x", step->label, error->status);
    } else {
        WAKE_LOG("%s ok", step->label);
    }
    s_setup_step_idx++;
    if (step->delay_after_ms > 0 && s_setup_step_idx < SETUP_STEP_COUNT) {
        WAKE_LOG("delaying %lu ms before next step (watching for early notify)",
                 (unsigned long)step->delay_after_ms);
        (void)esp_timer_start_once(s_setup_step_timer,
                                   (uint64_t)step->delay_after_ms * 1000);
    } else {
        issue_setup_step(s_setup_step_idx);
    }
    return 0;
}

static void apply_remote_setup(uint16_t conn_handle)
{
    if (s_setup_done_conn == conn_handle) return;
    s_setup_done_conn = conn_handle;
    s_setup_step_conn = conn_handle;
    s_setup_step_idx  = 0;
    issue_setup_step(0);
}

static void start_scan(void)
{
    struct ble_gap_disc_params params = {
        .filter_duplicates = 1,
        .passive           = 0,
    };
    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "scanning for %02x:%02x:%02x:%02x:%02x:%02x",
             TARGET_MAC[5], TARGET_MAC[4], TARGET_MAC[3],
             TARGET_MAC[2], TARGET_MAC[1], TARGET_MAC[0]);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        if (event->disc.event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
            event->disc.event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
            return 0;
        }
        if (memcmp(event->disc.addr.val, TARGET_MAC, 6) != 0) return 0;
        ESP_LOGI(TAG, "matching adv rssi=%d", event->disc.rssi);
        (void)ble_gap_disc_cancel();
        int rc = ble_gap_connect(s_own_addr_type, &event->disc.addr, 30000, NULL,
                                 gap_event_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_connect failed rc=%d", rc);
            start_scan();
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "connect failed status=%d", event->connect.status);
            start_scan();
            return 0;
        }
        s_connect_ms = now_ms();
        s_last_activity_ms = now_ms();
        s_active_conn = event->connect.conn_handle;
        WAKE_LOG("CONNECT conn_handle=%d", event->connect.conn_handle);
        (void)ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
        int sec_rc = ble_gap_security_initiate(event->connect.conn_handle);
        if (sec_rc != 0 && sec_rc != BLE_HS_EALREADY) {
            ESP_LOGE(TAG, "security_initiate failed rc=%d", sec_rc);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        WAKE_LOG("DISCONNECT reason=0x%04x", event->disconnect.reason);
        s_setup_done_conn = 0xFFFF;
        s_pending_setup_conn = 0xFFFF;
        s_active_conn = 0xFFFF;
        start_scan();
        return 0;

    case BLE_GAP_EVENT_MTU:
        WAKE_LOG("MTU=%d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
            WAKE_LOG("CONN_UPDATE status=%d itvl=%u (=%u.%ums) lat=%u sup=%ums",
                     event->conn_update.status,
                     desc.conn_itvl,
                     (desc.conn_itvl * 125) / 100, (desc.conn_itvl * 125) % 100,
                     desc.conn_latency,
                     desc.supervision_timeout * 10);
        } else {
            WAKE_LOG("CONN_UPDATE status=%d (conn_find failed)",
                     event->conn_update.status);
        }
        return 0;
    }

    case BLE_GAP_EVENT_ENC_CHANGE: {
        if (s_setup_done_conn != event->enc_change.conn_handle &&
            s_pending_setup_conn != event->enc_change.conn_handle) {
            // First sighting of this conn — stamp s_connect_ms here in case
            // ENC_CHANGE arrives before CONNECT (NimBLE bonded-resume path).
            s_connect_ms = now_ms();
        }
        WAKE_LOG("ENC_CHANGE status=%d %s",
                 event->enc_change.status,
                 s_setup_done_conn == event->enc_change.conn_handle ? "(post-setup)"
                 : s_pending_setup_conn == event->enc_change.conn_handle ? "(secondary)"
                 : "(initial)");
        if (event->enc_change.status != 0) return 0;
        if (s_setup_done_conn == event->enc_change.conn_handle) return 0;

        uint16_t conn = event->enc_change.conn_handle;
        if (s_pending_setup_conn != conn) {
            // First ENC_CHANGE — defer until secondary fires.
            s_pending_setup_conn = conn;
            return 0;
        }
        // Second ENC_CHANGE — run setup.
        s_pending_setup_conn = 0xFFFF;
        apply_remote_setup(conn);
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        s_last_activity_ms = now_ms();
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        uint8_t buf[64];
        if (len > sizeof(buf)) len = sizeof(buf);
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, buf, len, &copied) != 0) {
            ESP_LOGE(TAG, "mbuf_to_flat failed");
            return 0;
        }
        uint16_t h = event->notify_rx.attr_handle;

        // Touch (0x003D) is ~50/sec — skip the verbose log for it.
        if (h == 0x003D) return 0;

        // Hex dump.
        char hex[3 * 16 + 1];
        uint16_t n = copied < 16 ? copied : 16;
        for (uint16_t i = 0; i < n; i++) snprintf(&hex[i * 3], 4, "%02x ", buf[i]);
        hex[n * 3] = '\0';
        WAKE_LOG("NOTIFY h=0x%04x len=%u %s", h, copied, hex);

        // Decode button if it's the button handle.
        if (h == 0x0039) {
            const char *name = decode_button(buf, copied);
            if (name == NULL) {
                WAKE_LOG("  -> button RELEASE");
            } else {
                WAKE_LOG("  -> button PRESS: %s", name);
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            (void)ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        return 0;
    }
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset reason=%d", reason);
}

static void on_sync(void)
{
    if (ble_hs_util_ensure_addr(0) != 0) {
        ESP_LOGE(TAG, "ensure_addr failed");
        return;
    }
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
        ESP_LOGE(TAG, "id_infer_auto failed");
        return;
    }
    if (ble_att_set_preferred_mtu(247) != 0) {
        ESP_LOGW(TAG, "set_preferred_mtu(247) failed");
    }
    int bonded_count = 0;
    if (ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &bonded_count) == 0) {
        ESP_LOGI(TAG, "bond store: %d persisted peer-sec", bonded_count);
    }
    start_scan();
}

static void host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Periodic check: if connected and idle longer than IDLE_DISCONNECT_MS,
// terminate the link so the remote can deep-sleep. Next press is then a
// genuine wake-press scenario for testing.
static void idle_tick_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (IDLE_DISCONNECT_MS == 0) continue;
        if (s_active_conn == 0xFFFF) continue;
        if (s_setup_done_conn == 0xFFFF) continue;  // wait for setup to complete
        uint32_t idle = now_ms() - s_last_activity_ms;
        if (idle < IDLE_DISCONNECT_MS) continue;
        ESP_LOGI(TAG, "idle %lu ms — terminating connection so remote can sleep",
                 (unsigned long)idle);
        int rc = ble_gap_terminate(s_active_conn, 0x13);
        if (rc != 0 && rc != BLE_HS_ENOTCONN) {
            ESP_LOGW(TAG, "ble_gap_terminate rc=%d", rc);
        }
        // Avoid retriggering until disconnect event fires.
        s_last_activity_ms = now_ms();
    }
}

void ble_store_config_init(void);

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sm_io_cap         = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_sc             = 1;
    ble_hs_cfg.sm_mitm           = 0;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;
    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    // Register standard GAP/GATT services so Apple's accessory framework
    // gets responses to its periodic probes.
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_svc_gap_device_name_set("wake-min") != 0) {
        ESP_LOGW(TAG, "device_name_set failed");
    }

    ble_store_config_init();

    nimble_port_freertos_init(host_task);

    const esp_timer_create_args_t step_timer_args = {
        .callback = setup_step_resume_cb,
        .name     = "setup_step_resume",
    };
    ESP_ERROR_CHECK(esp_timer_create(&step_timer_args, &s_setup_step_timer));

    xTaskCreate(idle_tick_task, "idle_tick", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "wake_minimal started (idle_disconnect=%dms)", IDLE_DISCONNECT_MS);
}
