#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "siri_ble.h"
#include "esp_central.h"

static const char *TAG = "siri_ble";

// Gen-3 GATT handles, empirically verified in Phase 1 and reproducible across
// reconnects on the same unit. We skip GATT service discovery on the live path
// (~3.7 seconds saved per connection) and write directly to these handles.
// If a future unit has a different layout, set CONFIG_SIRI_BLE_VERBOSE_DISCOVERY
// in menuconfig to re-enable a one-time peer-layout dump on first connect.
#define BUTTON_CCCD_HANDLE   0x003A
#define TOUCH_CCCD_HANDLE    0x003E
#define BATTERY_CCCD_HANDLE  0x002F  // svc 0x180F, char 0x2A19 (battery level)
#define CHARGING_CCCD_HANDLE 0x0032  // svc 0x180F, char 0x2A1A (battery power state)
#define MAGIC_HANDLE         0x004D
static const uint8_t MAGIC_VALUE[2]   = {0xF0, 0x00};
static const uint8_t ENABLE_NOTIFY[2] = {0x01, 0x00};

static siri_ble_config_t s_cfg;
static uint8_t s_own_addr_type;
static uint8_t s_target_mac[6];
static uint32_t s_last_disconnect_ms;
static bool s_has_disconnected;
static uint16_t s_setup_done_conn = 0xFFFF;  // dedup: per-connection setup runs once
#ifdef CONFIG_DEBUG_WAKE_PROBE
static uint32_t s_connect_ms;  // BLE_GAP_EVENT_CONNECT timestamp; basis for t=Xms deltas
#endif

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

#ifdef CONFIG_DEBUG_WAKE_PROBE
#define WAKE_PROBE_LOG(fmt, ...) \
    ESP_LOGI(TAG, "wake_probe t=%lums " fmt, \
             (unsigned long)(now_ms() - s_connect_ms), ##__VA_ARGS__)
#else
#define WAKE_PROBE_LOG(fmt, ...) ((void)0)
#endif

// Parse "aa:bb:cc:dd:ee:ff" into NimBLE's little-endian 6-byte layout (byte 0 = 0xff).
static bool parse_mac(const char *str, uint8_t out[6])
{
    unsigned int a[6];
    if (sscanf(str, "%2x:%2x:%2x:%2x:%2x:%2x",
               &a[5], &a[4], &a[3], &a[2], &a[1], &a[0]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        out[i] = (uint8_t)a[i];
    }
    return true;
}

static void log_addr(const char *prefix, const uint8_t val[6])
{
    ESP_LOGI(TAG, "%s %02x:%02x:%02x:%02x:%02x:%02x",
             prefix, val[5], val[4], val[3], val[2], val[1], val[0]);
}

static void start_scan(void)
{
    struct ble_gap_disc_params params = {
        .itvl = 0,
        .window = 0,
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,  // active scan so we receive scan responses (carries the full name)
        .filter_duplicates = 1,
    };

    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "scanning for %02x:%02x:%02x:%02x:%02x:%02x",
             s_target_mac[5], s_target_mac[4], s_target_mac[3],
             s_target_mac[2], s_target_mac[1], s_target_mac[0]);
}

static bool should_connect(const struct ble_gap_disc_desc *disc)
{
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
        return false;
    }
    return memcmp(disc->addr.val, s_target_mac, 6) == 0;
}

static void connect_to(const struct ble_gap_disc_desc *disc)
{
    (void)ble_gap_disc_cancel();

    int rc = ble_gap_connect(s_own_addr_type, &disc->addr, 30000, NULL,
                             gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
        start_scan();
    }
}

// Setup steps are issued sequentially, one at a time. NimBLE's per-connection
// GATT procedure queue (CONFIG_BT_NIMBLE_GATT_MAX_PROCS, default 4) overflows
// when we fire all 5+ operations concurrently — by chaining via the completion
// callback we keep at most 1 procedure in flight regardless of how many steps
// the sequence grows to.
//
// Reads-after-subscribe matter for battery + charging: those characteristics
// support read+notify (props 0x12). The remote only notifies on *change*, so
// without an initial read the value stays empty in HA until the first state
// transition (e.g. plugging in). The reads pull the current value immediately
// and dispatch it through the same on_notify path.
typedef enum {
    SETUP_KIND_WRITE,
    SETUP_KIND_READ,
} setup_kind_t;

typedef struct {
    setup_kind_t   kind;
    uint16_t       handle;
    const uint8_t *value;  // SETUP_KIND_WRITE only
    size_t         len;    // SETUP_KIND_WRITE only
    const char    *label;
} setup_step_t;

static const setup_step_t SETUP_STEPS[] = {
    {SETUP_KIND_WRITE, BUTTON_CCCD_HANDLE,   ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "button CCCD"},
    {SETUP_KIND_WRITE, TOUCH_CCCD_HANDLE,    ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "touch CCCD"},
    {SETUP_KIND_WRITE, BATTERY_CCCD_HANDLE,  ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "battery CCCD"},
    {SETUP_KIND_WRITE, CHARGING_CCCD_HANDLE, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "charging CCCD"},
    {SETUP_KIND_WRITE, MAGIC_HANDLE,         MAGIC_VALUE,   sizeof(MAGIC_VALUE),   "magic unlock"},
    {SETUP_KIND_READ,  0x002E,               NULL,          0,                     "battery initial read"},
    {SETUP_KIND_READ,  0x0031,               NULL,          0,                     "charging initial read"},
#ifdef CONFIG_DEBUG_WAKE_PROBE
    // Unexplored HID Report CCCDs — gen-3 has 9 Report characteristics; we
    // already use 0x0035/0x0039/0x003D/0x004D. The rest are guess-CCCDs at
    // val_handle+1; some will fail (not actually CCCDs) and the chain will
    // log + skip via on_setup_step_done's error path.
    {SETUP_KIND_WRITE, 0x0042, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0042"},
    {SETUP_KIND_WRITE, 0x0046, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0046"},
    {SETUP_KIND_WRITE, 0x004A, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x004A"},
    {SETUP_KIND_WRITE, 0x0051, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0051"},
    {SETUP_KIND_WRITE, 0x0054, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0054"},
    {SETUP_KIND_WRITE, 0x0056, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0056"},
    {SETUP_KIND_WRITE, 0x0058, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0058"},
#endif
};
#define SETUP_STEP_COUNT (sizeof(SETUP_STEPS) / sizeof(SETUP_STEPS[0]))

static uint16_t s_setup_step_conn;
static size_t   s_setup_step_idx;

static int on_setup_step_done(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg);

static void issue_setup_step(size_t idx)
{
    if (idx >= SETUP_STEP_COUNT) {
        return;
    }
    const setup_step_t *step = &SETUP_STEPS[idx];
    int rc;
    if (step->kind == SETUP_KIND_WRITE) {
        rc = ble_gattc_write_flat(s_setup_step_conn, step->handle,
                                  step->value, step->len,
                                  on_setup_step_done, (void *)step);
    } else {
        rc = ble_gattc_read(s_setup_step_conn, step->handle,
                            on_setup_step_done, (void *)step);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "%s: queue failed: rc=%d", step->label, rc);
        // Skip this step rather than stalling the chain.
        s_setup_step_idx++;
        issue_setup_step(s_setup_step_idx);
    }
}

static int on_setup_step_done(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg)
{
    const setup_step_t *step = arg;
    WAKE_PROBE_LOG("setup step '%s' done status=0x%x", step->label, error->status);
    if (error->status != 0) {
        ESP_LOGE(TAG, "%s failed: status=0x%x att_handle=0x%04x",
                 step->label, error->status, error->att_handle);
    } else if (step->kind == SETUP_KIND_READ && attr != NULL && attr->om != NULL) {
        uint16_t len = OS_MBUF_PKTLEN(attr->om);
        uint8_t buf[16];
        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(attr->om, buf, len, &copied) == 0) {
            ESP_LOGI(TAG, "%s ok (handle=0x%04x len=%u)",
                     step->label, attr->handle, (unsigned)copied);
            if (s_cfg.on_notify != NULL) {
                s_cfg.on_notify(attr->handle, buf, copied, s_cfg.user);
            }
        }
    } else {
        ESP_LOGI(TAG, "%s ok (handle=0x%04x)",
                 step->label, attr != NULL ? attr->handle : 0);
    }
    s_setup_step_idx++;
    issue_setup_step(s_setup_step_idx);
    return 0;
}

// Fast-path connection setup. Subscribes to button / touch / battery / charging
// notifications and fires the magic unlock — all serialized via callback chain
// so NimBLE's GATT procedure queue is never saturated. Idempotent per
// connection: a second ENC_CHANGE for the same conn_handle (bonded resume edge
// case) is a no-op.
//
// Wake-press loss caveat: Apple's gen-3 firmware appears to gate HID notify
// delivery on the accessory-framework's secondary encryption phase (the second
// ENC_CHANGE that fires ~750–1200 ms after the first). On warm reconnects the
// press waiting in HID is delivered just after that completes; on cold-boot
// first-bond reconnects the longer handshake path drops the buffered press
// entirely. The pickup event still fires (driven by reconnect timing, not HID),
// so HA automations should treat pickup as the wake-intent signal — the
// specific button identity is not always recoverable. Use CONFIG_DEBUG_WAKE_PROBE
// to instrument the path with t=Xms timestamps and probe unexplored handles.
static void apply_remote_setup(uint16_t conn_handle)
{
    if (s_setup_done_conn == conn_handle) {
        return;
    }
    s_setup_done_conn = conn_handle;
    s_setup_step_conn = conn_handle;
    s_setup_step_idx  = 0;
    issue_setup_step(0);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        // Every non-matching adv is logged at DEBUG — bump esp_log_level_set("siri_ble", ESP_LOG_DEBUG)
        // if you need to see the full scan stream (e.g. hunting for a different remote's MAC).
        const uint8_t *a = event->disc.addr.val;
        if (!should_connect(&event->disc)) {
            ESP_LOGD(TAG, "adv %02x:%02x:%02x:%02x:%02x:%02x rssi=%d",
                     a[5], a[4], a[3], a[2], a[1], a[0], event->disc.rssi);
            return 0;
        }
        log_addr("matching adv", a);
        ESP_LOGI(TAG, "  rssi=%d", event->disc.rssi);
        connect_to(&event->disc);
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "connect failed: status=%d", event->connect.status);
            start_scan();
            return 0;
        }
        ESP_LOGI(TAG, "connected, conn_handle=%d", event->connect.conn_handle);
#ifdef CONFIG_DEBUG_WAKE_PROBE
        s_connect_ms = now_ms();
#endif
        WAKE_PROBE_LOG("CONNECT");

        if (peer_add(event->connect.conn_handle) != 0) {
            ESP_LOGE(TAG, "peer_add failed");
        }
        (void)ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
        // On bonded reconnect, NimBLE auto-kicks encryption from its host task
        // before our explicit call lands. The call returns BLE_HS_EALREADY (2)
        // and the automatic path completes encryption successfully. Treat that
        // as success — only log other failure modes.
        int sec_rc = ble_gap_security_initiate(event->connect.conn_handle);
        if (sec_rc != 0 && sec_rc != BLE_HS_EALREADY) {
            ESP_LOGE(TAG, "security_initiate failed: rc=%d", sec_rc);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected, reason=0x%04x", event->disconnect.reason);
        peer_delete(event->disconnect.conn.conn_handle);
        s_last_disconnect_ms = now_ms();
        s_has_disconnected   = true;
        s_setup_done_conn    = 0xFFFF;
        if (s_cfg.on_disconnected != NULL) {
            s_cfg.on_disconnected(s_cfg.user);
        }
        start_scan();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu=%d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        ESP_LOGI(TAG, "encryption changed, status=%d", event->enc_change.status);
        WAKE_PROBE_LOG("ENC_CHANGE status=%d %s",
                       event->enc_change.status,
                       s_setup_done_conn == event->enc_change.conn_handle ? "(post-setup)" : "(initial)");
        if (event->enc_change.status != 0) {
            return 0;
        }
        // Bonded reconnect can fire ENC_CHANGE twice in quick succession —
        // the second is Apple's accessory-framework re-encryption phase, and
        // on cold-boot connects from a sleeping remote, the wake-press notify
        // can be lost during this window (see CONFIG_DEBUG_WAKE_PROBE). Dedup
        // at the conn_handle level so on_connected + setup chain run exactly
        // once per connection.
        if (s_setup_done_conn == event->enc_change.conn_handle) {
            return 0;
        }
        uint32_t idle;
        if (s_has_disconnected) {
            idle = now_ms() - s_last_disconnect_ms;
        } else {
            // First connect since bridge boot. Use bridge uptime as a lower
            // bound on idle duration — the remote was unreachable for at
            // least this long. Lets pickup fire when the remote wakes after
            // a long sleep, even when the bridge restarted in between.
            idle = now_ms();
        }
        if (s_cfg.on_connected != NULL) {
            s_cfg.on_connected(idle, s_cfg.user);
        }
        apply_remote_setup(event->enc_change.conn_handle);
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        uint8_t buf[128];
        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, buf, len, &copied) != 0) {
            ESP_LOGE(TAG, "mbuf_to_flat failed");
            return 0;
        }
        // Diagnostic: log every non-touch notification with handle + first
        // bytes. Touch (0x003D) is ~50/sec and would flood UART; it has its
        // own gated dump in on_notify_cb (CONFIG_DEBUG_TOUCH_FRAMES).
        if (event->notify_rx.attr_handle != 0x003D) {
            char hex[3 * 16 + 1];
            uint16_t n = copied < 16 ? copied : 16;
            for (uint16_t i = 0; i < n; i++) {
                snprintf(&hex[i * 3], 4, "%02x ", buf[i]);
            }
            hex[n * 3] = '\0';
#ifdef CONFIG_DEBUG_WAKE_PROBE
            ESP_LOGI(TAG, "notify t=%lums h=0x%04x len=%u %s",
                     (unsigned long)(now_ms() - s_connect_ms),
                     event->notify_rx.attr_handle, copied, hex);
#else
            ESP_LOGI(TAG, "notify h=0x%04x len=%u %s",
                     event->notify_rx.attr_handle, copied, hex);
#endif
        }
        if (s_cfg.on_notify != NULL) {
            s_cfg.on_notify(event->notify_rx.attr_handle, buf, copied, s_cfg.user);
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

void siri_ble_idle_disconnect(void)
{
    uint16_t conn = s_setup_done_conn;
    if (conn == 0xFFFF) {
        return;
    }
    ESP_LOGI(TAG, "idle threshold reached, terminating connection so remote can sleep");
    int rc = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0 && rc != BLE_HS_ENOTCONN) {
        ESP_LOGW(TAG, "ble_gap_terminate failed: %d", rc);
    }
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset, reason=%d", reason);
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
    start_scan();
}

esp_err_t siri_ble_start(const siri_ble_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;

    if (!parse_mac(CONFIG_SIRI_REMOTE_MAC, s_target_mac)) {
        ESP_LOGE(TAG, "SIRI_REMOTE_MAC \"%s\" is malformed; expected aa:bb:cc:dd:ee:ff",
                 CONFIG_SIRI_REMOTE_MAC);
        return ESP_ERR_INVALID_ARG;
    }

    ble_hs_cfg.sm_io_cap         = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_sc             = 1;
    ble_hs_cfg.sm_mitm           = 0;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    // Register the standard GAP (0x1800) and GATT (0x1801) services so Apple's
    // accessory framework gets responses to its periodic probes. Without this,
    // the remote terminates the connection after ~20-30 seconds of failed
    // probe queries, even during active use. We never advertise — these are
    // server-side responses only.
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_svc_gap_device_name_set("siri-bridge") != 0) {
        ESP_LOGW(TAG, "ble_svc_gap_device_name_set failed");
    }

    if (peer_init(1, 64, 64, 64) != 0) {
        ESP_LOGE(TAG, "peer_init failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}
