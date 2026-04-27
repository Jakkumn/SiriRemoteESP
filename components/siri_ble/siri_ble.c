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

#include "siri_ble.h"
#include "esp_central.h"

static const char *TAG = "siri_ble";

// Gen-3 GATT handles, empirically verified in Phase 1 and reproducible across
// reconnects on the same unit. We skip GATT service discovery on the live path
// (~3.7 seconds saved per connection) and write directly to these handles.
// If a future unit has a different layout, set CONFIG_SIRI_BLE_VERBOSE_DISCOVERY
// in menuconfig to re-enable a one-time peer-layout dump on first connect.
#define BUTTON_CCCD_HANDLE  0x003A
#define TOUCH_CCCD_HANDLE   0x003E
#define MAGIC_HANDLE        0x004D
static const uint8_t MAGIC_VALUE[2]   = {0xF0, 0x00};
static const uint8_t ENABLE_NOTIFY[2] = {0x01, 0x00};

static siri_ble_config_t s_cfg;
static uint8_t s_own_addr_type;
static uint8_t s_target_mac[6];
static uint32_t s_last_disconnect_ms;
static bool s_has_disconnected;
static uint16_t s_setup_done_conn = 0xFFFF;  // dedup: per-connection setup runs once

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

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

static int on_write_done(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    const char *label = arg;
    if (error->status == 0) {
        ESP_LOGI(TAG, "%s write ok (handle=0x%04x)", label, attr->handle);
    } else {
        ESP_LOGE(TAG, "%s write failed: status=0x%x att_handle=0x%04x",
                 label, error->status, error->att_handle);
    }
    return 0;
}

static void enable_cccd(uint16_t conn_handle, uint16_t cccd_handle, const char *label)
{
    if (cccd_handle == 0) {
        ESP_LOGW(TAG, "%s: CCCD not found; skipping", label);
        return;
    }
    int rc = ble_gattc_write_flat(conn_handle, cccd_handle,
                                  ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY),
                                  on_write_done, (void *)label);
    if (rc != 0) {
        ESP_LOGE(TAG, "%s: write_flat queue failed: %d", label, rc);
    }
}

// Fast-path connection setup: subscribe to button + touch notifications and
// fire the magic unlock. Idempotent per connection — calling twice (e.g. if
// ENC_CHANGE fires multiple times during bonded resume) is a no-op the second
// time. Skips GATT service discovery entirely; Phase 1 verified the gen-3
// handle layout and it's reproducible per unit, so discovery is dead weight
// on the live path (~3.7 seconds per reconnect).
static void apply_remote_setup(uint16_t conn_handle)
{
    if (s_setup_done_conn == conn_handle) {
        return;
    }
    s_setup_done_conn = conn_handle;

    enable_cccd(conn_handle, BUTTON_CCCD_HANDLE, "button CCCD");
    enable_cccd(conn_handle, TOUCH_CCCD_HANDLE,  "touch CCCD");

    int rc = ble_gattc_write_flat(conn_handle, MAGIC_HANDLE,
                                  MAGIC_VALUE, sizeof(MAGIC_VALUE),
                                  on_write_done, (void *)"magic unlock");
    if (rc != 0) {
        ESP_LOGE(TAG, "magic unlock: write_flat queue failed: %d", rc);
    }
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

        if (peer_add(event->connect.conn_handle) != 0) {
            ESP_LOGE(TAG, "peer_add failed");
        }
        (void)ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
        if (ble_gap_security_initiate(event->connect.conn_handle) != 0) {
            // Benign on bonded reconnect: NimBLE auto-kicks encryption and our call
            // returns BLE_HS_EALREADY. Tracked in the Phase 3 hardening memory.
            ESP_LOGD(TAG, "security_initiate skipped (already in progress)");
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
        if (event->enc_change.status != 0) {
            return 0;
        }
        uint32_t idle = 0;
        if (s_has_disconnected) {
            idle = now_ms() - s_last_disconnect_ms;
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

    if (peer_init(1, 64, 64, 64) != 0) {
        ESP_LOGE(TAG, "peer_init failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}
