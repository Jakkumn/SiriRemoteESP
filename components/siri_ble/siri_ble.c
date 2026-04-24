#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"

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

// Expected gen-3 characteristic value handles (empirically verified).
// We look up CCCDs by walking the discovered peer for a char with this val_handle;
// if gen-3 firmware shifts handles, we log a mismatch warning and proceed with what we found.
#define EXPECTED_BUTTON_VAL_HANDLE 0x0039
#define EXPECTED_TOUCH_VAL_HANDLE  0x003D
#define EXPECTED_BUTTON_CCCD       0x003A
#define EXPECTED_TOUCH_CCCD        0x003E

// Vendor-specific unlock — no standard UUID, handle is hardcoded per Yanndroid gen-3 source.
#define MAGIC_HANDLE 0x004D
static const uint8_t MAGIC_VALUE[2]   = {0xF0, 0x00};
static const uint8_t ENABLE_NOTIFY[2] = {0x01, 0x00};

static siri_ble_config_t s_cfg;
static uint8_t s_own_addr_type;
static uint8_t s_target_mac[6];  // parsed from CONFIG_SIRI_REMOTE_MAC at startup

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);

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

static uint16_t find_cccd_for_val_handle(const struct peer *peer, uint16_t val_handle)
{
    struct peer_svc *svc;
    SLIST_FOREACH(svc, &peer->svcs, next) {
        struct peer_chr *chr;
        SLIST_FOREACH(chr, &svc->chrs, next) {
            if (chr->chr.val_handle != val_handle) {
                continue;
            }
            struct peer_dsc *dsc;
            SLIST_FOREACH(dsc, &chr->dscs, next) {
                if (ble_uuid_u16(&dsc->dsc.uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
                    return dsc->dsc.handle;
                }
            }
        }
    }
    return 0;
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

static void log_peer_layout(const struct peer *peer)
{
    ESP_LOGI(TAG, "peer layout:");
    struct peer_svc *svc;
    SLIST_FOREACH(svc, &peer->svcs, next) {
        char uuid_buf[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&svc->svc.uuid.u, uuid_buf);
        ESP_LOGI(TAG, "  svc %s  [0x%04x..0x%04x]",
                 uuid_buf, svc->svc.start_handle, svc->svc.end_handle);
        struct peer_chr *chr;
        SLIST_FOREACH(chr, &svc->chrs, next) {
            ble_uuid_to_str(&chr->chr.uuid.u, uuid_buf);
            ESP_LOGI(TAG, "    chr %s  def=0x%04x val=0x%04x props=0x%02x",
                     uuid_buf, chr->chr.def_handle, chr->chr.val_handle,
                     chr->chr.properties);
            struct peer_dsc *dsc;
            SLIST_FOREACH(dsc, &chr->dscs, next) {
                ble_uuid_to_str(&dsc->dsc.uuid.u, uuid_buf);
                ESP_LOGI(TAG, "      dsc %s  handle=0x%04x",
                         uuid_buf, dsc->dsc.handle);
            }
        }
    }
}

static void on_disc_complete(const struct peer *peer, int status, void *arg)
{
    if (status != 0) {
        ESP_LOGE(TAG, "service discovery failed: status=%d", status);
        (void)ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    log_peer_layout(peer);

    uint16_t cccd_btn   = find_cccd_for_val_handle(peer, EXPECTED_BUTTON_VAL_HANDLE);
    uint16_t cccd_touch = find_cccd_for_val_handle(peer, EXPECTED_TOUCH_VAL_HANDLE);

    if (cccd_btn != 0 && cccd_btn != EXPECTED_BUTTON_CCCD) {
        ESP_LOGW(TAG, "button CCCD is 0x%04x (expected 0x%04x)",
                 cccd_btn, EXPECTED_BUTTON_CCCD);
    }
    if (cccd_touch != 0 && cccd_touch != EXPECTED_TOUCH_CCCD) {
        ESP_LOGW(TAG, "touch CCCD is 0x%04x (expected 0x%04x)",
                 cccd_touch, EXPECTED_TOUCH_CCCD);
    }

    enable_cccd(peer->conn_handle, cccd_btn,   "button CCCD");
    enable_cccd(peer->conn_handle, cccd_touch, "touch CCCD");

    // Magic unlock has no discoverable UUID — hardcoded handle from gen-3 source.
    int rc = ble_gattc_write_flat(peer->conn_handle, MAGIC_HANDLE,
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
        start_scan();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu=%d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "encryption changed, status=%d", event->enc_change.status);
        if (event->enc_change.status == 0) {
            int rc = peer_disc_all(event->enc_change.conn_handle,
                                   on_disc_complete, NULL);
            if (rc != 0) {
                // BLE_HS_EBUSY (7) if MTU exchange is still pending. A subsequent
                // ENC_CHANGE fires after MTU completes and retries successfully.
                // Tracked in the Phase 3 hardening memory.
                ESP_LOGD(TAG, "peer_disc_all deferred: rc=%d", rc);
            }
        }
        return 0;

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
            s_cfg.on_notify(event->notify_rx.attr_handle, buf, copied);
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
