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

// Deferred-setup state. apply_remote_setup is held back until the *second*
// ENC_CHANGE for a connection — Apple's accessory framework re-encrypts the
// link as a secondary phase ~750-1200 ms after bond resumption, and any HID
// notify Apple buffered for us during that handshake is dropped if we touch
// the GATT layer (CCCD writes etc.) before the secondary phase completes.
// By waiting, the wake-press Apple has held since the user pressed a button
// to wake the remote is delivered cleanly once our CCCD subscribes land.
static uint16_t           s_pending_setup_conn = 0xFFFF;
static uint32_t           s_pending_setup_idle_ms;
static esp_timer_handle_t s_setup_fallback_timer;

// Worst-case wait between the two ENC_CHANGE events before we run setup
// anyway. Across captured gen-3 connects the gap is reliably 750-1200 ms;
// 2500 ms gives generous headroom while still bounding worst-case latency.
#define ENC_CHANGE_FALLBACK_MS 2500

// Apple custom service UUID `8341f2b4-c013-4f04-8197-c4cdb42e26dc` (LE byte
// order — NimBLE stores UUID128s LSB-first). Phase 1 dump found this service
// on the gen-3 remote but didn't enumerate its characteristics. CONFIG_DEBUG_WAKE_PROBE
// runs peer_disc_all post-setup and dumps the full layout so we can identify
// any wake-reason / accessory-state characteristics that might explain why
// the cold-boot first-bond wake-press is dropped on the HID path.
#ifdef CONFIG_DEBUG_WAKE_PROBE
static const ble_uuid128_t APPLE_CUSTOM_SVC_UUID =
    BLE_UUID128_INIT(0xdc, 0x26, 0x2e, 0xb4, 0xcd, 0xc4, 0x97, 0x81,
                     0x04, 0x4f, 0x13, 0xc0, 0xb4, 0xf2, 0x41, 0x83);

#endif

// Connection parameters requested AFTER setup completes. itvl_min/itvl_max
// match the remote's PPCP (read at handle 0x0007: 06 00 0c 00 50 00 58 02).
// We push slave_latency well above the remote's preferred 80 — Apple's
// peripheral accepts up to at least 400, verified empirically against a
// real gen-3 unit. At 15 ms intervals with latency 400 the peripheral may
// skip up to 400 conn events ≈ 6 s of effective deep-sleep, but breaks out
// instantly when it has a button press to send. This is what lets us keep
// the link alive forever (CONFIG_IDLE_DISCONNECT_MS=0) without melting the
// CR2032 — and avoids the wake-press loss entirely, since there's no
// disconnect/reconnect path for Apple's session state to expire across.
//
// supervision_timeout must satisfy spec rule:
//   timeout > 2 * itvl_max * (1 + latency)
// At itvl_max=12 (15ms) and latency=400 the lower bound is 12.03 s; 1500
// (15 s) gives ~25 % margin, the cost being that a runaway remote takes
// ~15 s to register as gone.
//
// Phase 3B.8 exposes itvl_max / latency / supervision_timeout as runtime
// MQTT Number entities so users can trade battery-life vs. responsiveness
// vs. disconnect-detection latency without reflashing.
static const struct ble_gap_upd_params LOW_POWER_CONN_PARAMS = {
    .itvl_min            = 6,     // 6 * 1.25ms = 7.5ms
    .itvl_max            = 12,    // 12 * 1.25ms = 15ms
    .latency             = 400,
    .supervision_timeout = 1500,  // 1500 * 10ms = 15s
    .min_ce_len          = 0,
    .max_ce_len          = 0,
};

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);
static void complete_setup_chain(uint16_t conn_handle, uint32_t idle_ms);
#ifdef CONFIG_DEBUG_WAKE_PROBE
static void kick_probe_discovery(uint16_t conn_handle);
#endif

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
    // CCCD probes for the three remaining notify-capable HID Reports on
    // gen-3 (0x0041/0x0045/0x0049). Confirmed real CCCDs via peer_disc_all.
    // 0x0051 used to be in this list but is Report Reference, not a CCCD.
    {SETUP_KIND_WRITE, 0x0042, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0042"},
    {SETUP_KIND_WRITE, 0x0046, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0046"},
    {SETUP_KIND_WRITE, 0x004A, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x004A"},
    // Audio HID Report CCCD (val=0x0035) — only remaining notify-capable
    // handle on the device that we don't normally subscribe to. The "wake-
    // info on the audio channel" hypothesis: Apple may multiplex a wake-
    // packet onto the Opus channel when the wake-press is suppressed from
    // the regular button path on cold-boot first-bond reconnect.
    {SETUP_KIND_WRITE, 0x0036, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0036 (audio)"},
    // 0x0025 (Apple-svc CCCD) is now subscribed in the EARLY block above.
    // GAP service reads — Apple's claim handshake might check these.
    {SETUP_KIND_READ,  0x0003, NULL, 0, "probe read GAP name 0x0003"},
    {SETUP_KIND_READ,  0x0005, NULL, 0, "probe read GAP appearance 0x0005"},
    {SETUP_KIND_READ,  0x0007, NULL, 0, "probe read GAP ppcp 0x0007"},
    {SETUP_KIND_READ,  0x0009, NULL, 0, "probe read GAP car 0x0009"},
    // Device Info Service reads — Apple's accessory framework on iOS hosts
    // reads these as part of standard claim. Strings (mfg/model/serial/
    // hw_rev/fw_rev) print as ASCII via the read-bytes log; PnP ID is a
    // 7-byte struct.
    {SETUP_KIND_READ,  0x000d, NULL, 0, "probe read mfg 0x000d"},
    {SETUP_KIND_READ,  0x000f, NULL, 0, "probe read model 0x000f"},
    {SETUP_KIND_READ,  0x0011, NULL, 0, "probe read serial 0x0011"},
    {SETUP_KIND_READ,  0x0013, NULL, 0, "probe read hw_rev 0x0013"},
    {SETUP_KIND_READ,  0x0015, NULL, 0, "probe read fw_rev 0x0015"},
    {SETUP_KIND_READ,  0x0017, NULL, 0, "probe read PnP 0x0017"},
    // Apple custom service reads — six read-only characteristics whose
    // contents Phase 1 never inspected. Reading them may also be part of
    // Apple's claim handshake.
    {SETUP_KIND_READ,  0x001a, NULL, 0, "probe read Apple 0x001a"},
    {SETUP_KIND_READ,  0x001c, NULL, 0, "probe read Apple 0x001c"},
    {SETUP_KIND_READ,  0x001e, NULL, 0, "probe read Apple 0x001e"},
    {SETUP_KIND_READ,  0x0020, NULL, 0, "probe read Apple 0x0020"},
    {SETUP_KIND_READ,  0x0022, NULL, 0, "probe read Apple 0x0022"},
    {SETUP_KIND_READ,  0x0024, NULL, 0, "probe read Apple 0x0024"},
    // Bond Management feature read.
    {SETUP_KIND_READ,  0x0028, NULL, 0, "probe read BondMgmt feat 0x0028"},
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
        // Setup chain complete — switch to the remote's preferred low-power
        // conn params so it can deep-sleep between events. Lets us keep the
        // link alive forever (which avoids the wake-press loss) without
        // burning the CR2032 to charge it every couple of weeks.
        int rc = ble_gap_update_params(s_setup_step_conn, &LOW_POWER_CONN_PARAMS);
        if (rc != 0 && rc != BLE_HS_ENOTCONN) {
            ESP_LOGW(TAG, "post-setup conn-param update rc=%d", rc);
        }
#ifdef CONFIG_DEBUG_WAKE_PROBE
        // Setup chain complete — kick off a one-shot peer_disc_all to dump
        // the full GATT layout and auto-subscribe to Apple custom service
        // notify chars. Investigates the cold-boot wake-press loss.
        kick_probe_discovery(s_setup_step_conn);
#endif
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
        uint8_t buf[64];
        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(attr->om, buf, len, &copied) == 0) {
#ifdef CONFIG_DEBUG_WAKE_PROBE
            char hex[3 * 32 + 1];
            uint16_t n = copied < 32 ? copied : 32;
            for (uint16_t i = 0; i < n; i++) {
                snprintf(&hex[i * 3], 4, "%02x ", buf[i]);
            }
            hex[n * 3] = '\0';
            ESP_LOGI(TAG, "%s ok (handle=0x%04x len=%u) %s",
                     step->label, attr->handle, (unsigned)copied, hex);
#else
            ESP_LOGI(TAG, "%s ok (handle=0x%04x len=%u)",
                     step->label, attr->handle, (unsigned)copied);
#endif
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
// connection: a second invocation for the same conn_handle is a no-op.
//
// IMPORTANT: only call this *after* the second BLE_GAP_EVENT_ENC_CHANGE has
// fired (Apple's accessory-framework secondary encryption). Subscribing CCCDs
// during that handshake window causes Apple to drop the wake-press it has
// buffered since the user pressed the button to wake the remote. The
// ENC_CHANGE handler in gap_event_cb defers this call accordingly; the
// esp_timer fallback at ENC_CHANGE_FALLBACK_MS ms acts as a safety net if
// the secondary phase ever fails to fire.
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

static void complete_setup_chain(uint16_t conn_handle, uint32_t idle_ms)
{
    if (s_cfg.on_connected != NULL) {
        s_cfg.on_connected(idle_ms, s_cfg.user);
    }
    apply_remote_setup(conn_handle);
}

// Fired from esp_timer task if the second ENC_CHANGE never arrives. Runs
// setup with the stashed idle so we don't end up wedged forever in pending
// state. ble_gattc_* and the on_connected callback are safe from this
// context (the former posts to the NimBLE host task, the latter just
// touches main.c state guarded by event_state's own lock).
static void setup_fallback_cb(void *arg)
{
    (void)arg;
    if (s_pending_setup_conn == 0xFFFF) {
        return;
    }
    ESP_LOGW(TAG, "second ENC_CHANGE timed out at %d ms; running setup anyway",
             ENC_CHANGE_FALLBACK_MS);
    uint16_t conn = s_pending_setup_conn;
    uint32_t idle = s_pending_setup_idle_ms;
    s_pending_setup_conn = 0xFFFF;
    complete_setup_chain(conn, idle);
}

#ifdef CONFIG_DEBUG_WAKE_PROBE
// peer_disc_all completion. Dumps the entire GATT layout to UART so we can
// identify Apple custom service handles, then auto-subscribes to every
// notify-capable characteristic in the Apple custom service. The hope: one
// of those characteristics carries the wake-reason / first-press payload
// that Apple's HID drops on cold-boot first-bond reconnects.
static void on_probe_disc_complete(const struct peer *peer, int status, void *arg)
{
    (void)arg;
    if (status != 0) {
        ESP_LOGW(TAG, "wake_probe: peer_disc_all failed status=%d", status);
        return;
    }
    ESP_LOGI(TAG, "wake_probe: GATT layout (conn=%d):", peer->conn_handle);
    const struct peer_svc *svc;
    SLIST_FOREACH(svc, &peer->svcs, next) {
        char uuid_buf[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&svc->svc.uuid.u, uuid_buf);
        ESP_LOGI(TAG, "  svc %s handles=0x%04x..0x%04x",
                 uuid_buf, svc->svc.start_handle, svc->svc.end_handle);
        const struct peer_chr *chr;
        SLIST_FOREACH(chr, &svc->chrs, next) {
            ble_uuid_to_str(&chr->chr.uuid.u, uuid_buf);
            ESP_LOGI(TAG, "    chr %s def=0x%04x val=0x%04x props=0x%02x",
                     uuid_buf, chr->chr.def_handle, chr->chr.val_handle,
                     chr->chr.properties);
            const struct peer_dsc *dsc;
            SLIST_FOREACH(dsc, &chr->dscs, next) {
                ble_uuid_to_str(&dsc->dsc.uuid.u, uuid_buf);
                ESP_LOGI(TAG, "      dsc %s handle=0x%04x",
                         uuid_buf, dsc->dsc.handle);
            }
        }
    }

    const struct peer_svc *apple = peer_svc_find_uuid(peer, &APPLE_CUSTOM_SVC_UUID.u);
    if (apple == NULL) {
        ESP_LOGW(TAG, "wake_probe: Apple custom service 8341f2b4-... not found");
        return;
    }
    const struct peer_chr *chr;
    SLIST_FOREACH(chr, &apple->chrs, next) {
        if (!(chr->chr.properties & BLE_GATT_CHR_PROP_NOTIFY)) {
            continue;
        }
        const struct peer_dsc *dsc;
        SLIST_FOREACH(dsc, &chr->dscs, next) {
            if (ble_uuid_u16(&dsc->dsc.uuid.u) != BLE_GATT_DSC_CLT_CFG_UUID16) {
                continue;
            }
            ESP_LOGI(TAG, "wake_probe: subscribing Apple-svc CCCD 0x%04x (val=0x%04x)",
                     dsc->dsc.handle, chr->chr.val_handle);
            int rc = ble_gattc_write_flat(peer->conn_handle, dsc->dsc.handle,
                                          ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY),
                                          NULL, NULL);
            if (rc != 0) {
                ESP_LOGW(TAG, "wake_probe: CCCD 0x%04x write failed rc=%d",
                         dsc->dsc.handle, rc);
            }
            break;
        }
    }
}

// Run a one-shot peer_disc_all after the setup chain completes. Walks the
// remote's full GATT tree, logs everything, and subscribes to every notify-
// capable Apple custom service characteristic via on_probe_disc_complete.
static void kick_probe_discovery(uint16_t conn_handle)
{
    int rc = peer_disc_all(conn_handle, on_probe_disc_complete, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "wake_probe: peer_disc_all kickoff failed rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "wake_probe: peer_disc_all kicked off (conn=%d)", conn_handle);
    }
}
#endif

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
        // Conn-param update is intentionally deferred until *after* the
        // setup chain completes (see issue_setup_step's terminator). At
        // connect time NimBLE's default fast intervals + zero slave latency
        // are what we want — the setup chain runs in ~1 s. Then we switch
        // to the remote's preferred low-power params for the long haul.
        // On bonded reconnect, NimBLE auto-kicks encryption from its host task
        // before our explicit call lands. The call returns BLE_HS_EALREADY (2)
        // and the automatic path completes encryption successfully. Treat that
        // as success — only log other failure modes.
        int sec_rc = ble_gap_security_initiate(event->connect.conn_handle);
        if (sec_rc != 0 && sec_rc != BLE_HS_EALREADY) {
            ESP_LOGE(TAG, "security_initiate failed: rc=%d", sec_rc);
        }
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
            WAKE_PROBE_LOG("CONN_UPDATE status=%d itvl=%u lat=%u sup=%u",
                           event->conn_update.status,
                           desc.conn_itvl, desc.conn_latency,
                           desc.supervision_timeout);
        } else {
            WAKE_PROBE_LOG("CONN_UPDATE status=%d (conn_find failed)",
                           event->conn_update.status);
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected, reason=0x%04x", event->disconnect.reason);
        peer_delete(event->disconnect.conn.conn_handle);
        s_last_disconnect_ms = now_ms();
        s_has_disconnected   = true;
        s_setup_done_conn    = 0xFFFF;
        if (s_pending_setup_conn != 0xFFFF) {
            (void)esp_timer_stop(s_setup_fallback_timer);
            s_pending_setup_conn = 0xFFFF;
        }
        if (s_cfg.on_disconnected != NULL) {
            s_cfg.on_disconnected(s_cfg.user);
        }
        start_scan();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu=%d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE: {
#ifdef CONFIG_DEBUG_WAKE_PROBE
        // ENC_CHANGE can fire *before* BLE_GAP_EVENT_CONNECT on bonded
        // reconnect (NimBLE resumes encryption from cached LTKs faster than
        // it dispatches GAP CONNECT). If this is our first sighting of this
        // conn_handle, stamp s_connect_ms here so the t=Xms delta on this
        // print — and on every probe log that follows — is relative to the
        // earliest observed event of the new connection cycle.
        if (s_setup_done_conn != event->enc_change.conn_handle &&
            s_pending_setup_conn != event->enc_change.conn_handle) {
            s_connect_ms = now_ms();
        }
#endif
        ESP_LOGI(TAG, "encryption changed, status=%d", event->enc_change.status);
        WAKE_PROBE_LOG("ENC_CHANGE status=%d %s",
                       event->enc_change.status,
                       s_setup_done_conn == event->enc_change.conn_handle ? "(post-setup)"
                       : s_pending_setup_conn == event->enc_change.conn_handle ? "(secondary)"
                       : "(initial)");
        if (event->enc_change.status != 0) {
            return 0;
        }
        if (s_setup_done_conn == event->enc_change.conn_handle) {
            // Setup already ran for this connection. Further ENC_CHANGEs are
            // mid-session re-keys (Apple's accessory framework periodically
            // refreshes encryption); ignore.
            return 0;
        }
        uint16_t conn = event->enc_change.conn_handle;
        if (s_pending_setup_conn != conn) {
            // First ENC_CHANGE for this connection. Stash idle now (the value
            // we want to publish reflects time-since-disconnect, not time-
            // since-secondary-phase-completed) and defer setup until the
            // secondary ENC_CHANGE fires. The fallback timer guarantees we
            // run setup eventually even if the secondary never arrives.
            uint32_t idle;
            if (s_has_disconnected) {
                idle = now_ms() - s_last_disconnect_ms;
            } else {
                // First connect since bridge boot. Use bridge uptime as a
                // lower bound on idle duration so pickup fires for cold-boot
                // wakes after long sleeps where the bridge restarted in
                // between.
                idle = now_ms();
            }
            s_pending_setup_conn    = conn;
            s_pending_setup_idle_ms = idle;
            (void)esp_timer_start_once(s_setup_fallback_timer,
                                       (uint64_t)ENC_CHANGE_FALLBACK_MS * 1000);
            return 0;
        }
        // Secondary ENC_CHANGE — Apple's accessory-framework handshake has
        // settled. Cancel fallback, run setup. Apple flushes the buffered
        // wake-press to us shortly after the button CCCD subscribe lands.
        (void)esp_timer_stop(s_setup_fallback_timer);
        uint32_t idle = s_pending_setup_idle_ms;
        s_pending_setup_conn = 0xFFFF;
        complete_setup_chain(conn, idle);
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
#ifdef CONFIG_DEBUG_WAKE_PROBE
        // Diagnostic dump of every non-touch notification with handle +
        // first bytes. Touch (0x003D) is ~50/sec and would flood UART;
        // for touch use CONFIG_DEBUG_TOUCH_FRAMES instead.
        if (event->notify_rx.attr_handle != 0x003D) {
            char hex[3 * 16 + 1];
            uint16_t n = copied < 16 ? copied : 16;
            for (uint16_t i = 0; i < n; i++) {
                snprintf(&hex[i * 3], 4, "%02x ", buf[i]);
            }
            hex[n * 3] = '\0';
            ESP_LOGI(TAG, "notify t=%lums h=0x%04x len=%u %s",
                     (unsigned long)(now_ms() - s_connect_ms),
                     event->notify_rx.attr_handle, copied, hex);
        }
#endif
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
    // Log persisted bond-store record counts at boot. Doesn't change BLE
    // behavior (NimBLE loads keys lazily anyway and Phase 3A.7 confirmed
    // pre-loading makes no difference) but the counts are useful at-a-
    // glance diagnostic for confirming a bond is actually persisted.
    int bonded_count = 0;
    if (ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &bonded_count) == 0) {
        ESP_LOGI(TAG, "bond store: %d persisted peer-sec record(s)", bonded_count);
    }
    int our_count = 0;
    if (ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &our_count) == 0) {
        ESP_LOGI(TAG, "bond store: %d persisted our-sec record(s)", our_count);
    }
    int cccd_count = 0;
    if (ble_store_util_count(BLE_STORE_OBJ_TYPE_CCCD, &cccd_count) == 0) {
        ESP_LOGI(TAG, "bond store: %d persisted CCCD record(s)", cccd_count);
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

    const esp_timer_create_args_t timer_args = {
        .callback = setup_fallback_cb,
        .name     = "siri_setup_fallback",
    };
    if (esp_timer_create(&timer_args, &s_setup_fallback_timer) != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}
