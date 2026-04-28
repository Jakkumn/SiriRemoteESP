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
#include "host/ble_hs_adv.h"
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
// First-bond connects in DISCOVERING mode still run peer_disc_all once for
// the fingerprint check, but bonded reconnects use these constants directly.
// Phase 3B.7 plans a runtime "discovery mode" Switch for cross-gen units
// where the layout differs.
#define BUTTON_CCCD_HANDLE   0x003A
#define TOUCH_CCCD_HANDLE    0x003E
#define BATTERY_CCCD_HANDLE  0x002F  // svc 0x180F, char 0x2A19 (battery level)
#define CHARGING_CCCD_HANDLE 0x0032  // svc 0x180F, char 0x2A1A (battery power state)
#define MAGIC_HANDLE         0x004D
static const uint8_t MAGIC_VALUE[2]   = {0xF0, 0x00};
static const uint8_t ENABLE_NOTIFY[2] = {0x01, 0x00};

static siri_ble_config_t s_cfg;
static uint8_t s_own_addr_type;
static uint32_t s_last_disconnect_ms;
static bool s_has_disconnected;
static uint16_t s_setup_done_conn = 0xFFFF;  // dedup: per-connection setup runs once
static uint16_t s_active_conn = 0xFFFF;      // for siri_ble_idle_disconnect / siri_ble_repair
#ifdef CONFIG_DEBUG_WAKE_PROBE
static uint32_t s_connect_ms;  // BLE_GAP_EVENT_CONNECT timestamp; basis for t=Xms deltas
#endif

// Discovery / bonded-reconnect state machine. Phase 3B.6 retired the hard-
// coded MAC: at boot we either reconnect to a stored bond or run a 5-minute
// active-scan window looking for an HID-advertising candidate that survives
// a post-connect fingerprint check.
typedef enum {
    MODE_DISCOVERING,       // active scan, HID-UUID + RSSI gate, fingerprint after connect
    MODE_BONDED_RECONNECT,  // ble_gap_connect direct to s_bonded_peer, IRK resumes encryption
    MODE_CONNECTED,         // happy path: link up, setup complete
    MODE_IDLE,              // no bond, discovery window expired, awaiting repair button
} siri_ble_mode_t;
static siri_ble_mode_t s_mode = MODE_IDLE;

// Bonded peer identity address, populated at boot from the NimBLE bond store
// or right after a fresh fingerprint pass. Used as the target of the direct
// `ble_gap_connect` in BONDED_RECONNECT mode (no scan necessary; NimBLE's
// resolving list translates outbound connect commands when the peer's
// random-resolvable address matches a bonded IRK).
static ble_addr_t s_bonded_peer;
static bool       s_have_bonded_peer;

// Address of the candidate we're currently connecting to / fingerprinting.
// Stashed at BLE_GAP_EVENT_DISC time so we can blacklist + delete-bond on
// fingerprint failure even though the NimBLE GAP event for the failure is
// the disconnect (which doesn't carry the peer addr).
static ble_addr_t s_pending_candidate;
static bool       s_pending_candidate_set;

// In-RAM candidate blacklist. Discovery walks every nearby HID adv; once
// we've rejected one (fingerprint mismatch, or LL connect-establish failure
// from a remote that's advertising but won't admit an unknown peer until
// the user enters pairing mode) we drop it briefly so we don't re-attempt
// every adv interval. Lost on reboot — fine, since reboot also clears any
// stuck NimBLE state.
typedef struct {
    uint8_t  addr[6];
    uint32_t expire_ms;
} blacklist_entry_t;
#define BLACKLIST_SIZE                   8
#define BLACKLIST_TTL_FINGERPRINT_MS     60000  // wrong device entirely
#define BLACKLIST_TTL_FAILED_CONNECT_MS  5000   // LL handshake didn't complete —
                                                // most often the remote isn't in
                                                // pairing mode for our identity yet.
                                                // Short TTL so the user pressing
                                                // Back+VolUp recovers quickly.
static blacklist_entry_t s_blacklist[BLACKLIST_SIZE];
static size_t            s_blacklist_next;  // ring index for replacement on overflow

// Discovery window — keeps the bridge from sitting in active scan forever
// after a failed/abandoned pairing attempt. Cancelled on fingerprint pass
// or repair-button press (both restart it from scratch).
#define DISCOVERY_WINDOW_MS  (5 * 60 * 1000)
#define DISCOVERY_RSSI_MIN   -55
static esp_timer_handle_t s_discovery_window_timer;

// HID service + Report char UUIDs (16-bit) for adv filter + fingerprint walk.
#define HID_SVC_UUID16              0x1812
#define HID_REPORT_UUID16           0x2A4D
#define HID_REPORT_BUTTON_VAL       0x0039  // gen-3 button bitmap value handle
#define APPLE_FINGERPRINT_MIN_REPORTS 3     // gen-3 has 9; require ≥3 for fingerprint pass

// Apple custom service UUID `8341f2b4-c013-4f04-8197-c4cdb42e26dc` (LE byte
// order). Required member of the gen-3 fingerprint; also used by the optional
// CONFIG_DEBUG_WAKE_PROBE path below for post-setup notify subscription.
static const ble_uuid128_t APPLE_CUSTOM_SVC_UUID =
    BLE_UUID128_INIT(0xdc, 0x26, 0x2e, 0xb4, 0xcd, 0xc4, 0x97, 0x81,
                     0x04, 0x4f, 0x13, 0xc0, 0xb4, 0xf2, 0x41, 0x83);

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
// Phase 3B.8 exposes slave_latency as a runtime MQTT Number entity so users
// can trade battery-life vs. disconnect-detection latency without reflashing.
// `siri_ble_set_slave_latency` mutates this struct + auto-recomputes
// supervision_timeout to satisfy the BLE spec rule with ~25 % margin.
static struct ble_gap_upd_params s_low_power_conn_params = {
    .itvl_min            = 6,     // 6 * 1.25ms = 7.5ms
    .itvl_max            = 12,    // 12 * 1.25ms = 15ms
    .latency             = 400,
    .supervision_timeout = 1500,  // 1500 * 10ms = 15s
    .min_ce_len          = 0,
    .max_ce_len          = 0,
};

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_discovery(void);
static void start_bonded_reconnect(void);
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

static void log_addr(const char *prefix, const uint8_t val[6])
{
    ESP_LOGI(TAG, "%s %02x:%02x:%02x:%02x:%02x:%02x",
             prefix, val[5], val[4], val[3], val[2], val[1], val[0]);
}

// --- Candidate blacklist (RAM only, ~60 s TTL) -----------------------------

static bool blacklist_contains(const uint8_t addr[6])
{
    uint32_t now = now_ms();
    for (size_t i = 0; i < BLACKLIST_SIZE; i++) {
        if (s_blacklist[i].expire_ms == 0) continue;
        if (s_blacklist[i].expire_ms <= now) {
            s_blacklist[i].expire_ms = 0;  // expired, evict lazily
            continue;
        }
        if (memcmp(s_blacklist[i].addr, addr, 6) == 0) return true;
    }
    return false;
}

static void blacklist_add(const uint8_t addr[6], uint32_t ttl_ms)
{
    s_blacklist[s_blacklist_next].expire_ms = now_ms() + ttl_ms;
    memcpy(s_blacklist[s_blacklist_next].addr, addr, 6);
    s_blacklist_next = (s_blacklist_next + 1) % BLACKLIST_SIZE;
}

static void blacklist_clear(void)
{
    memset(s_blacklist, 0, sizeof(s_blacklist));
    s_blacklist_next = 0;
}

// --- Adv parsing helper ----------------------------------------------------

// Returns true if the parsed adv fields advertise the HID service (UUID16
// 0x1812). Looks at both incomplete and complete UUID16 lists (NimBLE
// merges them into the same `uuids16` array).
static bool adv_advertises_hid(const struct ble_hs_adv_fields *f)
{
    for (uint8_t i = 0; i < f->num_uuids16; i++) {
        if (ble_uuid_u16(&f->uuids16[i].u) == HID_SVC_UUID16) {
            return true;
        }
    }
    return false;
}

// --- Scan / connect entry points -------------------------------------------

static void start_discovery(void)
{
    s_mode = MODE_DISCOVERING;
    s_pending_candidate_set = false;

    struct ble_gap_disc_params params = {
        .itvl = 0,
        .window = 0,
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,            // active scan to capture scan responses
        .filter_duplicates = 0,  // OFF — we want every adv from a candidate
                                 // until we connect, since blacklist eviction
                                 // depends on retrying after the TTL expires
    };

    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
        return;
    }

    if (s_discovery_window_timer != NULL) {
        (void)esp_timer_stop(s_discovery_window_timer);
        (void)esp_timer_start_once(s_discovery_window_timer,
                                   (uint64_t)DISCOVERY_WINDOW_MS * 1000);
    }
    ESP_LOGI(TAG, "DISCOVERING — for a NEW bridge, hold Back+VolUp on the remote ~5s to "
                  "enter pairing mode; for a previously-paired remote, any button press "
                  "should suffice (window=%d min, filter=HID UUID 0x1812, RSSI ≥ %d dBm)",
             DISCOVERY_WINDOW_MS / 60000, DISCOVERY_RSSI_MIN);
}

static void start_bonded_reconnect(void)
{
    if (!s_have_bonded_peer) {
        ESP_LOGW(TAG, "start_bonded_reconnect called with no stored peer; falling back to discovery");
        start_discovery();
        return;
    }
    s_mode = MODE_BONDED_RECONNECT;
    s_pending_candidate_set = false;

    log_addr("BONDED_RECONNECT to", s_bonded_peer.val);
    int rc = ble_gap_connect(s_own_addr_type, &s_bonded_peer, BLE_HS_FOREVER,
                             NULL, gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY && rc != BLE_HS_EBUSY) {
        ESP_LOGE(TAG, "ble_gap_connect (bonded) failed rc=%d; entering discovery", rc);
        s_have_bonded_peer = false;
        start_discovery();
    }
}

// Inspect an incoming adv. If it advertises HID and clears the RSSI / blacklist
// gates, cancel scan and connect. Stashes the candidate addr in
// s_pending_candidate so fingerprint failure can blacklist + delete bond.
static void try_connect_candidate(const struct ble_gap_disc_desc *disc,
                                   const struct ble_hs_adv_fields *fields)
{
    if (s_pending_candidate_set) {
        // Already mid-connect to a candidate; ignore concurrent advs.
        return;
    }
    if (disc->rssi < DISCOVERY_RSSI_MIN) {
        ESP_LOGD(TAG, "rejecting candidate: rssi=%d < %d", disc->rssi, DISCOVERY_RSSI_MIN);
        return;
    }
    if (blacklist_contains(disc->addr.val)) {
        return;  // already-rejected device, silently ignore
    }
    if (!adv_advertises_hid(fields)) {
        return;  // not an HID advertiser, silently ignore
    }

    log_addr("HID candidate", disc->addr.val);
    ESP_LOGI(TAG, "  rssi=%d, attempting connect+fingerprint", disc->rssi);

    s_pending_candidate     = disc->addr;
    s_pending_candidate_set = true;

    (void)ble_gap_disc_cancel();
    int rc = ble_gap_connect(s_own_addr_type, &disc->addr, 30000, NULL,
                             gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: rc=%d", rc);
        s_pending_candidate_set = false;
        start_discovery();
    }
}

static void discovery_window_expired_cb(void *arg)
{
    (void)arg;
    if (s_mode != MODE_DISCOVERING) {
        return;
    }
    ESP_LOGW(TAG, "discovery window (%d min) expired without finding a remote — "
                  "press the HA Repair button to retry",
             DISCOVERY_WINDOW_MS / 60000);
    (void)ble_gap_disc_cancel();
    s_mode = MODE_IDLE;
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
        int rc = ble_gap_update_params(s_setup_step_conn, &s_low_power_conn_params);
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
    s_mode = MODE_CONNECTED;
    if (s_cfg.on_connected != NULL) {
        s_cfg.on_connected(idle_ms, s_cfg.user);
    }
    apply_remote_setup(conn_handle);
}

// --- Discovery-mode fingerprint --------------------------------------------
//
// Run only on the *first* connect to a candidate (DISCOVERING mode).
// Walks the discovered GATT tree and confirms the candidate looks like a
// gen-3 Siri Remote: HID service present, ≥3 Report chars with notify, the
// button value handle 0x0039 specifically present, and the Apple custom
// service present. Bonded reconnects skip this — we trust prior bonds.

// Returns NULL on pass; otherwise a static string describing the failed check.
static const char *fingerprint_check(const struct peer *peer)
{
    const ble_uuid16_t hid_svc_uuid    = BLE_UUID16_INIT(HID_SVC_UUID16);
    const ble_uuid16_t hid_report_uuid = BLE_UUID16_INIT(HID_REPORT_UUID16);

    const struct peer_svc *hid = peer_svc_find_uuid(peer, &hid_svc_uuid.u);
    if (hid == NULL) {
        return "HID service 0x1812 missing";
    }

    int report_notify_count = 0;
    bool button_handle_present = false;
    const struct peer_chr *chr;
    SLIST_FOREACH(chr, &hid->chrs, next) {
        if (ble_uuid_cmp(&chr->chr.uuid.u, &hid_report_uuid.u) != 0) {
            continue;
        }
        if (chr->chr.properties & BLE_GATT_CHR_PROP_NOTIFY) {
            report_notify_count++;
        }
        if (chr->chr.val_handle == HID_REPORT_BUTTON_VAL) {
            button_handle_present = true;
        }
    }
    if (report_notify_count < APPLE_FINGERPRINT_MIN_REPORTS) {
        return "fewer than 3 notify-capable HID Report chars";
    }
    if (!button_handle_present) {
        return "HID button value handle 0x0039 missing";
    }
    if (peer_svc_find_uuid(peer, &APPLE_CUSTOM_SVC_UUID.u) == NULL) {
        return "Apple custom service 8341f2b4-... missing";
    }
    return NULL;
}

static void on_fingerprint_disc_complete(const struct peer *peer, int status, void *arg)
{
    (void)arg;
    uint16_t conn = peer->conn_handle;

    if (status != 0) {
        ESP_LOGW(TAG, "fingerprint: peer_disc_all failed status=%d — rejecting candidate",
                 status);
        goto reject;
    }

    const char *reason = fingerprint_check(peer);
    if (reason != NULL) {
        ESP_LOGI(TAG, "fingerprint FAIL: %s — rejecting candidate", reason);
        goto reject;
    }

    ESP_LOGI(TAG, "fingerprint PASS — gen-3 Siri Remote confirmed, proceeding with setup");
    if (s_pending_candidate_set) {
        s_bonded_peer       = s_pending_candidate;
        s_have_bonded_peer  = true;
        s_pending_candidate_set = false;
    }
    if (s_discovery_window_timer != NULL) {
        (void)esp_timer_stop(s_discovery_window_timer);
    }
    // Use idle=0 here: this is a brand-new bond, there's no meaningful
    // time-since-disconnect to publish. complete_setup_chain → on_connected
    // → main.c's pickup-event logic suppresses pickup for idle=0.
    complete_setup_chain(conn, 0);
    return;

reject:
    if (s_pending_candidate_set) {
        blacklist_add(s_pending_candidate.val, BLACKLIST_TTL_FINGERPRINT_MS);
        (void)ble_store_util_delete_peer(&s_pending_candidate);
        s_pending_candidate_set = false;
    }
    (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    // Disconnect handler will resume scanning while still inside the
    // discovery window.
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
    if (s_mode == MODE_DISCOVERING) {
        // Fallback during discovery: kick fingerprint manually, since the
        // secondary ENC_CHANGE that would have triggered it never arrived.
        int rc = peer_disc_all(conn, on_fingerprint_disc_complete, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "fingerprint: peer_disc_all kickoff failed rc=%d", rc);
            (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    } else {
        complete_setup_chain(conn, idle);
    }
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
        if (s_mode != MODE_DISCOVERING) {
            return 0;  // not actively scanning (e.g. idle / connected); ignore
        }
        if (event->disc.event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND) {
            // Only act on undirected connectable advs (ADV_IND). Skip:
            //   ADV_DIRECT_IND (0x01) — addressed to a specific peer, almost
            //     always the remote's prior bonded host. Connecting to one
            //     where we're not the addressed peer fails at LL with
            //     reason 0x023e ("connection failed to be established") and
            //     loops because the remote keeps sending the same DIR_IND.
            //     Wait for the ADV_IND fallback (Apple typically sends DIR_IND
            //     for ~1.28 s before falling back to ADV_IND for unbonded
            //     pairing — and explicit pairing-mode (TV+VolUp 5 s) goes
            //     straight to ADV_IND).
            //   ADV_SCAN_IND / ADV_NONCONN_IND / scan responses — skip.
            return 0;
        }
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                    event->disc.length_data) != 0) {
            return 0;  // malformed adv
        }
        try_connect_candidate(&event->disc, &fields);
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "connect failed: status=%d", event->connect.status);
            if (s_mode == MODE_DISCOVERING && s_pending_candidate_set) {
                blacklist_add(s_pending_candidate.val,
                              BLACKLIST_TTL_FAILED_CONNECT_MS);
            }
            s_pending_candidate_set = false;
            if (s_mode == MODE_BONDED_RECONNECT) {
                // Re-issue direct connect — peer will eventually advertise.
                start_bonded_reconnect();
            } else if (s_mode == MODE_DISCOVERING) {
                start_discovery();
            }
            return 0;
        }
        s_active_conn = event->connect.conn_handle;
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

    case BLE_GAP_EVENT_DISCONNECT: {
        bool failed_pre_fingerprint =
            (s_mode == MODE_DISCOVERING && s_pending_candidate_set &&
             s_setup_done_conn == 0xFFFF);
        ESP_LOGI(TAG, "disconnected, reason=0x%04x", event->disconnect.reason);
        peer_delete(event->disconnect.conn.conn_handle);
        s_last_disconnect_ms = now_ms();
        s_has_disconnected   = true;
        s_setup_done_conn    = 0xFFFF;
        s_active_conn        = 0xFFFF;
        if (s_pending_setup_conn != 0xFFFF) {
            (void)esp_timer_stop(s_setup_fallback_timer);
            s_pending_setup_conn = 0xFFFF;
        }
        if (failed_pre_fingerprint) {
            // Connect was attempted but the link never reached fingerprint —
            // most often the peer is advertising but won't admit us at LL
            // until the user puts the remote in pairing mode (Back+VolUp).
            // Short blacklist breaks the retry loop while still recovering
            // quickly when the user does press the combo.
            ESP_LOGI(TAG, "candidate failed pre-fingerprint — blacklisting %d s, "
                          "hold Back+VolUp ~5s on the remote to pair",
                     BLACKLIST_TTL_FAILED_CONNECT_MS / 1000);
            blacklist_add(s_pending_candidate.val,
                          BLACKLIST_TTL_FAILED_CONNECT_MS);
        }
        if (s_cfg.on_disconnected != NULL) {
            s_cfg.on_disconnected(s_cfg.user);
        }
        // Mode dispatch on disconnect:
        //   - DISCOVERING (fingerprint just rejected, or bonded peer
        //     advertising during discovery dropped us): keep scanning.
        //   - BONDED_RECONNECT (link dropped, we want it back): re-issue
        //     direct connect.
        //   - CONNECTED (idle disconnect or remote went away): if we have a
        //     bond, transition to BONDED_RECONNECT; otherwise this means the
        //     bond was wiped mid-session (siri_ble_repair after terminate),
        //     so enter discovery.
        //   - IDLE: shouldn't happen (no active conn), but just stay idle.
        if (s_mode == MODE_DISCOVERING) {
            start_discovery();
        } else if (s_mode == MODE_BONDED_RECONNECT) {
            start_bonded_reconnect();
        } else if (s_mode == MODE_CONNECTED) {
            if (s_have_bonded_peer) {
                start_bonded_reconnect();
            } else {
                start_discovery();
            }
        }
        return 0;
    }

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
        // settled. Cancel fallback. If we're discovering (no prior bond),
        // run a fingerprint check before committing to the bond — peer_disc_all
        // walks the GATT tree, then on_fingerprint_disc_complete either runs
        // setup or rejects+blacklists. If we already trust the bond, skip
        // straight to setup; Apple flushes the buffered wake-press to us
        // shortly after the button CCCD subscribe lands.
        (void)esp_timer_stop(s_setup_fallback_timer);
        uint32_t idle = s_pending_setup_idle_ms;
        s_pending_setup_conn = 0xFFFF;
        if (s_mode == MODE_DISCOVERING) {
            int rc = peer_disc_all(conn, on_fingerprint_disc_complete, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "fingerprint: peer_disc_all kickoff failed rc=%d", rc);
                if (s_pending_candidate_set) {
                    blacklist_add(s_pending_candidate.val, BLACKLIST_TTL_FINGERPRINT_MS);
                    s_pending_candidate_set = false;
                }
                (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
            }
        } else {
            complete_setup_chain(conn, idle);
        }
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

void siri_ble_repair(void)
{
    ESP_LOGI(TAG, "repair: clearing bond store and re-entering discovery");
    int rc = ble_store_clear();
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_store_clear rc=%d", rc);
    }
    s_have_bonded_peer      = false;
    s_pending_candidate_set = false;
    blacklist_clear();

    if (s_active_conn != 0xFFFF) {
        // Disconnect handler will see s_have_bonded_peer=false and route
        // into discovery on its way out. Setting mode here would race the
        // disconnect; let it dispatch.
        s_mode = MODE_DISCOVERING;
        int trc = ble_gap_terminate(s_active_conn, BLE_ERR_REM_USER_CONN_TERM);
        if (trc != 0 && trc != BLE_HS_ENOTCONN) {
            ESP_LOGW(TAG, "ble_gap_terminate (repair) rc=%d", trc);
        }
        return;
    }
    // No active connection — make sure scan is stopped (in case we're mid-
    // discovery already), then restart the window from scratch.
    (void)ble_gap_disc_cancel();
    start_discovery();
}

void siri_ble_set_slave_latency(uint16_t latency)
{
    // Spec rule: supervision_timeout > 2 * itvl_max * (1 + latency).
    // Compute with ~25% margin: ceil(2.5 * (1 + latency) * itvl_max),
    // expressed in 10 ms units (the supervision_timeout field unit).
    // itvl_max field is in 1.25 ms units; multiply by 125 then divide by
    // 100 to get ms. Combined: timeout_units_10ms = ceil(2.5 * (1+lat) * itvl_max * 1.25 / 10)
    //                                              = ceil((1+lat) * itvl_max * 25 / 80)
    // ...which we round up via integer math, then clamp to the 16-bit
    // field's safe upper bound (1500 = 15 s; longer means runaway-detection
    // delay > 15 s which is already the high end of acceptable).
    uint32_t numerator = (uint32_t)(1 + latency) * s_low_power_conn_params.itvl_max * 25;
    uint32_t timeout   = (numerator + 79) / 80;  // ceil(numerator / 80)
    if (timeout < 100) timeout = 100;            // 1 s floor
    if (timeout > 1500) timeout = 1500;          // 15 s ceiling

    s_low_power_conn_params.latency             = latency;
    s_low_power_conn_params.supervision_timeout = (uint16_t)timeout;

    ESP_LOGI(TAG, "slave_latency=%u sup_timeout=%lu (= %lu ms)",
             latency, (unsigned long)timeout, (unsigned long)timeout * 10);

    if (s_setup_done_conn != 0xFFFF) {
        int rc = ble_gap_update_params(s_setup_done_conn, &s_low_power_conn_params);
        if (rc != 0 && rc != BLE_HS_ENOTCONN) {
            ESP_LOGW(TAG, "live conn-param update rc=%d (cached value kept)", rc);
        }
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
    // Inspect the bond store. If a peer is persisted from a previous boot,
    // skip discovery and go straight to a direct connect — NimBLE's resolving
    // list translates the bonded identity address into the peer's current
    // resolvable random adv automatically. If the store is empty (first boot
    // or post-repair), enter the 5-minute discovery window.
    ble_addr_t peer_id_addrs[CONFIG_BT_NIMBLE_MAX_BONDS];
    int        num_bonded = 0;
    int rc = ble_store_util_bonded_peers(peer_id_addrs, &num_bonded,
                                          CONFIG_BT_NIMBLE_MAX_BONDS);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_store_util_bonded_peers rc=%d; entering discovery", rc);
        num_bonded = 0;
    }
    int our_count = 0;
    (void)ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &our_count);
    int cccd_count = 0;
    (void)ble_store_util_count(BLE_STORE_OBJ_TYPE_CCCD, &cccd_count);
    ESP_LOGI(TAG, "bond store: %d peer-sec / %d our-sec / %d CCCD record(s)",
             num_bonded, our_count, cccd_count);

    if (num_bonded > 0) {
        // Use the first bonded peer. We never expect more than one — the
        // bridge bonds to exactly one remote — but tolerate the array form.
        s_bonded_peer      = peer_id_addrs[0];
        s_have_bonded_peer = true;
        start_bonded_reconnect();
    } else {
        s_have_bonded_peer = false;
        start_discovery();
    }
}

esp_err_t siri_ble_start(const siri_ble_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;

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

    const esp_timer_create_args_t setup_timer_args = {
        .callback = setup_fallback_cb,
        .name     = "siri_setup_fallback",
    };
    if (esp_timer_create(&setup_timer_args, &s_setup_fallback_timer) != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create (setup_fallback) failed");
        return ESP_FAIL;
    }
    const esp_timer_create_args_t window_timer_args = {
        .callback = discovery_window_expired_cb,
        .name     = "siri_disc_window",
    };
    if (esp_timer_create(&window_timer_args, &s_discovery_window_timer) != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create (discovery_window) failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}
