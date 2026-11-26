#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "siri_ble.h"
#include "esp_central.h"

static const char *TAG = "siri_ble";

// IDF ships ble_store_config_init in libbt.a but doesn't publish a header.
void ble_store_config_init(void);

// The remote's discovered GATT layout. Declared here because resolve_handle()
// (below, with the setup-step table) reads it before the derivation helpers
// are defined. Invalid until a fingerprint pass or a successful NVS load.
static siri_handle_map_t s_map;
static bool s_map_valid;

// Post-pairing notification blackout — see pairing_flush_suppress_ms in
// siri_ble.h. Armed when a fresh bond's setup chain finishes, because that is
// the moment Apple flushes the buffered pairing-combo presses.
static uint32_t s_suppress_until_ms;
static uint32_t s_setup_idle_ms;

// Reference-unit handles, retained as documentation only — nothing reads them.
// The second gen-3 remote tested carried the same structure shifted by +1
// (button 0x003A, touch 0x003E, battery 0x002F, charging 0x0032, magic 0x004E),
// which is why these are discovered per-remote now rather than compiled in:
//   button CCCD 0x003A / touch CCCD 0x003E / battery CCCD 0x002F
//   charging CCCD 0x0032 / magic 0x004D
static const uint8_t MAGIC_VALUE[2] = {0xF0, 0x00};
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
static bool s_have_bonded_peer;

// Address of the candidate we're currently connecting to / fingerprinting.
// Stashed at BLE_GAP_EVENT_DISC time so we can blacklist + delete-bond on
// fingerprint failure even though the NimBLE GAP event for the failure is
// the disconnect (which doesn't carry the peer addr).
static ble_addr_t s_pending_candidate;
static bool s_pending_candidate_set;

// The same candidate's *identity* address, captured at its first ENC_CHANGE.
// The remote advertises with a resolvable private address, but NimBLE keys
// bond records by identity address — so deleting or reconnecting by the
// advertised address silently misses: the delete leaves the bond behind, and
// a direct connect stops matching as soon as the remote rotates its RPA.
// NimBLE persists the keys before raising ENC_CHANGE (ble_sm_process_result),
// so peer_id_addr is already the identity at that point.
static ble_addr_t s_pending_candidate_id;
static bool s_pending_candidate_id_set;

// The address that names the candidate's bond record: its identity address
// once pairing has distributed it, otherwise the advertised one.
static const ble_addr_t *candidate_bond_addr(void)
{
    return s_pending_candidate_id_set ? &s_pending_candidate_id : &s_pending_candidate;
}

// In-RAM candidate blacklist. Discovery walks every nearby HID adv; once
// we've rejected one (fingerprint mismatch, or LL connect-establish failure
// from a remote that's advertising but won't admit an unknown peer until
// the user enters pairing mode) we drop it briefly so we don't re-attempt
// every adv interval. Lost on reboot — fine, since reboot also clears any
// stuck NimBLE state.
typedef struct {
    uint8_t addr[6];
    uint32_t expire_ms;
} blacklist_entry_t;
#define BLACKLIST_SIZE 8
#define CANDIDATE_CONNECT_TIMEOUT_MS 5000
#define BLACKLIST_TTL_FINGERPRINT_MS 60000  // wrong device entirely
#define BLACKLIST_TTL_FAILED_CONNECT_MS                                                            \
    5000  // LL handshake didn't complete —
          // most often the remote isn't in
          // pairing mode for our identity yet.
          // Short TTL so the user pressing
          // Back+VolUp recovers quickly.
static blacklist_entry_t s_blacklist[BLACKLIST_SIZE];
static size_t s_blacklist_next;  // ring index for replacement on overflow
static portMUX_TYPE s_blacklist_mux = portMUX_INITIALIZER_UNLOCKED;
// protects s_blacklist + s_blacklist_next:
// writes/reads from BLE host task (DISC
// event, fingerprint reject, and repair,
// which is posted to it).

// --- Host-task timers and events --------------------------------------------
//
// Every timer and the Repair request run on the NimBLE host task: the timers
// are ble_npl_callouts on NimBLE's default event queue, and siri_ble_repair()
// only posts an event to that queue. That keeps every s_mode / s_pending_* /
// peer transition on one task. They were esp_timer callbacks and a direct
// call from the glue's loop / MQTT task, which could interleave with
// gap_event_cb — two setup chains on one link, or the discovery window
// dropping to IDLE mid-connect so a candidate skipped the fingerprint.
//
// ble_npl_callout_stop() stops the underlying timer but does not dequeue an
// event that already fired, so every callback re-checks the state it acts on.
static struct ble_npl_event s_repair_ev;

// Discovery window — keeps the bridge from sitting in active scan forever
// after a failed/abandoned pairing attempt. Cancelled on fingerprint pass
// or repair-button press (both restart it from scratch).
#define DISCOVERY_WINDOW_MS (5 * 60 * 1000)
#define DISCOVERY_RSSI_MIN -55
static struct ble_npl_callout s_discovery_window_co;
// While a candidate connect or fingerprint is in flight at window expiry,
// re-check after this long instead of dropping to IDLE under it.
#define DISCOVERY_WINDOW_GRACE_MS (30 * 1000)

// HID service + Report char UUIDs (16-bit) for adv filter + fingerprint walk.
#define HID_SVC_UUID16 0x1812
#define HID_REPORT_UUID16 0x2A4D

// Apple custom service UUID `8341f2b4-c013-4f04-8197-c4cdb42e26dc` (LE byte
// order). Required member of the gen-3 fingerprint; also used by the optional
// CONFIG_DEBUG_WAKE_PROBE path below for post-setup notify subscription.
static const ble_uuid128_t APPLE_CUSTOM_SVC_UUID = BLE_UUID128_INIT(
    0xdc, 0x26, 0x2e, 0xb4, 0xcd, 0xc4, 0x97, 0x81, 0x04, 0x4f, 0x13, 0xc0, 0xb4, 0xf2, 0x41, 0x83);

// Deferred-setup state. apply_remote_setup is held back until the *second*
// ENC_CHANGE for a connection — Apple's accessory framework re-encrypts the
// link as a secondary phase ~750-1200 ms after bond resumption, and any HID
// notify Apple buffered for us during that handshake is dropped if we touch
// the GATT layer (CCCD writes etc.) before the secondary phase completes.
// By waiting, the wake-press Apple has held since the user pressed a button
// to wake the remote is delivered cleanly once our CCCD subscribes land.
static uint16_t s_pending_setup_conn = 0xFFFF;
static uint32_t s_pending_setup_idle_ms;
// Set the moment setup is dispatched for a connection (secondary ENC_CHANGE or
// the fallback), cleared on DISCONNECT. s_setup_done_conn is only set once the
// GATT walk has finished and the chain starts — ~3.7 s later on a fingerprint
// or rediscovery. Without this marker, an ENC_CHANGE in that gap (Apple
// re-keys, or a late secondary after the fallback ran) looked like a *first*
// ENC_CHANGE, re-armed the fallback, and the fallback then called
// peer_disc_all again: that frees the in-flight walk's service list under it
// (peer.c), a use-after-free / assert and a reboot.
static uint16_t s_setup_dispatched_conn = 0xFFFF;

static bool setup_started(uint16_t conn)
{
    return conn == s_setup_dispatched_conn || conn == s_setup_done_conn;
}
static struct ble_npl_callout s_setup_fallback_co;

// Encryption watchdog. Nothing else bounds the time between CONNECT and the
// first successful ENC_CHANGE: a remote that was factory-reset or re-paired to
// an Apple TV no longer has our keys, so a bonded reconnect never encrypts,
// and a discovery candidate that never finishes pairing holds the link — and
// with scanning stopped, the whole discovery window — indefinitely. On expiry
// the link is terminated and the normal disconnect dispatch takes over
// (blacklist + rescan for a candidate, re-issue direct connect when bonded).
// Bonded resumption takes well under 1 s and a fresh just-works pairing a few
// seconds, so 10 s only ever fires on a link that is not going to encrypt.
#define ENC_WATCHDOG_MS 10000
static struct ble_npl_callout s_enc_watchdog_co;
static uint16_t s_enc_watchdog_conn = 0xFFFF;
// Consecutive pre-setup encryption failures; reset on success. Diagnostic only.
static uint32_t s_enc_fail_streak;

// Optional idle disconnect (s_cfg.idle_disconnect_ms > 0; the default 0 is
// always-connected). Activity is stamped on every delivered notification and
// at setup completion; the callout re-arms for the remainder rather than
// ticking, so an active link costs one timestamp write per notify.
static struct ble_npl_callout s_idle_co;
static uint32_t s_last_activity_ms;

// Stale-map detection. A map restored from NVS can outlive the layout it
// describes (a remote firmware update can shift handles, exactly as two units
// on different firmware differ). That brings back the original silent failure,
// so a setup step that fails with a "wrong handle" ATT error on a cached map
// erases it and reconnects to rediscover. Only for a cached map — a freshly
// derived one failing would fail again — and once per boot, so a remote that
// rejects some step for any other reason cannot loop.
static bool s_map_from_nvs;
static bool s_map_invalidated;

// Backoff for a bonded remote whose handle-map rediscovery keeps failing.
// Without it the bridge reconnects, spends ~3.7 s rediscovering, fails,
// disconnects and repeats indefinitely. Doubles from 5 s, capped at 5 min;
// reset by a successful rediscovery or by Repair.
#define REDISCOVERY_BACKOFF_BASE_MS 5000
#define REDISCOVERY_BACKOFF_MAX_MS (5 * 60 * 1000)
static struct ble_npl_callout s_reconnect_co;
static uint32_t s_rediscover_fail_streak;

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
// slave_latency is compile-time on both build paths (CONFIG_BLE_SLAVE_LATENCY_DEFAULT
// standalone, the `ble_slave_latency:` YAML key under ESPHome). It was briefly a
// runtime Number entity; the pulse refactor removed the whole Number platform, so
// trading battery-life against disconnect-detection latency now needs a reflash.
// It arrives as siri_ble_config_t.slave_latency; set_slave_latency() applies it
// to this struct at start and recomputes supervision_timeout to satisfy the BLE
// spec rule with ~25 % margin.
#define SLAVE_LATENCY_MAX 498
static struct ble_gap_upd_params s_low_power_conn_params = {
    .itvl_min = 6,   // 6 * 1.25ms = 7.5ms
    .itvl_max = 12,  // 12 * 1.25ms = 15ms
    .latency = 400,
    .supervision_timeout = 1500,  // 1500 * 10ms = 15s
    .min_ce_len = 0,
    .max_ce_len = 0,
};

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_discovery(bool restart_window);
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
#define WAKE_PROBE_LOG(fmt, ...)                                                                   \
    ESP_LOGI(TAG, "wake_probe t=%lums " fmt, (unsigned long)(now_ms() - s_connect_ms),             \
             ##__VA_ARGS__)
#else
#define WAKE_PROBE_LOG(fmt, ...) ((void)0)
#endif

static void log_addr(const char *prefix, const uint8_t val[6])
{
    ESP_LOGI(TAG, "%s %02x:%02x:%02x:%02x:%02x:%02x", prefix, val[5], val[4], val[3], val[2],
             val[1], val[0]);
}

// --- Candidate blacklist (RAM only, ~60 s TTL) -----------------------------

static bool blacklist_contains(const uint8_t addr[6])
{
    bool found = false;
    portENTER_CRITICAL(&s_blacklist_mux);
    uint32_t now = now_ms();
    for (size_t i = 0; i < BLACKLIST_SIZE; i++) {
        if (s_blacklist[i].expire_ms == 0)
            continue;
        if (s_blacklist[i].expire_ms <= now) {
            s_blacklist[i].expire_ms = 0;  // expired, evict lazily
            continue;
        }
        if (memcmp(s_blacklist[i].addr, addr, 6) == 0) {
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_blacklist_mux);
    return found;
}

static void blacklist_add(const uint8_t addr[6], uint32_t ttl_ms)
{
    portENTER_CRITICAL(&s_blacklist_mux);
    s_blacklist[s_blacklist_next].expire_ms = now_ms() + ttl_ms;
    memcpy(s_blacklist[s_blacklist_next].addr, addr, 6);
    s_blacklist_next = (s_blacklist_next + 1) % BLACKLIST_SIZE;
    portEXIT_CRITICAL(&s_blacklist_mux);
}

static void blacklist_clear(void)
{
    portENTER_CRITICAL(&s_blacklist_mux);
    memset(s_blacklist, 0, sizeof(s_blacklist));
    s_blacklist_next = 0;
    portEXIT_CRITICAL(&s_blacklist_mux);
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

// restart_window: true when entering discovery afresh (boot with no bond,
// Repair, a bond that turned out unusable), false for the rescan after a
// rejected or failed candidate. The 5-minute window bounds the whole attempt;
// restarting it on every rescan meant any nearby HID device that kept failing
// (a keyboard, or the remote outside pairing mode) kept the bridge scanning
// at full duty forever.
static void start_discovery(bool restart_window)
{
    if (!ble_hs_synced()) {
        // Mid host reset: anything issued now is lost when sync resets the
        // controller, while NimBLE's own bookkeeping keeps it "pending" — a
        // phantom that blocks every later scan/connect. on_sync re-dispatches.
        ESP_LOGW(TAG, "host not synced; discovery resumes on sync");
        return;
    }
    s_mode = MODE_DISCOVERING;
    s_pending_candidate_set = false;

    struct ble_gap_disc_params params = {
        .itvl = 0,
        .window = 0,
        .filter_policy = 0,
        .limited = 0,
        .passive = 1,            // passive: the HID-UUID match comes from the
                                 // ADV_IND payload, and gap_event_cb drops scan
                                 // responses anyway, so SCAN_REQs to every
                                 // nearby advertiser would be pure radio noise
        .filter_duplicates = 0,  // OFF — we want every adv from a candidate
                                 // until we connect, since blacklist eviction
                                 // depends on retrying after the TTL expires
    };

    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params, gap_event_cb, NULL);
    if (rc != 0) {
        // Without this the bridge would claim DISCOVERING with no scan
        // running, no window timer armed, and nothing scheduled to retry.
        // IDLE is the honest state: it is what Repair recovers from.
        ESP_LOGE(TAG, "ble_gap_disc failed: %d — idle; press the HA Repair button to retry", rc);
        s_mode = MODE_IDLE;
        return;
    }

    if (restart_window || !ble_npl_callout_is_active(&s_discovery_window_co)) {
        (void)ble_npl_callout_reset(&s_discovery_window_co,
                                    ble_npl_time_ms_to_ticks32(DISCOVERY_WINDOW_MS));
    }
    ESP_LOGI(TAG,
             "DISCOVERING — for a NEW bridge, hold Back+VolUp on the remote ~5s to "
             "enter pairing mode; for a previously-paired remote, any button press "
             "should suffice (window=%d min, filter=HID UUID 0x1812, RSSI ≥ %d dBm)",
             DISCOVERY_WINDOW_MS / 60000, DISCOVERY_RSSI_MIN);
}

static void start_bonded_reconnect(void)
{
    if (!ble_hs_synced()) {
        ESP_LOGW(TAG, "host not synced; bonded reconnect resumes on sync");  // see start_discovery
        return;
    }
    if (!s_have_bonded_peer) {
        ESP_LOGW(TAG,
                 "start_bonded_reconnect called with no stored peer; falling back to discovery");
        start_discovery(true);
        return;
    }
    s_mode = MODE_BONDED_RECONNECT;
    s_pending_candidate_set = false;

    log_addr("BONDED_RECONNECT to", s_bonded_peer.val);
    int rc =
        ble_gap_connect(s_own_addr_type, &s_bonded_peer, BLE_HS_FOREVER, NULL, gap_event_cb, NULL);
    if (rc == BLE_HS_EBUSY) {
        // A scan is running, so nothing is pending — treating this as success
        // would leave no connect issued at all. Stop the scan and retry once.
        (void)ble_gap_disc_cancel();
        rc = ble_gap_connect(s_own_addr_type, &s_bonded_peer, BLE_HS_FOREVER, NULL, gap_event_cb,
                             NULL);
    }
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_connect (bonded) failed rc=%d; entering discovery", rc);
        s_have_bonded_peer = false;
        start_discovery(true);
    }
}

// Disconnect-path entry to BONDED_RECONNECT: immediate normally, delayed while
// rediscovery is failing (see REDISCOVERY_BACKOFF_*).
static void schedule_bonded_reconnect(void)
{
    if (s_rediscover_fail_streak == 0) {
        start_bonded_reconnect();
        return;
    }
    uint32_t shift = s_rediscover_fail_streak - 1;
    uint32_t delay = REDISCOVERY_BACKOFF_MAX_MS;
    if (shift < 7 && (REDISCOVERY_BACKOFF_BASE_MS << shift) < REDISCOVERY_BACKOFF_MAX_MS) {
        delay = REDISCOVERY_BACKOFF_BASE_MS << shift;
    }
    s_mode = MODE_BONDED_RECONNECT;
    ESP_LOGW(TAG,
             "handle-map rediscovery failed %lu time(s) in a row — retrying the bonded "
             "connect in %lu s (press Repair to re-pair instead)",
             (unsigned long)s_rediscover_fail_streak, (unsigned long)(delay / 1000));
    (void)ble_npl_callout_reset(&s_reconnect_co, ble_npl_time_ms_to_ticks32(delay));
}

static void reconnect_cb(struct ble_npl_event *ev)
{
    (void)ev;
    // Repair (or anything else) may have moved on while we waited.
    if (s_mode != MODE_BONDED_RECONNECT || s_active_conn != 0xFFFF) {
        return;
    }
    start_bonded_reconnect();
}

// Inspect an incoming adv. If it advertises HID and clears the RSSI / blacklist
// gates, cancel scan and connect. Stashes the candidate addr in
// s_pending_candidate so fingerprint failure can blacklist + delete bond.
static void try_connect_candidate(const struct ble_gap_disc_desc *disc,
                                  const struct ble_hs_adv_fields *fields)
{
    // Pending / RSSI / blacklist gates run in gap_event_cb, before the
    // advertisement is parsed.
    if (!adv_advertises_hid(fields)) {
        return;  // not an HID advertiser, silently ignore
    }

    log_addr("HID candidate", disc->addr.val);
    ESP_LOGI(TAG, "  rssi=%d, attempting connect+fingerprint", disc->rssi);

    s_pending_candidate = disc->addr;
    s_pending_candidate_set = true;
    s_pending_candidate_id_set = false;

    (void)ble_gap_disc_cancel();
    // Scanning is cancelled for the duration of this connect, so its timeout
    // is how long the discovery window goes blind if the candidate stops
    // advertising (user let go of the combo, walked away). We just heard an
    // ADV_IND at >= -55 dBm and pairing-mode advertising runs at tens of ms,
    // so 5 s is still several advertising events even at a 1.28 s interval.
    // Timeout lands in the CONNECT-failed branch: short blacklist, rescan.
    int rc = ble_gap_connect(s_own_addr_type, &disc->addr, CANDIDATE_CONNECT_TIMEOUT_MS, NULL,
                             gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: rc=%d", rc);
        s_pending_candidate_set = false;
        start_discovery(false);
    }
}

static void discovery_window_expired_cb(struct ble_npl_event *ev)
{
    (void)ev;
    if (s_mode != MODE_DISCOVERING) {
        return;
    }
    if (s_pending_candidate_set) {
        // A candidate is mid-connect or mid-fingerprint. Going IDLE now would
        // strand it: its later ENC_CHANGE would see a non-DISCOVERING mode and
        // take the bonded path, skipping the fingerprint. Let it resolve
        // (pass stops this callout; fail rescans) and check again after.
        (void)ble_npl_callout_reset(&s_discovery_window_co,
                                    ble_npl_time_ms_to_ticks32(DISCOVERY_WINDOW_GRACE_MS));
        return;
    }
    ESP_LOGW(TAG,
             "discovery window (%d min) expired without finding a remote — "
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

// Which handle a step targets. Resolved from the discovered map when the step
// is issued, not when the table is written — the table is static but the
// layout is per-remote. H_LIT means "use the literal `handle` field", which
// only the reference-unit debug probes still do.
typedef enum {
    H_LIT = 0,
    H_BUTTON_CCCD,
    H_TOUCH_CCCD,
    H_BATTERY_CCCD,
    H_CHARGING_CCCD,
    H_AUDIO_CCCD,
    H_MAGIC,
    H_BATTERY_VAL,
    H_CHARGING_VAL,
} handle_sel_t;

typedef struct {
    setup_kind_t kind;
    handle_sel_t sel;
    uint16_t handle;       // H_LIT only
    const uint8_t *value;  // SETUP_KIND_WRITE only
    size_t len;            // SETUP_KIND_WRITE only
    const char *label;
} setup_step_t;

static uint16_t resolve_handle(const setup_step_t *step)
{
    switch (step->sel) {
    case H_LIT:
        return step->handle;
    case H_BUTTON_CCCD:
        return s_map.button_cccd;
    case H_TOUCH_CCCD:
        return s_map.touch_cccd;
    case H_BATTERY_CCCD:
        return s_map.battery_cccd;
    case H_CHARGING_CCCD:
        return s_map.charging_cccd;
    case H_AUDIO_CCCD:
        return s_map.audio_cccd;
    case H_MAGIC:
        return s_map.magic;
    case H_BATTERY_VAL:
        return s_map.battery_val;
    case H_CHARGING_VAL:
        return s_map.charging_val;
    }
    return 0;
}

// --- Discovered handle map -------------------------------------------------
//
// Replaces the former hardcoded constants. See siri_ble.h for why.

#define MAP_NVS_NAMESPACE "siri_ble"
#define MAP_NVS_KEY "hmapv1"

// A characteristic's CCCD is its own 0x2902 descriptor. Falls back to val+1,
// which is the usual layout, but only as a last resort — guessing is exactly
// what broke before.
static uint16_t find_cccd(const struct peer_chr *chr)
{
    const ble_uuid16_t cccd_uuid = BLE_UUID16_INIT(0x2902);
    const struct peer_dsc *dsc;
    SLIST_FOREACH(dsc, &chr->dscs, next)
    {
        if (ble_uuid_cmp(&dsc->dsc.uuid.u, &cccd_uuid.u) == 0) {
            return dsc->dsc.handle;
        }
    }
    return chr->chr.val_handle + 1;
}

// Build the map from a fully discovered GATT tree. Returns a failure reason,
// or NULL on success.
static const char *derive_handle_map(const struct peer *peer, siri_handle_map_t *out)
{
    const ble_uuid16_t batt_svc = BLE_UUID16_INIT(0x180F);
    const ble_uuid16_t batt_lvl = BLE_UUID16_INIT(0x2A19);
    const ble_uuid16_t batt_pwr = BLE_UUID16_INIT(0x2A1A);
    const ble_uuid16_t hid_svc = BLE_UUID16_INIT(HID_SVC_UUID16);
    const ble_uuid16_t hid_report = BLE_UUID16_INIT(HID_REPORT_UUID16);

    memset(out, 0, sizeof(*out));

    const struct peer_chr *c = peer_chr_find_uuid(peer, &batt_svc.u, &batt_lvl.u);
    if (c == NULL) {
        return "battery level 0x2A19 not found";
    }
    out->battery_val = c->chr.val_handle;
    out->battery_cccd = find_cccd(c);

    c = peer_chr_find_uuid(peer, &batt_svc.u, &batt_pwr.u);
    if (c == NULL) {
        return "battery power state 0x2A1A not found";
    }
    out->charging_val = c->chr.val_handle;
    out->charging_cccd = find_cccd(c);

    const struct peer_svc *hid = peer_svc_find_uuid(peer, &hid_svc.u);
    if (hid == NULL) {
        return "HID service 0x1812 not found";
    }

    // Reports arrive in handle order from peer_disc_all. The first three
    // notify-capable ones are audio/button/touch; the first non-notify one is
    // the vendor unlock. Ordinal rather than absolute, so a uniform shift in
    // the attribute table doesn't matter.
    const struct peer_chr *notify_reports[3] = {NULL, NULL, NULL};
    int n_notify = 0;
    const struct peer_chr *magic_chr = NULL;
    SLIST_FOREACH(c, &hid->chrs, next)
    {
        if (ble_uuid_cmp(&c->chr.uuid.u, &hid_report.u) != 0) {
            continue;
        }
        if (c->chr.properties & BLE_GATT_CHR_PROP_NOTIFY) {
            if (n_notify < 3) {
                notify_reports[n_notify] = c;
            }
            n_notify++;
        } else if (magic_chr == NULL) {
            magic_chr = c;
        }
    }
    if (n_notify < 3) {
        return "fewer than 3 notify-capable HID Reports";
    }
    if (magic_chr == NULL) {
        return "no non-notify HID Report for the vendor unlock";
    }

    out->audio_val = notify_reports[0]->chr.val_handle;
    out->audio_cccd = find_cccd(notify_reports[0]);
    out->button_val = notify_reports[1]->chr.val_handle;
    out->button_cccd = find_cccd(notify_reports[1]);
    out->touch_val = notify_reports[2]->chr.val_handle;
    out->touch_cccd = find_cccd(notify_reports[2]);
    out->magic = magic_chr->chr.val_handle;
    return NULL;
}

static void log_handle_map(const siri_handle_map_t *m)
{
    ESP_LOGI(TAG, "handle map: button=0x%04x/%04x touch=0x%04x/%04x audio=0x%04x/%04x",
             m->button_val, m->button_cccd, m->touch_val, m->touch_cccd, m->audio_val,
             m->audio_cccd);
    ESP_LOGI(TAG, "            battery=0x%04x/%04x charging=0x%04x/%04x magic=0x%04x",
             m->battery_val, m->battery_cccd, m->charging_val, m->charging_cccd, m->magic);
}

static void map_save(const siri_handle_map_t *m)
{
    nvs_handle_t h;
    if (nvs_open(MAP_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "handle map: nvs_open failed; will re-discover next boot");
        return;
    }
    if (nvs_set_blob(h, MAP_NVS_KEY, m, sizeof(*m)) == ESP_OK) {
        (void)nvs_commit(h);
        ESP_LOGI(TAG, "handle map cached to NVS");
    }
    nvs_close(h);
}

static bool map_load(siri_handle_map_t *m)
{
    nvs_handle_t h;
    if (nvs_open(MAP_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t len = sizeof(*m);
    bool ok = (nvs_get_blob(h, MAP_NVS_KEY, m, &len) == ESP_OK) && (len == sizeof(*m));
    nvs_close(h);
    return ok && m->button_val != 0 && m->magic != 0;
}

static void map_erase(void)
{
    nvs_handle_t h;
    if (nvs_open(MAP_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    (void)nvs_erase_key(h, MAP_NVS_KEY);
    (void)nvs_commit(h);
    nvs_close(h);
    s_map_valid = false;
}

// Physical handle -> logical SIRI_HANDLE_* so callers never see the layout.
static uint16_t canon_handle(uint16_t actual)
{
    if (!s_map_valid) {
        return actual;
    }
    if (actual == s_map.button_val)
        return SIRI_HANDLE_BUTTON;
    if (actual == s_map.touch_val)
        return SIRI_HANDLE_TOUCH;
    if (actual == s_map.audio_val)
        return SIRI_HANDLE_AUDIO;
    if (actual == s_map.battery_val)
        return SIRI_HANDLE_BATTERY;
    if (actual == s_map.charging_val)
        return SIRI_HANDLE_CHARGING;
    return actual;
}

static const setup_step_t SETUP_STEPS[] = {
    {SETUP_KIND_WRITE, H_BUTTON_CCCD, 0, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "button CCCD"},
    {SETUP_KIND_WRITE, H_TOUCH_CCCD, 0, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "touch CCCD"},
    {SETUP_KIND_WRITE, H_BATTERY_CCCD, 0, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "battery CCCD"},
    {SETUP_KIND_WRITE, H_CHARGING_CCCD, 0, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "charging CCCD"},
    {SETUP_KIND_WRITE, H_MAGIC, 0, MAGIC_VALUE, sizeof(MAGIC_VALUE), "magic unlock"},
    {SETUP_KIND_READ, H_BATTERY_VAL, 0, NULL, 0, "battery initial read"},
    {SETUP_KIND_READ, H_CHARGING_VAL, 0, NULL, 0, "charging initial read"},
#ifdef CONFIG_VOICE_ENABLED
    // Audio characteristic CCCD (val handle 0x0035, CCCD 0x0036). Subscribes
    // the bridge to the Opus-encoded voice stream that the remote emits
    // while the Mic button is held. siri_audio component decodes the frames
    // and dispatches PCM downstream.
    {SETUP_KIND_WRITE, H_AUDIO_CCCD, 0, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "audio CCCD"},
#endif
#ifdef CONFIG_DEBUG_WAKE_PROBE
    // CCCD probes for the three remaining notify-capable HID Reports on
    // gen-3 (0x0041/0x0045/0x0049). Confirmed real CCCDs via peer_disc_all.
    // 0x0051 used to be in this list but is Report Reference, not a CCCD.
    {SETUP_KIND_WRITE, H_LIT, 0x0042, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0042"},
    {SETUP_KIND_WRITE, H_LIT, 0x0046, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x0046"},
    {SETUP_KIND_WRITE, H_LIT, 0x004A, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY), "probe CCCD 0x004A"},
    // Audio HID Report CCCD (val=0x0035) — only remaining notify-capable
    // handle on the device that we don't normally subscribe to. The "wake-
    // info on the audio channel" hypothesis: Apple may multiplex a wake-
    // packet onto the Opus channel when the wake-press is suppressed from
    // the regular button path on cold-boot first-bond reconnect.
    {SETUP_KIND_WRITE, H_LIT, 0x0036, ENABLE_NOTIFY, sizeof(ENABLE_NOTIFY),
     "probe CCCD 0x0036 (audio)"},
    // 0x0025 (Apple-svc CCCD) is now subscribed in the EARLY block above.
    // GAP service reads — Apple's claim handshake might check these.
    {SETUP_KIND_READ, H_LIT, 0x0003, NULL, 0, "probe read GAP name 0x0003"},
    {SETUP_KIND_READ, H_LIT, 0x0005, NULL, 0, "probe read GAP appearance 0x0005"},
    {SETUP_KIND_READ, H_LIT, 0x0007, NULL, 0, "probe read GAP ppcp 0x0007"},
    {SETUP_KIND_READ, H_LIT, 0x0009, NULL, 0, "probe read GAP car 0x0009"},
    // Device Info Service reads — Apple's accessory framework on iOS hosts
    // reads these as part of standard claim. Strings (mfg/model/serial/
    // hw_rev/fw_rev) print as ASCII via the read-bytes log; PnP ID is a
    // 7-byte struct.
    {SETUP_KIND_READ, H_LIT, 0x000d, NULL, 0, "probe read mfg 0x000d"},
    {SETUP_KIND_READ, H_LIT, 0x000f, NULL, 0, "probe read model 0x000f"},
    {SETUP_KIND_READ, H_LIT, 0x0011, NULL, 0, "probe read serial 0x0011"},
    {SETUP_KIND_READ, H_LIT, 0x0013, NULL, 0, "probe read hw_rev 0x0013"},
    {SETUP_KIND_READ, H_LIT, 0x0015, NULL, 0, "probe read fw_rev 0x0015"},
    {SETUP_KIND_READ, H_LIT, 0x0017, NULL, 0, "probe read PnP 0x0017"},
    // Apple custom service reads — six read-only characteristics whose
    // contents Phase 1 never inspected. Reading them may also be part of
    // Apple's claim handshake.
    {SETUP_KIND_READ, H_LIT, 0x001a, NULL, 0, "probe read Apple 0x001a"},
    {SETUP_KIND_READ, H_LIT, 0x001c, NULL, 0, "probe read Apple 0x001c"},
    {SETUP_KIND_READ, H_LIT, 0x001e, NULL, 0, "probe read Apple 0x001e"},
    {SETUP_KIND_READ, H_LIT, 0x0020, NULL, 0, "probe read Apple 0x0020"},
    {SETUP_KIND_READ, H_LIT, 0x0022, NULL, 0, "probe read Apple 0x0022"},
    {SETUP_KIND_READ, H_LIT, 0x0024, NULL, 0, "probe read Apple 0x0024"},
    // Bond Management feature read.
    {SETUP_KIND_READ, H_LIT, 0x0028, NULL, 0, "probe read BondMgmt feat 0x0028"},
#endif
};
#define SETUP_STEP_COUNT (sizeof(SETUP_STEPS) / sizeof(SETUP_STEPS[0]))

static uint16_t s_setup_step_conn;
static size_t s_setup_step_idx;

static int on_setup_step_done(uint16_t conn_handle, const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg);

static bool is_wrong_handle_error(int status)
{
    return status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INVALID_HANDLE) ||
           status == BLE_HS_ATT_ERR(BLE_ATT_ERR_ATTR_NOT_FOUND) ||
           status == BLE_HS_ATT_ERR(BLE_ATT_ERR_READ_NOT_PERMITTED) ||
           status == BLE_HS_ATT_ERR(BLE_ATT_ERR_WRITE_NOT_PERMITTED);
}

// A step failed (status from the ATT callback or the queueing call). Returns
// true if the chain must stop here rather than skip to the next step.
static bool setup_step_failed(const setup_step_t *step, int status)
{
    if (status == BLE_HS_ENOTCONN || status == BLE_HS_ETIMEOUT) {
        // The link is gone (NimBLE fails in-flight procedures with ENOTCONN
        // before raising DISCONNECT; an ATT timeout ends the bearer). Every
        // later step would fail the same way, and the terminator would arm the
        // blackout and request conn params on a dead handle.
        ESP_LOGW(TAG, "%s: link lost (status=%d) — abandoning setup chain", step->label, status);
        return true;
    }
    if (step->sel != H_LIT && s_map_from_nvs && !s_map_invalidated &&
        is_wrong_handle_error(status)) {
        s_map_invalidated = true;
        ESP_LOGW(TAG,
                 "%s: ATT error 0x%02x on a cached handle — the remote's GATT layout has "
                 "likely changed (firmware update?); erasing the handle map and reconnecting "
                 "to rediscover",
                 step->label, (unsigned)(status - BLE_HS_ERR_ATT_BASE));
        map_erase();
        int rc = ble_gap_terminate(s_setup_step_conn, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0 && rc != BLE_HS_ENOTCONN) {
            ESP_LOGW(TAG, "ble_gap_terminate (stale map) rc=%d", rc);
        }
        return true;
    }
    return false;
}

static void issue_setup_step(size_t idx)
{
    if (idx >= SETUP_STEP_COUNT) {
        // Fresh bond (idle==0) means the user just held the pairing combo, and
        // Apple flushes those buffered presses once the button CCCD subscribe
        // lands. apply_remote_setup armed the blackout before step 0; re-arm
        // here so it also runs pairing_flush_suppress_ms past the chain's end.
        if (s_setup_idle_ms == 0 && s_cfg.pairing_flush_suppress_ms > 0) {
            s_suppress_until_ms = now_ms() + s_cfg.pairing_flush_suppress_ms;
            ESP_LOGI(TAG, "post-pairing notify blackout for %u ms",
                     (unsigned)s_cfg.pairing_flush_suppress_ms);
        }
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
    uint16_t handle = resolve_handle(step);
    int rc;
    if (handle == 0) {
        // No discovered handle for this step on this remote — skip rather
        // than writing to handle 0 and getting a confusing ATT error.
        ESP_LOGW(TAG, "%s: no handle in discovered map, skipping", step->label);
        s_setup_step_idx++;
        issue_setup_step(s_setup_step_idx);
        return;
    }
    if (step->kind == SETUP_KIND_WRITE) {
        rc = ble_gattc_write_flat(s_setup_step_conn, handle, step->value, step->len,
                                  on_setup_step_done, (void *)step);
    } else {
        rc = ble_gattc_read(s_setup_step_conn, handle, on_setup_step_done, (void *)step);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "%s: queue failed: rc=%d", step->label, rc);
        if (setup_step_failed(step, rc)) {
            return;
        }
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
        ESP_LOGE(TAG, "%s failed: status=0x%x att_handle=0x%04x", step->label, error->status,
                 error->att_handle);
        if (setup_step_failed(step, error->status)) {
            return 0;
        }
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
            ESP_LOGI(TAG, "%s ok (handle=0x%04x len=%u) %s", step->label, attr->handle,
                     (unsigned)copied, hex);
#else
            ESP_LOGI(TAG, "%s ok (handle=0x%04x len=%u)", step->label, attr->handle,
                     (unsigned)copied);
#endif
            if (s_cfg.on_notify != NULL) {
                s_cfg.on_notify(canon_handle(attr->handle), buf, copied, s_cfg.user);
            }
        }
    } else {
        ESP_LOGI(TAG, "%s ok (handle=0x%04x)", step->label, attr != NULL ? attr->handle : 0);
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
// fallback callout at ENC_CHANGE_FALLBACK_MS ms acts as a safety net if
// the secondary phase ever fails to fire.
static void apply_remote_setup(uint16_t conn_handle)
{
    if (s_setup_done_conn == conn_handle) {
        return;
    }
    s_setup_done_conn = conn_handle;
    s_setup_step_conn = conn_handle;
    s_setup_step_idx = 0;
    // Arm the post-pairing blackout before step 0, not just when the chain
    // ends: the flush follows the button CCCD subscribe, which is the first
    // step, and the remaining steps are several more ATT round-trips during
    // which flushed presses would otherwise reach on_notify. The terminator
    // re-arms it so the window still extends past the end of the chain.
    // Setup reads complete through on_setup_step_done, not NOTIFY_RX, so the
    // initial battery / charging values are unaffected.
    if (s_setup_idle_ms == 0 && s_cfg.pairing_flush_suppress_ms > 0) {
        s_suppress_until_ms = now_ms() + s_cfg.pairing_flush_suppress_ms;
    }
    issue_setup_step(0);
}

static void complete_setup_chain(uint16_t conn_handle, uint32_t idle_ms)
{
    s_mode = MODE_CONNECTED;
    s_setup_idle_ms = idle_ms;
    if (s_cfg.idle_disconnect_ms > 0) {
        s_last_activity_ms = now_ms();
        (void)ble_npl_callout_reset(&s_idle_co,
                                    ble_npl_time_ms_to_ticks32(s_cfg.idle_disconnect_ms));
    }
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
// Apple custom service present, and a derivable handle map. Bonded
// reconnects skip this — we trust prior bonds.

// Diagnostic dump of everything we discovered on a candidate. Only called on
// rejection: it is the only way to tell "not a Siri Remote" from "a Siri
// Remote whose layout our derivation doesn't understand" after the fact. A
// pass logs the derived handle map instead, and dumping dozens of lines on
// the NimBLE host task there would only delay the setup chain.
static void dump_gatt_tree(const struct peer *peer)
{
    const struct peer_svc *svc;
    ESP_LOGI(TAG, "  --- discovered GATT tree ---");
    SLIST_FOREACH(svc, &peer->svcs, next)
    {
        char uuid_buf[BLE_UUID_STR_LEN];
        ESP_LOGI(TAG, "  svc %s  handles %u..%u", ble_uuid_to_str(&svc->svc.uuid.u, uuid_buf),
                 (unsigned)svc->svc.start_handle, (unsigned)svc->svc.end_handle);
        const struct peer_chr *chr;
        SLIST_FOREACH(chr, &svc->chrs, next)
        {
            char cbuf[BLE_UUID_STR_LEN];
            ESP_LOGI(TAG, "    chr %s  val_handle=0x%04x props=0x%02x%s",
                     ble_uuid_to_str(&chr->chr.uuid.u, cbuf), (unsigned)chr->chr.val_handle,
                     (unsigned)chr->chr.properties,
                     (chr->chr.properties & BLE_GATT_CHR_PROP_NOTIFY) ? " [notify]" : "");
        }
    }
    ESP_LOGI(TAG, "  --- end GATT tree ---");
}

// Returns NULL on pass; otherwise a static string describing the failed check.
static const char *fingerprint_check(const struct peer *peer)
{
    const ble_uuid16_t hid_svc_uuid = BLE_UUID16_INIT(HID_SVC_UUID16);
    if (peer_svc_find_uuid(peer, &hid_svc_uuid.u) == NULL) {
        return "HID service 0x1812 missing";
    }
    // Deliberately no check for a Report at a specific handle. That used to be
    // here (0x0039) and it rejected every gen-3 unit whose attribute table is
    // shifted — which is what sent first-time pairing into an endless reject
    // loop while bonded reconnects, which skip this check, worked fine.
    // Identity is now structural: HID service, the Apple custom service, and a
    // derivable handle map (checked by the caller — derive_handle_map is what
    // requires the three notify-capable Reports, so they aren't counted here).
    if (peer_svc_find_uuid(peer, &APPLE_CUSTOM_SVC_UUID.u) == NULL) {
        return "Apple custom service 8341f2b4-... missing";
    }
    return NULL;
}

// The bridge bonds to exactly one remote, and on_sync reconnects to whichever
// bond the store lists first — so any other record left behind (an earlier
// remote, a candidate whose rejection predates identity-address deletes)
// could later win that race. Only called once the kept bond's identity
// address is known, so it can never delete the bond it was asked to keep.
static void prune_other_bonds(const ble_addr_t *keep)
{
    ble_addr_t addrs[CONFIG_BT_NIMBLE_MAX_BONDS];
    int n = 0;
    if (ble_store_util_bonded_peers(addrs, &n, CONFIG_BT_NIMBLE_MAX_BONDS) != 0) {
        return;
    }
    for (int i = 0; i < n; i++) {
        if (ble_addr_cmp(&addrs[i], keep) != 0) {
            log_addr("pruning stale bond", addrs[i].val);
            (void)ble_store_util_delete_peer(&addrs[i]);
        }
    }
}

static void on_fingerprint_disc_complete(const struct peer *peer, int status, void *arg)
{
    (void)arg;
    uint16_t conn = peer->conn_handle;

    // Link loss mid-discovery (NimBLE fails in-flight GATT procedures with
    // ENOTCONN before it raises DISCONNECT) or an ATT timeout says nothing
    // about what the device is. Blacklist it like a failed connect, not for
    // the fingerprint TTL — otherwise a genuine remote that dropped during
    // the ~3.7 s walk is locked out for a minute.
    uint32_t reject_ttl_ms = BLACKLIST_TTL_FINGERPRINT_MS;
    if (status != 0) {
        if (status == BLE_HS_ENOTCONN || status == BLE_HS_ETIMEOUT) {
            reject_ttl_ms = BLACKLIST_TTL_FAILED_CONNECT_MS;
        }
        ESP_LOGW(TAG, "fingerprint: peer_disc_all failed status=%d — rejecting candidate", status);
        goto reject;
    }

    const char *reason = fingerprint_check(peer);
    if (reason != NULL) {
        ESP_LOGI(TAG, "fingerprint FAIL: %s — rejecting candidate", reason);
        goto reject;
    }

    // Derivation is part of the verdict, not a later step: a remote we cannot
    // build a handle map for is one we cannot talk to, and failing here keeps
    // it in the reject/blacklist flow instead of bonding to something that
    // then silently never notifies.
    siri_handle_map_t derived;
    const char *derr = derive_handle_map(peer, &derived);
    if (derr != NULL) {
        ESP_LOGI(TAG, "fingerprint FAIL: handle map underivable (%s) — rejecting candidate", derr);
        goto reject;
    }
    s_map = derived;
    s_map_valid = true;
    s_map_from_nvs = false;
    ESP_LOGI(TAG, "fingerprint PASS — gen-3 Siri Remote confirmed, proceeding with setup");
    log_handle_map(&s_map);
    map_save(&s_map);
    if (s_pending_candidate_set) {
        // Identity, not advertised, address: a direct connect to the RPA would
        // stop matching once the remote rotates it, leaving a same-boot link
        // drop unable to reconnect until reboot.
        s_bonded_peer = *candidate_bond_addr();
        s_have_bonded_peer = true;
        if (s_pending_candidate_id_set) {
            prune_other_bonds(&s_bonded_peer);
        }
        s_pending_candidate_set = false;
    }
    ble_npl_callout_stop(&s_discovery_window_co);
    s_rediscover_fail_streak = 0;  // a new bond starts with a clean slate
    // Use idle=0 here: this is a brand-new bond, there's no meaningful
    // time-since-disconnect to publish. idle == 0 is also what arms the
    // post-pairing notify blackout at the end of the setup chain (Apple
    // flushes the buffered pairing-combo presses once the button CCCD
    // subscribes).
    complete_setup_chain(conn, 0);
    return;

reject:
    dump_gatt_tree(peer);
    if (s_pending_candidate_set) {
        // The blacklist matches advertisements, so it keys on the advertised
        // address; the bond record is keyed by identity.
        blacklist_add(s_pending_candidate.val, reject_ttl_ms);
        (void)ble_store_util_delete_peer(candidate_bond_addr());
        s_pending_candidate_set = false;
    }
    if (status != BLE_HS_ENOTCONN) {  // already gone: DISCONNECT is on its way
        (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    }
    // Disconnect handler will resume scanning while still inside the
    // discovery window.
}

static void on_bonded_disc_complete(const struct peer *peer, int status, void *arg)
{
    (void)arg;
    uint16_t conn = peer->conn_handle;
    if (status != 0) {
        if (status == BLE_HS_ENOTCONN || status == BLE_HS_ETIMEOUT) {
            // The link went away (out of range, Repair) — says nothing about
            // whether rediscovery works, so it must not feed the backoff.
            ESP_LOGW(TAG, "bonded rediscovery interrupted by link loss (status=%d)", status);
            if (status == BLE_HS_ETIMEOUT) {
                (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
            }
            return;
        }
        ESP_LOGE(TAG, "bonded rediscovery failed status=%d — terminating", status);
        s_rediscover_fail_streak++;
        (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    siri_handle_map_t derived;
    const char *derr = derive_handle_map(peer, &derived);
    if (derr != NULL) {
        ESP_LOGE(TAG, "bonded rediscovery: handle map underivable (%s) — terminating", derr);
        s_rediscover_fail_streak++;
        (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    s_rediscover_fail_streak = 0;
    s_map = derived;
    s_map_valid = true;
    s_map_from_nvs = false;
    ESP_LOGI(TAG, "handle map rediscovered for bonded remote");
    log_handle_map(&s_map);
    map_save(&s_map);
    complete_setup_chain(conn, s_pending_setup_idle_ms);
}

// Bonded path: run the setup chain, but rediscover first if we have no map.
static void setup_bonded(uint16_t conn, uint32_t idle)
{
    if (s_map_valid) {
        complete_setup_chain(conn, idle);
        return;
    }
    ESP_LOGI(TAG, "bonded remote but no cached handle map — rediscovering once");
    s_pending_setup_idle_ms = idle;
    int rc = peer_disc_all(conn, on_bonded_disc_complete, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "bonded rediscovery kickoff failed rc=%d — terminating", rc);
        s_rediscover_fail_streak++;
        (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    }
}

// Fired on the NimBLE host task if the second ENC_CHANGE never arrives. Runs
// setup with the stashed idle so we don't end up wedged forever in pending
// state. The s_pending_setup_conn check also covers a fire that was already
// queued when the secondary ENC_CHANGE or DISCONNECT stopped the callout.
static void setup_fallback_cb(struct ble_npl_event *ev)
{
    (void)ev;
    if (s_pending_setup_conn == 0xFFFF) {
        return;
    }
    ESP_LOGW(TAG, "second ENC_CHANGE timed out at %d ms; running setup anyway",
             ENC_CHANGE_FALLBACK_MS);
    uint16_t conn = s_pending_setup_conn;
    uint32_t idle = s_pending_setup_idle_ms;
    s_pending_setup_conn = 0xFFFF;
    s_setup_dispatched_conn = conn;
    if (s_mode == MODE_DISCOVERING) {
        // Fallback during discovery: kick fingerprint manually, since the
        // secondary ENC_CHANGE that would have triggered it never arrived.
        int rc = peer_disc_all(conn, on_fingerprint_disc_complete, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "fingerprint: peer_disc_all kickoff failed rc=%d", rc);
            (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    } else {
        setup_bonded(conn, idle);
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
    SLIST_FOREACH(svc, &peer->svcs, next)
    {
        char uuid_buf[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&svc->svc.uuid.u, uuid_buf);
        ESP_LOGI(TAG, "  svc %s handles=0x%04x..0x%04x", uuid_buf, svc->svc.start_handle,
                 svc->svc.end_handle);
        const struct peer_chr *chr;
        SLIST_FOREACH(chr, &svc->chrs, next)
        {
            ble_uuid_to_str(&chr->chr.uuid.u, uuid_buf);
            ESP_LOGI(TAG, "    chr %s def=0x%04x val=0x%04x props=0x%02x", uuid_buf,
                     chr->chr.def_handle, chr->chr.val_handle, chr->chr.properties);
            const struct peer_dsc *dsc;
            SLIST_FOREACH(dsc, &chr->dscs, next)
            {
                ble_uuid_to_str(&dsc->dsc.uuid.u, uuid_buf);
                ESP_LOGI(TAG, "      dsc %s handle=0x%04x", uuid_buf, dsc->dsc.handle);
            }
        }
    }

    const struct peer_svc *apple = peer_svc_find_uuid(peer, &APPLE_CUSTOM_SVC_UUID.u);
    if (apple == NULL) {
        ESP_LOGW(TAG, "wake_probe: Apple custom service 8341f2b4-... not found");
        return;
    }
    const struct peer_chr *chr;
    SLIST_FOREACH(chr, &apple->chrs, next)
    {
        if (!(chr->chr.properties & BLE_GATT_CHR_PROP_NOTIFY)) {
            continue;
        }
        const struct peer_dsc *dsc;
        SLIST_FOREACH(dsc, &chr->dscs, next)
        {
            if (ble_uuid_u16(&dsc->dsc.uuid.u) != BLE_GATT_DSC_CLT_CFG_UUID16) {
                continue;
            }
            ESP_LOGI(TAG, "wake_probe: subscribing Apple-svc CCCD 0x%04x (val=0x%04x)",
                     dsc->dsc.handle, chr->chr.val_handle);
            int rc = ble_gattc_write_flat(peer->conn_handle, dsc->dsc.handle, ENABLE_NOTIFY,
                                          sizeof(ENABLE_NOTIFY), NULL, NULL);
            if (rc != 0) {
                ESP_LOGW(TAG, "wake_probe: CCCD 0x%04x write failed rc=%d", dsc->dsc.handle, rc);
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

static void idle_cb(struct ble_npl_event *ev)
{
    (void)ev;
    uint16_t conn = s_setup_done_conn;
    if (conn == 0xFFFF) {
        return;  // disconnected after the callout fired
    }
    uint32_t elapsed = now_ms() - s_last_activity_ms;
    if (elapsed < s_cfg.idle_disconnect_ms) {
        (void)ble_npl_callout_reset(&s_idle_co,
                                    ble_npl_time_ms_to_ticks32(s_cfg.idle_disconnect_ms - elapsed));
        return;
    }
    ESP_LOGI(TAG, "idle threshold reached, terminating connection so remote can sleep");
    int rc = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0 && rc != BLE_HS_ENOTCONN) {
        ESP_LOGW(TAG, "ble_gap_terminate failed: %d", rc);
    }
}

static void enc_watchdog_cb(struct ble_npl_event *ev)
{
    (void)ev;
    uint16_t conn = s_enc_watchdog_conn;
    if (conn == 0xFFFF || conn != s_active_conn) {
        return;  // encrypted or disconnected after the callout fired
    }
    s_enc_watchdog_conn = 0xFFFF;
    ESP_LOGW(TAG, "link not encrypted %d ms after connect — terminating (mode=%d)", ENC_WATCHDOG_MS,
             (int)s_mode);
    if (s_mode == MODE_BONDED_RECONNECT) {
        ESP_LOGW(TAG, "if this repeats, the remote has likely lost our bond (factory reset or "
                      "re-paired elsewhere) — press Repair, then hold Back+VolUp ~5s");
    }
    int rc = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0 && rc != BLE_HS_ENOTCONN) {
        ESP_LOGW(TAG, "ble_gap_terminate (enc watchdog) rc=%d", rc);
    }
}

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
        // Cheap gates before parsing: with duplicate filtering off, every
        // nearby advertiser reports many times a second, and nearly all of
        // them fail one of these. Parsing walks the whole payload.
        if (s_pending_candidate_set) {
            return 0;  // already mid-connect to a candidate
        }
        if (event->disc.rssi < DISCOVERY_RSSI_MIN) {
            return 0;  // too far away to be the remote the user is holding
        }
        if (blacklist_contains(event->disc.addr.val)) {
            return 0;  // already-rejected device
        }
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0) {
            return 0;  // malformed adv
        }
        try_connect_candidate(&event->disc, &fields);
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            if (event->connect.status == BLE_HS_EAPP) {
                // Our own ble_gap_conn_cancel (siri_ble_repair), not a failure.
                ESP_LOGI(TAG, "pending connect cancelled");
            } else {
                ESP_LOGE(TAG, "connect failed: status=%d", event->connect.status);
            }
            if (s_mode == MODE_DISCOVERING && s_pending_candidate_set) {
                blacklist_add(s_pending_candidate.val, BLACKLIST_TTL_FAILED_CONNECT_MS);
            }
            s_pending_candidate_set = false;
            if (s_mode == MODE_BONDED_RECONNECT) {
                // Re-issue direct connect — peer will eventually advertise.
                start_bonded_reconnect();
            } else if (s_mode == MODE_DISCOVERING) {
                start_discovery(false);
            }
            return 0;
        }
        s_active_conn = event->connect.conn_handle;
        ESP_LOGI(TAG, "connected, conn_handle=%d", event->connect.conn_handle);
        // On bonded reconnect ENC_CHANGE can precede CONNECT (see the
        // ENC_CHANGE handler); arm only if encryption hasn't already landed.
        if (s_pending_setup_conn != event->connect.conn_handle &&
            !setup_started(event->connect.conn_handle)) {
            s_enc_watchdog_conn = event->connect.conn_handle;
            (void)ble_npl_callout_reset(&s_enc_watchdog_co,
                                        ble_npl_time_ms_to_ticks32(ENC_WATCHDOG_MS));
        }
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
        //
        // But if encryption already *completed* before CONNECT was dispatched
        // (NimBLE holds a central's CONNECT until Read Remote Version is done),
        // no procedure is pending and the call starts a fresh re-encryption.
        // Its ENC_CHANGE would then be taken for Apple's secondary and run
        // setup early — inside the window that loses the wake-press.
        struct ble_gap_conn_desc cdesc;
        bool already_encrypted =
            ble_gap_conn_find(event->connect.conn_handle, &cdesc) == 0 && cdesc.sec_state.encrypted;
        if (!already_encrypted && s_pending_setup_conn != event->connect.conn_handle) {
            int sec_rc = ble_gap_security_initiate(event->connect.conn_handle);
            if (sec_rc != 0 && sec_rc != BLE_HS_EALREADY) {
                ESP_LOGE(TAG, "security_initiate failed: rc=%d", sec_rc);
            }
        }
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
            WAKE_PROBE_LOG("CONN_UPDATE status=%d itvl=%u lat=%u sup=%u", event->conn_update.status,
                           desc.conn_itvl, desc.conn_latency, desc.supervision_timeout);
        } else {
            WAKE_PROBE_LOG("CONN_UPDATE status=%d (conn_find failed)", event->conn_update.status);
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        bool failed_pre_fingerprint =
            (s_mode == MODE_DISCOVERING && s_pending_candidate_set && s_setup_done_conn == 0xFFFF);
        ESP_LOGI(TAG, "disconnected, reason=0x%04x", event->disconnect.reason);
        peer_delete(event->disconnect.conn.conn_handle);
        s_last_disconnect_ms = now_ms();
        s_has_disconnected = true;
        s_setup_done_conn = 0xFFFF;
        s_setup_dispatched_conn = 0xFFFF;
        s_active_conn = 0xFFFF;
        if (s_pending_setup_conn != 0xFFFF) {
            ble_npl_callout_stop(&s_setup_fallback_co);
            s_pending_setup_conn = 0xFFFF;
        }
        ble_npl_callout_stop(&s_enc_watchdog_co);
        s_enc_watchdog_conn = 0xFFFF;
        ble_npl_callout_stop(&s_idle_co);
        if (failed_pre_fingerprint) {
            // Connect was attempted but the link never reached fingerprint —
            // most often the peer is advertising but won't admit us at LL
            // until the user puts the remote in pairing mode (Back+VolUp).
            // Short blacklist breaks the retry loop while still recovering
            // quickly when the user does press the combo.
            ESP_LOGI(TAG,
                     "candidate failed pre-fingerprint — blacklisting %d s, "
                     "hold Back+VolUp ~5s on the remote to pair",
                     BLACKLIST_TTL_FAILED_CONNECT_MS / 1000);
            blacklist_add(s_pending_candidate.val, BLACKLIST_TTL_FAILED_CONNECT_MS);
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
            start_discovery(false);
        } else if (s_mode == MODE_BONDED_RECONNECT) {
            schedule_bonded_reconnect();
        } else if (s_mode == MODE_CONNECTED) {
            if (s_have_bonded_peer) {
                schedule_bonded_reconnect();
            } else {
                start_discovery(true);
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
        if (!setup_started(event->enc_change.conn_handle) &&
            s_pending_setup_conn != event->enc_change.conn_handle) {
            s_connect_ms = now_ms();
        }
#endif
        ESP_LOGI(TAG, "encryption changed, status=%d", event->enc_change.status);
        WAKE_PROBE_LOG("ENC_CHANGE status=%d %s", event->enc_change.status,
                       setup_started(event->enc_change.conn_handle)            ? "(post-setup)"
                       : s_pending_setup_conn == event->enc_change.conn_handle ? "(secondary)"
                                                                               : "(initial)");
        if (event->enc_change.status != 0) {
            if (setup_started(event->enc_change.conn_handle)) {
                return 0;  // failed mid-session re-key; the controller handles the link
            }
            if (event->enc_change.status == BLE_HS_ENOTCONN) {
                // The link dropped mid-procedure (ble_sm_connection_broken
                // raises this before DISCONNECT). Not a key problem, and there
                // is nothing left to terminate.
                return 0;
            }
            // Pre-setup failure: the link can never carry HID (the remote's
            // reports need encryption), so don't wait out the watchdog. Most
            // often the remote no longer holds our keys (PIN/key missing).
            s_enc_fail_streak++;
            ESP_LOGW(TAG, "encryption failed (status=%d, %lu in a row) — terminating",
                     event->enc_change.status, (unsigned long)s_enc_fail_streak);
            if (s_mode == MODE_BONDED_RECONNECT) {
                ESP_LOGW(TAG, "the remote has likely lost our bond (factory reset or re-paired "
                              "elsewhere) — press Repair, then hold Back+VolUp ~5s");
            }
            ble_npl_callout_stop(&s_enc_watchdog_co);
            s_enc_watchdog_conn = 0xFFFF;
            int trc = ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            if (trc != 0 && trc != BLE_HS_ENOTCONN) {
                ESP_LOGW(TAG, "ble_gap_terminate (enc failure) rc=%d", trc);
            }
            return 0;
        }
        if (setup_started(event->enc_change.conn_handle)) {
            // Setup already dispatched for this connection. Further
            // ENC_CHANGEs are mid-session re-keys (Apple's accessory framework
            // periodically refreshes encryption), or a secondary that arrived
            // after the fallback already ran; ignore.
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
                // lower bound on idle duration. The value is mostly
                // diagnostic; only idle_ms == 0 (fresh fingerprint pass,
                // which arms the post-pairing blackout) carries meaning.
                idle = now_ms();
            }
            if (s_mode == MODE_DISCOVERING && s_pending_candidate_set) {
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(conn, &desc) == 0) {
                    s_pending_candidate_id = desc.peer_id_addr;
                    s_pending_candidate_id_set = true;
                }
            }
            s_pending_setup_conn = conn;
            s_pending_setup_idle_ms = idle;
            s_enc_fail_streak = 0;
            ble_npl_callout_stop(&s_enc_watchdog_co);
            s_enc_watchdog_conn = 0xFFFF;
            (void)ble_npl_callout_reset(&s_setup_fallback_co,
                                        ble_npl_time_ms_to_ticks32(ENC_CHANGE_FALLBACK_MS));
            return 0;
        }
        // Secondary ENC_CHANGE — Apple's accessory-framework handshake has
        // settled. Cancel fallback. If we're discovering (no prior bond),
        // run a fingerprint check before committing to the bond — peer_disc_all
        // walks the GATT tree, then on_fingerprint_disc_complete either runs
        // setup or rejects+blacklists. If we already trust the bond, skip
        // straight to setup; Apple flushes the buffered wake-press to us
        // shortly after the button CCCD subscribe lands.
        ble_npl_callout_stop(&s_setup_fallback_co);
        uint32_t idle = s_pending_setup_idle_ms;
        s_pending_setup_conn = 0xFFFF;
        s_setup_dispatched_conn = conn;
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
            setup_bonded(conn, idle);
        }
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        if (s_suppress_until_ms != 0) {
            if ((int32_t)(now_ms() - s_suppress_until_ms) < 0) {
                // Wrap-safe compare. Silent by design: the flush can be dozens
                // of packets and logging each would bury the pairing output.
                return 0;
            }
            s_suppress_until_ms = 0;
            ESP_LOGI(TAG, "post-pairing notify blackout over");
        }
        s_last_activity_ms = now_ms();
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
                     (unsigned long)(now_ms() - s_connect_ms), event->notify_rx.attr_handle, copied,
                     hex);
        }
#endif
        if (s_cfg.on_notify != NULL) {
            s_cfg.on_notify(canon_handle(event->notify_rx.attr_handle), buf, copied, s_cfg.user);
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

// Runs on the NimBLE host task; siri_ble_repair() posts it.
static void repair_ev_cb(struct ble_npl_event *ev)
{
    (void)ev;
    // The map belongs to the old remote; a different unit will have its own.
    map_erase();
    ESP_LOGI(TAG, "repair: clearing bond store and re-entering discovery");
    int rc = ble_store_clear();
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_store_clear rc=%d", rc);
    }
    s_have_bonded_peer = false;
    s_pending_candidate_set = false;
    blacklist_clear();
    s_rediscover_fail_streak = 0;
    ble_npl_callout_stop(&s_reconnect_co);
    // Repair always gets a full window. Stopping it here makes whichever path
    // starts the scan (directly below, or the cancelled-connect / disconnect
    // handler) arm it fresh, since start_discovery re-arms an inactive window.
    ble_npl_callout_stop(&s_discovery_window_co);

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
    // No active connection, but a connect may still be pending: the
    // BLE_HS_FOREVER direct connect of BONDED_RECONNECT, or a discovery
    // candidate's. NimBLE refuses to scan while one is (ble_gap_disc returns
    // BLE_HS_EBUSY), which is exactly the dead-or-replaced-remote case Repair
    // exists for. Cancelling is asynchronous — NimBLE reports it as a CONNECT
    // event with BLE_HS_EAPP once the controller confirms — so set the mode
    // first and let that handler's DISCOVERING branch start the scan.
    s_mode = MODE_DISCOVERING;
    if (ble_gap_conn_cancel() == 0) {
        ESP_LOGI(TAG, "repair: cancelling pending connect; discovery starts when it completes");
        return;
    }
    // Nothing pending (BLE_HS_EALREADY). Stop any scan in progress, then
    // restart the window from scratch.
    (void)ble_gap_disc_cancel();
    start_discovery(true);
}

void siri_ble_repair(void)
{
    // Callable from any task. A second press while one is still queued is a
    // no-op (eventq_put ignores an already-queued event), which is what a
    // double-tap of the HA button should do anyway.
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_repair_ev);
}

static void set_slave_latency(uint16_t latency)
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
    // NimBLE's ble_gap_validate_conn_params rejects latency > 499 and any
    // timeout <= (1 + latency) * itvl_max / 4. With itvl_max = 12 and the
    // 1500 ceiling below, 498 is the largest latency that passes; above it
    // ble_gap_update_params returns EINVAL and the link silently stays on
    // latency 0. Kconfig and the ESPHome schema cap at 498 too — this clamp
    // only guards a caller that bypasses them.
    if (latency > SLAVE_LATENCY_MAX) {
        ESP_LOGW(TAG, "slave_latency %u exceeds %u; clamping", latency, SLAVE_LATENCY_MAX);
        latency = SLAVE_LATENCY_MAX;
    }
    uint32_t numerator = (uint32_t)(1 + latency) * s_low_power_conn_params.itvl_max * 25;
    uint32_t timeout = (numerator + 79) / 80;  // ceil(numerator / 80)
    if (timeout < 100)
        timeout = 100;  // 1 s floor
    if (timeout > 1500)
        timeout = 1500;  // 15 s ceiling

    s_low_power_conn_params.latency = latency;
    s_low_power_conn_params.supervision_timeout = (uint16_t)timeout;

    ESP_LOGI(TAG, "slave_latency=%u sup_timeout=%lu (= %lu ms)", latency, (unsigned long)timeout,
             (unsigned long)timeout * 10);
}

// While the host is resetting, start_discovery / start_bonded_reconnect refuse
// to issue anything (ble_hs_synced() is false); on_sync re-dispatches from the
// bond store once the controller is back.
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
    // Inspect the bond store. If a peer is persisted from a previous boot,
    // skip discovery and go straight to a direct connect — NimBLE's resolving
    // list translates the bonded identity address into the peer's current
    // resolvable random adv automatically. If the store is empty (first boot
    // or post-repair), enter the 5-minute discovery window.
    ble_addr_t peer_id_addrs[CONFIG_BT_NIMBLE_MAX_BONDS];
    int num_bonded = 0;
    int rc = ble_store_util_bonded_peers(peer_id_addrs, &num_bonded, CONFIG_BT_NIMBLE_MAX_BONDS);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_store_util_bonded_peers rc=%d; entering discovery", rc);
        num_bonded = 0;
    }
    int our_count = 0;
    (void)ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &our_count);
    int cccd_count = 0;
    (void)ble_store_util_count(BLE_STORE_OBJ_TYPE_CCCD, &cccd_count);
    ESP_LOGI(TAG, "bond store: %d peer-sec / %d our-sec / %d CCCD record(s)", num_bonded, our_count,
             cccd_count);

    if (num_bonded > 0) {
        // Use the first bonded peer. We never expect more than one — the
        // bridge bonds to exactly one remote — but tolerate the array form.
        s_bonded_peer = peer_id_addrs[0];
        s_have_bonded_peer = true;
        start_bonded_reconnect();
    } else {
        s_have_bonded_peer = false;
        start_discovery(true);
    }
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t siri_ble_start(const siri_ble_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    set_slave_latency(s_cfg.slave_latency);

    // Takes effect in the standalone build only. ESPHome compiles IDF logging
    // without dynamic level control, where esp_log_level_set() is a no-op and
    // CONFIG_LOG_MAXIMUM_LEVEL is the only filter (see _pin_sdkconfig).
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }
    ble_store_config_init();

    // Restore the remote's discovered GATT layout, if we have one. Absent on
    // first boot and after a repair; the bonded path rediscovers in that case.
    if (map_load(&s_map)) {
        s_map_valid = true;
        s_map_from_nvs = true;
        ESP_LOGI(TAG, "handle map restored from NVS");
        log_handle_map(&s_map);
    } else {
        ESP_LOGI(TAG, "no cached handle map — will discover on first bond");
    }

    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sync_cb = on_sync;
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

    // nimble_port_init() created the default event queue above; the host task
    // that drains it starts below, so nothing can fire before it is set up.
    struct ble_npl_eventq *evq = nimble_port_get_dflt_eventq();
    if (ble_npl_callout_init(&s_setup_fallback_co, evq, setup_fallback_cb, NULL) != 0 ||
        ble_npl_callout_init(&s_discovery_window_co, evq, discovery_window_expired_cb, NULL) != 0 ||
        ble_npl_callout_init(&s_enc_watchdog_co, evq, enc_watchdog_cb, NULL) != 0 ||
        ble_npl_callout_init(&s_idle_co, evq, idle_cb, NULL) != 0 ||
        ble_npl_callout_init(&s_reconnect_co, evq, reconnect_cb, NULL) != 0) {
        ESP_LOGE(TAG, "ble_npl_callout_init failed");
        return ESP_FAIL;
    }
    ble_npl_event_init(&s_repair_ev, repair_ev_cb, NULL);

    nimble_port_freertos_init(nimble_host_task);
    return ESP_OK;
}
