#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Logical stream identifiers — NOT literal GATT handles.
//
// They started life as hardcoded handles for one gen-3 unit, and that was
// wrong: a second gen-3 remote with different firmware carries the identical
// attribute *structure* shifted by one handle, so every hardcoded number
// pointed at its neighbour. The symptoms were silent (a CCCD write landing on
// a Report value handle "succeeds" and simply never notifies), which made it
// expensive to diagnose.
//
// Handles are now discovered once per bond and cached in NVS (see
// siri_handle_map_t). siri_ble translates every real handle to one of these
// IDs before invoking on_notify, so callers dispatch on a stable value and
// never see the physical layout. The numbers are the reference unit's handles,
// kept only so existing switch statements keep compiling.
#define SIRI_HANDLE_BUTTON 0x0039
#define SIRI_HANDLE_TOUCH 0x003D
#define SIRI_HANDLE_AUDIO 0x0035
#define SIRI_HANDLE_AUDIO_CCCD 0x0036
#define SIRI_HANDLE_BATTERY 0x002E
#define SIRI_HANDLE_CHARGING 0x0031

// The physical layout discovered from a specific remote. Derived from the
// GATT tree at first bond, cached in NVS, reloaded on reconnect so the
// discovery cost (~3.7 s) is paid once rather than every connect.
//
// Derivation rules, all structural rather than positional:
//   battery/charging — by UUID (0x2A19 / 0x2A1A) inside service 0x180F
//   audio/button/touch — the first three notify-capable Report (0x2A4D)
//     characteristics inside the HID service 0x1812, in handle order
//   magic — the first Report characteristic *without* notify
//   every CCCD — that characteristic's own 0x2902 descriptor
typedef struct {
    uint16_t button_val, button_cccd;
    uint16_t touch_val, touch_cccd;
    uint16_t audio_val, audio_cccd;
    uint16_t battery_val, battery_cccd;
    uint16_t charging_val, charging_cccd;
    uint16_t magic;
} siri_handle_map_t;

// The active map, or NULL if no remote has been bonded yet. Diagnostic.
const siri_handle_map_t *siri_ble_handle_map(void);

// Raw notification from the remote. `attr_handle` is a *logical* SIRI_HANDLE_*
// ID, already translated from the remote's physical handle — callers must not
// assume it matches any particular unit's GATT layout.
// `data` is owned by NimBLE and valid only for the duration of the call.
typedef void (*siri_ble_notify_cb_t)(uint16_t attr_handle, const uint8_t *data, size_t len,
                                     void *user);

// Fired on successful encrypted connection (after the post-secondary
// ENC_CHANGE fingerprint pass on first-pair, or after a bonded reconnect's
// setup chain in always-connected mode).
//
// `idle_ms_since_disconnect` is overloaded:
//   - **0** — fresh first-bond from the DISCOVERING flow. Caller may use
//     this as a discriminator to suppress buffered HID notifies (the
//     pairing-combo flush) for ~1.5 s.
//   - **>0** — warm reconnect after the link dropped (Wi-Fi outage,
//     remote out of range, bridge reboot). Value reflects elapsed wall
//     time since the previous disconnect. Mostly diagnostic — in
//     always-connected mode (`CONFIG_IDLE_DISCONNECT_MS=0`, default) the
//     link doesn't deliberately drop, so this path is exceptional.
typedef void (*siri_ble_connected_cb_t)(uint32_t idle_ms_since_disconnect, void *user);

// Fired immediately on BLE disconnect event.
typedef void (*siri_ble_disconnected_cb_t)(void *user);

typedef struct {
    siri_ble_notify_cb_t on_notify;
    siri_ble_connected_cb_t on_connected;        // optional
    siri_ble_disconnected_cb_t on_disconnected;  // optional
    void *user;

    // After a *fresh* bond's setup chain completes, drop every incoming
    // notification for this many milliseconds. 0 disables.
    //
    // This does not stop the remote entering pairing mode — that is its own
    // firmware and none of our business. It discards the echo: the remote
    // buffers the Back+VolUp presses you physically made during pairing and
    // Apple flushes them the instant the button CCCD subscribe lands, so they
    // arrive as ordinary button notifications the user never meant as
    // commands. Under the pulse model those would fire whatever automation is
    // bound to those buttons, so pairing the remote could change your TV
    // volume. Everything is dropped rather than just buttons, because the
    // flush is not guaranteed to be buttons-only.
    //
    // Setup-chain reads (battery/charging) are issued by us and are delivered
    // regardless — they are not part of the flush.
    uint32_t pairing_flush_suppress_ms;
} siri_ble_config_t;

// Install callbacks and configure the BLE host. Call after nimble_port_init
// but before spawning the host task.
esp_err_t siri_ble_start(const siri_ble_config_t *cfg);

// Proactively terminate the active BLE connection so the remote can enter
// deep sleep. No-op if no connection is active. Reconnect happens on the
// next button press from the remote (re-advertises automatically).
void siri_ble_idle_disconnect(void);

// Forget the current bond and re-enter the 5-minute discovery window.
// Called when the user presses `button.siri_remote_repair` in HA, or when
// they want to switch to a different physical remote. Wipes the NimBLE bond
// store, terminates the active connection (if any), clears the in-RAM
// candidate blacklist, and starts a fresh active scan filtered on the HID
// service UUID. The user is expected to put the remote into Apple-pairing
// mode (Back + Vol Up held ~5 s) during the window.
void siri_ble_repair(void);

// Update slave-latency on the post-setup low-power conn params. Supervision
// timeout is auto-recomputed from latency to satisfy the BLE spec rule
// (timeout > 2 * itvl_max * (1 + latency)) with ~25 % margin, capped at
// 1500 (15 s). If a connection is currently active and post-setup, applies
// via ble_gap_update_params immediately; otherwise the new value takes
// effect on the next bonded reconnect. Apple may reject the requested
// params (CONN_UPDATE status != 0); the cached value is kept either way
// and the controller falls back to whatever was previously negotiated.
void siri_ble_set_slave_latency(uint16_t latency);

#ifdef __cplusplus
}
#endif
