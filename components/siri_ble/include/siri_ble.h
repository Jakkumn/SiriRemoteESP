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
//   - **0** — fresh first-bond from the DISCOVERING flow. siri_ble itself
//     suppresses the buffered pairing-combo flush that follows (see
//     pairing_flush_suppress_ms); callers need not act on it.
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

    // Terminate the link after this many milliseconds without a notification,
    // so the remote can deep-sleep; it reconnects on the next button press.
    // 0 (the default) is always-connected mode — see README before changing:
    // Apple's firmware discards the wake-press after a long disconnect.
    uint32_t idle_disconnect_ms;

    // Peripheral latency requested once the setup chain completes, 0..498.
    // Supervision timeout is derived from it (timeout > 2 * itvl_max *
    // (1 + latency), ~25 % margin, capped at 15 s). Above 498 NimBLE rejects
    // the update and the link silently stays at latency 0, so it is clamped.
    uint16_t slave_latency;
} siri_ble_config_t;

// Bring up the whole BLE stack: NimBLE port, bond store, host configuration,
// and the host task. Call once, after nvs_flash_init(), and do not call
// nimble_port_init() / nimble_port_freertos_init() yourself.
esp_err_t siri_ble_start(const siri_ble_config_t *cfg);

// Forget the current bond and re-enter the 5-minute discovery window.
// Called when the user presses `button.siri_remote_repair` in HA, or when
// they want to switch to a different physical remote. Wipes the NimBLE bond
// store, terminates the active connection (if any), clears the in-RAM
// candidate blacklist, and starts a fresh passive scan filtered on the HID
// service UUID. The user is expected to put the remote into Apple-pairing
// mode (Back + Vol Up held ~5 s) during the window.
//
// Safe from any task, and asynchronous: it posts the request to the NimBLE
// host task, where the work runs alongside every other BLE state change.
void siri_ble_repair(void);

#ifdef __cplusplus
}
#endif
