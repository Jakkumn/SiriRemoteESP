#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Gen-3 Siri Remote GATT *value* handles. Empirically verified in Phase 1
// and reproducible across reconnects on the same unit; main.c uses these
// to dispatch on `attr_handle` from notify / read callbacks. CCCD handles
// for these characteristics are siri_ble's internal concern (the bridge
// subscribes during the setup chain) and stay private to the component.
#define SIRI_HANDLE_BUTTON      0x0039  // 16-bit button bitmap (notify)
#define SIRI_HANDLE_TOUCH       0x003D  // 11-byte touch frames (notify, ~50/sec)
#define SIRI_HANDLE_AUDIO       0x0035  // Opus audio (Phase 5)
#define SIRI_HANDLE_AUDIO_CCCD  0x0036  // CCCD for SIRI_HANDLE_AUDIO
#define SIRI_HANDLE_BATTERY     0x002E  // single-byte percentage (read + notify)
#define SIRI_HANDLE_CHARGING    0x0031  // BLE-standard 0x2A1A power-state byte (read + notify)

// Raw notification from the remote. `attr_handle` lets the caller
// distinguish button / touch / audio / battery / charging etc — see the
// SIRI_HANDLE_* constants above.
// `data` is owned by NimBLE and valid only for the duration of the call.
typedef void (*siri_ble_notify_cb_t)(uint16_t attr_handle, const uint8_t *data,
                                     size_t len, void *user);

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
    siri_ble_notify_cb_t       on_notify;
    siri_ble_connected_cb_t    on_connected;     // optional
    siri_ble_disconnected_cb_t on_disconnected;  // optional
    void                      *user;
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
