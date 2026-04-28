#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Raw notification from the remote. `attr_handle` lets the caller
// distinguish button (0x0039) / touch (0x003D) / audio (0x0035) etc.
// `data` is owned by NimBLE and valid only for the duration of the call.
typedef void (*siri_ble_notify_cb_t)(uint16_t attr_handle, const uint8_t *data,
                                     size_t len, void *user);

// Fired on successful encrypted connection. `idle_ms_since_disconnect` is the
// time since the last disconnect in ms, or 0 on first-boot connect.
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
