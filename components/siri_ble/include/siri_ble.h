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
// mode (TV + Vol Up held ~5 s) during the window.
void siri_ble_repair(void);

#ifdef __cplusplus
}
#endif
