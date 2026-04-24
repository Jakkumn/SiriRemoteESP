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
typedef void (*siri_ble_notify_cb_t)(uint16_t attr_handle, const uint8_t *data, size_t len);

typedef struct {
    siri_ble_notify_cb_t on_notify;
} siri_ble_config_t;

// Kick off scanning. Must be called after NimBLE is synced (from ble_hs_cfg.sync_cb).
esp_err_t siri_ble_start(const siri_ble_config_t *cfg);

#ifdef __cplusplus
}
#endif
