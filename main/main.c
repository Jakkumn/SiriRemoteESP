#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "siri_ble.h"

// Forward-declared — ESP-IDF ships the implementation in libbt.a but doesn't
// expose the header via include path.
void ble_store_config_init(void);

static const char *TAG = "main";

static void on_notify(uint16_t attr_handle, const uint8_t *data, size_t len)
{
    char hex[3 * 128 + 1];
    size_t pos = 0;
    size_t cap = len > 128 ? 128 : len;
    for (size_t i = 0; i < cap; i++) {
        int n = snprintf(hex + pos, sizeof(hex) - pos, "%02x ", data[i]);
        if (n <= 0 || (size_t)n >= sizeof(hex) - pos) {
            break;
        }
        pos += (size_t)n;
    }
    if (pos > 0 && hex[pos - 1] == ' ') {
        hex[pos - 1] = '\0';
    } else {
        hex[pos] = '\0';
    }
    ESP_LOGI(TAG, "notify handle=0x%04x len=%u data=%s",
             attr_handle, (unsigned)len, hex);
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    // NimBLE's own INFO-level chatter ("GATT procedure initiated: ...") is
    // noisy and duplicates what our siri_ble logs say more concisely.
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(nimble_port_init());

    // NVS-backed bond key store — must be called before any pairing occurs.
    ble_store_config_init();

    const siri_ble_config_t cfg = {.on_notify = on_notify};
    ESP_ERROR_CHECK(siri_ble_start(&cfg));

    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "booted");
}
