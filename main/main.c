#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"

static const char *TAG = "siri-bridge";

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);

static void fmt_addr(char *out, size_t out_len, const uint8_t val[6])
{
    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
             val[5], val[4], val[3], val[2], val[1], val[0]);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        char addr_str[18];
        fmt_addr(addr_str, sizeof(addr_str), event->disc.addr.val);

        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                         event->disc.length_data);
        char name[32] = "";
        if (rc == 0 && fields.name != NULL && fields.name_len > 0) {
            size_t n = fields.name_len < sizeof(name) - 1
                           ? fields.name_len
                           : sizeof(name) - 1;
            memcpy(name, fields.name, n);
            name[n] = '\0';
        }

        ESP_LOGI(TAG, "adv %s rssi=%d name=\"%s\" len=%u",
                 addr_str, event->disc.rssi, name,
                 (unsigned)event->disc.length_data);
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "scan complete, restarting");
        start_scan();
        return 0;

    default:
        ESP_LOGD(TAG, "gap event %d", event->type);
        return 0;
    }
}

static void start_scan(void)
{
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    struct ble_gap_disc_params params = {
        .itvl = 0,
        .window = 0,
        .filter_policy = 0,
        .limited = 0,
        .passive = 1,
        .filter_duplicates = 0,
    };

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "scanning");
    }
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr failed: %d", rc);
        return;
    }
    start_scan();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset, reason=%d", reason);
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "booted");
}
