#include "mqtt_entity.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "mqtt_entity";

#define TOPIC_PREFIX     "siri_remote"
#define DISCOVERY_PREFIX "homeassistant"
#define NVS_NAMESPACE    "mqtt_entity"

struct mqtt_entity {
    mqtt_entity_config_t cfg;
    int32_t              value;
    char                 cmd_topic[64];
    char                 state_topic[64];
    char                 config_topic[80];
    struct mqtt_entity  *next;
};

static struct mqtt_entity      *s_entities;
static esp_mqtt_client_handle_t s_client;

static const char *kind_str(mqtt_entity_kind_t k)
{
    switch (k) {
    case MQTT_ENTITY_SWITCH: return "switch";
    case MQTT_ENTITY_NUMBER: return "number";
    case MQTT_ENTITY_BUTTON: return "button";
    }
    return "unknown";
}

static void persist_value(const mqtt_entity_t *e)
{
    if (e->cfg.nvs_key == NULL || e->cfg.kind == MQTT_ENTITY_BUTTON) {
        return;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "%s: nvs_open failed", e->cfg.unique_id);
        return;
    }
    if (nvs_set_i32(h, e->cfg.nvs_key, e->value) == ESP_OK) {
        (void)nvs_commit(h);
    }
    nvs_close(h);
}

static void load_value(mqtt_entity_t *e)
{
    if (e->cfg.nvs_key == NULL || e->cfg.kind == MQTT_ENTITY_BUTTON) {
        return;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;  // first boot — namespace doesn't exist yet
    }
    int32_t v;
    if (nvs_get_i32(h, e->cfg.nvs_key, &v) == ESP_OK) {
        e->value = v;
    }
    nvs_close(h);
}

static void publish_state(const mqtt_entity_t *e)
{
    if (s_client == NULL || e->cfg.kind == MQTT_ENTITY_BUTTON) {
        return;
    }
    char buf[16];
    int  n;
    if (e->cfg.kind == MQTT_ENTITY_SWITCH) {
        n = snprintf(buf, sizeof(buf), "%s", e->value ? "ON" : "OFF");
    } else {
        n = snprintf(buf, sizeof(buf), "%" PRId32, e->value);
    }
    esp_mqtt_client_publish(s_client, e->state_topic, buf, n, 1, /*retain*/ 1);
}

static void publish_discovery(const mqtt_entity_t *e)
{
    if (s_client == NULL) {
        return;
    }
    char json[512];
    int  n = 0;

    n += snprintf(json + n, sizeof(json) - n,
                  "{\"name\":\"%s\","
                  "\"unique_id\":\"%s\","
                  "\"command_topic\":\"%s\"",
                  e->cfg.display_name, e->cfg.unique_id, e->cmd_topic);

    if (e->cfg.kind != MQTT_ENTITY_BUTTON) {
        n += snprintf(json + n, sizeof(json) - n,
                      ",\"state_topic\":\"%s\"", e->state_topic);
    }
    if (e->cfg.kind == MQTT_ENTITY_NUMBER) {
        n += snprintf(json + n, sizeof(json) - n,
                      ",\"min\":%" PRId32 ",\"max\":%" PRId32
                      ",\"step\":%" PRId32 ",\"mode\":\"slider\"",
                      e->cfg.min_value, e->cfg.max_value, e->cfg.step_value);
        if (e->cfg.unit_of_measurement != NULL) {
            n += snprintf(json + n, sizeof(json) - n,
                          ",\"unit_of_measurement\":\"%s\"",
                          e->cfg.unit_of_measurement);
        }
    }
    if (e->cfg.icon != NULL) {
        n += snprintf(json + n, sizeof(json) - n,
                      ",\"icon\":\"%s\"", e->cfg.icon);
    }
    if (e->cfg.device_id != NULL) {
        n += snprintf(json + n, sizeof(json) - n,
                      ",\"device\":{\"identifiers\":[\"%s\"]}",
                      e->cfg.device_id);
    }
    if (e->cfg.availability_topic != NULL) {
        n += snprintf(json + n, sizeof(json) - n,
                      ",\"availability_topic\":\"%s\","
                      "\"payload_available\":\"online\","
                      "\"payload_not_available\":\"offline\"",
                      e->cfg.availability_topic);
    }
    n += snprintf(json + n, sizeof(json) - n,
                  ",\"entity_category\":\"config\"}");

    if (n >= (int)sizeof(json)) {
        ESP_LOGW(TAG, "%s: discovery JSON truncated (%d bytes)",
                 e->cfg.unique_id, n);
        return;
    }
    esp_mqtt_client_publish(s_client, e->config_topic, json, n, 1, /*retain*/ 1);
}

static int parse_command(const mqtt_entity_t *e, const char *data, int len, int32_t *out)
{
    if (e->cfg.kind == MQTT_ENTITY_SWITCH) {
        if (len == 2 && (memcmp(data, "ON", 2) == 0 || memcmp(data, "on", 2) == 0)) {
            *out = 1;
            return 0;
        }
        if (len == 3 && (memcmp(data, "OFF", 3) == 0 || memcmp(data, "off", 3) == 0)) {
            *out = 0;
            return 0;
        }
        return -1;
    }
    if (e->cfg.kind == MQTT_ENTITY_NUMBER) {
        char buf[16];
        if (len <= 0 || len >= (int)sizeof(buf)) {
            return -1;
        }
        memcpy(buf, data, len);
        buf[len] = '\0';
        char *end;
        long  v = strtol(buf, &end, 10);
        if (end == buf) {
            return -1;
        }
        if (v < e->cfg.min_value) v = e->cfg.min_value;
        if (v > e->cfg.max_value) v = e->cfg.max_value;
        *out = (int32_t)v;
        return 0;
    }
    // BUTTON: any payload is a press; the value is unused.
    *out = 0;
    return 0;
}

mqtt_entity_t *mqtt_entity_create(const mqtt_entity_config_t *cfg)
{
    if (cfg == NULL || cfg->unique_id == NULL || cfg->display_name == NULL) {
        return NULL;
    }
    mqtt_entity_t *e = calloc(1, sizeof(*e));
    if (e == NULL) {
        return NULL;
    }
    e->cfg   = *cfg;
    e->value = cfg->default_value;

    snprintf(e->cmd_topic, sizeof(e->cmd_topic),
             "%s/cmd/%s", TOPIC_PREFIX, cfg->unique_id);
    if (cfg->kind != MQTT_ENTITY_BUTTON) {
        snprintf(e->state_topic, sizeof(e->state_topic),
                 "%s/state/%s", TOPIC_PREFIX, cfg->unique_id);
    }
    snprintf(e->config_topic, sizeof(e->config_topic),
             "%s/%s/%s/config", DISCOVERY_PREFIX, kind_str(cfg->kind),
             cfg->unique_id);

    load_value(e);

    e->next     = s_entities;
    s_entities  = e;
    return e;
}

int32_t mqtt_entity_get_value(const mqtt_entity_t *e)
{
    return e != NULL ? e->value : 0;
}

void mqtt_entity_set_value(mqtt_entity_t *e, int32_t value)
{
    if (e == NULL || e->cfg.kind == MQTT_ENTITY_BUTTON) {
        return;
    }
    if (e->value == value) {
        return;
    }
    e->value = value;
    persist_value(e);
    publish_state(e);
}

void mqtt_entity_on_mqtt_connected(esp_mqtt_client_handle_t client)
{
    s_client = client;
    for (mqtt_entity_t *e = s_entities; e != NULL; e = e->next) {
        publish_discovery(e);
        publish_state(e);  // no-op for Button
        esp_mqtt_client_subscribe(client, e->cmd_topic, 1);
    }
}

bool mqtt_entity_dispatch_data(const char *topic, int topic_len,
                                const char *data, int data_len)
{
    for (mqtt_entity_t *e = s_entities; e != NULL; e = e->next) {
        if (topic_len != (int)strlen(e->cmd_topic)) continue;
        if (memcmp(topic, e->cmd_topic, topic_len) != 0) continue;

        int32_t v;
        if (parse_command(e, data, data_len, &v) != 0) {
            ESP_LOGW(TAG, "%s: failed to parse command (%d bytes)",
                     e->cfg.unique_id, data_len);
            return true;
        }
        if (e->cfg.kind != MQTT_ENTITY_BUTTON) {
            e->value = v;
            persist_value(e);
        }
        if (e->cfg.on_change != NULL) {
            e->cfg.on_change(v, e->cfg.user);
        }
        if (e->cfg.kind != MQTT_ENTITY_BUTTON) {
            publish_state(e);
        }
        return true;
    }
    return false;
}
