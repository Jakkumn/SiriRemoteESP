#include "mqtt_entity.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "mqtt_entity";

#define TOPIC_PREFIX "siri_remote"
#define DISCOVERY_PREFIX "homeassistant"
#define NVS_NAMESPACE "mqtt_entity"

struct mqtt_entity {
    mqtt_entity_config_t cfg;
    int32_t value;
    char cmd_topic[64];
    char state_topic[64];
    char config_topic[80];
    struct mqtt_entity *next;
};

static struct mqtt_entity *s_entities;
static esp_mqtt_client_handle_t s_client;

static const char *kind_str(mqtt_entity_kind_t k)
{
    switch (k) {
    case MQTT_ENTITY_SWITCH:
        return "switch";
    case MQTT_ENTITY_NUMBER:
        return "number";
    case MQTT_ENTITY_BUTTON:
        return "button";
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
        // Clamp to current bounds: NVS may carry a value persisted under a
        // wider min/max range than this build ships, and parse_command's
        // clamp only fires on inbound commands.
        if (e->cfg.kind == MQTT_ENTITY_NUMBER) {
            if (v < e->cfg.min_value)
                v = e->cfg.min_value;
            if (v > e->cfg.max_value)
                v = e->cfg.max_value;
        }
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
    int n;
    if (e->cfg.kind == MQTT_ENTITY_SWITCH) {
        n = snprintf(buf, sizeof(buf), "%s", e->value ? "ON" : "OFF");
    } else {
        n = snprintf(buf, sizeof(buf), "%" PRId32, e->value);
    }
    esp_mqtt_client_publish(s_client, e->state_topic, buf, n, 1, /*retain*/ 1);
}

// Bounded JSON builder. The previous `n += snprintf(json + n, sizeof(json) -
// n, ...)` chain only checked for truncation at the end: once one append was
// cut short, n passed the buffer size, `sizeof(json) - n` wrapped to a huge
// size_t, and the next append wrote past the stack buffer. Today's only entity
// (the Repair button, ~330 B) never got there; a Number with a unit and longer
// names would have.
typedef struct {
    char *buf;
    size_t cap;
    size_t len;
    bool overflow;
} json_buf_t;

__attribute__((format(printf, 2, 3))) static void json_append(json_buf_t *j, const char *fmt, ...)
{
    if (j->overflow) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(j->buf + j->len, j->cap - j->len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= j->cap - j->len) {
        j->overflow = true;
        return;
    }
    j->len += (size_t)n;
}

static void publish_discovery(const mqtt_entity_t *e)
{
    if (s_client == NULL) {
        return;
    }
    char json[512];
    json_buf_t j = {.buf = json, .cap = sizeof(json)};

    json_append(&j,
                "{\"name\":\"%s\","
                "\"unique_id\":\"%s\","
                "\"command_topic\":\"%s\"",
                e->cfg.display_name, e->cfg.unique_id, e->cmd_topic);

    if (e->cfg.kind != MQTT_ENTITY_BUTTON) {
        json_append(&j, ",\"state_topic\":\"%s\"", e->state_topic);
    }
    if (e->cfg.kind == MQTT_ENTITY_NUMBER) {
        json_append(
            &j, ",\"min\":%" PRId32 ",\"max\":%" PRId32 ",\"step\":%" PRId32 ",\"mode\":\"slider\"",
            e->cfg.min_value, e->cfg.max_value, e->cfg.step_value);
        if (e->cfg.unit_of_measurement != NULL) {
            json_append(&j, ",\"unit_of_measurement\":\"%s\"", e->cfg.unit_of_measurement);
        }
    }
    if (e->cfg.icon != NULL) {
        json_append(&j, ",\"icon\":\"%s\"", e->cfg.icon);
    }
    if (e->cfg.device_id != NULL) {
        json_append(&j, ",\"device\":{\"identifiers\":[\"%s\"]}", e->cfg.device_id);
    }
    if (e->cfg.availability_topic != NULL) {
        json_append(&j,
                    ",\"availability_topic\":\"%s\","
                    "\"payload_available\":\"online\","
                    "\"payload_not_available\":\"offline\"",
                    e->cfg.availability_topic);
    }
    json_append(&j, ",\"entity_category\":\"config\"}");

    if (j.overflow) {
        ESP_LOGE(TAG, "%s: discovery JSON exceeds %u bytes; not published", e->cfg.unique_id,
                 (unsigned)sizeof(json));
        return;
    }
    esp_mqtt_client_publish(s_client, e->config_topic, json, (int)j.len, 1, /*retain*/ 1);
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
        long v = strtol(buf, &end, 10);
        if (end == buf) {
            return -1;
        }
        if (v < e->cfg.min_value)
            v = e->cfg.min_value;
        if (v > e->cfg.max_value)
            v = e->cfg.max_value;
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
    e->cfg = *cfg;
    e->value = cfg->default_value;

    // A truncated topic would make HA publish to one topic while we subscribe
    // to another — silently dead. Refuse the entity instead.
    int n1 =
        snprintf(e->cmd_topic, sizeof(e->cmd_topic), "%s/cmd/%s", TOPIC_PREFIX, cfg->unique_id);
    int n2 = 0;
    if (cfg->kind != MQTT_ENTITY_BUTTON) {
        n2 = snprintf(e->state_topic, sizeof(e->state_topic), "%s/state/%s", TOPIC_PREFIX,
                      cfg->unique_id);
    }
    int n3 = snprintf(e->config_topic, sizeof(e->config_topic), "%s/%s/%s/config", DISCOVERY_PREFIX,
                      kind_str(cfg->kind), cfg->unique_id);
    if (n1 < 0 || n1 >= (int)sizeof(e->cmd_topic) || n2 < 0 || n2 >= (int)sizeof(e->state_topic) ||
        n3 < 0 || n3 >= (int)sizeof(e->config_topic)) {
        ESP_LOGE(TAG, "%s: unique_id too long for its MQTT topics", cfg->unique_id);
        free(e);
        return NULL;
    }

    load_value(e);

    e->next = s_entities;
    s_entities = e;
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

bool mqtt_entity_dispatch_data(const char *topic, int topic_len, const char *data, int data_len,
                               bool retained)
{
    for (mqtt_entity_t *e = s_entities; e != NULL; e = e->next) {
        if (topic_len != (int)strlen(e->cmd_topic))
            continue;
        if (memcmp(topic, e->cmd_topic, topic_len) != 0)
            continue;

        if (retained && e->cfg.kind == MQTT_ENTITY_BUTTON) {
            // A button press is an event, never state. A retained message on
            // the command topic (published by hand, or by an automation with
            // retain: true) is redelivered on every subscribe — i.e. every
            // MQTT reconnect — and for the Repair button that would wipe the
            // bond on each Wi-Fi blip. HA's own button never retains.
            ESP_LOGW(TAG,
                     "%s: ignoring retained command (clear it with an empty retained "
                     "publish to %s)",
                     e->cfg.unique_id, e->cmd_topic);
            return true;
        }

        int32_t v;
        if (parse_command(e, data, data_len, &v) != 0) {
            ESP_LOGW(TAG, "%s: failed to parse command (%d bytes)", e->cfg.unique_id, data_len);
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
