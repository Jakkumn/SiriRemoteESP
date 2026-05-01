#pragma once

// Lightweight helper for runtime HA-MQTT entities (Switch, Number, Button).
// Each entity registers once at boot. The helper does:
//   - persists Switch/Number values to NVS, restores on boot
//   - publishes HA auto-discovery JSON on every MQTT (re)connect
//   - subscribes to the command topic and dispatches inbound commands to a
//     user callback, then echoes the new state back to HA
//
// Standalone-only — Phase 4's ESPHome wrapper replaces this layer entirely.
// Topic conventions (relative to the bridge's MQTT prefix):
//   homeassistant/<kind>/<unique_id>/config   discovery, retained
//   siri_remote/cmd/<unique_id>               HA -> ESP commands
//   siri_remote/state/<unique_id>             ESP -> HA state echo, retained
//                                              (Switch + Number only)

#include <stdbool.h>
#include <stdint.h>

#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MQTT_ENTITY_SWITCH,
    MQTT_ENTITY_NUMBER,
    MQTT_ENTITY_BUTTON,
} mqtt_entity_kind_t;

// Fired whenever HA wants something to change. For Switch/Number, `value`
// is the new state (0/1 for Switch, the new integer for Number) and the
// helper has *already* updated the entity's stored value and persisted
// to NVS by the time this fires. For Button, `value` is always 0 — the
// callback is just a "press happened" signal.
typedef void (*mqtt_entity_on_change_fn)(int32_t value, void *user);

typedef struct {
    mqtt_entity_kind_t kind;

    // Required for all kinds.
    const char *unique_id;     // e.g. "siri_remote_raw_stream"
    const char *display_name;  // e.g. "Raw Touch Stream"

    // Optional but recommended.
    const char *icon;                // mdi:... (omitted from discovery if NULL)
    const char *device_id;           // HA device identifier (NULL = no device grouping)
    const char *availability_topic;  // bridge LWT topic (NULL = no availability gating)

    // Number-only.
    int32_t min_value;
    int32_t max_value;
    int32_t step_value;
    const char *unit_of_measurement;  // e.g. "ms", "%" (NULL = unitless)

    // Switch + Number persistence + initial value.
    const char *nvs_key;    // NULL = don't persist (Button always)
    int32_t default_value;  // used if NVS is empty or read fails

    // Inbound command callback.
    mqtt_entity_on_change_fn on_change;
    void *user;
} mqtt_entity_config_t;

typedef struct mqtt_entity mqtt_entity_t;

// Allocate + register an entity. Strings in cfg must outlive the entity
// (string literals are fine). Returns NULL on allocation failure.
//
// Switch/Number values are loaded from NVS here; the on_change callback
// is *not* fired on boot (the caller already knows the initial value via
// mqtt_entity_get_value()).
mqtt_entity_t *mqtt_entity_create(const mqtt_entity_config_t *cfg);

// Current value (Switch: 0/1, Number: integer, Button: always 0).
int32_t mqtt_entity_get_value(const mqtt_entity_t *e);

// Update value from firmware side (e.g. user pressed the physical button
// behind a Switch). Persists to NVS and publishes the new state. No-op
// for Button entities.
void mqtt_entity_set_value(mqtt_entity_t *e, int32_t value);

// MQTT lifecycle hooks. Call these from main.c's mqtt_event_handler.
// `on_mqtt_connected` (re)publishes discovery configs, restores state to
// the broker, and subscribes to all command topics. `dispatch_data`
// returns true if an entity claimed the topic.
void mqtt_entity_on_mqtt_connected(esp_mqtt_client_handle_t client);
bool mqtt_entity_dispatch_data(const char *topic, int topic_len, const char *data, int data_len);

#ifdef __cplusplus
}
#endif
