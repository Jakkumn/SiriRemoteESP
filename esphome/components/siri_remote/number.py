"""siri_remote number platform — six runtime-tunable thresholds backed by
ESPHome Preferences (NVS). User config is just `type:` + `name:`; min/max/
step/initial defaults are filled in per-type by codegen, but any of them
may be overridden in YAML."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_INITIAL_VALUE,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_RESTORE_VALUE,
    CONF_STEP,
    CONF_TYPE,
)

from . import CONF_SIRI_REMOTE_ID, SiriRemoteHub, siri_remote_ns

CONF_SWIPE_Y_PRIORITY = "swipe_y_priority"
CONF_SWIPE_MIN_DISTANCE = "swipe_min_distance"
CONF_DOUBLE_CLICK_WINDOW_MS = "double_click_window_ms"
CONF_HOLD_THRESHOLD_MS = "hold_threshold_ms"
CONF_BATTERY_LOW_PCT = "battery_low_pct"
CONF_BLE_SLAVE_LATENCY = "ble_slave_latency"

SiriRemoteNumber = siri_remote_ns.class_(
    "SiriRemoteNumber", number.Number, cg.Component
)
SiriRemoteKnob = siri_remote_ns.enum("SiriRemoteKnob", is_class=True)

# (min, max, step, initial, kind enum, hub-setter method name)
TYPES = {
    CONF_SWIPE_Y_PRIORITY: (
        0, 200, 5, 30,
        SiriRemoteKnob.SwipeYPriority, "set_swipe_y_pri_number",
    ),
    CONF_SWIPE_MIN_DISTANCE: (
        0, 500, 10, 40,
        SiriRemoteKnob.SwipeMinDistance, "set_swipe_dist_number",
    ),
    CONF_DOUBLE_CLICK_WINDOW_MS: (
        0, 2000, 50, 300,
        SiriRemoteKnob.DoubleWindowMs, "set_dbl_ms_number",
    ),
    CONF_HOLD_THRESHOLD_MS: (
        0, 10000, 100, 700,
        SiriRemoteKnob.HoldThresholdMs, "set_hold_ms_number",
    ),
    CONF_BATTERY_LOW_PCT: (
        0, 100, 5, 20,
        SiriRemoteKnob.BatteryLowPct, "set_bat_low_number",
    ),
    CONF_BLE_SLAVE_LATENCY: (
        0, 500, 20, 400,
        SiriRemoteKnob.BleSlaveLatency, "set_ble_lat_number",
    ),
}

CONFIG_SCHEMA = (
    number.number_schema(SiriRemoteNumber)
    .extend({
        cv.GenerateID(CONF_SIRI_REMOTE_ID): cv.use_id(SiriRemoteHub),
        cv.Required(CONF_TYPE): cv.one_of(*TYPES.keys(), lower=True),
        cv.Optional(CONF_MIN_VALUE): cv.float_,
        cv.Optional(CONF_MAX_VALUE): cv.float_,
        cv.Optional(CONF_STEP): cv.positive_float,
        cv.Optional(CONF_INITIAL_VALUE): cv.float_,
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
    })
)


async def to_code(config):
    type_ = config[CONF_TYPE]
    min_v, max_v, step_v, init_v, kind, setter_name = TYPES[type_]

    min_value = config.get(CONF_MIN_VALUE, min_v)
    max_value = config.get(CONF_MAX_VALUE, max_v)
    step = config.get(CONF_STEP, step_v)
    initial = config.get(CONF_INITIAL_VALUE, init_v)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await number.register_number(
        var, config, min_value=min_value, max_value=max_value, step=step,
    )

    cg.add(var.set_kind(kind))
    cg.add(var.set_initial_value(float(initial)))
    cg.add(var.set_restore_value(config[CONF_RESTORE_VALUE]))

    hub = await cg.get_variable(config[CONF_SIRI_REMOTE_ID])
    cg.add(var.set_hub(hub))
    cg.add(getattr(hub, setter_name)(var))
