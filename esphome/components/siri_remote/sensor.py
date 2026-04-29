"""siri_remote sensor platform — battery percentage."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_TYPE,
    DEVICE_CLASS_BATTERY,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
)

from . import CONF_SIRI_REMOTE_ID, SiriRemoteHub

CONF_BATTERY = "battery"

TYPES = [CONF_BATTERY]

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        sensor.Sensor,
        device_class=DEVICE_CLASS_BATTERY,
        unit_of_measurement=UNIT_PERCENT,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=0,
    )
    .extend({
        cv.GenerateID(CONF_SIRI_REMOTE_ID): cv.use_id(SiriRemoteHub),
        cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
    })
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    hub = await cg.get_variable(config[CONF_SIRI_REMOTE_ID])
    if config[CONF_TYPE] == CONF_BATTERY:
        cg.add(hub.set_battery_sensor(var))
