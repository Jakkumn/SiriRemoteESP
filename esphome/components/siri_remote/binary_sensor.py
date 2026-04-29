"""siri_remote binary_sensor platform — exposes runtime state derived
from BLE notifies as ESPHome BinarySensor entities."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_TYPE

from . import CONF_SIRI_REMOTE_ID, SiriRemoteHub

CONF_VOICE_ACTIVE = "voice_active"

TYPES = [CONF_VOICE_ACTIVE]

CONFIG_SCHEMA = (
    binary_sensor.binary_sensor_schema(binary_sensor.BinarySensor)
    .extend({
        cv.GenerateID(CONF_SIRI_REMOTE_ID): cv.use_id(SiriRemoteHub),
        cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
    })
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    hub = await cg.get_variable(config[CONF_SIRI_REMOTE_ID])
    if config[CONF_TYPE] == CONF_VOICE_ACTIVE:
        cg.add(hub.set_voice_active_sensor(var))
