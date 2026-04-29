"""siri_remote text_sensor platform — charging state string."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_TYPE

from . import CONF_SIRI_REMOTE_ID, SiriRemoteHub

CONF_CHARGING = "charging"

TYPES = [CONF_CHARGING]

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(text_sensor.TextSensor)
    .extend({
        cv.GenerateID(CONF_SIRI_REMOTE_ID): cv.use_id(SiriRemoteHub),
        cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
    })
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    hub = await cg.get_variable(config[CONF_SIRI_REMOTE_ID])
    if config[CONF_TYPE] == CONF_CHARGING:
        cg.add(hub.set_charging_text_sensor(var))
