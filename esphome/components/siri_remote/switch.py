"""siri_remote switch platform — raw_stream toggle gates touch-frame UART logging."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_TYPE

from . import CONF_SIRI_REMOTE_ID, SiriRemoteHub, siri_remote_ns

CONF_RAW_STREAM = "raw_stream"

TYPES = [CONF_RAW_STREAM]

SiriRemoteRawStreamSwitch = siri_remote_ns.class_(
    "SiriRemoteRawStreamSwitch", switch.Switch
)

CONFIG_SCHEMA = (
    switch.switch_schema(
        SiriRemoteRawStreamSwitch,
        default_restore_mode="RESTORE_DEFAULT_OFF",
        icon="mdi:gesture-tap-hold",
    )
    .extend({
        cv.GenerateID(CONF_SIRI_REMOTE_ID): cv.use_id(SiriRemoteHub),
        cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
    })
)


async def to_code(config):
    var = await switch.new_switch(config)
    hub = await cg.get_variable(config[CONF_SIRI_REMOTE_ID])
    cg.add(var.set_hub(hub))
