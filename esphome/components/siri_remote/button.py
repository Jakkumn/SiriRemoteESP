"""siri_remote button platform — re-pair the remote (destructive)."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_TYPE

from . import CONF_SIRI_REMOTE_ID, SiriRemoteHub, siri_remote_ns

CONF_REPAIR = "repair"

TYPES = [CONF_REPAIR]

SiriRemoteRepairButton = siri_remote_ns.class_(
    "SiriRemoteRepairButton", button.Button
)

CONFIG_SCHEMA = (
    button.button_schema(
        SiriRemoteRepairButton,
        icon="mdi:bluetooth-refresh",
    )
    .extend({
        cv.GenerateID(CONF_SIRI_REMOTE_ID): cv.use_id(SiriRemoteHub),
        cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
    })
)


async def to_code(config):
    await button.new_button(config)
    # Hub doesn't track the button — press_action() calls siri_ble_repair()
    # directly. We resolve the hub id only to enforce the cv.use_id binding.
    await cg.get_variable(config[CONF_SIRI_REMOTE_ID])
