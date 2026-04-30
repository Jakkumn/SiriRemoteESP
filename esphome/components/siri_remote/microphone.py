"""siri_remote microphone platform — exposes the gen-3 Opus-decoded PCM
stream as an ESPHome Microphone, so voice_assistant can route it into HA
Assist."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import audio, microphone
from esphome.const import CONF_ID

from . import CONF_SIRI_REMOTE_ID, SiriRemoteHub, siri_remote_ns

SiriRemoteMicrophone = siri_remote_ns.class_(
    "SiriRemoteMicrophone", microphone.Microphone, cg.Component
)

# gen-3 Opus decode produces exactly 16 kHz / 16-bit / mono — pin every
# limit so the audio framework's MicrophoneSource validator sees a fully
# specified stream and voice_assistant matches without conversion.
def _set_stream_limits(config):
    # `audio.set_stream_limits(...)` returns a setter that mutates config in
    # place but doesn't return it, so the cv.All chain breaks on a None
    # return. Match the i2s_audio pattern: invoke + explicitly return.
    audio.set_stream_limits(
        min_bits_per_sample=16, max_bits_per_sample=16,
        min_channels=1, max_channels=1,
        min_sample_rate=16000, max_sample_rate=16000,
    )(config)
    return config


CONFIG_SCHEMA = cv.All(
    microphone.MICROPHONE_SCHEMA.extend({
        cv.GenerateID(CONF_ID): cv.declare_id(SiriRemoteMicrophone),
        cv.GenerateID(CONF_SIRI_REMOTE_ID): cv.use_id(SiriRemoteHub),
    }).extend(cv.COMPONENT_SCHEMA),
    _set_stream_limits,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await microphone.register_microphone(var, config)

    hub = await cg.get_variable(config[CONF_SIRI_REMOTE_ID])
    cg.add(hub.set_microphone(var))
