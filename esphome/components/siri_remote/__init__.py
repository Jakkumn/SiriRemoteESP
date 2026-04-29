"""siri_remote — ESPHome external component bridging an Apple Siri Remote
(gen 3) to Home Assistant.

Shares the C components (siri_ble, siri_audio, report_decoder, event_state)
with the standalone idf.py build at <repo>/components/ via in-tree
symlinks; both build paths consume the same source files."""

import os

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID, CONF_PORT
from esphome.core import CORE

CODEOWNERS = ["@cmehl"]
DEPENDENCIES = ["esp32"]

siri_remote_ns = cg.esphome_ns.namespace("siri_remote")
SiriRemoteHub = siri_remote_ns.class_("SiriRemoteHub", cg.Component)

CONF_SIRI_REMOTE_ID = "siri_remote_id"
CONF_VOICE_ENABLED = "voice_enabled"
CONF_IDLE_DISCONNECT_MS = "idle_disconnect_ms"
CONF_PAIRING_FLUSH_SUPPRESS_MS = "pairing_flush_suppress_ms"
CONF_DEBUG_TOUCH_FRAMES = "debug_touch_frames"
CONF_DEBUG_WAKE_PROBE = "debug_wake_probe"
CONF_DEBUG_PCM_TCP = "debug_pcm_tcp"
CONF_HOST = "host"

DEBUG_PCM_TCP_SCHEMA = cv.Schema({
    cv.Required(CONF_HOST): cv.ipv4address,
    cv.Optional(CONF_PORT, default=8000): cv.port,
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SiriRemoteHub),
    cv.Optional(CONF_VOICE_ENABLED, default=True): cv.boolean,
    cv.Optional(CONF_IDLE_DISCONNECT_MS, default=0): cv.uint32_t,
    cv.Optional(CONF_PAIRING_FLUSH_SUPPRESS_MS, default=1500): cv.uint32_t,
    cv.Optional(CONF_DEBUG_TOUCH_FRAMES, default=False): cv.boolean,
    cv.Optional(CONF_DEBUG_WAKE_PROBE, default=False): cv.boolean,
    cv.Optional(CONF_DEBUG_PCM_TCP): DEBUG_PCM_TCP_SCHEMA,
}).extend(cv.COMPONENT_SCHEMA)


def _pin_sdkconfig():
    """Pin sdkconfig to match the standalone build. Without this,
    ESPHome's defaults leave Bluedroid on, PSRAM off, and the partition
    layout at the IDF stock — none of which work for a Siri Remote bridge.
    Flash size + partition table come from the user's `esp32:` block."""
    so = esp32.add_idf_sdkconfig_option

    # NimBLE host (Bluedroid off — central role lives on NimBLE only).
    so("CONFIG_BT_ENABLED", True)
    so("CONFIG_BT_NIMBLE_ENABLED", True)
    so("CONFIG_BT_BLUEDROID_ENABLED", False)

    # Peripheral role is enabled for the passive GATT-server stub Apple's
    # accessory framework probes (it disconnects after a window of failed
    # probes if we don't answer).
    so("CONFIG_BT_NIMBLE_ROLE_CENTRAL", True)
    so("CONFIG_BT_NIMBLE_ROLE_OBSERVER", True)
    so("CONFIG_BT_NIMBLE_ROLE_PERIPHERAL", True)
    so("CONFIG_BT_NIMBLE_ROLE_BROADCASTER", False)

    so("CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU", 247)
    so("CONFIG_BT_NIMBLE_SM_SC", True)
    so("CONFIG_BT_NIMBLE_MAX_BONDS", 1)
    so("CONFIG_BT_NIMBLE_MAX_CONNECTIONS", 1)
    so("CONFIG_BT_NIMBLE_NVS_PERSIST", True)

    # PSRAM is NOT used implicitly by malloc — lwIP/MQTT TX paths are
    # timing-sensitive to heap latency and PSRAM SPI access caused MQTT
    # keepalive timeouts during Phase 3.M bring-up. siri_audio opts in
    # via heap_caps_malloc(MALLOC_CAP_SPIRAM) for the decoder only.
    so("CONFIG_SPIRAM", True)
    so("CONFIG_SPIRAM_MODE_OCT", True)
    so("CONFIG_SPIRAM_TYPE_AUTO", True)
    so("CONFIG_SPIRAM_SPEED_80M", True)

    so("CONFIG_ESP_COEX_SW_COEXIST_ENABLE", True)


async def to_code(config):
    if not (CORE.is_esp32 and not CORE.using_arduino):
        raise cv.Invalid(
            "siri_remote requires `framework: type: esp-idf` on an ESP32 "
            "(Arduino framework lacks the NimBLE central API surface we use)"
        )

    _pin_sdkconfig()

    # Opus decoder, sourced via the IDF Component Manager registry
    # (`esphome/micro-opus`). The registry archive bundles the upstream
    # xiph/opus + ogg-demuxer source — no submodule init step needed.
    # Pinned identically to the standalone `components/siri_audio/idf_component.yml`.
    esp32.add_idf_component(
        name="esphome/micro-opus",
        ref="~0.3.3",
    )

    # esp_central.h is an internal header used by siri_ble.c + peer.c.
    # We deliberately don't symlink it into the component dir — otherwise
    # ESPHome's esphome.h auto-include pulls it into main.cpp where the
    # NimBLE prereq symbols aren't visible. Inject the parent dir as an
    # include path so the C sources still find it.
    component_dir = os.path.dirname(os.path.abspath(__file__))
    siri_ble_internal = os.path.normpath(
        os.path.join(component_dir, "..", "..", "..", "components", "siri_ble")
    )
    cg.add_build_flag(f"-I{siri_ble_internal}")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_voice_enabled(config[CONF_VOICE_ENABLED]))
    cg.add(var.set_idle_disconnect_ms(config[CONF_IDLE_DISCONNECT_MS]))
    cg.add(var.set_pairing_flush_suppress_ms(
        config[CONF_PAIRING_FLUSH_SUPPRESS_MS]))
    cg.add(var.set_debug_touch_frames(config[CONF_DEBUG_TOUCH_FRAMES]))

    if CONF_DEBUG_PCM_TCP in config:
        pcm = config[CONF_DEBUG_PCM_TCP]
        cg.add(var.set_debug_pcm_tcp(str(pcm[CONF_HOST]), pcm[CONF_PORT]))
        cg.add_build_flag("-DSIRI_REMOTE_DEBUG_PCM_TCP")

    # The shared siri_ble.c uses #ifdef CONFIG_VOICE_ENABLED to gate the
    # audio CCCD subscribe and #ifdef CONFIG_DEBUG_WAKE_PROBE for probe
    # CCCDs. Keep the macro names so the C file is identical across both
    # build paths.
    if config[CONF_VOICE_ENABLED]:
        cg.add_build_flag("-DCONFIG_VOICE_ENABLED=1")
    if config[CONF_DEBUG_WAKE_PROBE]:
        cg.add_build_flag("-DCONFIG_DEBUG_WAKE_PROBE=1")

    # Enable the HA service-call / event-fire path. ESPHome 2026.4 made
    # this opt-in via api: homeassistant_services: true; emit_event needs
    # it unconditionally, so add the define here regardless of the api:
    # block's value.
    cg.add_define("USE_API_HOMEASSISTANT_SERVICES")
