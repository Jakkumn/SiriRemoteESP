"""siri_remote — ESPHome external component bridging an Apple Siri Remote
(gen 3) to Home Assistant.

Shares the C components (siri_ble, siri_audio, report_decoder, button_pulse)
with the standalone idf.py build at <repo>/components/ via in-tree
symlinks; both build paths consume the same source files."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32, voice_assistant
from esphome.const import CONF_ID, CONF_PORT
from esphome.core import CORE

CODEOWNERS = ["@cmehl"]
DEPENDENCIES = ["esp32"]

# siri_remote_hub.cpp always includes siri_remote_microphone.h (the hub's
# on_pcm fan-out calls into the mic component). AUTO_LOAD forces the
# microphone module onto the include path even when the user doesn't add
# a `microphone:` block. Negligible build cost; no runtime cost.
AUTO_LOAD = ["microphone"]

siri_remote_ns = cg.esphome_ns.namespace("siri_remote")
SiriRemoteHub = siri_remote_ns.class_("SiriRemoteHub", cg.Component)

CONF_SIRI_REMOTE_ID = "siri_remote_id"
CONF_VOICE_ENABLED = "voice_enabled"
CONF_IDLE_DISCONNECT_MS = "idle_disconnect_ms"
CONF_PAIRING_FLUSH_SUPPRESS_MS = "pairing_flush_suppress_ms"
CONF_DEBUG_TOUCH_FRAMES = "debug_touch_frames"
CONF_REPEAT_INTERVAL_MS = "repeat_interval_ms"
CONF_BLE_SLAVE_LATENCY = "ble_slave_latency"
CONF_DEBUG_WAKE_PROBE = "debug_wake_probe"
CONF_DEBUG_PCM_TCP = "debug_pcm_tcp"
CONF_HOST = "host"
CONF_VOICE_ASSISTANT_ID = "voice_assistant_id"
CONF_AUTO_FINISH_RESPONSE = "auto_finish_response"

DEBUG_PCM_TCP_SCHEMA = cv.Schema({
    cv.Required(CONF_HOST): cv.ipv4address,
    cv.Optional(CONF_PORT, default=8000): cv.port,
})

def _validate(config):
    # auto_finish_response only does anything when voice_assistant_id is
    # also set (the codegen below is gated on it). Catch the
    # explicitly-set-without-VA case so the user gets a clear error
    # rather than silent no-op.
    if (CONF_AUTO_FINISH_RESPONSE in config
            and CONF_VOICE_ASSISTANT_ID not in config):
        raise cv.Invalid(
            f"{CONF_AUTO_FINISH_RESPONSE} requires {CONF_VOICE_ASSISTANT_ID} "
            "to also be set"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(SiriRemoteHub),
        cv.Optional(CONF_VOICE_ENABLED, default=True): cv.boolean,
        cv.Optional(CONF_IDLE_DISCONNECT_MS, default=0): cv.uint32_t,
        cv.Optional(CONF_PAIRING_FLUSH_SUPPRESS_MS, default=1500): cv.uint32_t,
        # How often a held button re-emits. A rate, not a policy — HA decides
        # what a hold means from the `held_ms` each pulse carries. Floored at
        # the 50 ms tick period, since a shorter interval cannot be honoured.
        # 0 disables repeats entirely (press-only).
        cv.Optional(CONF_REPEAT_INTERVAL_MS, default=100): cv.Any(
            cv.int_range(min=0, max=0), cv.int_range(min=50, max=60000)
        ),
        # Was a runtime Number entity; now compile-time. Load-bearing for
        # always-connected wake behaviour — see README before changing.
        # 498, not 500: NimBLE rejects latency > 499, and 499 fails its
        # supervision-timeout check against siri_ble's 15 s ceiling — the
        # conn-param update would return EINVAL and leave latency at 0.
        cv.Optional(CONF_BLE_SLAVE_LATENCY, default=400): cv.int_range(
            min=0, max=498
        ),
        cv.Optional(CONF_DEBUG_TOUCH_FRAMES, default=False): cv.boolean,
        cv.Optional(CONF_DEBUG_WAKE_PROBE, default=False): cv.boolean,
        cv.Optional(CONF_DEBUG_PCM_TCP): DEBUG_PCM_TCP_SCHEMA,
        cv.Optional(CONF_VOICE_ASSISTANT_ID): cv.use_id(voice_assistant.VoiceAssistant),
        # No default — the codegen treats absence as "user did not opt in".
        # See _validate above for the auto_finish_response/voice_assistant_id
        # interlock.
        cv.Optional(CONF_AUTO_FINISH_RESPONSE): cv.boolean,
    }).extend(cv.COMPONENT_SCHEMA),
    _validate,
)


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
    # NOT 1, despite the bridge only ever bonding one remote. NimBLE sizes
    # ble_store_config_rpa_recs[] by MAX_BONDS, so 1 leaves a single
    # RPA-record slot. Every Apple device advertises with a rotating private
    # address, so in an RF-dense room the first candidate we connect to
    # consumes that slot; from then on every subsequent candidate's RPA write
    # fails with BLE_HS_ESTORE_CAP (27) and NimBLE terminates the link ~17 ms
    # after encryption — before peer_disc_all runs, so fingerprint_check never
    # gets to identify the real remote. The slot is only freed in the
    # fingerprint-reject path, which is unreachable in that state: a deadlock
    # that presents as an endless connect/disconnect loop with no explanation.
    # Headroom for discovery churn; siri_ble still tracks exactly one
    # s_bonded_peer. Keep in sync with sdkconfig.defaults.
    so("CONFIG_BT_NIMBLE_MAX_BONDS", 8)
    so("CONFIG_BT_NIMBLE_MAX_CONNECTIONS", 1)
    so("CONFIG_BT_NIMBLE_NVS_PERSIST", True)

    # Unlike standalone, PSRAM *is* in the malloc pool here. SPIRAM_USE_MALLOC
    # is the IDF default once SPIRAM is on, and ESPHome builds have always run
    # with it; it is pinned so that stays true rather than depending on the
    # default. Standalone turns it off because PSRAM-backed lwIP/MQTT buffers
    # caused MQTT keepalive timeouts during Phase 3.M bring-up. That hazard
    # doesn't carry over: TRY_ALLOCATE_WIFI_LWIP stays off, so the network
    # stack is internal, and allocations under SPIRAM_MALLOC_ALWAYSINTERNAL
    # (16 KiB) never leave internal RAM. Switching it off here would push
    # every large ESPHome/voice_assistant buffer into internal SRAM, which
    # this firmware has not been tested with. siri_audio requests PSRAM
    # explicitly (MALLOC_CAP_SPIRAM) on both paths either way.
    so("CONFIG_SPIRAM", True)
    so("CONFIG_SPIRAM_MODE_OCT", True)
    so("CONFIG_SPIRAM_TYPE_AUTO", True)
    so("CONFIG_SPIRAM_SPEED_80M", True)
    so("CONFIG_SPIRAM_USE_MALLOC", True)

    so("CONFIG_ESP_COEX_SW_COEXIST_ENABLE", True)

    # Raise ESP-IDF's log ceiling so our pure-C components stay diagnosable.
    #
    # esphome/core/log.h does `#undef ESP_LOGI` and redirects it to ESPHome's
    # own logger — but only for translation units that include it, i.e. the C++
    # glue. siri_ble.c, siri_audio.c and mqtt_entity.c are pure C and get the
    # real ESP-IDF macros, which are compiled out entirely when
    # CONFIG_LOG_MAXIMUM_LEVEL is below the call's level.
    #
    # ESPHome defaults that ceiling to ERROR(1), which silently deletes every
    # ESP_LOGI in siri_ble.c — including the whole pairing/discovery
    # narrative ("disconnected, reason=0x…", "candidate failed pre-fingerprint
    # — hold Back+VolUp…").
    #
    # INFO(3), not DEBUG(4) as standalone uses: under ESPHome this ceiling is
    # the *only* filter on ESP-IDF output. ESPHome builds with
    # CONFIG_LOG_DYNAMIC_LEVEL_CONTROL off and TAG_LEVEL_IMPL_NONE, where
    # esp_log_is_tag_loggable() returns true unconditionally — so
    # CONFIG_LOG_DEFAULT_LEVEL and esp_log_level_set() are no-ops, and every
    # IDF component logs at whatever the ceiling compiles in. The closed-source
    # Wi-Fi driver also derives its own level from this ceiling
    # (esp_wifi_set_log_level in wifi_init.c). At DEBUG it emits several lines
    # per beacon-timing adjustment ("bcnd: 2720 -> 2704", "bssid equal"),
    # continuously. siri_ble's diagnostics are INFO; the one ESP_LOGD there
    # (sub-threshold RSSI rejects) is not worth that.
    #
    # Both choice symbols and their ints must be written, and the ERROR choice
    # cleared, or kconfiglib keeps the old selection. DEFAULT is set to match
    # for the day dynamic control is enabled; today it has no effect.
    so("CONFIG_LOG_MAXIMUM_LEVEL_ERROR", False)
    so("CONFIG_LOG_MAXIMUM_LEVEL_INFO", True)
    so("CONFIG_LOG_MAXIMUM_LEVEL", 3)
    so("CONFIG_LOG_DEFAULT_LEVEL_ERROR", False)
    so("CONFIG_LOG_DEFAULT_LEVEL_INFO", True)
    so("CONFIG_LOG_DEFAULT_LEVEL", 3)


async def to_code(config):
    if not (CORE.is_esp32 and not CORE.using_arduino):
        raise cv.Invalid(
            "siri_remote requires `framework: type: esp-idf` on an ESP32 "
            "(Arduino framework lacks the NimBLE central API surface we use)"
        )

    _pin_sdkconfig()

    # ESPHome 2026.9 started excluding built-in ESP-IDF components by default to
    # cut build time, and both of these are on that list:
    #
    #   "bt",        # re-included by request_bluetooth()
    #   "esp_coex",  # re-included by esp32_ble_tracker, zigbee
    #
    # This component drives NimBLE directly rather than going through ESPHome's
    # esp32_ble, so nothing else in the build asks for them and ESP-IDF drops
    # the component entirely. The failure is a confusing one — every siri_ble.c
    # include of host/ble_hs.h fails with "Maybe one of the components needs to
    # add the missing header directory", pointing at INCLUDE_DIRS rather than at
    # a missing component.
    #
    # Setting CONFIG_BT_ENABLED in _pin_sdkconfig() is NOT enough: sdkconfig
    # options don't un-exclude a component. Both helpers exist back to 2026.7.x,
    # where the exclusion lists didn't yet contain these, so calling them is
    # correct on old and new ESPHome alike.
    esp32.request_bluetooth()
    esp32.request_software_coexistence()

    # Opus decoder, sourced via the IDF Component Manager registry
    # (`esphome/micro-opus`). The registry archive bundles the upstream
    # xiph/opus + ogg-demuxer source — no submodule init step needed.
    # Pinned identically to the standalone `components/siri_audio/idf_component.yml`.
    esp32.add_idf_component(
        name="esphome/micro-opus",
        ref="~0.3.3",
    )

    # esp_central.h is an internal header used by siri_ble.c + peer.c. It is
    # symlinked into this component dir so it resolves from the component's own
    # (always-present) include path rather than a -I build flag pointing outside
    # the copied `src` tree — that flag stopped reaching the ESP-IDF component
    # compile on the 2026.07 toolchain. The header guards its body behind
    # `#ifndef __cplusplus`, so ESPHome's esphome.h auto-include into main.cpp
    # (C++) expands to nothing and the NimBLE-only types stay out of scope there.

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_voice_enabled(config[CONF_VOICE_ENABLED]))
    cg.add(var.set_idle_disconnect_ms(config[CONF_IDLE_DISCONNECT_MS]))
    cg.add(var.set_pairing_flush_suppress_ms(
        config[CONF_PAIRING_FLUSH_SUPPRESS_MS]))
    cg.add(var.set_debug_touch_frames(config[CONF_DEBUG_TOUCH_FRAMES]))
    cg.add(var.set_repeat_interval_ms(config[CONF_REPEAT_INTERVAL_MS]))
    cg.add(var.set_ble_slave_latency(config[CONF_BLE_SLAVE_LATENCY]))

    if CONF_DEBUG_PCM_TCP in config:
        pcm = config[CONF_DEBUG_PCM_TCP]
        cg.add(var.set_debug_pcm_tcp(str(pcm[CONF_HOST]), pcm[CONF_PORT]))
        cg.add_build_flag("-DSIRI_REMOTE_DEBUG_PCM_TCP")

    if CONF_VOICE_ASSISTANT_ID in config:
        va = await cg.get_variable(config[CONF_VOICE_ASSISTANT_ID])
        cg.add(var.set_voice_assistant(va))
        # auto_finish_response defaults to True when voice_assistant_id is
        # set but auto_finish_response is omitted — matches the prior
        # behavior. The schema validator forbids the inverse.
        cg.add(var.set_auto_finish_response(
            config.get(CONF_AUTO_FINISH_RESPONSE, True)))

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
