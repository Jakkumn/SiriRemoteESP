# siri-remote-ha-bridge

An ESP32 BLE bridge that pairs with an Apple Siri Remote (gen 3) and forwards
button + clickpad events to Home Assistant via MQTT, with Home Assistant
auto-discovery so the remote shows up as a device automatically.

**Status:** Phase 2 — Wi-Fi + MQTT + HA integration. Buttons, swipes, and
pickup events publish to MQTT; HA auto-discovers an Event entity and a
runtime Switch for the optional raw touch stream. Touch coordinate decoding
and battery/charging publishing are deferred to Phase 3.

## Hardware

- **MCU (development)**: ESP32-WROOM-32 DevKit (4 MB flash)
- **MCU (eventual target)**: ESP32-S3 DevKit-C (`make set-esp32s3` to retarget)
- **Remote**: Apple Siri Remote, 3rd generation (2022, USB-C)

## Prerequisites

- macOS or Linux
- `cmake`, `ninja`, `dfu-util` (on macOS: `brew install cmake ninja dfu-util`)
- ESP-IDF **v5.5** cloned to `~/esp/esp-idf` — see
  [Espressif's setup guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/)
- `export IDF_PATH=~/esp/esp-idf` in your shell (or source
  `~/esp/esp-idf/export.sh` the traditional way)
- A running Home Assistant instance with the **MQTT integration** enabled
  (Mosquitto add-on or any external broker)

## Quick start

```bash
make test          # host-side unit tests, no hardware required
make menuconfig    # set Wi-Fi SSID/password + MQTT broker URI/credentials
make build         # compile firmware
make fm            # flash + monitor the connected device
```

Run `make` with no arguments to see all targets.

## Configuration knobs (`menuconfig`)

| Menu | Key | Default | Purpose |
|---|---|---|---|
| Siri Bridge Configuration | `WIFI_SSID` / `WIFI_PASSWORD` | empty | Wi-Fi credentials |
| Siri Bridge Configuration | `MQTT_BROKER_URI` | `mqtt://homeassistant.local:1883` | MQTT broker |
| Siri Bridge Configuration | `MQTT_USERNAME` / `MQTT_PASSWORD` | empty | MQTT auth |
| Siri BLE Bridge | `SIRI_REMOTE_MAC` | dev-paired MAC | Target remote |
| Event state machine | `EVENT_DOUBLE_WINDOW_MS` | 300 | Double-click window |
| Event state machine | `EVENT_HOLD_THRESHOLD_MS` | 700 | Hold detection |
| Event state machine | `EVENT_SWIPE_MIN_DISTANCE` | 40 | Swipe threshold |
| Event state machine | `EVENT_PICKUP_IDLE_THRESHOLD_MS` | 30000 | Pickup idle gate |
| Siri Bridge Configuration | `IDLE_DISCONNECT_MS` | **0** (always-connected) | See "Always-connected mode" below |

`= 0` on the timing fields disables that feature (clean kill-switch).

## Always-connected mode (default)

The bridge keeps a permanent BLE link with the remote and uses the remote's preferred low-power conn params (15 ms intervals, slave latency 80) so the remote can deep-sleep ~99 % of the time while the link stays alive. Button presses break out of the slave-latency window and arrive within ~15 ms.

The reason we stay connected: Apple's BLE accessory firmware has an undocumented server-side timer that drops the *wake-press* (the press that wakes the remote from sleep) if the bridge has been disconnected for more than ~30 s. We tested every reasonable workaround at the GATT layer — different chain orders, claim writes to the Apple custom service, conn-param tuning, peer_disc_all priming — and none of them recovered the press once Apple discarded it. Implementing the full MagicPairing accessory-authentication protocol (which doesn't even gate HID delivery, per the [WiSec '20 paper](https://arxiv.org/pdf/2005.07255)) is not feasible without a hardware BLE sniffer and access to Apple's per-device LTK material. So we accept the trade-off: keep the link alive, button identity is always preserved.

**Battery cost:** ~15–25 µA average draw on the remote in connected-low-power mode vs. ~1 µA disconnected. CR2032 (≈ 225 mAh) lifetime drops from ~18–24 months to ~10–15 months. About 1.5× drain for 100 % wake-press reliability.

**Opting out:** set `CONFIG_IDLE_DISCONNECT_MS` to a non-zero value (e.g. 60000) via `idf.py menuconfig`. The bridge will proactively terminate the link after that many milliseconds of inactivity, the remote enters its deepest sleep, and battery improves — but any button press after >30 s of disconnect arrives without the button identity. The synthesized `pickup` event still fires as a generic "user activated remote" signal in HA. Choose this if battery matters more than knowing which specific button was pressed to wake.

## MQTT topics published

| Topic | Retained | Purpose |
|---|---|---|
| `siri_remote/event` | no | Discrete events (click/double_click/hold_start/hold_end/swipe_*/pickup) |
| `siri_remote/touch_raw` | no | Raw touch frames at ~50/sec — only when the HA Switch is on |
| `siri_remote/connection` | yes (LWT) | `online` / `offline` |
| `siri_remote/state/raw_stream` | yes | Current Switch state, mirrored from NVS |

## Home Assistant automation patterns

The bridge auto-discovers as a single **Event entity** with these `event_type`
values: `click`, `double_click`, `hold_start`, `hold_end`, `swipe_up`,
`swipe_down`, `swipe_left`, `swipe_right`, `pickup`. Event payloads include
`button` (for button events), `duration_ms` (for click/hold_end), `distance`
(for swipes), and `idle_duration_ms` (for pickup).

**Toggle a light on click:**
```yaml
trigger:
  platform: state
  entity_id: event.siri_remote
condition: >
  {{ trigger.to_state.attributes.event_type == 'click'
     and trigger.to_state.attributes.button == 'volume_up' }}
action:
  service: light.toggle
  target:
    entity_id: light.living_room
```

**Dim while holding** (start a loop on `hold_start`, stop on `hold_end`):
```yaml
- alias: "Vol Down hold dims"
  trigger:
    platform: state
    entity_id: event.siri_remote
  condition: >
    {{ trigger.to_state.attributes.event_type == 'hold_start'
       and trigger.to_state.attributes.button == 'volume_down' }}
  action:
    repeat:
      while: "{{ states('input_boolean.dimming') == 'on' }}"
      sequence:
        - service: light.turn_on
          data:
            entity_id: light.living_room
            brightness_step_pct: -5
        - delay: "00:00:00.1"
```

**Welcome-home on long pickup:**
```yaml
- alias: "Remote pickup after 1h triggers welcome scene"
  trigger:
    platform: state
    entity_id: event.siri_remote
  condition: >
    {{ trigger.to_state.attributes.event_type == 'pickup'
       and trigger.to_state.attributes.idle_duration_ms | int > 3600000 }}
  action:
    service: scene.turn_on
    target:
      entity_id: scene.evening_lights
```

**Magnitude-aware swipe:**
```yaml
condition: >
  {{ trigger.to_state.attributes.event_type == 'swipe_up'
     and trigger.to_state.attributes.distance | int > 100 }}
```

## Project layout

```
components/
  siri_ble/         # BLE central — scan, pair, discover, magic unlock, notify dispatch
  report_decoder/   # Pure-C parsers (button bitmap, touch frame, name lookup)
  event_state/      # Pure-C state machine — derives semantic events from raw input
main/
  main.c            # Wi-Fi, MQTT, NVS, HA auto-discovery, glue
  Kconfig.projbuild # Wi-Fi + MQTT credential config
tests/host/         # Host unit tests (no hardware required) — ASan + UBSan enabled
  fixtures/         # Captured per-button + per-swipe byte sequences from real hardware
```

## Acknowledgments

This project ports existing Siri Remote protocol work to an ESP32. The chain
of prior art:

- [Jack-R1](https://github.com/Jack-R1) — original reverse-engineering of the
  Apple TV 4th-gen Siri Remote protocol (Opus voice decoder, macOS voice
  control, Windows filter driver).
- [Yanndroid/SiriRemote-Linux](https://github.com/Yanndroid/SiriRemote-Linux) —
  Python/bluepy port for Linux that documents the GATT handle map, the
  vendor-specific "magic write," button bitmask, and touch/clickpad report
  format. Gen 2/3 coverage lives on the `gen-3` branch.

This project translates that protocol knowledge into ESP-IDF + NimBLE C
running on an ESP32.

## License

MIT — see [LICENSE](LICENSE).
