# siri-remote-ha-bridge

An ESP32 BLE bridge that pairs with an Apple Siri Remote (gen 3) and forwards
button + clickpad events to Home Assistant via MQTT, with Home Assistant
auto-discovery so the remote shows up as a device automatically.

**Status:** Phase 3B in progress — buttons, swipes, battery, and
charging all publish to MQTT with HA auto-discovery (Event entity, Switch
for raw touch stream, Sensors for battery + charging, Button to re-pair).
The bridge auto-discovers the remote on first boot — no hardcoded MAC, no
menuconfig step. Connected-low-power mode is the default to ensure 100 %
wake-press reliability — see [Always-connected mode](#always-connected-mode-default)
below.

## Hardware

- **MCU**: XIAO ESP32-S3 (Seeed Studio 3-pack — 8 MB flash, 8 MB octal PSRAM,
  native USB-C, BLE 5.0). Project migrated to this board in Phase 3.M; the
  ESP32-WROOM-32 path was the development target through Phase 3B but is no
  longer the primary build.
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

## Pairing your remote

The bridge has no hardcoded MAC. On first boot (empty NVS) it enters a 5-minute
**discovery window**: an active BLE scan filtered on the HID service UUID
(`0x1812`), with a proximity gate (RSSI ≥ −55 dBm) and a post-connect
fingerprint check (HID + ≥3 Report chars + Apple custom service + button
value handle `0x0039`).

### First-time pair (or pair to a new bridge)

The remote treats every distinct BLE peer identity as a separate "host" and
will not admit a connect request from an unknown peer at the LL layer until
it's been put into pairing mode. This is verified empirically: with a fresh
ESP32-S3 (different factory MAC than the previously-bonded ESP32), button
presses alone produce undirected advs but connect attempts fail with HCI
reason `0x3E` ("connection failed to be established") in a tight retry loop.
The fix is to put the remote into pairing mode:

1. Power the bridge on within ~1 metre of the remote.
2. Hold **Back + Volume Up** on the remote for ~5 seconds. The remote enters
   Apple's native pairing-mode advertising and admits unknown peers.
3. The bridge connects, runs the fingerprint check, bonds, and starts
   forwarding events. The whole dance takes <10 seconds. The bond is
   persisted in NVS — subsequent reboots reconnect directly without scanning.

If the bridge logs `candidate failed pre-fingerprint — blacklisting 5 s,
hold Back+VolUp …`, that's the signal: the remote is advertising but
rejecting your bridge's identity. Press the combo and the next retry will
succeed.

If the wrong device is in range (an iPhone, Apple Watch, Magic Keyboard…)
the bridge will either reject it via the HID-UUID filter (most Apple devices
don't advertise HID) or via the post-connect fingerprint (Magic Keyboard
passes the UUID gate but fails the Apple-custom-service check). Fingerprint
failures are blacklisted for 60 s; LL connect failures for 5 s (so a
correctly-pressed pairing combo recovers quickly).

### Re-pair on the same bridge (already bonded once)

The remote remembers bonded peers on its side too. If the bond is still
present on the remote and you press the HA repair button on the bridge:

- In Home Assistant, press **`button.siri_remote_repair`** (icon
  `mdi:bluetooth-refresh`). The bridge wipes the bond, terminates the active
  connection, and re-enters the 5-minute discovery window.
- Press **any button** on the remote (no combo needed — the remote still has
  the bridge in its bond list and will admit the connection for re-keying).

If the discovery window expires without finding a remote, the bridge logs a
warning and goes idle until the next repair-button press.

## Configuration knobs (`menuconfig`)

| Menu | Key | Default | Purpose |
|---|---|---|---|
| Siri Bridge Configuration | `WIFI_SSID` / `WIFI_PASSWORD` | empty | Wi-Fi credentials |
| Siri Bridge Configuration | `MQTT_BROKER_URI` | `mqtt://homeassistant.local:1883` | MQTT broker |
| Siri Bridge Configuration | `MQTT_USERNAME` / `MQTT_PASSWORD` | empty | MQTT auth |
| Event state machine | `EVENT_DOUBLE_WINDOW_MS` | 300 | Double-click window |
| Event state machine | `EVENT_HOLD_THRESHOLD_MS` | 700 | Hold detection |
| Event state machine | `EVENT_SWIPE_MIN_DISTANCE` | 40 | Swipe threshold |
| Siri Bridge Configuration | `IDLE_DISCONNECT_MS` | **0** (always-connected) | See "Always-connected mode" below |

`= 0` on the timing fields disables that feature (clean kill-switch).

## Always-connected mode (default)

The bridge keeps a permanent BLE link with the remote and pushes the connection to high-latency low-power params (15 ms intervals, slave latency 400, 15 s supervision timeout) so the remote can deep-sleep up to ~6 s between mandatory radio events while the link stays alive. Button presses break out of the slave-latency window and arrive within ~15 ms. The remote's PPCP advertises latency 80 as its preference, but Apple's peripheral accepts at least 400 — verified empirically and pushed into firmware after the latency-sweep experiment in `experiments/wake_minimal/`.

The reason we stay connected: Apple's BLE accessory firmware has an undocumented server-side timer that drops the *wake-press* (the press that wakes the remote from sleep) if the bridge has been disconnected for more than ~30 s. We tested every reasonable workaround at the GATT layer — different chain orders, claim writes to the Apple custom service, conn-param tuning, peer_disc_all priming — and none of them recovered the press once Apple discarded it. Implementing the full MagicPairing accessory-authentication protocol (which doesn't even gate HID delivery, per the [WiSec '20 paper](https://arxiv.org/pdf/2005.07255)) is not feasible without a hardware BLE sniffer and access to Apple's per-device LTK material. So we accept the trade-off: keep the link alive, button identity is always preserved.

**Battery cost:** ~5–10 µA average draw on the remote in connected-low-power mode at latency 400, vs. ~1 µA disconnected. CR2032 (≈ 225 mAh) lifetime drops from ~18–24 months to ~14–18 months. Roughly 1.2× drain for 100 % wake-press reliability — a Phase 3B Number entity will let you tune slave latency live to trade battery vs. disconnect-detection latency without reflashing.

**Opting out:** set `CONFIG_IDLE_DISCONNECT_MS` to a non-zero value (e.g. 60000) via `idf.py menuconfig`. The bridge will proactively terminate the link after that many milliseconds of inactivity, the remote enters its deepest sleep, and battery improves — but any button press after >30 s of disconnect arrives without the button identity. Choose this if battery matters more than knowing which specific button was pressed to wake.

## MQTT topics published

| Topic | Retained | Purpose |
|---|---|---|
| `siri_remote/event` | no | Discrete events (click/double_click/hold_start/hold_end/swipe_*) |
| `siri_remote/touch_raw` | no | Raw touch frames at ~50/sec — only when the HA Switch is on |
| `siri_remote/connection` | yes (LWT) | `online` / `offline` |
| `siri_remote/state/raw_stream` | yes | Current Switch state, mirrored from NVS |

## Home Assistant automation patterns

The bridge auto-discovers as a single **Event entity** with these `event_type`
values: `click`, `double_click`, `hold_start`, `hold_end`, `swipe_up`,
`swipe_down`, `swipe_left`, `swipe_right`. Event payloads include
`button` (for button events), `duration_ms` (for click/hold_end), and
`distance` (for swipes).

For "bridge came back online" automations (Wi-Fi outage recovery, bridge
reboot, etc.), bind to the bridge LWT availability topic
`siri_remote/connection` and watch for an `offline → online` transition —
the previously-emitted `pickup` event was a duplicate of this signal with
worse semantics in always-connected mode and was removed in Phase 3.N.

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

**Welcome-home on bridge recovery:** subscribe to MQTT topic
`siri_remote/connection` and trigger on the `offline → online` transition.
The bridge LWT publishes `online` when it boots and the broker pushes
`offline` after the keepalive expires; HA's `mqtt.state` trigger handles
this directly without needing a synthetic event.

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
