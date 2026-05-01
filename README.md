# SiriRemoteESP

An ESP32 BLE bridge that pairs with an Apple Siri Remote (gen 3) and surfaces
button + clickpad + voice events to Home Assistant. Two install paths: an
ESPHome external component for end users, and a standalone ESP-IDF build for
hacking on the C components.

**Status:** Phase 5 shipped. ESPHome wrap is the recommended install path;
the standalone build remains available for development. The full pipeline
— button + clickpad + voice capture with on-device Opus decode feeding
ESPHome's `voice_assistant` — is verified end-to-end. Outstanding work is
limited to research items (accelerometer / motion telemetry probe).

## Hardware

- **MCU**: XIAO ESP32-S3 (Seeed Studio 3-pack — 8 MB flash, 8 MB octal PSRAM,
  native USB-C, BLE 5.0).
- **Remote**: Apple Siri Remote, 3rd generation (2022, USB-C).

## Install — ESPHome (recommended)

The bridge ships as an ESPHome external component. Drop it into a YAML config,
fill in your secrets, run `esphome run`. No ESP-IDF or `make` knowledge needed.

1. Clone (or pin via the GitHub form below).
   ```bash
   git clone https://github.com/Jakkumn/SiriRemoteESP
   cd SiriRemoteESP
   ```
2. Create your secrets file from the template:
   ```bash
   cp secrets.yaml.example secrets.yaml
   $EDITOR secrets.yaml
   ```
   `secrets.yaml.example` documents how to generate the API encryption key.
3. Flash:
   ```bash
   uv run esphome run example.yaml
   ```

After the bridge boots, follow [Pairing your remote](#pairing-your-remote)
below.

### Pinning a release

To consume the component without cloning, point ESPHome at the GitHub repo:

```yaml
external_components:
  - source: github://Jakkumn/SiriRemoteESP@v0.0.1
    path: esphome/components
```

Pinning to a tag (rather than `@main`) is recommended so a downstream change
won't break your install on the next `esphome run`.

### What you get in Home Assistant

The wrap registers seven entity types under one device:

- `binary_sensor.siri_remote_voice_active` — `on` while Mic is held.
- `sensor.siri_remote_battery` — battery level (%).
- `text_sensor.siri_remote_charging` — `charging` / `discharging` / `plugged_in`.
- `switch.siri_remote_raw_touch_stream` — toggles UART logging of the touchpad
  packets (diagnostic; off by default).
- `button.siri_remote_re_pair` — wipes the BLE bond and re-enters discovery.
- Six `number.siri_remote_*` sliders for runtime-tunable thresholds (swipe
  sensitivity, double-click window, hold threshold, battery-low %, BLE slave
  latency). All persist across reboot via ESPHome Preferences.

Discrete remote events (clicks, double-clicks, holds, swipes) fire as a Home
Assistant event called `esphome.siri_remote_event` with full payload — see
[Home Assistant automation patterns](#home-assistant-automation-patterns).

### Voice in HA Assist (optional)

Holding the Mic button on the remote pushes decoded audio into Home
Assistant's Assist pipeline as push-to-talk — wake-word disabled. The
`microphone:` and `voice_assistant:` blocks in `example.yaml` enable
this. Comment them out (and remove `voice_assistant_id:` from the
`siri_remote:` block) if you don't want voice — the rest of the bridge
keeps working.

Mic press fires `voice_assistant.start` directly from the firmware
(see `voice_assistant_id:` in `example.yaml`). voice_assistant's
built-in VAD finalizes STT when it detects end-of-speech silence. Mic
*release* is intentionally **not** wired to `voice_assistant.stop` —
that action is a hard abort that drops the in-flight utterance if the
user releases before VAD completes. Trade-off: pressing Mic and
staying silent lets voice_assistant sit in `STREAMING_MICROPHONE` for
a second or two until VAD's no-voice timeout fires. Acceptable for
push-to-talk.

After Mic-release the firmware also injects 20 ms-paced silence
frames into the audio stream (since Apple stops sending audio packets
at release) so HA's VAD has a continuous signal to analyze and can
detect end-of-speech naturally. Without this the satellite would sit
in HA's "Listening" state until its `stt-stream-failed` timeout (~9 s)
fired and discarded the captured utterance. The silence stops as soon
as voice_assistant finishes the cycle.

#### TTS playback options

The XIAO S3 has no audio-out hardware, so the bridge by itself can't
play voice responses. There are three usable patterns; pick whichever
fits your install. Don't try to remove TTS from the Assist pipeline
itself — ESPHome's `voice_assistant` always advertises that it can
receive audio responses (`FEATURE_API_AUDIO` is hardcoded), and HA's
pipeline validator rejects pipelines without TTS for satellites that
advertise it (`validation-error - the pipeline does not support
text-to-speech`).

**A — local speaker on the bridge.** If you have I²S audio hardware
wired up, add an ESPHome `speaker:` block and reference it from
`voice_assistant.speaker:`. voice_assistant handles playback +
end-of-response signalling itself — set
`siri_remote: auto_finish_response: false` in this case so we don't
send a duplicate `VoiceAssistantAnnounceFinished` message.

**B — route TTS to an existing HA media_player.** If you have a Sonos,
Echo, Voice PE, etc. already in HA and want voice responses to play
there, add a top-level automation hook on the voice_assistant block:

```yaml
voice_assistant:
  id: siri_voice_assistant
  microphone:
    microphone: siri_remote_mic
  use_wake_word: false
  on_tts_end:
    - homeassistant.service:
        service: media_player.play_media
        data:
          entity_id: media_player.kitchen
          media_content_id: !lambda 'return x;'
          media_content_type: music
```

Leave `auto_finish_response: true` (the default) so HA's
`assist_satellite` UI returns to Idle promptly — voice_assistant's
built-in playback timeout doesn't run when no local speaker /
media_player is attached, so the firmware fan-out fills the gap.

**C — no speaker anywhere (the default).** Voice commands process and
intents run, but no audio response is generated locally. HA still
generates the TTS audio (a few hundred ms of compute on the HA side);
the bridge silently discards it. The
`siri_remote: voice_assistant_id: siri_voice_assistant` opt-in in
`example.yaml` enables this with `auto_finish_response: true` (the
default), so the firmware mirrors what `voice_assistant`'s playback
timeout would have done if a media_player were attached: it sends
`VoiceAssistantAnnounceFinished` when HA reports TTS_END, and the
`assist_satellite.<bridge>` entity in HA cleanly transitions
`responding` → `idle` instead of getting stuck on Responding.

#### Reducing voice latency

Press → audio in HA Assist takes ~150–250 ms today (firmware + HA
round-trip). What feels slower is user reaction time after press
(500–1500 ms) and HA's VAD silence-detection lag (600 ms+). The wins
below stack:

1. **HA Aggressive VAD.** Settings → Voice Assistants → your pipeline
   → **Finished speaking detection** → **Aggressive**. Saves
   ~200–400 ms on end-of-utterance. Slow speakers may get the last
   syllable clipped — fall back to Default if it bites.
2. **HA cue automation.** Bind to `assist_satellite.<bridge>` state
   transitions. Once `auto_finish_response: true` is on (Scenario C
   above), the satellite cleanly cycles `idle` → `listening` →
   `processing` → `idle` per command, so HA-side automations on those
   transitions actually work. Example — flash a hue lamp while listening:
   ```yaml
   automation:
     - alias: "Siri bridge listening cue"
       trigger:
         platform: state
         entity_id: assist_satellite.siri_bridge
         to: "listening"
       action:
         service: light.turn_on
         target: { entity_id: light.living_room_lamp }
         data: { rgb_color: [0, 100, 255], brightness: 80 }
   ```
   The auto-derived satellite name is `assist_satellite.<esphome_name>`.
3. **Firmware-side direct trigger** (already wired in `example.yaml`).
   Setting `voice_assistant_id:` on the `siri_remote:` block fires
   `voice_assistant.start` directly from the BLE callback, skipping
   the binary_sensor → on_press automation hop. ~5–15 ms saving;
   sub-perceptual but stacks with the rest.

What's deliberately out of scope: continuous-stream pre-warm
(privacy + 24/7 HA STT compute on silence), cross-task direct calls
into voice_assistant (ESPHome thread model forbids), and the HA-side
network round-trip itself (~100–200 ms — controlled by HA's STT
engine choice, not the bridge).

### Caveats

- **Don't add `esp32_ble_tracker:` to the same YAML.** NimBLE is single-host
  and the bridge owns it. Mixing the two crashes at bring-up.
- **Don't combine with another voice satellite component on the same chip.**
  The siri_audio decoder owns one CPU's free cycles when Mic is held.

## Install — standalone (for ESP-IDF developers)

Use this path if you want to hack on the C components, run `make test`, or
build firmware without ESPHome's YAML layer.

### Prerequisites

- macOS or Linux
- `cmake`, `ninja`, `dfu-util` (on macOS: `brew install cmake ninja dfu-util`)
- ESP-IDF **v5.5** cloned to `~/esp/esp-idf` — see
  [Espressif's setup guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/)
- `export IDF_PATH=~/esp/esp-idf` in your shell (or source
  `~/esp/esp-idf/export.sh` the traditional way)
- A running Home Assistant instance with the **MQTT integration** enabled
  (Mosquitto add-on or any external broker)

### Quick start

```bash
make test          # host-side unit tests, no hardware required
make menuconfig    # set Wi-Fi SSID/password + MQTT broker URI/credentials
make build         # compile firmware
make fm            # flash + monitor the connected device
```

Run `make` with no arguments to see all targets.

### Standalone configuration knobs (`menuconfig`)

| Menu | Key | Default | Purpose |
|---|---|---|---|
| Siri Bridge Configuration | `WIFI_SSID` / `WIFI_PASSWORD` | empty | Wi-Fi credentials |
| Siri Bridge Configuration | `MQTT_BROKER_URI` | `mqtt://homeassistant.local:1883` | MQTT broker |
| Siri Bridge Configuration | `MQTT_USERNAME` / `MQTT_PASSWORD` | empty | MQTT auth |
| Event state machine | `EVENT_DOUBLE_WINDOW_MS` | 300 | Double-click window |
| Event state machine | `EVENT_HOLD_THRESHOLD_MS` | 700 | Hold detection |
| Event state machine | `EVENT_SWIPE_MIN_DISTANCE` | 40 | Swipe threshold |
| Siri Bridge Configuration | `IDLE_DISCONNECT_MS` | **0** (always-connected) | See "Always-connected mode" below |

`= 0` on the timing fields disables that feature (clean kill-switch). The
ESPHome path exposes the same knobs as runtime Number entities; the table
above only applies to the standalone build.

### Standalone-only MQTT topics

The standalone build publishes these topics directly. The ESPHome path does
not — events flow over the ESPHome API instead. Bind to these only if you're
running the standalone firmware.

| Topic | Retained | Purpose |
|---|---|---|
| `siri_remote/event` | no | Discrete events (click/double_click/hold_start/hold_end/swipe_*) |
| `siri_remote/touch_raw` | no | Raw touch frames at ~50/sec — only when the HA Switch is on |
| `siri_remote/connection` | yes (LWT) | `online` / `offline` |
| `siri_remote/state/raw_stream` | yes | Current Switch state, mirrored from NVS |

## Pairing your remote

Both build paths use the same pairing flow. The bridge has no hardcoded MAC.
On first boot (empty NVS) it enters a 5-minute **discovery window**: an active
BLE scan filtered on the HID service UUID (`0x1812`), with a proximity gate
(RSSI ≥ −55 dBm) and a post-connect fingerprint check (HID + ≥3 Report chars
+ Apple custom service + button value handle `0x0039`).

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

- In Home Assistant, press **`button.siri_remote_re_pair`** (icon
  `mdi:bluetooth-refresh`). The bridge wipes the bond, terminates the active
  connection, and re-enters the 5-minute discovery window.
- Press **any button** on the remote (no combo needed — the remote still has
  the bridge in its bond list and will admit the connection for re-keying).

If the discovery window expires without finding a remote, the bridge logs a
warning and goes idle until the next repair-button press.

## Always-connected mode (default)

The bridge keeps a permanent BLE link with the remote and pushes the connection to high-latency low-power params (15 ms intervals, slave latency 400, 15 s supervision timeout) so the remote can deep-sleep up to ~6 s between mandatory radio events while the link stays alive. Button presses break out of the slave-latency window and arrive within ~15 ms. The remote's PPCP advertises latency 80 as its preference, but Apple's peripheral accepts at least 400 — verified empirically and pushed into firmware after the latency-sweep experiment in `experiments/wake_minimal/`.

The reason we stay connected: Apple's BLE accessory firmware has an undocumented server-side timer that drops the *wake-press* (the press that wakes the remote from sleep) if the bridge has been disconnected for more than ~30 s. We tested every reasonable workaround at the GATT layer — different chain orders, claim writes to the Apple custom service, conn-param tuning, peer_disc_all priming — and none of them recovered the press once Apple discarded it. Implementing the full MagicPairing accessory-authentication protocol (which doesn't even gate HID delivery, per the [WiSec '20 paper](https://arxiv.org/pdf/2005.07255)) is not feasible without a hardware BLE sniffer and access to Apple's per-device LTK material. So we accept the trade-off: keep the link alive, button identity is always preserved.

**Battery cost:** ~5–10 µA average draw on the remote in connected-low-power mode at latency 400, vs. ~1 µA disconnected. CR2032 (≈ 225 mAh) lifetime drops from ~18–24 months to ~14–18 months. Roughly 1.2× drain for 100 % wake-press reliability — the ESPHome path exposes `number.siri_remote_ble_slave_latency` as a live slider so you can trade battery vs. disconnect-detection latency without reflashing.

**Opting out:**

- **ESPHome path:** set `idle_disconnect_ms:` on the `siri_remote:` block in
  YAML to a non-zero value (e.g. `60000`).
- **Standalone path:** set `CONFIG_IDLE_DISCONNECT_MS` via `idf.py menuconfig`.

The bridge will proactively terminate the link after that many milliseconds of inactivity, the remote enters its deepest sleep, and battery improves — but any button press after >30 s of disconnect arrives without the button identity. Choose this if battery matters more than knowing which specific button was pressed to wake.

## Home Assistant automation patterns

### ESPHome path

Discrete remote events fire as a Home Assistant event `esphome.siri_remote_event`
with the following `event_data` shape:

| Field | Type | Present on |
|---|---|---|
| `action` | string | always — one of `click`, `double_click`, `hold_start`, `hold_end`, `swipe_up`, `swipe_down`, `swipe_left`, `swipe_right` |
| `button` | string | button events only — `select`, `tv`, `mic`, `volume_up`, `volume_down`, `back`, `play_pause`, etc. |
| `duration_ms` | integer | `click` and `hold_end` only |
| `distance` | integer | swipe events only |

**Toggle a light on a click:**

```yaml
trigger:
  platform: event
  event_type: esphome.siri_remote_event
  event_data:
    action: click
    button: volume_up
action:
  service: light.toggle
  target:
    entity_id: light.living_room
```

**Dim while holding** (start a loop on `hold_start`, stop on `hold_end`):

```yaml
- alias: "Vol Down hold dims"
  trigger:
    platform: event
    event_type: esphome.siri_remote_event
    event_data:
      action: hold_start
      button: volume_down
  action:
    repeat:
      until:
        - platform: event
          event_type: esphome.siri_remote_event
          event_data:
            action: hold_end
            button: volume_down
      sequence:
        - service: light.turn_on
          data:
            entity_id: light.living_room
            brightness_step_pct: -5
        - delay: "00:00:00.1"
```

**Magnitude-aware swipe** (HA template trigger — swipe_up only fires when
distance > 100):

```yaml
trigger:
  platform: event
  event_type: esphome.siri_remote_event
  event_data:
    action: swipe_up
condition: "{{ trigger.event.data.distance | int > 100 }}"
```

**Pause media while voice is active:**

```yaml
trigger:
  platform: state
  entity_id: binary_sensor.siri_remote_voice_active
  to: "on"
action:
  service: media_player.media_pause
  target:
    entity_id: media_player.living_room
```

### Standalone path

The MQTT-only build publishes events on `siri_remote/event` and an Event
entity is auto-discovered with the same `event_type` values as above. Bind
to the entity directly:

```yaml
trigger:
  platform: state
  entity_id: event.siri_remote
condition: >
  {{ trigger.to_state.attributes.event_type == 'click'
     and trigger.to_state.attributes.button == 'volume_up' }}
```

For "bridge came back online" automations (Wi-Fi outage recovery, bridge
reboot, etc.), bind to the bridge LWT availability topic
`siri_remote/connection` and watch for an `offline → online` transition. The
ESPHome path uses the ESPHome API's native availability signal instead and
does not need a separate topic subscription.

## Project layout

```
components/                       # standalone-only IDF C
  siri_ble/                       # BLE central — scan, pair, discover, magic unlock, notify dispatch
  siri_audio/                     # Opus decode pipeline (gen-3 audio packet parser + decoder task)
  report_decoder/                 # Pure-C parsers (button bitmap, touch frame, charging-state, name lookup)
  event_state/                    # Pure-C state machine — derives semantic events from raw input
  mqtt_entity/                    # MQTT helper for Switch / Number / Button HA-discovery (standalone-only)
main/
  main.c                          # standalone build — Wi-Fi, MQTT, NVS, HA auto-discovery, glue
  Kconfig.projbuild               # standalone Wi-Fi + MQTT credential config
esphome/components/siri_remote/   # ESPHome external component
  __init__.py                     # parent component schema, sdkconfig pinning
  binary_sensor.py / sensor.py / text_sensor.py / switch.py / button.py / number.py / microphone.py
  siri_remote_hub.{h,cpp}         # Component subclass — orchestrates BLE + audio + entity bridging
  siri_remote_{switch,button,number,microphone}.{h,cpp}  # entity subclasses
  *.c, *.h                        # symlinks to ../../components/*/ — both build paths share C
example.yaml                      # working ESPHome config (the recommended user starting point)
secrets.yaml.example              # template for the secrets ESPHome reads
tests/host/                       # Host unit tests (no hardware required) — ASan + UBSan enabled
  fixtures/                       # Captured per-button + per-swipe byte sequences from real hardware
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
running on an ESP32, with an ESPHome external-component layer on top.

## License

MIT — see [LICENSE](LICENSE).
