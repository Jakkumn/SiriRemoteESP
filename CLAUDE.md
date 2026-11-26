# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An ESP32-S3 BLE central that bonds with an Apple Siri Remote (gen 3), decodes its
HID/touch/audio notifications, and surfaces them to Home Assistant. Read `README.md`
first — it is the user-facing spec (pairing flow, HA entity list, event payload shape,
voice/TTS trade-offs) and is kept current.

## Two build paths, one C core

This is the single most important structural fact:

- `components/` — ESP-IDF components, built by `idf.py` for the **standalone** firmware
  (`main/main.c`, Wi-Fi + MQTT + HA MQTT-discovery).
- `esphome/components/siri_remote/` — the **ESPHome external component** (the shipped
  install path). Its `*.c` / `*.h` files are **symlinks back into `components/`**.

Both paths compile the same C sources. **Always edit the real file under `components/`** —
never the symlink path. A change to `report_decoder.c` lands in both firmwares at once.

Differences between the paths are confined to the glue layer:

| Concern | Standalone | ESPHome |
|---|---|---|
| Entrypoint | `main/main.c` | `siri_remote_hub.cpp` (`Component` subclass) |
| Config | `idf.py menuconfig` → `main/Kconfig.projbuild` | YAML schema in `__init__.py` + `*.py` platforms |
| Output | MQTT topics (`siri_remote/event`, …) | `esphome.siri_remote_button` HA event + native entities |
| Tunables | compile-time `CONFIG_*` | compile-time YAML keys on `siri_remote:` |

`siri_remote_hub.cpp` is deliberately a near-mirror of `main.c` (same callback names, same
locking discipline). When changing one, check whether the other needs the same change.

## Component responsibilities

- `components/report_decoder/` — pure C, no ESP-IDF. Button bitmap (`SIRI_HANDLE_BUTTON`),
  11-byte touch frame (`SIRI_HANDLE_TOUCH`), BLE `0x2A1A` charging byte, enum→string names.
- `components/button_pulse/` — pure C, no ESP-IDF. IR-style pulse emitter. While a button
  is held it re-emits `{button, repeat, held_ms}` every `repeat_interval_ms`; **there is no
  release event** — release is inferred downstream from the pulses stopping, exactly as an
  IR receiver infers it from repeat frames ending. Requires `button_pulse_tick()` calls,
  because the remote sends HID reports on change, not while held. Both paths drive it from
  a **one-shot** `esp_timer` re-armed after every feed/tick for `button_pulse_next_due_ms()`
  — exact repeat cadence, and no wakeups at all while nothing is held. Don't go back to a
  periodic tick: it quantised repeats to the tick period.

  **Nothing here classifies input.** `repeat` and `held_ms` are measurements; every judgment
  built from them ("is 220 ms a click or a hold?") lives in Home Assistant, which has the
  per-automation and per-context knowledge firmware does not. Adding a threshold to this
  component is almost certainly a design mistake — see `PULSE_REFACTOR_PLAN.md` for the
  reasoning, including why real IR receivers and tvOS both work this way.

  Touch/gesture support was deliberately removed. The touch CCCD stays subscribed (removing
  its `SETUP_STEPS[]` entry would perturb the fragile setup chain) but notifies are discarded
  unless `debug_touch_frames` is on.
- `components/siri_audio/` — Opus decode. `siri_audio_validate.c` is the pure-C header
  validator (host-testable); `siri_audio.c` owns the decoder task pinned to CPU1 with
  decoder state in PSRAM (`MALLOC_CAP_SPIRAM`). `siri_audio_pcm_tcp.c` is the shared
  debug PCM-over-TCP sink both glues call.
- `components/siri_ble/` — NimBLE central. The bulk of the protocol work lives here,
  including everything below the glue: `siri_ble_start()` brings up the NimBLE port, bond
  store and host task itself, and the idle-disconnect timer and slave latency arrive as
  `siri_ble_config_t` fields. The glue only does NVS init and supplies config + callbacks.
- `components/mqtt_entity/` — HA MQTT-discovery helper; **standalone only**, unused by ESPHome.
- `components/siri_ble/peer.c` + `esp_central.h` — vendored from IDF's `blecent` example.
  Kept verbatim so they can be diffed against upstream; excluded from `make lint` and from
  the `-Werror` source properties.

### siri_ble specifics

- A 4-state machine: `MODE_DISCOVERING` (passive scan, HID-UUID filter, RSSI ≥ −55 dBm,
  post-connect fingerprint) → `MODE_BONDED_RECONNECT` (direct connect to the stored bond)
  → `MODE_CONNECTED`, with `MODE_IDLE` when the 5-minute discovery window lapses.
- **GATT handles are discovered once per bond and cached in NVS**, never hardcoded: two
  gen-3 units with different firmware carry the same attribute structure shifted by one
  handle, and a hardcoded CCCD write onto the neighbour "succeeds" and silently never
  notifies. `derive_handle_map()` builds a `siri_handle_map_t` structurally (by UUID and
  notify property, in handle order) at the first-bond fingerprint; it is saved under NVS
  namespace `siri_ble`, key `hmapv1`, and reloaded on bonded reconnect so the ~3.7 s
  `peer_disc_all` is paid once. A bonded connect with no cached map rediscovers; Repair
  erases it. `SIRI_HANDLE_*` in `include/siri_ble.h` are now *logical* IDs — siri_ble
  translates every real handle to one before `on_notify`, so callers never see the layout.
- Post-connect work is a sequential `SETUP_STEPS[]` table (CCCD writes, the vendor "magic
  unlock" write `0xF0 0x00` to the first non-notify HID Report — `0x004D` on the
  reference unit — and initial reads). Steps are chained one-at-a-time
  through `on_setup_step_done` — add new steps to the table rather than issuing ad-hoc GATT ops.
- Default is **always-connected** with slave latency 400; the link is never dropped
  deliberately. This is load-bearing: Apple's firmware discards the wake-press if the
  bridge has been disconnected >~30 s. See README "Always-connected mode" before touching
  conn-param or disconnect logic.

## Threading

Three contexts touch shared state; get this wrong and you get rare crashes, not test failures.

- **NimBLE host task** — all `on_notify` / connect / disconnect callbacks, and every
  `siri_ble` timer and request: its timeouts are `ble_npl_callout`s on NimBLE's default
  event queue and `siri_ble_repair()` posts an event there, so all BLE state machine
  transitions happen on this one task. Don't add an `esp_timer` or a direct cross-task call
  into `siri_ble` state; add a callout or event. `ble_npl_callout_stop()` does not dequeue
  an already-fired event, so callbacks must re-check their state. Never issue a GAP
  procedure while `!ble_hs_synced()` (mid host reset): it is lost when sync resets the
  controller but stays "pending" in NimBLE, wedging every later scan/connect.
  `start_discovery()` / `start_bonded_reconnect()` enforce this; `on_sync` re-dispatches.
- **ESPHome loop task** (or MQTT handler in standalone) — entity writes, HA event sends.
- **siri_audio decode task** (CPU1) — `on_pcm` callbacks, for decoded audio and for the
  post-release silence (`siri_audio_silence_start()`), so the ESPHome microphone has a
  single producer.

Rules in force: `button_pulse` has no internal lock — callers hold `pulse_lock_` (ESPHome) or
`s_pulse_lock` (standalone) around every feed/tick. Entity publishes from the BLE task go
through `defer_publish_()` / `this->defer()`, never direct. Cross-task scalar flags are
`volatile` single words (atomic on Xtensa).

**Never publish from inside the emit callback.** It runs under the pulse lock, and at a
100 ms repeat interval it runs ~10x/sec on the high-priority `esp_timer` task. Standalone
stages into `s_pulse_stage[]` and `flush_pulses()` queues the batch after releasing the
lock — otherwise `esp_mqtt_client_publish` (which takes the MQTT client mutex and can block
on the outbox) would be called with our lock held, against the NimBLE notify path. ESPHome
writes the pulse into `pulse_ring_` (a lock-free SPSC ring) and calls
`enable_loop_soon_any_context()`; `loop()` drains and publishes it, then `disable_loop()`s.
Not `defer()` per pulse: that heap-allocates and takes the scheduler lock under ours.

Standalone never calls `esp_mqtt_client_publish` from the NimBLE host or `esp_timer` tasks
at all: it sends synchronously and can block for the ~10 s network timeout. Runtime
publishes go through `mqtt_publish()`, which copies into `s_pub_queue` without blocking, and
`publisher_task` does the send. Only the MQTT event handler, already on the MQTT task, uses
`mqtt_publish_now()` directly. ESPHome hands pulses
to the loop task for the same reason, and additionally gates on
`api::global_api_server->is_connected()` so a held button across an HA restart doesn't make
`APIServer` log a dropped-event warning per pulse.

Pairing-combo suppression lives in `siri_ble.c` (`s_suppress_until_ms`); the glue only
supplies `pairing_flush_suppress_ms`. It is armed when a fresh bond's (`idle_ms == 0`)
setup chain finishes and drops notifies in `NOTIFY_RX` before they reach `on_notify` — so
`button_pulse` never learns the combo was held and cannot start pulsing when the window
closes. Both its write and read are on the NimBLE host task, so it needs no lock. Keep it
out of `button_pulse`, which must stay free of judgment logic.

## Commands

Standalone / C development (needs ESP-IDF **v5.5**, `export IDF_PATH=~/esp/esp-idf`):

```bash
make test                       # host unit tests — no hardware, no ESP-IDF needed
make lint                       # clang-format --dry-run -Werror over our C (not peer.c)
make build                      # idf.py build
make fm                         # flash + monitor (PORT=/dev/cu.usbserial-XXXX to pin)
make erase                      # erase flash — wipes the NVS bond, forces re-pair
make menuconfig                 # standalone Wi-Fi/MQTT/pulse-interval config
make                            # list all targets
```

Run one host test (the CMake project builds one executable per test file):

```bash
ctest --test-dir tests/build -R test_button_pulse --output-on-failure
./tests/build/test_button_pulse         # or run the binary directly
```

ESPHome path (uv-managed; `uv sync` first if `.venv` is stale):

```bash
cp secrets.yaml.example secrets.yaml    # then fill it in; secrets.yaml is gitignored
uv run esphome config example.yaml      # validate YAML + codegen without compiling
uv run esphome run example.yaml         # compile + flash + log
uv run esphome clean example.yaml
```

`builder.yaml` is the copy-paste config for the HA ESPHome Device Builder add-on (pins the
component by git tag); keep it in sync with `example.yaml` when the schema changes.

## Testing

`tests/host/` builds the pure-C components as plain static libraries with
`-Wall -Wextra -Wpedantic -Werror` plus ASan + UBSan. The CMakeLists comment states the
invariant these tests exist to defend: **`report_decoder`, `button_pulse`, and
`siri_audio_validate` must not include ESP-IDF headers.** If you add an `esp_*` include to
one of them, `make test` breaks at compile time — that is the intended signal, not a build
problem to work around.

`tests/host/fixtures/` holds real captured byte sequences per button and per swipe. Prefer
extending a fixture over inventing synthetic bytes. Note the fixtures carry **no timestamps**
and there is no loader — they are hand-transcribed into tests. Bitmaps can come from them;
all timing in `test_button_pulse.c` is necessarily synthetic. The swipe fixtures are retained
only as protocol documentation for `report_decoder`, since gesture support is gone.

There is no on-device test harness; anything touching `siri_ble.c` or `siri_audio.c` needs
`make fm` against real hardware.

## Conventions

- C style: `.clang-format` (LLVM base, 4-space, 100 cols, Linux braces, `SortIncludes: false`
  — include order is intentional). `make lint` is the gate.
- Every component builds with `-Wall -Wextra -Werror`; don't relax it. The one sanctioned
  exception is the `-Wno-maybe-uninitialized` on vendored micro-opus in the root `CMakeLists.txt`.
- The ESPHome C++ follows ESPHome's own style (2-space, `trailing_` member underscores), not
  `.clang-format` — it is not in the lint set.
- Shared `#ifdef` names stay identical across both paths: `__init__.py` emits
  `-DCONFIG_VOICE_ENABLED` / `-DCONFIG_DEBUG_WAKE_PROBE` as build flags so `siri_ble.c` is
  byte-identical whether Kconfig or ESPHome codegen supplied them. Preserve that when adding
  new conditional features.
- `esphome/components/siri_remote/__init__.py::_pin_sdkconfig()` forces NimBLE on, Bluedroid
  off, PSRAM octal-mode on, and raises the ESP-IDF log ceiling. ESPHome defaults do not work
  for this component; changes there must match `sdkconfig.defaults` on the standalone side.
- **`to_code()` must call `esp32.request_bluetooth()` and
  `esp32.request_software_coexistence()`.** ESPHome 2026.9 began excluding built-in IDF
  components by default, and `bt` / `esp_coex` are on that list. We drive NimBLE directly
  instead of via ESPHome's `esp32_ble`, so nothing else requests them and ESP-IDF drops the
  component — every `host/ble_hs.h` include then fails with a message that blames
  `INCLUDE_DIRS`. Setting `CONFIG_BT_ENABLED` does **not** help: sdkconfig options cannot
  un-exclude a component.
- **Track the current ESPHome release line; older releases are unsupported.** The floor in
  `pyproject.toml` is 2026.11.0 (the microphone uses `publish_audio_()`, the hub uses
  `fire_homeassistant_event`). Keep `uv.lock` on the release users' Builder add-ons run —
  v0.1.0 shipped broken because every check ran on a stale locked 2026.7.1 while users were
  on 2026.9.x. When a new ESPHome release lands, compile against it before tagging:
  `uvx --from esphome==<ver> esphome compile` is an isolated check, then bump the lock with
  `uv lock --upgrade-package esphome`. `[tool.uv] environments` excludes Intel macOS, where
  ESPHome 2026.11's own dependency pins are unsatisfiable.
- Comments in this codebase record *why* (empirical findings, Apple firmware behaviour,
  rejected alternatives). Keep that density; don't strip them as noise.

## Attribution

The protocol knowledge is ported, not originated. The README's Acknowledgments section
credits Jack-R1 (original Siri Remote reverse-engineering / Opus decoder) and
Yanndroid/SiriRemote-Linux (GATT handle map, magic write, button bitmask, touch format).
That chain must stay in the README.

## Scratch areas

- `experiments/wake_minimal/` — a separate, self-contained IDF project (BLE only, no Wi-Fi /
  MQTT / button_pulse) used to isolate wake-press behaviour. Not built by the root project;
  it has its own `Makefile` and `sdkconfig`.
- `blueprints/automation/siri_remote/` — two HA blueprints shipped to users: `button.yaml`
  (one button, the common case) and `controller.yaml` (all 13 buttons). Update both when the
  event payload or button set changes. Note HA matches `event_data` with `==` and the ESPHome
  API carries every value as a *string*, so numeric comparisons need `| int` and literal
  filters need quoting (`repeat: "0"`). Getting this wrong fails silently.
