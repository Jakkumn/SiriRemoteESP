# siri-remote-ha-bridge

An ESP32 BLE bridge that pairs with an Apple Siri Remote (gen 3) and forwards
button/touch events to Home Assistant via MQTT.

**Status:** Phase 0 — BLE bring-up. The firmware boots, initializes NimBLE,
and passively scans for advertisements. Siri Remote pairing is not yet wired
up.

## Hardware

- **MCU (development)**: ESP32-WROOM-32 DevKit
- **MCU (eventual target)**: ESP32-S3 DevKit-C (`make set-esp32s3` to retarget)
- **Remote**: Apple Siri Remote, 3rd generation (2022, USB-C)

## Prerequisites

- macOS or Linux
- `cmake`, `ninja`, `dfu-util` (on macOS: `brew install cmake ninja dfu-util`)
- ESP-IDF **v5.5** cloned to `~/esp/esp-idf` — see
  [Espressif's setup guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/)
- `export IDF_PATH=~/esp/esp-idf` in your shell (or source
  `~/esp/esp-idf/export.sh` the traditional way)

## Quick start

```bash
make test          # host-side unit tests, no hardware required
make set-esp32     # target WROOM-32 (one-time per target switch)
make build         # compile firmware
make fm            # flash + monitor the connected device
```

Run `make` with no arguments to see all targets.

## Project layout

```
components/report_decoder/  # pure-C HID report decoder (no IDF deps)
main/                       # firmware entry point + NimBLE bring-up
tests/host/                 # host-side unit tests (CMake + assert())
```

## License

MIT — see [LICENSE](LICENSE).
