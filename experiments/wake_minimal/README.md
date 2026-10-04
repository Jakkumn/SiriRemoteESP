# wake_minimal — minimal BLE-only Siri Remote bridge

Standalone ESP-IDF project that strips everything *except* the BLE central + setup chain. No Wi-Fi, no MQTT, no button_pulse, no MQTT entity helpers. Just connect to the bonded gen-3 Siri Remote, run the same setup chain (CCCDs + magic unlock + deferred-second-ENC_CHANGE flow), and log every non-touch HID notify to UART with a decoded button name when applicable.

## Why this exists

The main project's wake-press investigation hit a wall: cold-boot / long-idle wake-presses are dropped before they reach our handler, but every interim hypothesis (subscribe more, read more, write claims, faster conn intervals) failed to recover them. Last avenue worth exploring before declaring it Apple-firmware-side: maybe the loss is caused by something *we* are doing — Wi-Fi/BT coexistence, MQTT-handler CPU time, pulse-emitter lock contention — rather than by Apple. This minimal build removes all those.

If this build also loses the cold-boot wake-press, the issue is definitively in the BLE layer (Apple-side). If it captures the wake-press cleanly when the main build doesn't, we've found the culprit at the application layer.

## Build / flash

Same ESP-IDF env as the main project. From this directory:

```sh
source $IDF_PATH/export.sh
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

Partition layout is identical to the main project (`partitions.csv`), so flashing this image over the same chip preserves the NVS bond — no re-pairing required. If you do want a clean state, `idf.py erase-flash` first, then on first boot hold the remote's home + volume-up buttons for 5 seconds to enter pairing mode.

## Test protocol

Same as the main project's wake-press tests:

1. Power-cycle the bridge. Wait for `bond store: 1 persisted peer-sec` followed by `scanning for ...`.
2. Wait long enough for the remote to time out of Apple's session cache (≥5 minutes with no contact between bridge and remote).
3. Press a button to wake the remote. Hold the capture for ≥15 s after the connection establishes.
4. Look for `t=Xms NOTIFY h=0x0039 ...` followed by `-> button PRESS: power` (or whatever button you used).

The `t=Xms` timestamps are relative to `BLE_GAP_EVENT_CONNECT`, same convention as the main project's `wake_probe`. Compare the timing of `button CCCD ok` and the first `NOTIFY h=0x0039` (if any) against the main project's logs.

## Outcomes to look for

- **Wake-press lands within ~5 ms of `button CCCD ok`** → main build was blocking it somehow. We dig into the application layer.
- **Wake-press never arrives, same as main build** → confirms Apple-side discard. Closes the investigation; we ship the warm-reconnect fix and document the limitation.
