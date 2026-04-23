#pragma once

#include <stdint.h>
#include <stdbool.h>

// Button bitmask bits carried in byte 1 of every HID report from handle 0x0023.
// Values documented in the Yanndroid/SiriRemote-Linux repo; gen-3 mappings
// still to be verified empirically (see siri-remote-ha-bridge.md).
typedef enum {
    SIRI_BTN_AIRPLAY    = 0x01,
    SIRI_BTN_VOL_UP     = 0x02,
    SIRI_BTN_VOL_DOWN   = 0x04,
    SIRI_BTN_PLAY_PAUSE = 0x08,
    SIRI_BTN_SIRI       = 0x10,
    SIRI_BTN_MENU       = 0x20,
    SIRI_BTN_POWER      = 0x40,
    SIRI_BTN_TOUCH      = 0x80,
} siri_button_bit_t;

bool siri_button_is_pressed(uint8_t buttons, siri_button_bit_t bit);
