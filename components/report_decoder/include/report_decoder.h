#pragma once

#include <stdint.h>
#include <stdbool.h>

// Button bitmap carried in a 2-byte HID report from handle 0x0039 on gen-3.
// Empirically verified — each bit maps to one physical control. The report
// is a 16-bit mask; multiple bits can be set simultaneously (chord presses).
typedef enum {
    // Byte 0
    SIRI_BTN_TV         = 0x0001,
    SIRI_BTN_VOL_UP     = 0x0002,
    SIRI_BTN_VOL_DOWN   = 0x0004,
    SIRI_BTN_SELECT     = 0x0008,  // clickpad center click
    SIRI_BTN_POWER      = 0x0010,
    SIRI_BTN_MIC        = 0x0020,  // Siri button
    SIRI_BTN_BACK       = 0x0040,
    SIRI_BTN_MUTE       = 0x0080,
    // Byte 1
    SIRI_BTN_PLAY_PAUSE = 0x0100,
    SIRI_BTN_UP         = 0x0200,  // clickpad directional click
    SIRI_BTN_RIGHT      = 0x0400,
    SIRI_BTN_DOWN       = 0x0800,
    SIRI_BTN_LEFT       = 0x1000,
} siri_button_bit_t;

bool siri_button_is_pressed(uint16_t buttons, siri_button_bit_t bit);
