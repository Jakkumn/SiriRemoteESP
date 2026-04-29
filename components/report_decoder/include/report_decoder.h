#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

// Parsed 11-byte touch/clickpad frame from handle 0x003D on gen-3.
typedef struct {
    int32_t  x;
    int32_t  y;
    uint8_t  pressure;
    bool     finger_down;
    uint32_t remote_counter;
} siri_touch_frame_t;

bool siri_button_is_pressed(uint16_t buttons, siri_button_bit_t bit);

// Convert a 2-byte HID report into a 16-bit button bitmap. Returns 0 if len < 2.
uint16_t siri_decode_button_bytes(const uint8_t *data, size_t len);

// Parse an 11-byte touch frame. Returns false if len != 11 or byte 0 != 0x32.
bool siri_decode_touch_frame(const uint8_t *data, size_t len, siri_touch_frame_t *out);

// Enum → string for JSON serialization: "volume_up", "play_pause", etc.
// Returns NULL for an unknown or zero value.
const char *siri_button_name(siri_button_bit_t bit);

#ifdef __cplusplus
}
#endif
