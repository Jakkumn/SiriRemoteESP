#include "report_decoder.h"

bool siri_button_is_pressed(uint16_t buttons, siri_button_bit_t bit)
{
    return (buttons & (uint16_t)bit) != 0;
}

uint16_t siri_decode_button_bytes(const uint8_t *data, size_t len)
{
    if (data == NULL || len < 2) {
        return 0;
    }
    return (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
}

bool siri_decode_touch_frame(const uint8_t *data, size_t len, siri_touch_frame_t *out)
{
    if (data == NULL || out == NULL || len != 11 || data[0] != 0x32) {
        return false;
    }
    // X encoding (gen-1-derived; verified against gen-3 fixtures during Phase 2).
    // The low 3 bits of byte 5 extend byte 4 to form a ~11-bit value.
    out->x = (int32_t)(data[4] | ((uint32_t)(data[5] & 0x07) << 8));
    // Y is a signed 8-bit value. Positive = upward on the touchpad.
    out->y = (int32_t)(int8_t)data[6];
    // Bytes 7-8 go to zero when the finger lifts.
    out->finger_down    = !(data[7] == 0 && data[8] == 0);
    out->pressure       = data[9];
    out->remote_counter = (uint32_t)data[1]
                        | ((uint32_t)data[2] << 8)
                        | ((uint32_t)data[3] << 16);
    return true;
}

const char *siri_button_name(siri_button_bit_t bit)
{
    switch (bit) {
    case SIRI_BTN_TV:         return "tv";
    case SIRI_BTN_VOL_UP:     return "volume_up";
    case SIRI_BTN_VOL_DOWN:   return "volume_down";
    case SIRI_BTN_SELECT:     return "select";
    case SIRI_BTN_POWER:      return "power";
    case SIRI_BTN_MIC:        return "mic";
    case SIRI_BTN_BACK:       return "back";
    case SIRI_BTN_MUTE:       return "mute";
    case SIRI_BTN_PLAY_PAUSE: return "play_pause";
    case SIRI_BTN_UP:         return "up";
    case SIRI_BTN_RIGHT:      return "right";
    case SIRI_BTN_DOWN:       return "down";
    case SIRI_BTN_LEFT:       return "left";
    }
    return NULL;
}
