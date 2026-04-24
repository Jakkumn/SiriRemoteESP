#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "report_decoder.h"

static void test_single_button_pressed(void)
{
    assert(siri_button_is_pressed(SIRI_BTN_VOL_UP, SIRI_BTN_VOL_UP));
    assert(!siri_button_is_pressed(SIRI_BTN_VOL_UP, SIRI_BTN_VOL_DOWN));
}

static void test_no_buttons_pressed(void)
{
    assert(!siri_button_is_pressed(0x0000, SIRI_BTN_VOL_UP));
    assert(!siri_button_is_pressed(0x0000, SIRI_BTN_BACK));
    assert(!siri_button_is_pressed(0x0000, SIRI_BTN_PLAY_PAUSE));
}

static void test_chord_across_bytes(void)
{
    // Back (byte 0) + Play/Pause (byte 1) simultaneously.
    uint16_t chord = (uint16_t)(SIRI_BTN_BACK | SIRI_BTN_PLAY_PAUSE);
    assert(siri_button_is_pressed(chord, SIRI_BTN_BACK));
    assert(siri_button_is_pressed(chord, SIRI_BTN_PLAY_PAUSE));
    assert(!siri_button_is_pressed(chord, SIRI_BTN_MIC));
    assert(!siri_button_is_pressed(chord, SIRI_BTN_VOL_DOWN));
}

static void test_all_bits_set(void)
{
    uint16_t all = 0xFFFF;
    assert(siri_button_is_pressed(all, SIRI_BTN_TV));
    assert(siri_button_is_pressed(all, SIRI_BTN_POWER));
    assert(siri_button_is_pressed(all, SIRI_BTN_MUTE));
    assert(siri_button_is_pressed(all, SIRI_BTN_LEFT));
}

// Captured from a real gen-3 remote: `0x02 0x00` on handle 0x0039
// corresponds to Vol Up press. Decoded from the 2-byte wire format as
// low byte in bits 0..7 and high byte in bits 8..15 of a uint16_t.
static void test_wire_format_vol_up(void)
{
    uint8_t wire[2] = {0x02, 0x00};
    uint16_t buttons = wire[0] | ((uint16_t)wire[1] << 8);
    assert(siri_button_is_pressed(buttons, SIRI_BTN_VOL_UP));
    assert(!siri_button_is_pressed(buttons, SIRI_BTN_VOL_DOWN));
}

// Captured gen-3: Play/Pause press is `0x00 0x01` — confirms byte 1 bit 0.
static void test_wire_format_play_pause(void)
{
    uint8_t wire[2] = {0x00, 0x01};
    uint16_t buttons = wire[0] | ((uint16_t)wire[1] << 8);
    assert(siri_button_is_pressed(buttons, SIRI_BTN_PLAY_PAUSE));
    assert(!siri_button_is_pressed(buttons, SIRI_BTN_TV));
}

int main(void)
{
    test_single_button_pressed();
    test_no_buttons_pressed();
    test_chord_across_bytes();
    test_all_bits_set();
    test_wire_format_vol_up();
    test_wire_format_play_pause();
    printf("test_report_decoder: ok\n");
    return 0;
}
