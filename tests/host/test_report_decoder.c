#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
    uint16_t chord = (uint16_t)(SIRI_BTN_BACK | SIRI_BTN_PLAY_PAUSE);
    assert(siri_button_is_pressed(chord, SIRI_BTN_BACK));
    assert(siri_button_is_pressed(chord, SIRI_BTN_PLAY_PAUSE));
    assert(!siri_button_is_pressed(chord, SIRI_BTN_MIC));
}

static void test_all_bits_set(void)
{
    uint16_t all = 0xFFFF;
    assert(siri_button_is_pressed(all, SIRI_BTN_TV));
    assert(siri_button_is_pressed(all, SIRI_BTN_POWER));
    assert(siri_button_is_pressed(all, SIRI_BTN_LEFT));
}

// --- siri_decode_button_bytes ---

static void test_decode_button_single(void)
{
    // Vol Up press: 0x02 0x00 on the wire.
    uint8_t wire[2] = {0x02, 0x00};
    uint16_t btns = siri_decode_button_bytes(wire, 2);
    assert(btns == SIRI_BTN_VOL_UP);
}

static void test_decode_button_byte1(void)
{
    // Play/Pause press: 0x00 0x01.
    uint8_t wire[2] = {0x00, 0x01};
    uint16_t btns = siri_decode_button_bytes(wire, 2);
    assert(btns == SIRI_BTN_PLAY_PAUSE);
}

static void test_decode_button_chord(void)
{
    // Vol Up + Up (clickpad): byte 0 = 0x02, byte 1 = 0x02.
    uint8_t wire[2] = {0x02, 0x02};
    uint16_t btns = siri_decode_button_bytes(wire, 2);
    assert((btns & SIRI_BTN_VOL_UP) != 0);
    assert((btns & SIRI_BTN_UP) != 0);
}

static void test_decode_button_short_input(void)
{
    uint8_t wire[1] = {0x02};
    assert(siri_decode_button_bytes(wire, 1) == 0);
    assert(siri_decode_button_bytes(NULL, 5) == 0);
}

// --- siri_decode_touch_frame ---

static void test_decode_touch_frame_valid(void)
{
    // First frame of the captured swipe_up fixture:
    // 32 f2 76 01 51 de c0 51 1e 06 eb
    uint8_t frame[11] = {
        0x32, 0xf2, 0x76, 0x01, 0x51, 0xde, 0xc0, 0x51, 0x1e, 0x06, 0xeb};
    siri_touch_frame_t t;
    assert(siri_decode_touch_frame(frame, 11, &t));
    // X = 0x51 | (0xde & 0x07) << 8 = 81 | 1536 = 1617... actually (0xde & 0x07)=6
    // 0x51 | (6 << 8) = 81 | 1536 = 1617
    assert(t.x == (int32_t)(0x51 | ((0xde & 0x07) << 8)));
    // Y = (int8_t)0xc0 = -64
    assert(t.y == -64);
    // Bytes 7,8 = 0x51, 0x1e (non-zero) → finger down
    assert(t.finger_down);
    assert(t.pressure == 0x06);
    // Counter: f2 76 01 (LE) = 1 << 16 | 0x76 << 8 | 0xf2 = 96498
    assert(t.remote_counter == (uint32_t)(0xf2 | (0x76 << 8) | (0x01 << 16)));
}

static void test_decode_touch_frame_finger_up(void)
{
    // Last frame of swipe_up fixture: 32 ae 7a 00 f6 1e 0a 00 00 00 e8
    uint8_t frame[11] = {
        0x32, 0xae, 0x7a, 0x00, 0xf6, 0x1e, 0x0a, 0x00, 0x00, 0x00, 0xe8};
    siri_touch_frame_t t;
    assert(siri_decode_touch_frame(frame, 11, &t));
    assert(t.y == 10);
    assert(!t.finger_down);
    assert(t.pressure == 0);
}

static void test_decode_touch_frame_rejects_bad_input(void)
{
    siri_touch_frame_t t;
    uint8_t wrong_len[10] = {0x32, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert(!siri_decode_touch_frame(wrong_len, 10, &t));
    uint8_t wrong_magic[11] = {0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert(!siri_decode_touch_frame(wrong_magic, 11, &t));
    uint8_t good[11] = {0x32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert(!siri_decode_touch_frame(NULL, 11, &t));
    assert(!siri_decode_touch_frame(good, 11, NULL));
}

// --- siri_button_name ---

static void test_button_name_returns_strings(void)
{
    assert(strcmp(siri_button_name(SIRI_BTN_TV), "tv") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_VOL_UP), "volume_up") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_VOL_DOWN), "volume_down") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_SELECT), "select") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_POWER), "power") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_MIC), "mic") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_BACK), "back") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_MUTE), "mute") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_PLAY_PAUSE), "play_pause") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_UP), "up") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_RIGHT), "right") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_DOWN), "down") == 0);
    assert(strcmp(siri_button_name(SIRI_BTN_LEFT), "left") == 0);
}

static void test_button_name_returns_null_for_unknown(void)
{
    assert(siri_button_name(0) == NULL);
    assert(siri_button_name(0x2000) == NULL);
}

int main(void)
{
    test_single_button_pressed();
    test_no_buttons_pressed();
    test_chord_across_bytes();
    test_all_bits_set();

    test_decode_button_single();
    test_decode_button_byte1();
    test_decode_button_chord();
    test_decode_button_short_input();

    test_decode_touch_frame_valid();
    test_decode_touch_frame_finger_up();
    test_decode_touch_frame_rejects_bad_input();

    test_button_name_returns_strings();
    test_button_name_returns_null_for_unknown();

    printf("test_report_decoder: ok\n");
    return 0;
}
