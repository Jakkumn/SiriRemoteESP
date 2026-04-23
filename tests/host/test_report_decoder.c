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
    assert(!siri_button_is_pressed(0x00, SIRI_BTN_VOL_UP));
    assert(!siri_button_is_pressed(0x00, SIRI_BTN_MENU));
    assert(!siri_button_is_pressed(0x00, SIRI_BTN_TOUCH));
}

static void test_chord_press(void)
{
    uint8_t chord = (uint8_t)(SIRI_BTN_VOL_UP | SIRI_BTN_MENU);
    assert(siri_button_is_pressed(chord, SIRI_BTN_VOL_UP));
    assert(siri_button_is_pressed(chord, SIRI_BTN_MENU));
    assert(!siri_button_is_pressed(chord, SIRI_BTN_SIRI));
    assert(!siri_button_is_pressed(chord, SIRI_BTN_VOL_DOWN));
}

static void test_all_buttons_pressed(void)
{
    uint8_t all = 0xFF;
    assert(siri_button_is_pressed(all, SIRI_BTN_AIRPLAY));
    assert(siri_button_is_pressed(all, SIRI_BTN_POWER));
    assert(siri_button_is_pressed(all, SIRI_BTN_TOUCH));
}

int main(void)
{
    test_single_button_pressed();
    test_no_buttons_pressed();
    test_chord_press();
    test_all_buttons_pressed();
    printf("test_report_decoder: ok\n");
    return 0;
}
