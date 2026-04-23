#include "report_decoder.h"

bool siri_button_is_pressed(uint8_t buttons, siri_button_bit_t bit)
{
    return (buttons & (uint8_t)bit) != 0;
}
