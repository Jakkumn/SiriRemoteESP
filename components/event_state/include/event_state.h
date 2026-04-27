#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "report_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EVT_CLICK,
    EVT_DOUBLE_CLICK,
    EVT_HOLD_START,
    EVT_HOLD_END,
    EVT_SWIPE_UP,
    EVT_SWIPE_DOWN,
    EVT_SWIPE_LEFT,
    EVT_SWIPE_RIGHT,
    EVT_PICKUP,
} event_action_t;

typedef struct {
    uint32_t double_click_max_ms;        // 0 disables DOUBLE_CLICK; CLICK fires instantly
    uint32_t hold_min_ms;                // 0 disables HOLD_START/HOLD_END
    int32_t  swipe_min_distance;         // minimum axis delta to count as a swipe
    int32_t  swipe_y_priority_threshold; // |dy| >= this forces vertical classification; 0 disables
    uint32_t pickup_idle_threshold_ms;   // 0 disables PICKUP
} event_state_config_t;

typedef struct {
    event_action_t action;
    uint32_t now_ms;
    // Button events (CLICK, DOUBLE_CLICK, HOLD_START, HOLD_END):
    siri_button_bit_t button;
    uint32_t duration_ms;
    // Swipe events (SWIPE_*):
    int32_t distance;
    // Pickup event:
    uint32_t idle_duration_ms;
} event_state_event_t;

typedef void (*event_state_emit_fn)(const event_state_event_t *evt, void *user);

typedef struct event_state event_state_t;

event_state_t *event_state_create(const event_state_config_t *cfg,
                                  event_state_emit_fn emit,
                                  void *user);
void event_state_destroy(event_state_t *es);

void event_state_feed_buttons(event_state_t *es, uint16_t buttons, uint32_t now_ms);
void event_state_feed_touch(event_state_t *es, const siri_touch_frame_t *frame, uint32_t now_ms);
void event_state_feed_connect(event_state_t *es, uint32_t idle_ms, uint32_t now_ms);
void event_state_tick(event_state_t *es, uint32_t now_ms);
void event_state_reset(event_state_t *es, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
