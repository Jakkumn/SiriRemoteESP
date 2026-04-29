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
} event_action_t;

typedef struct {
    uint32_t double_click_max_ms;        // 0 disables DOUBLE_CLICK; CLICK fires instantly
    uint32_t hold_min_ms;                // 0 disables HOLD_START/HOLD_END
    int32_t  swipe_min_distance;         // minimum axis delta to count as a swipe
    int32_t  swipe_y_priority_threshold; // |dy| >= this forces vertical classification; 0 disables
} event_state_config_t;

typedef struct {
    event_action_t action;
    uint32_t now_ms;
    // Button events (CLICK, DOUBLE_CLICK, HOLD_START, HOLD_END):
    siri_button_bit_t button;
    uint32_t duration_ms;
    // Swipe events (SWIPE_*):
    int32_t distance;
} event_state_event_t;

typedef void (*event_state_emit_fn)(const event_state_event_t *evt, void *user);

typedef struct event_state event_state_t;

event_state_t *event_state_create(const event_state_config_t *cfg,
                                  event_state_emit_fn emit,
                                  void *user);
void event_state_destroy(event_state_t *es);

// Canonical string for an event_action_t — "click", "swipe_up", etc.
// Returns NULL for an unknown value. Both build paths use the same
// string set; HA event names depend on it.
const char *event_state_action_name(event_action_t action);

void event_state_feed_buttons(event_state_t *es, uint16_t buttons, uint32_t now_ms);
void event_state_feed_touch(event_state_t *es, const siri_touch_frame_t *frame, uint32_t now_ms);
void event_state_tick(event_state_t *es, uint32_t now_ms);
void event_state_reset(event_state_t *es, uint32_t now_ms);

// Runtime-tunable thresholds. Each writes one cfg field; the new value
// takes effect on the next event_state_feed_* call. Caller must hold the
// same mutex it uses around the feed/tick functions — there is no
// internal lock. Field assignment is atomic on Xtensa, so the lock is
// for ordering with concurrent reads, not tearing.
void event_state_set_swipe_y_priority(event_state_t *es, int32_t value);
void event_state_set_swipe_min_distance(event_state_t *es, int32_t value);
void event_state_set_double_click_max_ms(event_state_t *es, uint32_t value);
void event_state_set_hold_min_ms(event_state_t *es, uint32_t value);

#ifdef __cplusplus
}
#endif
