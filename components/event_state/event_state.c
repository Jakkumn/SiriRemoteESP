#include "event_state.h"

#include <stdlib.h>
#include <string.h>

#define NUM_BUTTONS 13

// siri_button_bit_t values are single-bit masks; the button's index in our
// per-button state array is the bit position (0..12).
static const siri_button_bit_t INDEX_TO_BIT[NUM_BUTTONS] = {
    SIRI_BTN_TV,         // 0
    SIRI_BTN_VOL_UP,     // 1
    SIRI_BTN_VOL_DOWN,   // 2
    SIRI_BTN_SELECT,     // 3
    SIRI_BTN_POWER,      // 4
    SIRI_BTN_MIC,        // 5
    SIRI_BTN_BACK,       // 6
    SIRI_BTN_MUTE,       // 7
    SIRI_BTN_PLAY_PAUSE, // 8
    SIRI_BTN_UP,         // 9
    SIRI_BTN_RIGHT,      // 10
    SIRI_BTN_DOWN,       // 11
    SIRI_BTN_LEFT,       // 12
};

typedef enum {
    BTN_IDLE,
    BTN_PRESSED,
    BTN_HELD,
} button_state_t;

typedef struct {
    button_state_t state;
    uint32_t       press_started_ms;
    // A CLICK pending emission — we're waiting out the double-click window.
    // When non-zero, a click at `pending_click_duration_ms` duration is scheduled
    // to emit at `pending_emit_at_ms + double_click_max_ms` unless a second press
    // lands first (in which case this becomes a DOUBLE_CLICK).
    uint32_t       pending_emit_at_ms;
    uint32_t       pending_click_duration_ms;
    // Set when the current press is the second of a potential double.
    bool           second_press_in_progress;
} per_button_t;

typedef struct {
    bool     active;
    int32_t  x_start, y_start;
    int32_t  x_last, y_last;
    uint32_t start_ms;
} touch_state_t;

struct event_state {
    event_state_config_t  cfg;
    event_state_emit_fn   emit;
    void                 *user;
    per_button_t          buttons[NUM_BUTTONS];
    touch_state_t         touch;
    uint16_t              prev_buttons;
};

static void emit(const event_state_t *es, const event_state_event_t *evt)
{
    if (es->emit != NULL) {
        es->emit(evt, es->user);
    }
}

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

event_state_t *event_state_create(const event_state_config_t *cfg,
                                  event_state_emit_fn emit_fn,
                                  void *user)
{
    if (cfg == NULL) {
        return NULL;
    }
    event_state_t *es = calloc(1, sizeof(*es));
    if (es == NULL) {
        return NULL;
    }
    es->cfg  = *cfg;
    es->emit = emit_fn;
    es->user = user;
    return es;
}

void event_state_destroy(event_state_t *es)
{
    free(es);
}

static void emit_pending_click(event_state_t *es, per_button_t *b,
                               siri_button_bit_t bit, uint32_t now_ms)
{
    event_state_event_t evt = {
        .action       = EVT_CLICK,
        .now_ms       = now_ms,
        .button       = bit,
        .duration_ms  = b->pending_click_duration_ms,
    };
    emit(es, &evt);
    b->pending_emit_at_ms        = 0;
    b->pending_click_duration_ms = 0;
}

static void handle_press(event_state_t *es, int idx, uint32_t now_ms)
{
    per_button_t *b = &es->buttons[idx];

    if (b->pending_emit_at_ms != 0) {
        // Second press arrived inside the double-click window: cancel the
        // scheduled CLICK (we'll treat this press as the second of a double).
        b->second_press_in_progress = true;
        b->pending_emit_at_ms       = 0;
        // Keep pending_click_duration_ms in case this second press turns into
        // a HOLD — in that case we retroactively emit the first CLICK before
        // the HOLD_START.
    } else {
        b->second_press_in_progress = false;
    }
    b->state            = BTN_PRESSED;
    b->press_started_ms = now_ms;
}

static void handle_release(event_state_t *es, int idx, uint32_t now_ms)
{
    per_button_t *b = &es->buttons[idx];
    siri_button_bit_t bit = INDEX_TO_BIT[idx];
    uint32_t duration = now_ms - b->press_started_ms;

    if (b->state == BTN_HELD) {
        event_state_event_t evt = {
            .action      = EVT_HOLD_END,
            .now_ms      = now_ms,
            .button      = bit,
            .duration_ms = duration,
        };
        emit(es, &evt);
        b->second_press_in_progress = false;
    } else if (b->state == BTN_PRESSED) {
        if (b->second_press_in_progress) {
            event_state_event_t evt = {
                .action = EVT_DOUBLE_CLICK,
                .now_ms = now_ms,
                .button = bit,
            };
            emit(es, &evt);
            b->second_press_in_progress = false;
        } else if (es->cfg.double_click_max_ms > 0) {
            b->pending_emit_at_ms        = now_ms;
            b->pending_click_duration_ms = duration;
        } else {
            event_state_event_t evt = {
                .action      = EVT_CLICK,
                .now_ms      = now_ms,
                .button      = bit,
                .duration_ms = duration,
            };
            emit(es, &evt);
        }
    }
    b->state = BTN_IDLE;
}

void event_state_feed_buttons(event_state_t *es, uint16_t buttons, uint32_t now_ms)
{
    if (es == NULL) {
        return;
    }
    uint16_t rising  = buttons & ~es->prev_buttons;
    uint16_t falling = ~buttons & es->prev_buttons;
    es->prev_buttons = buttons;

    uint16_t r = rising;
    while (r) {
        int idx = __builtin_ctz(r);
        r &= (uint16_t)(r - 1);
        if (idx < NUM_BUTTONS) {
            handle_press(es, idx, now_ms);
        }
    }
    uint16_t f = falling;
    while (f) {
        int idx = __builtin_ctz(f);
        f &= (uint16_t)(f - 1);
        if (idx < NUM_BUTTONS) {
            handle_release(es, idx, now_ms);
        }
    }
}

void event_state_feed_touch(event_state_t *es, const siri_touch_frame_t *frame, uint32_t now_ms)
{
    if (es == NULL || frame == NULL) {
        return;
    }
    touch_state_t *t = &es->touch;

    if (frame->finger_down) {
        if (!t->active) {
            t->active   = true;
            t->x_start  = frame->x;
            t->y_start  = frame->y;
            t->start_ms = now_ms;
        }
        t->x_last = frame->x;
        t->y_last = frame->y;
        return;
    }

    if (!t->active) {
        return;
    }
    t->active = false;

    int32_t dx = t->x_last - t->x_start;
    int32_t dy = t->y_last - t->y_start;
    int32_t abs_dx = abs32(dx);
    int32_t abs_dy = abs32(dy);
    int32_t magnitude = abs_dx > abs_dy ? abs_dx : abs_dy;

    if (magnitude < es->cfg.swipe_min_distance) {
        return;
    }

    event_state_event_t evt = {
        .now_ms   = now_ms,
        .distance = magnitude,
    };
    if (abs_dy > abs_dx) {
        // Y axis: positive = up on the touchpad.
        evt.action = dy > 0 ? EVT_SWIPE_UP : EVT_SWIPE_DOWN;
    } else {
        // X axis: positive = left on the touchpad (observed from gen-3 fixtures).
        evt.action = dx > 0 ? EVT_SWIPE_LEFT : EVT_SWIPE_RIGHT;
    }
    emit(es, &evt);
}

void event_state_feed_connect(event_state_t *es, uint32_t idle_ms, uint32_t now_ms)
{
    if (es == NULL || es->cfg.pickup_idle_threshold_ms == 0) {
        return;
    }
    if (idle_ms < es->cfg.pickup_idle_threshold_ms) {
        return;
    }
    event_state_event_t evt = {
        .action           = EVT_PICKUP,
        .now_ms           = now_ms,
        .idle_duration_ms = idle_ms,
    };
    emit(es, &evt);
}

void event_state_tick(event_state_t *es, uint32_t now_ms)
{
    if (es == NULL) {
        return;
    }

    for (int idx = 0; idx < NUM_BUTTONS; idx++) {
        per_button_t *b = &es->buttons[idx];
        siri_button_bit_t bit = INDEX_TO_BIT[idx];

        // PRESSED crossing the hold threshold → HOLD_START
        if (b->state == BTN_PRESSED && es->cfg.hold_min_ms > 0 &&
            (now_ms - b->press_started_ms) >= es->cfg.hold_min_ms) {
            if (b->second_press_in_progress && b->pending_click_duration_ms > 0) {
                // The first press was a quick tap; the second became a hold.
                // Emit the deferred CLICK so the sequence is visible.
                emit_pending_click(es, b, bit, now_ms);
            }
            b->second_press_in_progress = false;
            event_state_event_t evt = {
                .action = EVT_HOLD_START,
                .now_ms = now_ms,
                .button = bit,
            };
            emit(es, &evt);
            b->state = BTN_HELD;
        }

        // IDLE with pending CLICK past the double-click window → emit CLICK
        if (b->state == BTN_IDLE && b->pending_emit_at_ms != 0 &&
            (now_ms - b->pending_emit_at_ms) >= es->cfg.double_click_max_ms) {
            emit_pending_click(es, b, bit, now_ms);
        }
    }
}

void event_state_reset(event_state_t *es, uint32_t now_ms)
{
    if (es == NULL) {
        return;
    }
    for (int idx = 0; idx < NUM_BUTTONS; idx++) {
        per_button_t *b = &es->buttons[idx];
        siri_button_bit_t bit = INDEX_TO_BIT[idx];

        if (b->state == BTN_HELD) {
            event_state_event_t evt = {
                .action      = EVT_HOLD_END,
                .now_ms      = now_ms,
                .button      = bit,
                .duration_ms = now_ms - b->press_started_ms,
            };
            emit(es, &evt);
        }
        if (b->pending_emit_at_ms != 0) {
            emit_pending_click(es, b, bit, now_ms);
        }
        memset(b, 0, sizeof(*b));
    }
    memset(&es->touch, 0, sizeof(es->touch));
    es->prev_buttons = 0;
}
