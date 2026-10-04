#include "button_pulse.h"

#include <stdlib.h>
#include <string.h>

#define NUM_BUTTONS 13

// siri_button_bit_t values are single-bit masks; the button's index in our
// per-button state array is the bit position (0..12).
static const siri_button_bit_t INDEX_TO_BIT[NUM_BUTTONS] = {
    SIRI_BTN_TV,          // 0
    SIRI_BTN_VOL_UP,      // 1
    SIRI_BTN_VOL_DOWN,    // 2
    SIRI_BTN_SELECT,      // 3
    SIRI_BTN_POWER,       // 4
    SIRI_BTN_MIC,         // 5
    SIRI_BTN_BACK,        // 6
    SIRI_BTN_MUTE,        // 7
    SIRI_BTN_PLAY_PAUSE,  // 8
    SIRI_BTN_UP,          // 9
    SIRI_BTN_RIGHT,       // 10
    SIRI_BTN_DOWN,        // 11
    SIRI_BTN_LEFT,        // 12
};

typedef struct {
    // Explicit flag rather than treating held_since_ms == 0 as "idle": 0 is a
    // legitimate timestamp in the first millisecond after boot, and this device
    // is always-connected, so that window is reachable on a cold start.
    bool held;
    uint32_t held_since_ms;
    uint32_t last_pulse_ms;
    uint32_t repeat;
} per_button_t;

struct button_pulse {
    button_pulse_config_t cfg;
    button_pulse_emit_fn emit;
    void *user;
    per_button_t buttons[NUM_BUTTONS];
    uint16_t prev_buttons;
};

static void emit(const button_pulse_t *bp, const button_pulse_event_t *evt)
{
    if (bp->emit != NULL) {
        bp->emit(evt, bp->user);
    }
}

button_pulse_t *button_pulse_create(const button_pulse_config_t *cfg, button_pulse_emit_fn emit_fn,
                                    void *user)
{
    if (cfg == NULL) {
        return NULL;
    }
    button_pulse_t *bp = calloc(1, sizeof(*bp));
    if (bp == NULL) {
        return NULL;
    }
    bp->cfg = *cfg;
    bp->emit = emit_fn;
    bp->user = user;
    return bp;
}

void button_pulse_destroy(button_pulse_t *bp)
{
    free(bp);
}

void button_pulse_feed_buttons(button_pulse_t *bp, uint16_t buttons, uint32_t now_ms)
{
    if (bp == NULL) {
        return;
    }

    // The remote packs every control into one bitmap, so a single report can
    // change several bits at once. Diff rather than scanning for "the" button.
    uint16_t changed = (uint16_t)(buttons ^ bp->prev_buttons);
    if (changed == 0) {
        return;
    }

    for (int idx = 0; idx < NUM_BUTTONS; idx++) {
        siri_button_bit_t bit = INDEX_TO_BIT[idx];
        if ((changed & (uint16_t)bit) == 0) {
            continue;
        }
        per_button_t *b = &bp->buttons[idx];

        if ((buttons & (uint16_t)bit) != 0) {
            b->held = true;
            b->held_since_ms = now_ms;
            b->last_pulse_ms = now_ms;
            b->repeat = 0;
            button_pulse_event_t evt = {
                .button = bit,
                .repeat = 0,
                .held_ms = 0,
                .now_ms = now_ms,
            };
            emit(bp, &evt);
        } else {
            // Release emits nothing. Downstream infers it from the pulses
            // stopping, the same way an IR receiver times out on repeat frames.
            memset(b, 0, sizeof(*b));
        }
    }

    bp->prev_buttons = buttons;
}

void button_pulse_tick(button_pulse_t *bp, uint32_t now_ms)
{
    if (bp == NULL || bp->cfg.repeat_interval_ms == 0) {
        return;
    }

    for (int idx = 0; idx < NUM_BUTTONS; idx++) {
        per_button_t *b = &bp->buttons[idx];
        if (!b->held) {
            continue;
        }
        siri_button_bit_t bit = INDEX_TO_BIT[idx];

        // Mic drives the voice session (siri_audio_session_start/end) and is
        // already surfaced to HA as binary_sensor.voice_active. Pulsing it would
        // put a 10 Hz event stream on the same API socket that is carrying the
        // audio, while the Opus decoder owns CPU1 — all cost, no benefit. The
        // press itself still emits, so a Mic-press automation remains possible.
        if (bit == SIRI_BTN_MIC) {
            continue;
        }

        // Wrap-safe: now_ms is a uint32 millisecond counter that rolls over
        // every ~49.7 days, and this device is never deliberately disconnected.
        // Unsigned subtraction stays correct across the wrap; `now >= last +
        // interval` would not.
        if ((uint32_t)(now_ms - b->last_pulse_ms) < bp->cfg.repeat_interval_ms) {
            continue;
        }

        // Advance from `now`, not by += interval. A late tick should emit once
        // and resume, not fire a catch-up burst — a burst would spike whatever
        // the pulse drives (volume steps, scroll) long after the user expected.
        b->last_pulse_ms = now_ms;
        b->repeat++;

        button_pulse_event_t evt = {
            .button = bit,
            .repeat = b->repeat,
            .held_ms = (uint32_t)(now_ms - b->held_since_ms),
            .now_ms = now_ms,
        };
        emit(bp, &evt);
    }
}

void button_pulse_reset(button_pulse_t *bp)
{
    if (bp == NULL) {
        return;
    }
    // Deliberately silent. event_state used to synthesise a HOLD_END here; the
    // pulse model has no release event to synthesise, and stopping is already
    // the signal.
    memset(bp->buttons, 0, sizeof(bp->buttons));
    bp->prev_buttons = 0;
}
