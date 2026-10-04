#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "report_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

// IR-style button pulse emitter. Replaces the gesture state machine that used
// to live in components/event_state/.
//
// The model is NEC IR's repeat frame: while a button is held, emit the same
// event every `repeat_interval_ms`, and let the receiver (Home Assistant)
// decide what holding means. There is deliberately NO release event — release
// is inferred downstream from the pulses stopping, exactly as an IR receiver
// infers it from the absence of repeat frames.
//
// Nothing here classifies input. `repeat` and `held_ms` are measurements;
// every judgment built from them ("is 220 ms a click or a hold?") belongs in
// HA. Adding a threshold to this file is almost certainly a design mistake —
// see PULSE_REFACTOR_PLAN.md.
//
// No internal locking: callers hold their own mutex around every feed/tick
// pair, because feeds arrive on the NimBLE host task and ticks on a timer task.

typedef struct {
    // How often a held button re-emits. A rate, not a policy. Bounded below by
    // the caller's tick period (50 ms in both build paths) — a shorter interval
    // cannot be honoured and simply emits once per tick.
    //
    // 0 disables repeats entirely: the press still emits with repeat == 0, but
    // nothing follows it. Matches the "0 disables this feature" convention the
    // old event_state config used.
    uint32_t repeat_interval_ms;
} button_pulse_config_t;

typedef struct {
    siri_button_bit_t button;
    uint32_t repeat;   // 0 = the press itself; increments once per interval
    uint32_t held_ms;  // 0 on the press; measured elapsed hold on repeats
    uint32_t now_ms;   // emission timestamp, as supplied by the caller
} button_pulse_event_t;

typedef void (*button_pulse_emit_fn)(const button_pulse_event_t *evt, void *user);

typedef struct button_pulse button_pulse_t;

button_pulse_t *button_pulse_create(const button_pulse_config_t *cfg, button_pulse_emit_fn emit,
                                    void *user);
void button_pulse_destroy(button_pulse_t *bp);

// Feed a raw 16-bit button bitmap (handle 0x0039). Newly-pressed bits emit
// immediately with repeat == 0; newly-released bits go quiet with no event.
void button_pulse_feed_buttons(button_pulse_t *bp, uint16_t buttons, uint32_t now_ms);

// Drive the repeat stream. Must be called periodically — repeats exist only
// because of this, since the remote sends HID reports on change, not while held.
void button_pulse_tick(button_pulse_t *bp, uint32_t now_ms);

// Drop all held state without emitting. Called on BLE disconnect: the pulses
// simply stop, which is already how release is signalled.
void button_pulse_reset(button_pulse_t *bp);

#ifdef __cplusplus
}
#endif
