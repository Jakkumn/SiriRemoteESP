#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "button_pulse.h"

#define MAX_CAPTURED 256
static button_pulse_event_t captured[MAX_CAPTURED];
static size_t captured_count;

static void capture_cb(const button_pulse_event_t *evt, void *user)
{
    (void)user;
    assert(captured_count < MAX_CAPTURED);
    captured[captured_count++] = *evt;
}

static void reset_capture(void)
{
    captured_count = 0;
}

static button_pulse_t *make_bp(uint32_t interval_ms)
{
    button_pulse_config_t cfg = {
        .repeat_interval_ms = interval_ms,
    };
    button_pulse_t *bp = button_pulse_create(&cfg, capture_cb, NULL);
    assert(bp != NULL);
    reset_capture();
    return bp;
}

// Drive ticks at the 50 ms cadence both build paths use, so the tests exercise
// the same granularity the firmware actually has.
#define TICK_MS 50
static void run_ticks(button_pulse_t *bp, uint32_t from_ms, uint32_t to_ms)
{
    for (uint32_t t = from_ms + TICK_MS; t <= to_ms; t += TICK_MS) {
        button_pulse_tick(bp, t);
    }
}

static void test_press_emits_once(void)
{
    button_pulse_t *bp = make_bp(100);

    button_pulse_feed_buttons(bp, SIRI_BTN_VOL_UP, 0);
    assert(captured_count == 1);
    assert(captured[0].button == SIRI_BTN_VOL_UP);
    assert(captured[0].repeat == 0);
    assert(captured[0].held_ms == 0);

    // Released before the first interval elapses — the press is all there is.
    run_ticks(bp, 0, 50);
    button_pulse_feed_buttons(bp, 0, 60);
    run_ticks(bp, 60, 500);
    assert(captured_count == 1);

    button_pulse_destroy(bp);
}

static void test_release_emits_nothing(void)
{
    button_pulse_t *bp = make_bp(100);

    button_pulse_feed_buttons(bp, SIRI_BTN_SELECT, 0);
    run_ticks(bp, 0, 300);
    size_t before_release = captured_count;
    assert(before_release > 1);

    button_pulse_feed_buttons(bp, 0, 320);
    assert(captured_count == before_release);  // no release event

    run_ticks(bp, 320, 1000);
    assert(captured_count == before_release);  // and no further pulses

    button_pulse_destroy(bp);
}

static void test_hold_repeats_monotonic(void)
{
    button_pulse_t *bp = make_bp(100);

    button_pulse_feed_buttons(bp, SIRI_BTN_VOL_DOWN, 0);
    run_ticks(bp, 0, 1000);

    // Press at t=0 plus one pulse per 100 ms through t=1000.
    assert(captured_count == 11);
    for (size_t i = 0; i < captured_count; i++) {
        assert(captured[i].button == SIRI_BTN_VOL_DOWN);
        assert(captured[i].repeat == (uint32_t)i);
    }
    // held_ms strictly increases and tracks elapsed within one tick of slop.
    for (size_t i = 1; i < captured_count; i++) {
        assert(captured[i].held_ms > captured[i - 1].held_ms);
        uint32_t expected = (uint32_t)i * 100;
        uint32_t actual = captured[i].held_ms;
        uint32_t slop = actual > expected ? actual - expected : expected - actual;
        assert(slop <= TICK_MS);
    }

    button_pulse_destroy(bp);
}

static void test_two_buttons_independent(void)
{
    button_pulse_t *bp = make_bp(100);

    button_pulse_feed_buttons(bp, SIRI_BTN_VOL_UP, 0);
    run_ticks(bp, 0, 300);
    // Second button joins mid-hold; the bitmap now carries both bits.
    button_pulse_feed_buttons(bp, SIRI_BTN_VOL_UP | SIRI_BTN_BACK, 300);
    run_ticks(bp, 300, 600);

    uint32_t last_up_repeat = 0, last_back_repeat = 0;
    uint32_t last_up_held = 0, last_back_held = 0;
    for (size_t i = 0; i < captured_count; i++) {
        if (captured[i].button == SIRI_BTN_VOL_UP) {
            last_up_repeat = captured[i].repeat;
            last_up_held = captured[i].held_ms;
        } else if (captured[i].button == SIRI_BTN_BACK) {
            last_back_repeat = captured[i].repeat;
            last_back_held = captured[i].held_ms;
        } else {
            assert(0 && "unexpected button");
        }
    }
    // Counters and durations are per-button, not shared.
    assert(last_up_repeat == 6);
    assert(last_back_repeat == 3);
    assert(last_up_held == 600);
    assert(last_back_held == 300);

    button_pulse_destroy(bp);
}

static void test_two_bits_in_one_feed(void)
{
    button_pulse_t *bp = make_bp(100);

    // A single HID report can set several bits at once — the pairing combo
    // (Back + Vol Up) is exactly this shape.
    button_pulse_feed_buttons(bp, SIRI_BTN_BACK | SIRI_BTN_VOL_UP, 0);
    assert(captured_count == 2);
    assert(captured[0].repeat == 0 && captured[1].repeat == 0);
    assert(captured[0].button != captured[1].button);

    // And a single report can clear both again.
    reset_capture();
    button_pulse_feed_buttons(bp, 0, 40);
    assert(captured_count == 0);

    button_pulse_destroy(bp);
}

static void test_repress_restarts_repeat(void)
{
    button_pulse_t *bp = make_bp(100);

    button_pulse_feed_buttons(bp, SIRI_BTN_PLAY_PAUSE, 0);
    run_ticks(bp, 0, 400);
    button_pulse_feed_buttons(bp, 0, 420);

    reset_capture();
    button_pulse_feed_buttons(bp, SIRI_BTN_PLAY_PAUSE, 800);
    assert(captured_count == 1);
    assert(captured[0].repeat == 0);
    assert(captured[0].held_ms == 0);

    button_pulse_destroy(bp);
}

static void test_mic_never_repeats(void)
{
    button_pulse_t *bp = make_bp(100);

    button_pulse_feed_buttons(bp, SIRI_BTN_MIC, 0);
    run_ticks(bp, 0, 5000);

    // Mic drives voice capture; a held Mic must not put a pulse stream on the
    // same socket as the audio. The press still emits.
    assert(captured_count == 1);
    assert(captured[0].button == SIRI_BTN_MIC);
    assert(captured[0].repeat == 0);

    button_pulse_destroy(bp);
}

static void test_mic_does_not_suppress_others(void)
{
    button_pulse_t *bp = make_bp(100);

    button_pulse_feed_buttons(bp, SIRI_BTN_MIC | SIRI_BTN_VOL_UP, 0);
    run_ticks(bp, 0, 300);

    size_t mic_events = 0, vol_events = 0;
    for (size_t i = 0; i < captured_count; i++) {
        if (captured[i].button == SIRI_BTN_MIC) {
            mic_events++;
        } else if (captured[i].button == SIRI_BTN_VOL_UP) {
            vol_events++;
        }
    }
    assert(mic_events == 1);
    assert(vol_events == 4);  // press + three pulses

    button_pulse_destroy(bp);
}

static void test_reset_is_silent(void)
{
    button_pulse_t *bp = make_bp(100);

    button_pulse_feed_buttons(bp, SIRI_BTN_DOWN, 0);
    run_ticks(bp, 0, 300);
    reset_capture();

    // Called on BLE disconnect. No synthetic release — stopping is the signal.
    button_pulse_reset(bp);
    assert(captured_count == 0);

    run_ticks(bp, 300, 1000);
    assert(captured_count == 0);

    // Baseline is cleared too, so the same bitmap reads as a fresh press.
    button_pulse_feed_buttons(bp, SIRI_BTN_DOWN, 1000);
    assert(captured_count == 1);
    assert(captured[0].repeat == 0);

    button_pulse_destroy(bp);
}

static void test_zero_interval_disables_repeats(void)
{
    button_pulse_t *bp = make_bp(0);

    button_pulse_feed_buttons(bp, SIRI_BTN_TV, 0);
    assert(captured_count == 1);

    // Must not divide, spin, or emit once per tick.
    run_ticks(bp, 0, 5000);
    assert(captured_count == 1);

    button_pulse_destroy(bp);
}

static void test_interval_below_tick_emits_once_per_tick(void)
{
    // A sub-tick interval cannot be honoured; it degrades to one pulse per
    // tick rather than misbehaving.
    button_pulse_t *bp = make_bp(10);

    button_pulse_feed_buttons(bp, SIRI_BTN_RIGHT, 0);
    run_ticks(bp, 0, 500);
    assert(captured_count == 11);  // press + one per 50 ms tick

    button_pulse_destroy(bp);
}

static void test_now_ms_wraparound(void)
{
    button_pulse_t *bp = make_bp(100);

    // uint32 ms wraps every ~49.7 days and this bridge is never deliberately
    // disconnected, so a hold can straddle the rollover.
    const uint32_t start = UINT32_MAX - 150;
    button_pulse_feed_buttons(bp, SIRI_BTN_MUTE, start);
    assert(captured_count == 1);

    for (uint32_t i = 1; i <= 10; i++) {
        button_pulse_tick(bp, start + i * TICK_MS);  // wraps partway through
    }

    // No burst, no stall: two pulses in 500 ms at a 100 ms interval would be
    // 5, and each held_ms must stay small rather than reading ~4 billion.
    assert(captured_count == 6);
    for (size_t i = 1; i < captured_count; i++) {
        assert(captured[i].repeat == (uint32_t)i);
        assert(captured[i].held_ms == (uint32_t)i * 100);
    }

    button_pulse_destroy(bp);
}

static void test_idle_tick_emits_nothing(void)
{
    button_pulse_t *bp = make_bp(100);

    run_ticks(bp, 0, 2000);
    assert(captured_count == 0);

    // A repeated bitmap with no change is also a no-op.
    button_pulse_feed_buttons(bp, 0, 2000);
    assert(captured_count == 0);

    button_pulse_destroy(bp);
}

static void test_unchanged_bitmap_is_noop(void)
{
    button_pulse_t *bp = make_bp(100);

    button_pulse_feed_buttons(bp, SIRI_BTN_LEFT, 0);
    assert(captured_count == 1);

    // Re-feeding the same bitmap must not re-emit the press or reset held_ms.
    button_pulse_feed_buttons(bp, SIRI_BTN_LEFT, 20);
    button_pulse_feed_buttons(bp, SIRI_BTN_LEFT, 40);
    assert(captured_count == 1);

    run_ticks(bp, 0, 100);
    assert(captured_count == 2);
    assert(captured[1].held_ms == 100);  // measured from the original press

    button_pulse_destroy(bp);
}

static void test_null_safe(void)
{
    // Every entry point short-circuits on NULL — no assertion, just no crash.
    button_pulse_feed_buttons(NULL, SIRI_BTN_TV, 0);
    button_pulse_tick(NULL, 100);
    button_pulse_reset(NULL);
    button_pulse_destroy(NULL);
    assert(button_pulse_create(NULL, capture_cb, NULL) == NULL);
}

static void test_null_emit_callback(void)
{
    button_pulse_config_t cfg = {.repeat_interval_ms = 100};
    button_pulse_t *bp = button_pulse_create(&cfg, NULL, NULL);
    assert(bp != NULL);

    button_pulse_feed_buttons(bp, SIRI_BTN_UP, 0);
    button_pulse_tick(bp, 100);

    button_pulse_destroy(bp);
}

int main(void)
{
    test_press_emits_once();
    test_release_emits_nothing();
    test_hold_repeats_monotonic();
    test_two_buttons_independent();
    test_two_bits_in_one_feed();
    test_repress_restarts_repeat();
    test_mic_never_repeats();
    test_mic_does_not_suppress_others();
    test_reset_is_silent();
    test_zero_interval_disables_repeats();
    test_interval_below_tick_emits_once_per_tick();
    test_now_ms_wraparound();
    test_idle_tick_emits_nothing();
    test_unchanged_bitmap_is_noop();
    test_null_safe();
    test_null_emit_callback();
    printf("test_button_pulse: ok\n");
    return 0;
}
