#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "event_state.h"

#define MAX_CAPTURED 64
static event_state_event_t captured[MAX_CAPTURED];
static size_t captured_count;

static void capture_cb(const event_state_event_t *evt, void *user)
{
    (void)user;
    assert(captured_count < MAX_CAPTURED);
    captured[captured_count++] = *evt;
}

static void reset_capture(void) { captured_count = 0; }

static event_state_t *make_es(uint32_t double_ms, uint32_t hold_ms)
{
    event_state_config_t cfg = {
        .double_click_max_ms        = double_ms,
        .hold_min_ms                = hold_ms,
        .swipe_min_distance         = 40,
        .swipe_y_priority_threshold = 30,
    };
    event_state_t *es = event_state_create(&cfg, capture_cb, NULL);
    assert(es != NULL);
    reset_capture();
    return es;
}

static void test_single_click(void)
{
    event_state_t *es = make_es(300, 700);

    event_state_feed_buttons(es, SIRI_BTN_VOL_UP, 0);
    event_state_feed_buttons(es, 0, 100);
    // At t=100, release captured; CLICK emits after double window (300ms) expires.
    event_state_tick(es, 200);
    assert(captured_count == 0);
    event_state_tick(es, 400);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_CLICK);
    assert(captured[0].button == SIRI_BTN_VOL_UP);
    assert(captured[0].duration_ms == 100);

    event_state_destroy(es);
}

static void test_double_click(void)
{
    event_state_t *es = make_es(300, 700);

    event_state_feed_buttons(es, SIRI_BTN_SELECT, 0);
    event_state_feed_buttons(es, 0, 100);
    // Second press within the double window.
    event_state_feed_buttons(es, SIRI_BTN_SELECT, 200);
    event_state_feed_buttons(es, 0, 300);

    assert(captured_count == 1);
    assert(captured[0].action == EVT_DOUBLE_CLICK);
    assert(captured[0].button == SIRI_BTN_SELECT);
    // No lingering CLICK should emit after the window.
    event_state_tick(es, 1000);
    assert(captured_count == 1);

    event_state_destroy(es);
}

static void test_hold(void)
{
    event_state_t *es = make_es(300, 700);

    event_state_feed_buttons(es, SIRI_BTN_MIC, 0);
    event_state_tick(es, 500);
    assert(captured_count == 0);
    event_state_tick(es, 700);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_HOLD_START);
    assert(captured[0].button == SIRI_BTN_MIC);

    event_state_feed_buttons(es, 0, 1500);
    assert(captured_count == 2);
    assert(captured[1].action == EVT_HOLD_END);
    assert(captured[1].button == SIRI_BTN_MIC);
    assert(captured[1].duration_ms == 1500);

    // Tick past the double window — no stray CLICK.
    event_state_tick(es, 2500);
    assert(captured_count == 2);

    event_state_destroy(es);
}

static void test_mid_length_press(void)
{
    // A 650ms press (above the double window, below hold threshold) still
    // emits a CLICK — there's no "medium press" category.
    event_state_t *es = make_es(300, 700);

    event_state_feed_buttons(es, SIRI_BTN_PLAY_PAUSE, 0);
    event_state_feed_buttons(es, 0, 650);
    event_state_tick(es, 700);  // < 650+300, no emit yet
    assert(captured_count == 0);
    event_state_tick(es, 950);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_CLICK);
    assert(captured[0].duration_ms == 650);

    event_state_destroy(es);
}

static void test_chord_independent(void)
{
    event_state_t *es = make_es(300, 700);

    // Press Vol Up at t=0, press Vol Down at t=50 (chord), release both at t=150.
    event_state_feed_buttons(es, SIRI_BTN_VOL_UP, 0);
    event_state_feed_buttons(es, SIRI_BTN_VOL_UP | SIRI_BTN_VOL_DOWN, 50);
    event_state_feed_buttons(es, 0, 150);
    event_state_tick(es, 500);

    // Both buttons emit their own CLICK, independent durations.
    assert(captured_count == 2);
    bool saw_vu = false, saw_vd = false;
    for (size_t i = 0; i < captured_count; i++) {
        assert(captured[i].action == EVT_CLICK);
        if (captured[i].button == SIRI_BTN_VOL_UP) {
            assert(captured[i].duration_ms == 150);
            saw_vu = true;
        } else if (captured[i].button == SIRI_BTN_VOL_DOWN) {
            assert(captured[i].duration_ms == 100);
            saw_vd = true;
        }
    }
    assert(saw_vu && saw_vd);

    event_state_destroy(es);
}

static void test_double_click_disabled(void)
{
    // double_click_max_ms=0 → CLICK fires immediately, no delay.
    event_state_t *es = make_es(0, 700);

    event_state_feed_buttons(es, SIRI_BTN_BACK, 0);
    event_state_feed_buttons(es, 0, 100);
    // No tick needed — CLICK should be immediate.
    assert(captured_count == 1);
    assert(captured[0].action == EVT_CLICK);
    assert(captured[0].duration_ms == 100);

    event_state_destroy(es);
}

static void test_hold_disabled(void)
{
    // hold_min_ms=0 → no HOLD_START/HOLD_END; long press becomes a CLICK.
    event_state_t *es = make_es(300, 0);

    event_state_feed_buttons(es, SIRI_BTN_TV, 0);
    event_state_tick(es, 1000);
    event_state_tick(es, 2000);
    assert(captured_count == 0);  // no HOLD_START ever

    event_state_feed_buttons(es, 0, 3000);
    event_state_tick(es, 3400);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_CLICK);
    assert(captured[0].duration_ms == 3000);

    event_state_destroy(es);
}

static void test_click_then_hold(void)
{
    // Quick tap, then a second press that becomes a hold. The first click
    // should emit before the HOLD_START.
    event_state_t *es = make_es(300, 700);

    event_state_feed_buttons(es, SIRI_BTN_MUTE, 0);
    event_state_feed_buttons(es, 0, 100);
    event_state_feed_buttons(es, SIRI_BTN_MUTE, 200);  // second press, inside window
    event_state_tick(es, 900);  // now-200 = 700ms → hold crosses

    assert(captured_count == 2);
    assert(captured[0].action == EVT_CLICK);
    assert(captured[0].duration_ms == 100);
    assert(captured[1].action == EVT_HOLD_START);

    event_state_feed_buttons(es, 0, 1500);
    assert(captured_count == 3);
    assert(captured[2].action == EVT_HOLD_END);
    assert(captured[2].duration_ms == 1300);

    event_state_destroy(es);
}

static void test_reset_mid_hold(void)
{
    event_state_t *es = make_es(300, 700);

    event_state_feed_buttons(es, SIRI_BTN_POWER, 0);
    event_state_tick(es, 700);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_HOLD_START);

    // BLE disconnects while still held — reset should synthesize HOLD_END.
    event_state_reset(es, 1200);
    assert(captured_count == 2);
    assert(captured[1].action == EVT_HOLD_END);
    assert(captured[1].duration_ms == 1200);

    event_state_destroy(es);
}

static void test_reset_flushes_pending_click(void)
{
    event_state_t *es = make_es(300, 700);

    event_state_feed_buttons(es, SIRI_BTN_VOL_DOWN, 0);
    event_state_feed_buttons(es, 0, 100);
    // CLICK is pending (awaiting double window).
    assert(captured_count == 0);

    event_state_reset(es, 150);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_CLICK);
    assert(captured[0].duration_ms == 100);

    event_state_destroy(es);
}

static void test_swipe_up(void)
{
    event_state_t *es = make_es(300, 700);

    // Simulate a swipe: finger down at (100, -50), lifts at (110, 50) → dy > 0 = UP
    siri_touch_frame_t f = {.x = 100, .y = -50, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.x = 110; f.y = 0;
    event_state_feed_touch(es, &f, 50);
    f.x = 110; f.y = 50;
    event_state_feed_touch(es, &f, 100);
    // Finger lifts
    f.finger_down = false; f.pressure = 0;
    event_state_feed_touch(es, &f, 150);

    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_UP);
    assert(captured[0].distance == 100);  // |50 - (-50)|

    event_state_destroy(es);
}

static void test_swipe_down(void)
{
    event_state_t *es = make_es(300, 700);

    siri_touch_frame_t f = {.x = 100, .y = 80, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.y = -40;
    event_state_feed_touch(es, &f, 100);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 150);

    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_DOWN);
    assert(captured[0].distance == 120);

    event_state_destroy(es);
}

static void test_swipe_left_right(void)
{
    event_state_t *es = make_es(300, 700);

    // Positive dx = RIGHT (signed Cartesian: +X is right of center).
    siri_touch_frame_t f = {.x = 100, .y = 0, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.x = 300;
    event_state_feed_touch(es, &f, 100);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 150);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_RIGHT);
    assert(captured[0].distance == 200);

    reset_capture();

    // Negative dx = LEFT.
    siri_touch_frame_t g = {.x = 500, .y = 0, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &g, 200);
    g.x = 200;
    event_state_feed_touch(es, &g, 300);
    g.finger_down = false;
    event_state_feed_touch(es, &g, 350);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_LEFT);
    assert(captured[0].distance == 300);

    event_state_destroy(es);
}

static void test_y_priority_overrides_drifty_x(void)
{
    // Regression: a swipe-down with significant horizontal drift (|dx| > |dy|)
    // should still classify as DOWN as long as |dy| crosses the Y-priority
    // threshold (default 30). Without the bias, this swipe would misclassify
    // as swipe-left because positive dx wins by raw magnitude.
    event_state_t *es = make_es(300, 700);

    siri_touch_frame_t f = {.x = 100, .y = 50, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.x = 250; f.y = -10;  // dx=+150, dy=-60 — X dominant by raw magnitude
    event_state_feed_touch(es, &f, 100);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 150);

    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_DOWN);
    assert(captured[0].distance == 60);

    event_state_destroy(es);
}

static void test_y_priority_disabled_falls_back(void)
{
    // With the priority threshold = 0, classification reverts to plain
    // |dy| > |dx|, meaning the same gesture would now misclassify as swipe-left.
    event_state_config_t cfg = {
        .double_click_max_ms        = 300,
        .hold_min_ms                = 700,
        .swipe_min_distance         = 40,
        .swipe_y_priority_threshold = 0,  // disabled
    };
    event_state_t *es = event_state_create(&cfg, capture_cb, NULL);
    reset_capture();

    siri_touch_frame_t f = {.x = 100, .y = 50, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.x = 250; f.y = -10;  // dx=+150, dy=-60
    event_state_feed_touch(es, &f, 100);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 150);

    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_RIGHT);  // unbiased: +dx = right
    assert(captured[0].distance == 150);

    event_state_destroy(es);
}

static void test_micro_touch_below_threshold(void)
{
    event_state_t *es = make_es(300, 700);

    // Tiny travel — below the swipe_min_distance of 40.
    siri_touch_frame_t f = {.x = 100, .y = 0, .pressure = 30, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.x = 120; f.y = 10;
    event_state_feed_touch(es, &f, 50);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 100);

    assert(captured_count == 0);

    event_state_destroy(es);
}

static void test_diagonal_uses_larger_axis(void)
{
    // More Y than X → classify as vertical (UP/DOWN).
    event_state_t *es = make_es(300, 700);

    siri_touch_frame_t f = {.x = 100, .y = 0, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.x = 150; f.y = 120;  // dx = 50, dy = 120
    event_state_feed_touch(es, &f, 100);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 150);

    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_UP);
    assert(captured[0].distance == 120);

    event_state_destroy(es);
}

// --- Phase 3B.8 setter tests --------------------------------------------
//
// Setters mutate the cached cfg directly. Asserting via behavior change
// (post-setter input produces a different outcome than pre-setter input)
// is more robust than reaching into the private struct.

static void test_set_swipe_min_distance(void)
{
    event_state_t *es = make_es(300, 700);
    // Default min_distance from make_es is 40. A swipe of 50 should fire.
    siri_touch_frame_t f = {.x = 0, .y = 0, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.x = 50;
    event_state_feed_touch(es, &f, 50);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 60);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_RIGHT);

    // Raise threshold above the same swipe — second swipe must not fire.
    reset_capture();
    event_state_set_swipe_min_distance(es, 200);
    f = (siri_touch_frame_t){.x = 0, .y = 0, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 1000);
    f.x = 50;
    event_state_feed_touch(es, &f, 1050);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 1060);
    assert(captured_count == 0);
    event_state_destroy(es);
}

static void test_set_double_click_max_ms(void)
{
    event_state_t *es = make_es(300, 700);
    // Disable double-click via setter — second click should fire as a CLICK
    // immediately rather than waiting for the double-window.
    event_state_set_double_click_max_ms(es, 0);

    event_state_feed_buttons(es, 0x0001, 0);    // press TV
    event_state_feed_buttons(es, 0x0000, 50);   // release
    assert(captured_count == 1);
    assert(captured[0].action == EVT_CLICK);
    event_state_destroy(es);
}

static void test_set_hold_min_ms(void)
{
    event_state_t *es = make_es(300, 700);
    // Lower hold threshold via setter — a 200 ms press should now register
    // as HOLD_START → HOLD_END (with default 700 it wouldn't).
    event_state_set_hold_min_ms(es, 100);

    event_state_feed_buttons(es, 0x0001, 0);    // press
    event_state_tick(es, 150);                  // hold_min_ms=100 elapsed → HOLD_START
    event_state_feed_buttons(es, 0x0000, 200);  // release → HOLD_END
    // Expect: HOLD_START (at tick), HOLD_END (at release).
    assert(captured_count == 2);
    assert(captured[0].action == EVT_HOLD_START);
    assert(captured[1].action == EVT_HOLD_END);
    event_state_destroy(es);
}

static void test_set_swipe_y_priority(void)
{
    event_state_t *es = make_es(300, 700);
    // make_es starts with swipe_y_priority_threshold=30. With dy=35, dx=50,
    // priority is engaged so we'd classify vertical. Disable priority via
    // setter — same input now classifies horizontal (dx > dy).
    event_state_set_swipe_y_priority(es, 0);

    siri_touch_frame_t f = {.x = 0, .y = 0, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.x = 50; f.y = 35;
    event_state_feed_touch(es, &f, 100);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 150);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_RIGHT);
    event_state_destroy(es);
}

static void test_setters_null_safe(void)
{
    event_state_set_swipe_y_priority(NULL, 50);
    event_state_set_swipe_min_distance(NULL, 50);
    event_state_set_double_click_max_ms(NULL, 50);
    event_state_set_hold_min_ms(NULL, 50);
    // No assertion — just verify no crash. Setters must short-circuit on NULL.
}

int main(void)
{
    test_single_click();
    test_double_click();
    test_hold();
    test_mid_length_press();
    test_chord_independent();
    test_double_click_disabled();
    test_hold_disabled();
    test_click_then_hold();
    test_reset_mid_hold();
    test_reset_flushes_pending_click();
    test_swipe_up();
    test_swipe_down();
    test_swipe_left_right();
    test_y_priority_overrides_drifty_x();
    test_y_priority_disabled_falls_back();
    test_micro_touch_below_threshold();
    test_diagonal_uses_larger_axis();
    test_set_swipe_min_distance();
    test_set_double_click_max_ms();
    test_set_hold_min_ms();
    test_set_swipe_y_priority();
    test_setters_null_safe();
    printf("test_event_state: ok\n");
    return 0;
}
