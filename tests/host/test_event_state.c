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
        .double_click_max_ms       = double_ms,
        .hold_min_ms               = hold_ms,
        .swipe_min_distance        = 40,
        .pickup_idle_threshold_ms  = 30000,
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

static void test_pickup_below_threshold(void)
{
    event_state_t *es = make_es(300, 700);

    event_state_feed_connect(es, 10000, 0);  // 10s idle, below default 30s
    assert(captured_count == 0);

    event_state_destroy(es);
}

static void test_pickup_above_threshold(void)
{
    event_state_t *es = make_es(300, 700);

    event_state_feed_connect(es, 60000, 0);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_PICKUP);
    assert(captured[0].idle_duration_ms == 60000);

    event_state_destroy(es);
}

static void test_pickup_zero_idle(void)
{
    // First boot: idle_ms=0 → no pickup (threshold is 30s).
    event_state_t *es = make_es(300, 700);

    event_state_feed_connect(es, 0, 0);
    assert(captured_count == 0);

    event_state_destroy(es);
}

static void test_pickup_disabled(void)
{
    event_state_config_t cfg = {
        .double_click_max_ms      = 300,
        .hold_min_ms              = 700,
        .swipe_min_distance       = 40,
        .pickup_idle_threshold_ms = 0,  // disabled
    };
    event_state_t *es = event_state_create(&cfg, capture_cb, NULL);
    reset_capture();

    event_state_feed_connect(es, 1000000, 0);
    assert(captured_count == 0);

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

    // Positive dx = LEFT (per gen-3 empirical observation).
    siri_touch_frame_t f = {.x = 100, .y = 0, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &f, 0);
    f.x = 300;
    event_state_feed_touch(es, &f, 100);
    f.finger_down = false;
    event_state_feed_touch(es, &f, 150);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_LEFT);
    assert(captured[0].distance == 200);

    reset_capture();

    // Negative dx = RIGHT.
    siri_touch_frame_t g = {.x = 500, .y = 0, .pressure = 50, .finger_down = true};
    event_state_feed_touch(es, &g, 200);
    g.x = 200;
    event_state_feed_touch(es, &g, 300);
    g.finger_down = false;
    event_state_feed_touch(es, &g, 350);
    assert(captured_count == 1);
    assert(captured[0].action == EVT_SWIPE_RIGHT);
    assert(captured[0].distance == 300);

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
    test_pickup_below_threshold();
    test_pickup_above_threshold();
    test_pickup_zero_idle();
    test_pickup_disabled();
    test_swipe_up();
    test_swipe_down();
    test_swipe_left_right();
    test_micro_touch_below_threshold();
    test_diagonal_uses_larger_axis();
    printf("test_event_state: ok\n");
    return 0;
}
