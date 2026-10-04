#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include <drivers/sensor/mpr121.h>
#include <zmk/events/mpr121_events.h>

LOG_MODULE_REGISTER(mpr121_subsystem, CONFIG_MPR121_SUBSYSTEM_LOG_LEVEL);

#define TOUCH_THRESHOLD      CONFIG_MPR121_TOUCH_THRESHOLD
#define DEADZONE             (CONFIG_MPR121_DEADZONE / 10.0f)
#define SWIPE_THRESHOLD      (CONFIG_MPR121_SWIPE_THRESHOLD / 10.0f)
#define TAP_MAX_DURATION_MS  CONFIG_MPR121_TAP_MAX_DURATION_MS
#define TAP_MAX_MOVEMENT     (CONFIG_MPR121_TAP_MAX_MOVEMENT / 10.0f)
#define ROTARY_THRESHOLD_RAD (CONFIG_MPR121_ROTARY_THRESHOLD_RAD / 100.0f)
#define CURSOR_SENSITIVITY   (CONFIG_MPR121_CURSOR_SENSITIVITY / 10.0f)
#define DECAY_FACTOR         0.8f
#define DECAY_INTERVAL_MS    20

struct disc_state {
    bool scroll_mode;
    uint8_t prox_state;
    bool is_touched;
    int64_t touch_start_time;
    float start_x, start_y;
    float last_x, last_y;
    float max_dx, max_dy;
    float angle_sum;
    float last_angle;
    bool is_perimeter;
    float vel_x, vel_y;
};

static struct disc_state state = {
    .scroll_mode = false,
    .prox_state = MPR121_PROX_FAR,
};

static const struct device *mpr121_dev;
static const struct device *input_dev;
static struct k_work_delayable decay_work;
static struct k_timer sleep_timer;

/* ------------------------------------------------------------------ */
/* Input reporting via Zephyr Input Subsystem                         */
/* ------------------------------------------------------------------ */

static void send_pointer(int16_t x, int16_t y)
{
    if (!input_dev || !device_is_ready(input_dev)) return;

    if (state.scroll_mode) {
        input_report_rel(input_dev, INPUT_REL_WHEEL, -y, false, K_FOREVER);
    } else {
        input_report_rel(input_dev, INPUT_REL_X, x, false, K_FOREVER);
        input_report_rel(input_dev, INPUT_REL_Y, y, false, K_FOREVER);
    }
    input_report_sync(input_dev, K_FOREVER);
}

static void decay_handler(struct k_work *work)
{
    if (fabsf(state.vel_x) > 0.5f || fabsf(state.vel_y) > 0.5f) {
        state.vel_x *= DECAY_FACTOR;
        state.vel_y *= DECAY_FACTOR;
        send_pointer((int16_t)state.vel_x, (int16_t)state.vel_y);
        k_work_schedule(&decay_work, K_MSEC(DECAY_INTERVAL_MS));
    } else {
        state.vel_x = 0;
        state.vel_y = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Event emission helpers                                             */
/* ------------------------------------------------------------------ */

static void emit_gesture(uint8_t type, uint8_t zone, uint8_t direction)
{
    struct mpr121_gesture_event *ev = new_mpr121_gesture_event();
    if (!ev) return;
    ev->type = type;
    ev->zone = zone;
    ev->direction = direction;
    ZMK_EVENT_RAISE(ev);
}

static void emit_proximity(uint8_t prox_state)
{
    struct mpr121_proximity_event *ev = new_mpr121_proximity_event();
    if (!ev) return;
    ev->state = prox_state;
    ZMK_EVENT_RAISE(ev);
}

/* ------------------------------------------------------------------ */
/* Proximity timer                                                    */
/* ------------------------------------------------------------------ */

static void sleep_timer_expiry(struct k_timer *timer)
{
    state.prox_state = MPR121_PROX_SLEEP;
    if (mpr121_dev) mpr121_set_esi(mpr121_dev, MPR121_ESI_128MS);
    emit_proximity(MPR121_PROX_SLEEP);
    LOG_INF("MPR121 -> SLEEP (128ms)");
}

/* ------------------------------------------------------------------ */
/* Mode toggle                                                        */
/* ------------------------------------------------------------------ */

void mpr121_toggle_mode(void)
{
    state.scroll_mode = !state.scroll_mode;
    struct mpr121_mode_event *ev = new_mpr121_mode_event();
    if (ev) {
        ev->scroll_mode = state.scroll_mode;
        ZMK_EVENT_RAISE(ev);
    }
    LOG_INF("Mode: %s", state.scroll_mode ? "SCROLL" : "CURSOR");
}

/* ------------------------------------------------------------------ */
/* Sensor data callback                                               */
/* ------------------------------------------------------------------ */

static void on_mpr121_data(const struct device *dev,
                           struct mpr121_sample *sample, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    /* Proximity handling */
    if (sample->proximity_active) {
        if (state.prox_state != MPR121_PROX_NEAR) {
            state.prox_state = MPR121_PROX_NEAR;
            k_timer_stop(&sleep_timer);
            if (mpr121_dev) mpr121_set_esi(mpr121_dev, MPR121_ESI_1MS);
            emit_proximity(MPR121_PROX_NEAR);
        }
    } else {
        if (state.prox_state == MPR121_PROX_NEAR) {
            state.prox_state = MPR121_PROX_FAR;
            k_timer_start(&sleep_timer, K_SECONDS(30), K_NO_WAIT);
            emit_proximity(MPR121_PROX_FAR);
        }
    }

    /* Centroid calculation */
    float sum_wx = 0, cx = 0, sum_wy = 0, cy = 0;
    bool any_touch = false;

    for (int i = 0; i < 4; i++) {
        int16_t delta = (int16_t)sample->baseline[i] - (int16_t)sample->filtered[i];
        if (delta > TOUCH_THRESHOLD) {
            sum_wx += delta;
            cx += i * delta;
            any_touch = true;
        }
    }
    for (int i = 0; i < 4; i++) {
        int16_t delta = (int16_t)sample->baseline[4 + i] -
                        (int16_t)sample->filtered[4 + i];
        if (delta > TOUCH_THRESHOLD) {
            sum_wy += delta;
            cy += i * delta;
            any_touch = true;
        }
    }

    float x = (sum_wx > 0) ? (cx / sum_wx) : -1.0f;
    float y = (sum_wy > 0) ? (cy / sum_wy) : -1.0f;

    /* Touch state machine */
    if (any_touch) {
        k_work_cancel_delayable(&decay_work);

        if (!state.is_touched) {
            state.is_touched = true;
            state.touch_start_time = k_uptime_get();
            state.start_x = x; state.start_y = y;
            state.last_x = x; state.last_y = y;
            state.max_dx = 0; state.max_dy = 0;
            state.angle_sum = 0;
            state.last_angle = atan2f(y - 1.5f, x - 1.5f);
            state.is_perimeter = (fabsf(x - 1.5f) > 1.0f ||
                                  fabsf(y - 1.5f) > 1.0f);
        } else {
            float dx = x - state.start_x;
            float dy = y - state.start_y;
            state.max_dx = fmaxf(state.max_dx, fabsf(dx));
            state.max_dy = fmaxf(state.max_dy, fabsf(dy));

            if (state.is_perimeter) {
                float angle = atan2f(y - 1.5f, x - 1.5f);
                float delta_a = angle - state.last_angle;
                if (delta_a > M_PI) delta_a -= 2 * M_PI;
                if (delta_a < -M_PI) delta_a += 2 * M_PI;
                state.angle_sum += delta_a;
                state.last_angle = angle;
            }

            state.last_x = x; state.last_y = y;

            float cdx = x - 1.5f;
            float cdy = y - 1.5f;

            if (fabsf(cdx) > DEADZONE || fabsf(cdy) > DEADZONE) {
                float vx = (cdx > 0 ? cdx - DEADZONE : cdx + DEADZONE) *
                           CURSOR_SENSITIVITY;
                float vy = (cdy > 0 ? cdy - DEADZONE : cdy + DEADZONE) *
                           CURSOR_SENSITIVITY;
                state.vel_x = vx;
                state.vel_y = vy;
                send_pointer((int16_t)state.vel_x, (int16_t)state.vel_y);
            }
        }
    } else if (state.is_touched) {
        state.is_touched = false;
        int64_t duration = k_uptime_get() - state.touch_start_time;

        bool is_tap = (duration < TAP_MAX_DURATION_MS &&
                       state.max_dx < TAP_MAX_MOVEMENT &&
                       state.max_dy < TAP_MAX_MOVEMENT);
        bool is_swipe = (!is_tap &&
                         (state.max_dx > SWIPE_THRESHOLD ||
                          state.max_dy > SWIPE_THRESHOLD));
        bool is_rotary = (!is_tap && state.is_perimeter &&
                          fabsf(state.angle_sum) > ROTARY_THRESHOLD_RAD);

        if (is_tap) {
            uint8_t zone = MPR121_ZONE_CENTER;
            if (state.start_y < 1.0f)      zone = MPR121_ZONE_TOP;
            else if (state.start_y > 2.0f) zone = MPR121_ZONE_BOTTOM;
            else if (state.start_x < 1.0f) zone = MPR121_ZONE_LEFT;
            else if (state.start_x > 2.0f) zone = MPR121_ZONE_RIGHT;
            emit_gesture(MPR121_GESTURE_TAP, zone, MPR121_DIR_NONE);
        } else if (is_rotary) {
            uint8_t dir = (state.angle_sum > 0) ? MPR121_DIR_CW : MPR121_DIR_CCW;
            float sa = atan2f(state.start_y - 1.5f, state.start_x - 1.5f);
            uint8_t zone;
            if (sa > -M_PI/4 && sa <= M_PI/4)       zone = MPR121_ZONE_RIGHT;
            else if (sa > M_PI/4 && sa <= 3*M_PI/4) zone = MPR121_ZONE_BOTTOM;
            else if (sa > 3*M_PI/4 || sa <= -3*M_PI/4) zone = MPR121_ZONE_LEFT;
            else                                    zone = MPR121_ZONE_TOP;
            emit_gesture(MPR121_GESTURE_ROTARY, zone, dir);
        } else if (is_swipe) {
            float dx = state.last_x - state.start_x;
            float dy = state.last_y - state.start_y;
            uint8_t dir;
            if (fabsf(dx) > fabsf(dy)) {
                dir = (dx > 0) ? MPR121_DIR_RIGHT : MPR121_DIR_LEFT;
            } else {
                dir = (dy > 0) ? MPR121_DIR_DOWN : MPR121_DIR_UP;
            }
            emit_gesture(MPR121_GESTURE_SWIPE, MPR121_ZONE_CENTER, dir);
        }

        if (fabsf(state.vel_x) > 0.5f || fabsf(state.vel_y) > 0.5f) {
            k_work_schedule(&decay_work, K_MSEC(DECAY_INTERVAL_MS));
        }
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                               */
/* ------------------------------------------------------------------ */

static int mpr121_subsystem_init(void)
{
    mpr121_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(mpr121));
    input_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(mpr121_input));

    if (!mpr121_dev || !device_is_ready(mpr121_dev)) {
        LOG_WRN("MPR121 device not found");
        return -ENODEV;
    }

    k_timer_init(&sleep_timer, sleep_timer_expiry, NULL);
    k_work_init_delayable(&decay_work, decay_handler);
    mpr121_register_data_ready_cb(mpr121_dev, on_mpr121_data, NULL);

    LOG_INF("MPR121 subsystem initialised");
    return 0;
}

SYS_INIT(mpr121_subsystem_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
