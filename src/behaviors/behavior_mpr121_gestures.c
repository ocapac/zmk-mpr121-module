#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/events/mpr121_events.h>

LOG_MODULE_REGISTER(mpr121_gestures_beh, CONFIG_ZMK_LOG_LEVEL);

#define NUM_GESTURE_BINDINGS 17
static struct zmk_behavior_binding bindings[NUM_GESTURE_BINDINGS];
static bool bindings_loaded;

static int gesture_index(uint8_t type, uint8_t zone, uint8_t direction)
{
    switch (type) {
    case MPR121_GESTURE_TAP:    return zone;
    case MPR121_GESTURE_SWIPE:  return 5 + (direction - MPR121_DIR_UP);
    case MPR121_GESTURE_ROTARY:
        return 9 + (zone - MPR121_ZONE_TOP) * 2 +
               (direction == MPR121_DIR_CW ? 0 : 1);
    default: return -1;
    }
}

static int on_gesture_event(const zmk_event_t *eh)
{
    struct mpr121_gesture_event *ev = as_mpr121_gesture_event(eh);
    if (!ev || !bindings_loaded) return 0;

    int idx = gesture_index(ev->type, ev->zone, ev->direction);
    if (idx < 0 || idx >= NUM_GESTURE_BINDINGS) return 0;

    struct zmk_behavior_binding *binding = &bindings[idx];
    if (binding->behavior == NULL) return 0;

    struct zmk_behavior_binding_event event = {
        .layer = 0,
        .position = 0,
        .timestamp = k_uptime_get(),
    };

    behavior_keymap_binding_pressed(binding, event);
    behavior_keymap_binding_released(binding, event);
    return 0;
}

ZMK_LISTENER(mpr121_gestures_beh, on_gesture_event);
ZMK_SUBSCRIPTION(mpr121_gestures_beh, mpr121_gesture_event);

static int behavior_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
}

static int binding_pressed(const struct device *behavior,
                           struct zmk_behavior_binding *binding,
                           struct zmk_behavior_binding_event event)
{
    ARG_UNUSED(behavior); ARG_UNUSED(binding); ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int binding_released(const struct device *behavior,
                            struct zmk_behavior_binding *binding,
                            struct zmk_behavior_binding_event event)
{
    ARG_UNUSED(behavior); ARG_UNUSED(binding); ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .binding_pressed = binding_pressed,
    .binding_released = binding_released,
};

BEHAVIOR_DEFINE(mpr121_gestures, behavior_init, binding_pressed,
                 binding_released, &api);
