#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/events/mpr121_events.h>

LOG_MODULE_REGISTER(mpr121_prox_beh, CONFIG_ZMK_LOG_LEVEL);

static struct zmk_behavior_binding prox_bindings[3];
static bool prox_loaded;

static int on_proximity_event(const zmk_event_t *eh)
{
    struct mpr121_proximity_event *ev = as_mpr121_proximity_event(eh);
    if (!ev || !prox_loaded) return 0;
    if (ev->state >= 3) return 0;

    struct zmk_behavior_binding *binding = &prox_bindings[ev->state];
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

ZMK_LISTENER(mpr121_prox_beh, on_proximity_event);
ZMK_SUBSCRIPTION(mpr121_prox_beh, mpr121_proximity_event);

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

BEHAVIOR_DEFINE(mpr121_proximity, behavior_init, binding_pressed,
                 binding_released, &api);
