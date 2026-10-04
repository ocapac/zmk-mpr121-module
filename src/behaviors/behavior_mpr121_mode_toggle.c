#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>

LOG_MODULE_REGISTER(mpr121_mode_beh, CONFIG_ZMK_LOG_LEVEL);

extern void mpr121_toggle_mode(void);

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
    mpr121_toggle_mode();
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

BEHAVIOR_DEFINE(mpr121_mode_toggle, behavior_init, binding_pressed,
                 binding_released, &api);
