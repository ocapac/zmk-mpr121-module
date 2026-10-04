#pragma once

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>

struct mpr121_data_event {
    ZMK_EVENT_HEADER
    uint16_t filtered[8];
    uint16_t baseline[8];
    uint16_t touch_status;
    bool proximity_active;
};
ZMK_EVENT_DECLARE(mpr121_data_event);

#define MPR121_GESTURE_TAP     0
#define MPR121_GESTURE_SWIPE   1
#define MPR121_GESTURE_ROTARY  2

#define MPR121_ZONE_CENTER 0
#define MPR121_ZONE_TOP    1
#define MPR121_ZONE_BOTTOM 2
#define MPR121_ZONE_LEFT   3
#define MPR121_ZONE_RIGHT  4

#define MPR121_DIR_NONE  0
#define MPR121_DIR_UP    1
#define MPR121_DIR_DOWN  2
#define MPR121_DIR_LEFT  3
#define MPR121_DIR_RIGHT 4
#define MPR121_DIR_CW    5
#define MPR121_DIR_CCW   6

struct mpr121_gesture_event {
    ZMK_EVENT_HEADER
    uint8_t type;
    uint8_t zone;
    uint8_t direction;
};
ZMK_EVENT_DECLARE(mpr121_gesture_event);

#define MPR121_PROX_NEAR  0
#define MPR121_PROX_FAR   1
#define MPR121_PROX_SLEEP 2

struct mpr121_proximity_event {
    ZMK_EVENT_HEADER
    uint8_t state;
};
ZMK_EVENT_DECLARE(mpr121_proximity_event);

struct mpr121_mode_event {
    ZMK_EVENT_HEADER
    bool scroll_mode;
};
ZMK_EVENT_DECLARE(mpr121_mode_event);
