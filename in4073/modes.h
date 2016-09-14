#ifndef MODES_H
#define MODES_H

typedef enum quadcopter_mode {
    MODE_0_SAFE         = 0,
    MODE_1_PANIC        = 1,
    MODE_2_MANUAL       = 2,
    MODE_3_CALIBRATE    = 3,
    MODE_4_YAW          = 4,
    MODE_5_FULL_CONTROL = 5,
    MODE_6_RAW          = 6,
    MODE_7_HEIGHT       = 7,
    MODE_8_WIRELESS     = 8
} quadcopter_mode_t;


#endif // MODES_H
