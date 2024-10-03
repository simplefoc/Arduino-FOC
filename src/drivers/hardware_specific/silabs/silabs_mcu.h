
#pragma once

#include "../../hardware_api.h"
#include <inttypes.h>

#if defined(ARDUINO_ARCH_SILABS)


typedef struct SilabsDriverParams {
    int pins[6];
    uint8_t timer[6];
    uint8_t channel[6];
    long pwm_frequency;
    float dead_zone;
};



#endif