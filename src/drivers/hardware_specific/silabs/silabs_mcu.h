
#pragma once

#include "../../hardware_api.h"
#include <inttypes.h>

#if defined(ARDUINO_ARCH_SILABS)

#include "em_timer.h"

typedef struct SilabsDriverParams {
    int pins[6];
    TIMER_TypeDef* timer[6];
    uint8_t channel[6];
    long pwm_frequency;
    float dead_zone;
};



#endif