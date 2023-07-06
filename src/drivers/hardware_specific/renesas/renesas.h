#pragma once


#include "../../hardware_api.h"


#if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)

// uncomment to enable debug output from Renesas driver
// can set this as build-flag in Arduino IDE or PlatformIO
#define SIMPLEFOC_RENESAS_DEBUG

#define RENESAS_DEFAULT_PWM_FREQUENCY 24000
#define RENESAS_DEFAULT_DEAD_ZONE 0.05f

struct RenesasTimerConfig;

typedef struct RenesasHardwareDriverParams {
    uint8_t pins[6];
    uint8_t channels[6];
    RenesasTimerConfig* timer_config[6];
    long pwm_frequency;
    float dead_zone;
} RenesasHardwareDriverParams;



#endif