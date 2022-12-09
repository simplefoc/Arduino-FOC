#include "../../hardware_api.h"

#if defined(__arm__) && defined(CORE_TEENSY)

#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

// debugging output 
#define SIMPLEFOC_TEENSY_DEBUG 

//  configure High PWM frequency
void _setHighFrequency(const long freq, const int pin);

#endif