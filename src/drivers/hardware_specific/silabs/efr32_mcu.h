#ifndef EFR32_DRIVER_MCU_H
#define EFR32_DRIVER_MCU_H

#include "../../hardware_api.h"

#if defined(ARDUINO_ARCH_SILABS)
#include "efr32_pwm.h"

#ifndef SILABS_DEFAULT_PWM_PERPHERAL
#define SILABS_DEFAULT_PWM_PERPHERAL TIMER0
#endif

#ifndef SILABS_SECOND_PWM_PERPHERAL
#define SILABS_SECOND_PWM_PERPHERAL TIMER1
#endif

#ifndef SILABS_PWM_PRS_CHANNEL
#define SILABS_PWM_PRS_CHANNEL 0
#endif

#ifndef SILABS_DEFAULT_PWM_FREQUENCY
#define SILABS_DEFAULT_PWM_FREQUENCY 50000
#endif

#ifndef SILABS_DEFAULT_DEAD_ZONE
#define SILABS_DEFAULT_DEAD_ZONE 0.02f
#endif

typedef struct EFR32DriverParams {
  EFR32PwmInstance inst[4];
  uint8_t noPwmChannel;
  bool lowside;
  long pwm_frequency;
  float dead_zone;
} EFR32DriverParams;

#endif

#endif
