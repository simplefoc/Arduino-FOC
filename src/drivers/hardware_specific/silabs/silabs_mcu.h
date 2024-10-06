/**
 * Silabs support for SimpleFOC library
 * 
 * The Silabs Gecko EFR32 procossors as used in the Arduino Nano Matter have the following
 * features:
 *  - 2 × 32-bit Timer/Counter with 3 Compare/Capture/PWM channels (TIMER0, TIMER1)
 *  - 3 × 16-bit Timer/Counter with 3 Compare/Capture/PWM channels (TIMER2, TIMER3, TIMER4)
 *  - All timers support dead-time insertion (DTI)
 *  - Centre-aligned PWM mode
 *  - Buffered compare register to ensure glitch-free update of compare values
 *  - Timers can trigger ADC and DMA via reflex system
 *  - Timers can be started/stopped synchronously by other timers
 * 
 * The MCU is very flexible regarding its pin assigments. The following is the possible
 * assignments:
 * 
 *  GPIO port A: TIMER0, TIMER1, TIMER2, TIMER4
 *  GPIO port B: TIMER0, TIMER1, TIMER2, TIMER4
 *  GPIO port C: TIMER0, TIMER1, TIMER3
 *  GPIO port D: TIMER0, TIMER1, TIMER3
 *  
 * Any pin of these ports can be used with any of the 3 channels or the inverted 
 * channels of the possible timers.
 * 
 * This suggests the following usage paradigm:
 *  - 3-PWM or 6-PWM use a single timer, all 3 channels
 *  - 2-PWM uses two channels of a single timer
 *  - 1-PWM uses a single channel
 *  - 4-PWM uses two timers, two channels each (we could try to sync them)
 *  - TIMER0 and TIMER1 are preferred, since they can be used with all GPIO ports
 * 
 * Nano Matter Pin/Port assignments:
 *  D0 TX: PA04
 *  D1 RX: PA05
 *  D3: PC06
 *  D4 SDA1: PC07
 *  D5 SCL1: PC08
 *  D6: PC09
 *  D7: PD02
 *  D8: PD03
 *  D9: PTI_DATA: PD04
 *  D10 SS PTI_SYNC: PD05
 *  D11 MOSI: PA09
 *  D12 MISO: PA08
 *  D13 CLK: PB04
 *  A0: PB00
 *  A1: PB02
 *  A2: PB05
 *  A3: PC00
 *  A4 SDA: PA06
 *  A5 SCL: PA07
 *  A6: PB01
 *  A7: PB03
 *  RGB_R: PC01
 *  RGB_G: PC02
 *  RGB_B: PC03
 * 
 * So the Nano Matter could use any of TIMER0, TIMER1, or TIMER3 with 
 * the SimpleFOC Nano Shield (3-PWM mode).
 * 
 */


#pragma once

#include "../../hardware_api.h"
#include <inttypes.h>

#if defined(ARDUINO_ARCH_SILABS)

#include "em_timer.h"

typedef struct SilabsDriverParams {
    int pins[6];
    TIMER_TypeDef* timer[6];
    uint8_t channel[6];
    uint32_t pwm_frequency;
    uint32_t resolution;
    float dead_zone;
};



#endif