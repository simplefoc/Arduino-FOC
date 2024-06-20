#ifndef ESP32_DRIVER_MCPWM_H
#define ESP32_DRIVER_MCPWM_H

#include "../../hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "esp_idf_version.h"  

// version check - this mcpwm driver is specific for ESP-IDF 5.x and arduino-esp32 3.x
#if ESP_IDF_VERSION  < ESP_IDF_VERSION_VAL(5, 0, 0) 
#error SimpleFOC: ESP-IDF version 4 or lower detected. Please update to ESP-IDF 5.x and Arduino-esp32 3.0 (or higher)
#endif

#ifndef SIMPLEFOC_ESP32_HW_DEADTIME
  #define SIMPLEFOC_ESP32_HW_DEADTIME true // TODO: Change to false when sw-deadtime & phase_state is approved ready for general use.
#endif

//!< ESP32 MCPWM driver parameters
typedef struct ESP32MCPWMDriverParams {
  long pwm_frequency; //!< frequency of the pwm signal
  int group_id; //!< group of the mcpwm
  mcpwm_timer_handle_t timers[2]; //!< timers of the mcpwm
  mcpwm_oper_handle_t oper[3]; //!< operators of the mcpwm
  mcpwm_cmpr_handle_t comparator[6]; //!< comparators of the mcpwm
  mcpwm_gen_handle_t generator[6]; //!< generators of the mcpwm
  uint32_t mcpwm_period; //!< period of the pwm signal
  float dead_zone; //!< dead zone of the pwm signal
} ESP32MCPWMDriverParams; 


#define SIMPLEFOC_ESP32_DEBUG(tag, str)\
    SimpleFOCDebug::println( "ESP32-"+String(tag)+ ": "+ String(str));

#define SIMPLEFOC_ESP32_DRV_DEBUG(str)\
   SIMPLEFOC_ESP32_DEBUG("DRV", str);\

// macro for checking the error of the mcpwm functions
// if the function returns an error the function will return SIMPLEFOC_DRIVER_INIT_FAILED
#define CHECK_ERR(func_call, message) \
  if ((func_call) != ESP_OK) { \
    SIMPLEFOC_ESP32_DRV_DEBUG("ERROR - " + String(message)); \
    return SIMPLEFOC_DRIVER_INIT_FAILED; \
  }


// ABI bus frequency - would be better to take it from somewhere
// but I did nto find a good exposed variable
#define _MCPWM_FREQ 160e6f
#define _PWM_TIMEBASE_RESOLUTION_HZ (_MCPWM_FREQ) /*!< Resolution of MCPWM */
// pwm frequency settings
#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50kHz


// low-level configuration API 

/**
 * checking if group has pins available
 * @param group - group of the mcpwm
 * @param no_pins - number of pins
 * @returns true if pins are available, false otherwise
 */
bool _hasAvailablePins(int group, int no_pins);
/**
 * function finding the last timer in the group
 * @param group - group of the mcpwm
 * @returns index of the last timer in the group
 *       -1 if no timer instantiated yet
 */
uint8_t _findLastTimer(int group);

/**
 * function finding the next timer in the group
 * @param group - group of the mcpwm
 * @returns index of the next timer in the group
 *        -1 if all timers are used
 */
uint8_t _findNextTimer(int group);


/**
 * function finding the best group and timer for the pwm signals
 *  
 * @param no_pins - number of pins
 * @param pwm_freq - frequency of the pwm signal
 * @param group - pointer to the group
 * @param timer - pointer to the timer
 * @returns 
 *  1 if solution found in one group
 *  2 if solution requires using both groups
 *  0 if no solution possible
 */
int _findBestGroup(int no_pins, long pwm_freq, int* group, int* timer);


/**
 * function configuring the center alignement and inversion of a pwm signal
 * @param gena - mcpwm generator handle
 * @param cmpa - mcpwm comparator handle
 * @param inverted - true if the signal is inverted, false otherwise
 */
int _configureCenterAlign(mcpwm_gen_handle_t gena, mcpwm_cmpr_handle_t cmpa, bool inverted);

/**
 * function calculating the pwm period
 * @param pwm_frequency - frequency of the pwm signal
 * @return uint32_t - period of the pwm signal
 */
uint32_t _calcPWMPeriod(long pwm_frequency);
/**
 * function calculating the pwm frequency
 * @param pwm_period - period of the pwm signal
 * @return long - frequency of the pwm signal
 */
long _calcPWMFreq(long pwm_period);

/**
 * function configuring the MCPWM for 6pwm
 * @param pwm_frequency - frequency of the pwm signal
 * @param mcpwm_group - group of the mcpwm
 * @param timer_no - timer number
 * @param dead_zone - dead zone of the pwm signal
 * @param pins - array of pins
 * @return ESP32MCPWMDriverParams* - pointer to the driver parameters if successful, -1 if failed
 */
void* _configure6PWMPinsMCPWM(long pwm_frequency, int mcpwm_group, int timer_no, float dead_zone, int* pins);
/**
 * function configuring the MCPWM for pwm generation
 * @param pwm_frequency - frequency of the pwm signal
 * @param mcpwm_group - group of the mcpwm
 * @param timer_no - timer number
 * @param no_pins - number of pins
 * @param pins - array of pins
 * @return ESP32MCPWMDriverParams* - pointer to the driver parameters if successful, -1 if failed
 */
void* _configurePinsMCPWM(long pwm_frequency, int mcpwm_group, int timer_no, int no_pins, int* pins);
/**
 * function setting the duty cycle to the MCPWM channel
 * @param cmpr - mcpwm channel handle
 * @param mcpwm_period - period of the pwm signal
 * @param duty_cycle - duty cycle of the pwm signal
 */
void _setDutyCycle(mcpwm_cmpr_handle_t cmpr, uint32_t mcpwm_period, float duty_cycle);

/**
 * function setting the phase state 
 * @param generator_high - mcpwm generator handle for the high side
 * @param generator_low - mcpwm generator handle for the low side
 * @param phase_state - phase state
 */
void _forcePhaseState(mcpwm_gen_handle_t generator_high, mcpwm_gen_handle_t generator_low, PhaseState phase_state);

#endif
#endif