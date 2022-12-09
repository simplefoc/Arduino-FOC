#ifndef ESP32_DRIVER_MCPWM_H
#define ESP32_DRIVER_MCPWM_H

#include "../../hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

// empty motor slot
#define _EMPTY_SLOT -20
#define _TAKEN_SLOT -21

// ABI bus frequency - would be better to take it from somewhere
// but I did nto find a good exposed variable
#define _MCPWM_FREQ 160e6f

// preferred pwm resolution default
#define _PWM_RES_DEF 4096
// min resolution
#define _PWM_RES_MIN 3000
// max resolution
#define _PWM_RES_MAX 8000
// pwm frequency
#define _PWM_FREQUENCY 25000 // default
#define _PWM_FREQUENCY_MAX 50000 // mqx

// structure containing motor slot configuration
// this library supports up to 4 motors
typedef struct {
  int pinA;
  mcpwm_dev_t* mcpwm_num;
  mcpwm_unit_t mcpwm_unit;
  mcpwm_operator_t mcpwm_operator;
  mcpwm_io_signals_t mcpwm_a;
  mcpwm_io_signals_t mcpwm_b;
  mcpwm_io_signals_t mcpwm_c;
} bldc_3pwm_motor_slots_t;

typedef struct {
  int pin1A;
  mcpwm_dev_t* mcpwm_num;
  mcpwm_unit_t mcpwm_unit;
  mcpwm_operator_t mcpwm_operator1;
  mcpwm_operator_t mcpwm_operator2;
  mcpwm_io_signals_t mcpwm_1a;
  mcpwm_io_signals_t mcpwm_1b;
  mcpwm_io_signals_t mcpwm_2a;
  mcpwm_io_signals_t mcpwm_2b;
} stepper_4pwm_motor_slots_t;

typedef struct {
  int pin1pwm;
  mcpwm_dev_t* mcpwm_num;
  mcpwm_unit_t mcpwm_unit;
  mcpwm_operator_t mcpwm_operator;
  mcpwm_io_signals_t mcpwm_a;
  mcpwm_io_signals_t mcpwm_b;
} stepper_2pwm_motor_slots_t;

typedef struct {
  int pinAH;
  mcpwm_dev_t* mcpwm_num;
  mcpwm_unit_t mcpwm_unit;
  mcpwm_operator_t mcpwm_operator1;
  mcpwm_operator_t mcpwm_operator2;
  mcpwm_io_signals_t mcpwm_ah;
  mcpwm_io_signals_t mcpwm_bh;
  mcpwm_io_signals_t mcpwm_ch;
  mcpwm_io_signals_t mcpwm_al;
  mcpwm_io_signals_t mcpwm_bl;
  mcpwm_io_signals_t mcpwm_cl;
} bldc_6pwm_motor_slots_t;


typedef struct ESP32MCPWMDriverParams {
  long pwm_frequency;
  mcpwm_dev_t* mcpwm_dev;
  mcpwm_unit_t mcpwm_unit;
  mcpwm_operator_t mcpwm_operator1;
  mcpwm_operator_t mcpwm_operator2;
} ESP32MCPWMDriverParams;


#endif
#endif