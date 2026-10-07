
#include "../../hardware_api.h"

#if defined(ARDUINO_ARCH_ZEPHYR)

#pragma message("")
#pragma message("SimpleFOC: compiling for Arduino Zephyr core (ArduinoCore-zephyr)")
#pragma message("")

// The Arduino Zephyr core (https://github.com/arduino/ArduinoCore-zephyr) runs on top of
// the Zephyr RTOS. To get true runtime control of the PWM frequency (which the wiring
// analogWrite() API does not offer, as its period is fixed by the devicetree), we drive the
// Zephyr PWM timers directly through the native pwm_set_pulse_dt() API - the same method used
// by the Arduino_HardwareServo library (https://github.com/arduino-libraries/Arduino_HardwareServo).
//
// The requested frequency is converted to a period (ns) and stored, once at configuration
// time, into a per-pin copy of the channel's pwm_dt_spec. The duty-cycle updates then only
// push the new pulse width with pwm_set_pulse_dt(), reusing that stored period.
//
// The PWM channels are taken from the `pwms` property of the board's `zephyr,user` devicetree
// node, and mapped to Arduino pin numbers through the `pwm-pin-gpios` property. Make sure the
// driver pins you use are declared there in your board overlay (see README.md).

#include <Arduino.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>
#include <zephyrPinctrl.h>   // zephyr::arduino::init_dev_apply_channel_pinctrl(), state_pin_index_from_spec_index()

// Build a pwm_dt_spec table and a pin->channel map from the devicetree, exactly like
// Arduino_HardwareServo / the Arduino Zephyr core do internally.
#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), pwms) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), pwm_pin_gpios)

#define SIMPLEFOC_ZEPHYR_HAS_PWM 1

#define _SIMPLEFOC_PWM_DT_SPEC(n, p, i) PWM_DT_SPEC_GET_BY_IDX(n, i),
#define _SIMPLEFOC_PWM_PINS(n, p, i)                                                                \
  DIGITAL_PIN_GPIOS_FIND_PIN(DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PATH(zephyr_user), p, i)),            \
                             DT_PHA_BY_IDX(DT_PATH(zephyr_user), p, i, pin)),

static const struct pwm_dt_spec _foc_pwm[] = {
  DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), pwms, _SIMPLEFOC_PWM_DT_SPEC)
};

// maps Arduino digital pin numbers to indices into _foc_pwm[]
static const pin_size_t _foc_pwm_pins[] = {
  DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), pwm_pin_gpios, _SIMPLEFOC_PWM_PINS)
};

// returns the index of the pin into the pwm spec table, or -1 if the pin is not a PWM channel
static int _foc_pwm_index(pin_size_t pinNumber) {
  for (size_t i = 0; i < ARRAY_SIZE(_foc_pwm_pins); i++) {
    if (_foc_pwm_pins[i] == pinNumber) return (int)i;
  }
  return -1;
}

#endif // devicetree has pwms


// analogWrite resolution used only for the fallback (pins not mapped to a PWM channel in
// the devicetree). Overridable from the build environment.
#ifndef SIMPLEFOC_ZEPHYR_PWM_RESOLUTION
#define SIMPLEFOC_ZEPHYR_PWM_RESOLUTION 12
#endif

// hardware specific parameters for the Zephyr driver
typedef struct ZephyrDriverParams {
  int pins[6];              // Arduino pin numbers
  bool has_pwm[6];          // true if pins[i] maps to a hardware PWM channel
#ifdef SIMPLEFOC_ZEPHYR_HAS_PWM
  struct pwm_dt_spec pwm[6];// per-pin channel spec copy, with .period set to the requested one
#endif
  long pwm_frequency;       // requested PWM frequency [Hz]
  uint32_t period_ns;       // PWM period in nanoseconds = 1e9 / pwm_frequency
  float dead_zone;
  uint32_t pwm_range;       // max value for the analogWrite fallback = 2^resolution - 1
} ZephyrDriverParams;


// default PWM frequency if none (or an invalid one) is requested
#define SIMPLEFOC_ZEPHYR_DEFAULT_PWM_FREQ 25000L

static inline float _clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// write a [0,1] duty cycle either through the Zephyr PWM timer (pulse only, reusing the
// period stored in the spec) or, for unmapped pins, through the analogWrite() fallback
static inline void _writeDuty(ZephyrDriverParams* p, int i, float dc) {
  dc = _clampf(dc, 0.0f, 1.0f);
#ifdef SIMPLEFOC_ZEPHYR_HAS_PWM
  if (p->has_pwm[i]) {
    uint32_t pulse_ns = (uint32_t)(dc * (float)p->pwm[i].period);
    pwm_set_pulse_dt(&p->pwm[i], pulse_ns);
    return;
  }
#endif
  analogWrite(p->pins[i], (int)(dc * (float)p->pwm_range));
}

// common configuration helper for all N-PWM modes
static ZephyrDriverParams* _configurePWM(long pwm_frequency, const int* pins, int n) {
  if (pwm_frequency <= 0) pwm_frequency = SIMPLEFOC_ZEPHYR_DEFAULT_PWM_FREQ;

  analogWriteResolution(SIMPLEFOC_ZEPHYR_PWM_RESOLUTION);

  ZephyrDriverParams* params = new ZephyrDriverParams();
  params->pwm_frequency = pwm_frequency;
  params->period_ns = (uint32_t)(1000000000UL / (uint32_t)pwm_frequency);
  params->dead_zone = NOT_SET;
  params->pwm_range = (1UL << SIMPLEFOC_ZEPHYR_PWM_RESOLUTION) - 1UL;

  for (int i = 0; i < 6; i++) { params->pins[i] = NOT_SET; params->has_pwm[i] = false; }

  for (int i = 0; i < n; i++) {
    params->pins[i] = pins[i];
#ifdef SIMPLEFOC_ZEPHYR_HAS_PWM
    int idx = _foc_pwm_index((pin_size_t)pins[i]);
    if (idx >= 0) {
      // Route the timer channel to the physical pin. The Arduino Zephyr core leaves the pins
      // as plain GPIO at boot and applies the PWM pin mux lazily inside analogWrite(); we must
      // do the same or the timer runs but its output never reaches the pin.
      zephyr::arduino::init_dev_apply_channel_pinctrl(
          _foc_pwm[idx].dev,
          zephyr::arduino::state_pin_index_from_spec_index(_foc_pwm, idx));
      if (!pwm_is_ready_dt(&_foc_pwm[idx]))
        SIMPLEFOC_DEBUG("ZEPHYR: PWM device not ready for pin: ", pins[i]);

      params->pwm[i] = _foc_pwm[idx];        // copy the devicetree channel spec ...
      params->pwm[i].period = params->period_ns; // ... and apply the requested frequency
      params->has_pwm[i] = true;
      continue;
    }
    SIMPLEFOC_DEBUG("ZEPHYR: pin not mapped to a PWM channel, using analogWrite fallback: ", pins[i]);
#else
    SIMPLEFOC_DEBUG("ZEPHYR: no `pwms` in the devicetree, using analogWrite fallback (no frequency control).");
#endif
    pinMode(pins[i], OUTPUT);   // fallback (non-PWM) pins only
  }
  return params;
}


// Configuring PWM - 1PWM setting (single phase)
void* _configure1PWM(long pwm_frequency, const int pinA) {
  const int pins[] = { pinA };
  return _configurePWM(pwm_frequency, pins, 1);
}

// Configuring PWM - 2PWM setting (Stepper)
void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  const int pins[] = { pinA, pinB };
  return _configurePWM(pwm_frequency, pins, 2);
}

// Configuring PWM - 3PWM setting (BLDC)
void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
  const int pins[] = { pinA, pinB, pinC };
  return _configurePWM(pwm_frequency, pins, 3);
}

// Configuring PWM - 4PWM setting (Stepper)
void* _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
  const int pins[] = { pin1A, pin1B, pin2A, pin2B };
  return _configurePWM(pwm_frequency, pins, 4);
}

// Configuring PWM - 6PWM setting (BLDC, complementary)
//
// Complementary 6-PWM with hardware dead-time is timer/board specific and cannot be produced
// portably: pwm_set_pulse_dt() drives each channel independently and inserts no dead-time, so
// a software complementary pair would risk shoot-through. 6-PWM is therefore reported as
// unsupported on this core - use two 3-PWM half-bridges configured in the devicetree instead.
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
  _UNUSED(pwm_frequency);
  _UNUSED(dead_zone);
  _UNUSED(pinA_h);
  _UNUSED(pinA_l);
  _UNUSED(pinB_h);
  _UNUSED(pinB_l);
  _UNUSED(pinC_h);
  _UNUSED(pinC_l);

  SIMPLEFOC_DEBUG("ZEPHYR: 6-PWM not supported. Use a 3-PWM driver, or hardware complementary PWM via the devicetree.");
  return SIMPLEFOC_DRIVER_INIT_FAILED;
}


// Writing PWM duty cycles ------------------------------------------------------

void _writeDutyCycle1PWM(float dc_a, void* params) {
  ZephyrDriverParams* p = (ZephyrDriverParams*)params;
  _writeDuty(p, 0, dc_a);
}

void _writeDutyCycle2PWM(float dc_a, float dc_b, void* params) {
  ZephyrDriverParams* p = (ZephyrDriverParams*)params;
  _writeDuty(p, 0, dc_a);
  _writeDuty(p, 1, dc_b);
}

void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void* params) {
  ZephyrDriverParams* p = (ZephyrDriverParams*)params;
  _writeDuty(p, 0, dc_a);
  _writeDuty(p, 1, dc_b);
  _writeDuty(p, 2, dc_c);
}

void _writeDutyCycle4PWM(float dc_1a, float dc_1b, float dc_2a, float dc_2b, void* params) {
  ZephyrDriverParams* p = (ZephyrDriverParams*)params;
  _writeDuty(p, 0, dc_1a);
  _writeDuty(p, 1, dc_1b);
  _writeDuty(p, 2, dc_2a);
  _writeDuty(p, 3, dc_2b);
}

// 6-PWM is not supported on this core (see _configure6PWM above)
void _writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c, PhaseState *phase_state, void* params) {
  _UNUSED(dc_a);
  _UNUSED(dc_b);
  _UNUSED(dc_c);
  _UNUSED(phase_state);
  _UNUSED(params);
}

#endif
