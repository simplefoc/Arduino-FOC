// File: efr32_mcu.cpp

#if defined(ARDUINO_ARCH_SILABS)

#include <em_device.h>
#include <em_prs.h>
#include "efr32_pwm.h"
#include "efr32_mcu.h"

void _getPrsSourceAndUnderflowSignal(
  TIMER_TypeDef *timer,
  uint32_t *source,
  uint32_t *signal
) {
  if (!timer || !source || !signal) return;

  switch ((uint32_t) timer) {
    case TIMER0_BASE:
      *source = PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER0;
      *signal = PRS_ASYNC_CH_CTRL_SIGSEL_TIMER0UF;
      break;

    case TIMER1_BASE:
      *source = PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER1;
      *signal = PRS_ASYNC_CH_CTRL_SIGSEL_TIMER1UF;
      break;

    case TIMER2_BASE:
      *source = PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER2;
      *signal = PRS_ASYNC_CH_CTRL_SIGSEL_TIMER2UF;
      break;

    default:
      break;
  }
}

// Callbacks
static void _alignPWMTimers(
  TIMER_InitCC_TypeDef *initCC,
  void *params
) {
  EFR32PwmInstance *p = (EFR32PwmInstance*)params;
  if (!p || !initCC) return;

  CMU_ClockEnable(cmuClock_PRS, true);

  uint32_t prsSource, prsSignal;

  initCC->prsInputType = timerPrsInputAsyncPulse;
  initCC->prsInput = true;
  initCC->prsSel = SILABS_PWM_PRS_CHANNEL;

  _getPrsSourceAndUnderflowSignal(p->h.timer, &prsSource, &prsSignal);
  PRS_SourceAsyncSignalSet(SILABS_PWM_PRS_CHANNEL, prsSource, prsSignal);
}

static void _configPWMMode(
  TIMER_Init_TypeDef *init,
  void *params
) {
  if (!init) return;
  _UNUSED(params);
  init->mode = timerModeUpDown;
}

static void _alignPWMStart(
  TIMER_Init_TypeDef *init,
  void *params
) {
  if (!init) return;
  _UNUSED(params);
  init->mode = timerModeUpDown;
  init->riseAction = timerInputActionReloadStart;
}

static void _setSinglePhaseState(EFR32PwmInstance *inst, PhaseState state) {
  if (!inst) return;

  switch (state) {
    case PhaseState::PHASE_OFF:
      pwmOff(inst);
      break;

    default:
      pwmOn(inst);
      break;
  }
}

void *_configure1PWM(long pwm_frequency, const int pinA) {
  EFR32DriverParams* params = new EFR32DriverParams;
  if (!params) return SIMPLEFOC_DRIVER_INIT_FAILED;

  if (!pwm_frequency || !_isset(pwm_frequency)) pwm_frequency = SILABS_DEFAULT_PWM_FREQUENCY;
  else pwm_frequency = _constrain(pwm_frequency, 0, SILABS_DEFAULT_PWM_FREQUENCY);

  params->pwm_frequency = pwm_frequency;
  params->noPwmChannel = 1;

  // Ensure all PWMs use the same TIMER instance
  pwmHiConfig(&params->inst[0], SILABS_DEFAULT_PWM_PERPHERAL, pinA, 0);

  // Initialize PWM
  EFR32PwmConfig pwmConfig;
  pwmConfig.frequency = pwm_frequency;
  pwmConfig.polarity = PWM_P_ACTIVE_LOW;
  pwmConfig.outInvert = true;

  pwmHiInit(&params->inst[0], &pwmConfig, NULL, NULL);

  // PWM On
  pwmHiOn(&params->inst[0]);

  // Start PWM
  pwmStart(&params->inst[0], _configPWMMode, NULL);

  return params;
}

void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  EFR32DriverParams *params = new EFR32DriverParams;
  if (!params) return SIMPLEFOC_DRIVER_INIT_FAILED;

  if (!pwm_frequency || !_isset(pwm_frequency)) pwm_frequency = SILABS_DEFAULT_PWM_FREQUENCY;
  else pwm_frequency = _constrain(pwm_frequency, 0, SILABS_DEFAULT_PWM_FREQUENCY);

  params->pwm_frequency = pwm_frequency;
  params->noPwmChannel = 2;

  // Ensure all PWMs use the same TIMER instance
  pwmHiConfig(&params->inst[0], SILABS_DEFAULT_PWM_PERPHERAL, pinA, 0);
  pwmHiConfig(&params->inst[1], SILABS_DEFAULT_PWM_PERPHERAL, pinB, 1);

  // Initialize PWM
  EFR32PwmConfig pwmConfig;
  pwmConfig.frequency = pwm_frequency << 1;
  pwmConfig.polarity = PWM_P_ACTIVE_LOW;
  pwmConfig.outInvert = true;

  pwmHiInit(&params->inst[0], &pwmConfig, NULL, NULL);
  pwmHiInit(&params->inst[1], &pwmConfig, NULL, NULL);

  // PWM On
  pwmHiOn(&params->inst[0]);
  pwmHiOn(&params->inst[1]);

  // Start PWM
  pwmStart(&params->inst[0], _configPWMMode, NULL);

  return params;
}

void* _configure3PWM(
  long pwm_frequency,
  const int pinA,
  const int pinB,
  const int pinC
) {
  EFR32DriverParams *params = new EFR32DriverParams;
  if (!params) return SIMPLEFOC_DRIVER_INIT_FAILED;

  if (!pwm_frequency || !_isset(pwm_frequency)) pwm_frequency = SILABS_DEFAULT_PWM_FREQUENCY;
  else pwm_frequency = _constrain(pwm_frequency, 0, SILABS_DEFAULT_PWM_FREQUENCY);

  params->pwm_frequency = pwm_frequency;
  params->noPwmChannel = 3;

  // Ensure all PWMs use the same TIMER instance
  pwmHiConfig(&params->inst[0], SILABS_DEFAULT_PWM_PERPHERAL, pinA, 0);
  pwmHiConfig(&params->inst[1], SILABS_DEFAULT_PWM_PERPHERAL, pinB, 1);
  pwmHiConfig(&params->inst[2], SILABS_DEFAULT_PWM_PERPHERAL, pinC, 2);

  // Initialize PWM
  EFR32PwmConfig pwmConfig;
  pwmConfig.frequency = pwm_frequency << 1;
  pwmConfig.polarity = PWM_P_ACTIVE_LOW;
  pwmConfig.outInvert = true;

  pwmHiInit(&params->inst[0], &pwmConfig, NULL, NULL);
  pwmHiInit(&params->inst[1], &pwmConfig, NULL, NULL);
  pwmHiInit(&params->inst[2], &pwmConfig, NULL, NULL);

  // PWM On
  pwmHiOn(&params->inst[0]);
  pwmHiOn(&params->inst[1]);
  pwmHiOn(&params->inst[2]);

  // Start PWM
  pwmStart(&params->inst[0], _configPWMMode, NULL);

  return params;
}

void* _configure4PWM(
  long pwm_frequency,
  const int pin1A,
  const int pin1B,
  const int pin2A,
  const int pin2B
) {
  EFR32DriverParams *params = new EFR32DriverParams;
  if (!params) return SIMPLEFOC_DRIVER_INIT_FAILED;

  if (!pwm_frequency || !_isset(pwm_frequency)) pwm_frequency = SILABS_DEFAULT_PWM_FREQUENCY;
  else pwm_frequency = _constrain(pwm_frequency, 0, SILABS_DEFAULT_PWM_FREQUENCY);

  params->pwm_frequency = pwm_frequency;
  params->noPwmChannel = 4;

  // Ensure all PWMs use the same TIMER instance
  pwmHiConfig(&params->inst[0], SILABS_DEFAULT_PWM_PERPHERAL, pin1A, 0);
  pwmHiConfig(&params->inst[1], SILABS_DEFAULT_PWM_PERPHERAL, pin1B, 1);
  pwmHiConfig(&params->inst[2], SILABS_DEFAULT_PWM_PERPHERAL, pin2A, 2);
  pwmHiConfig(&params->inst[3], SILABS_SECOND_PWM_PERPHERAL , pin2B, 0);

  // Initialize PWM
  EFR32PwmConfig pwmConfig;
  pwmConfig.frequency = pwm_frequency << 1;
  pwmConfig.polarity = PWM_P_ACTIVE_LOW;
  pwmConfig.outInvert = true;

  pwmHiInit(&params->inst[0], &pwmConfig, NULL, NULL);
  pwmHiInit(&params->inst[1], &pwmConfig, NULL, NULL);
  pwmHiInit(&params->inst[2], &pwmConfig, NULL, NULL);
  pwmHiInit(&params->inst[3], &pwmConfig, _alignPWMTimers, &params->inst[0]);

  // PWM On
  pwmHiOn(&params->inst[0]);
  pwmHiOn(&params->inst[1]);
  pwmHiOn(&params->inst[2]);
  pwmHiOn(&params->inst[3]);

  // Start PWM
  pwmStart(&params->inst[0], _configPWMMode, NULL);
  pwmStart(&params->inst[3], _alignPWMStart, NULL);

  return params;
}

void* _configure6PWM(
  long pwm_frequency,
  float dead_zone,
  const int pinA_h,
  const int pinA_l,
  const int pinB_h,
  const int pinB_l,
  const int pinC_h,
  const int pinC_l
) {
  EFR32DriverParams *params = new EFR32DriverParams;
  if (!params) return SIMPLEFOC_DRIVER_INIT_FAILED;

  if (!pwm_frequency || !_isset(pwm_frequency)) pwm_frequency = SILABS_DEFAULT_PWM_FREQUENCY;
  else pwm_frequency = _constrain(pwm_frequency, 0, SILABS_DEFAULT_PWM_FREQUENCY);

  params->pwm_frequency = pwm_frequency;
  params->dead_zone = (dead_zone == NOT_SET) ? SILABS_DEFAULT_DEAD_ZONE : dead_zone;
  params->lowside = true;
  params->noPwmChannel = 3;

  // Ensure all PWMs use the same TIMER instance
  pwmHiConfig(&params->inst[0], SILABS_DEFAULT_PWM_PERPHERAL, pinA_h, 0);
  pwmLoConfig(&params->inst[0], pinA_l);

  pwmHiConfig(&params->inst[1], SILABS_DEFAULT_PWM_PERPHERAL, pinB_h, 1);
  pwmLoConfig(&params->inst[1], pinB_l);

  pwmHiConfig(&params->inst[2], SILABS_DEFAULT_PWM_PERPHERAL, pinC_h, 2);
  pwmLoConfig(&params->inst[2], pinC_l);

  // Initialize PWM
  EFR32PwmConfig pwmConfig;
  pwmConfig.frequency = pwm_frequency << 1;
  pwmConfig.polarity = PWM_P_ACTIVE_LOW;
  pwmConfig.outInvert = true;

  pwmInit(&params->inst[0], &pwmConfig, NULL, NULL);
  pwmInit(&params->inst[1], &pwmConfig, NULL, NULL);
  pwmInit(&params->inst[2], &pwmConfig, NULL, NULL);

  // Dead Time PWM
  uint32_t deadTimeNs = (float) (1e9f / pwm_frequency) * dead_zone;
  EFR32PwmDeadTimeConfig deadTimeConfig;
  deadTimeConfig.deadTimeNs = deadTimeNs >> 1;
  deadTimeConfig.outputMask = TIMER_DTOGEN_DTOGCC0EN
                              | TIMER_DTOGEN_DTOGCC1EN
                              | TIMER_DTOGEN_DTOGCC2EN
                              | TIMER_DTOGEN_DTOGCDTI0EN
                              | TIMER_DTOGEN_DTOGCDTI1EN
                              | TIMER_DTOGEN_DTOGCDTI2EN;
  pwmDeadTimeInit(&params->inst[0], &deadTimeConfig);

  // PWM On
  pwmOn(&params->inst[0]);
  pwmOn(&params->inst[1]);
  pwmOn(&params->inst[2]);

  // Start PWM
  pwmStart(&params->inst[0], _configPWMMode, NULL);

  return params;
}

void _writeDutyCycle1PWM(float dc_a, void* params) {
  EFR32DriverParams *p = (EFR32DriverParams*) params;
  if (!p) return;

  pwmHiSetDutyCycle(&p->inst[0], dc_a * 100.0f);
}

void _writeDutyCycle2PWM(float dc_a, float dc_b, void* params) {
  EFR32DriverParams *p = (EFR32DriverParams*) params;
  if (!p) return;

  pwmHiSetDutyCycle(&p->inst[0], dc_a * 100.0f);
  pwmHiSetDutyCycle(&p->inst[1], dc_b * 100.0f);
}

void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void* params) {
  EFR32DriverParams *p = (EFR32DriverParams*) params;
  if (!p) return;

  pwmHiSetDutyCycle(&p->inst[0], dc_a * 100.0f);
  pwmHiSetDutyCycle(&p->inst[1], dc_b * 100.0f);
  pwmHiSetDutyCycle(&p->inst[2], dc_c * 100.0f);
}

void _writeDutyCycle4PWM(
  float dc_1a,
  float dc_1b,
  float dc_2a,
  float dc_2b,
  void* params
) {
  EFR32DriverParams *p = (EFR32DriverParams*) params;
  if (!p) return;

  pwmHiSetDutyCycle(&p->inst[0], dc_1a * 100.0f);
  pwmHiSetDutyCycle(&p->inst[1], dc_1b * 100.0f);
  pwmHiSetDutyCycle(&p->inst[2], dc_2a * 100.0f);
  pwmHiSetDutyCycle(&p->inst[3], dc_2b * 100.0f);
}

void _writeDutyCycle6PWM(
  float dc_a,
  float dc_b,
  float dc_c,
  PhaseState *phase_state,
  void* params
) {
  EFR32DriverParams *p = (EFR32DriverParams*) params;
  if (!p || !phase_state) return;

  _setSinglePhaseState(&p->inst[0], phase_state[0]);
  if (phase_state[0] == PhaseState::PHASE_OFF) dc_a = 0.0f;
  pwmHiSetDutyCycle(&p->inst[0], dc_a * 100);

  _setSinglePhaseState(&p->inst[1], phase_state[1]);
  if (phase_state[1] == PhaseState::PHASE_OFF) dc_b = 0.0f;
  pwmHiSetDutyCycle(&p->inst[1], dc_b * 100.0f);

  _setSinglePhaseState(&p->inst[2], phase_state[2]);
  if (phase_state[2] == PhaseState::PHASE_OFF) dc_c = 0.0f;
  pwmHiSetDutyCycle(&p->inst[2], dc_c * 100.0f);
}

#endif