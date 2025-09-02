#include <pinDefinitions.h>

#if defined(ARDUINO_ARCH_SILABS)

#include <em_device.h>
#include <em_gpio.h>
#include <em_bus.h>
#include <em_cmu.h>
#include "efr32_pwm.h"

static CMU_Clock_TypeDef _getTimerClock(TIMER_TypeDef *timer) {
#if defined(_CMU_HFCLKSEL_MASK) || defined(_CMU_CMD_HFCLKSEL_MASK)
  CMU_Clock_TypeDef timerClock = cmuClock_HF;
#elif defined(_CMU_SYSCLKCTRL_MASK)
  CMU_Clock_TypeDef timerClock = cmuClock_SYSCLK;
#else
#error "Unknown root of clock tree"
#endif

  switch ((uint32_t)timer) {
#if defined(TIMER0_BASE)
    case TIMER0_BASE:
      timerClock = cmuClock_TIMER0;
      break;
#endif
#if defined(TIMER1_BASE)
    case TIMER1_BASE:
      timerClock = cmuClock_TIMER1;
      break;
#endif
#if defined(TIMER2_BASE)
    case TIMER2_BASE:
      timerClock = cmuClock_TIMER2;
      break;
#endif
#if defined(TIMER3_BASE)
    case TIMER3_BASE:
      timerClock = cmuClock_TIMER3;
      break;
#endif
#if defined(TIMER4_BASE)
    case TIMER4_BASE:
      timerClock = cmuClock_TIMER4;
      break;
#endif
#if defined(TIMER5_BASE)
    case TIMER5_BASE:
      timerClock = cmuClock_TIMER5;
      break;
#endif
#if defined(TIMER6_BASE)
    case TIMER6_BASE:
      timerClock = cmuClock_TIMER6;
      break;
#endif
#if defined(WTIMER0_BASE)
    case WTIMER0_BASE:
      timerClock = cmuClock_WTIMER0;
      break;
#endif
#if defined(WTIMER1_BASE)
    case WTIMER1_BASE:
      timerClock = cmuClock_WTIMER1;
      break;
#endif
#if defined(WTIMER2_BASE)
    case WTIMER2_BASE:
      timerClock = cmuClock_WTIMER2;
      break;
#endif
#if defined(WTIMER3_BASE)
    case WTIMER3_BASE:
      timerClock = cmuClock_WTIMER3;
      break;
#endif
    default:
      break;
  }
  return timerClock;
}

void pwmHiConfig(
  EFR32PwmInstance *inst,
  TIMER_TypeDef *timer,
  const int pin,
  const uint8_t channel
) {
  if (!inst) return;

  inst->h.timer = timer;
  inst->h.port = getSilabsPortFromArduinoPin(pinToPinName(pin));
  inst->h.pin = getSilabsPinFromArduinoPin(pinToPinName(pin));
  inst->h.channel = channel;
}

void pwmHiInit(
  EFR32PwmInstance *inst,
  EFR32PwmConfig *config,
  prevTimerInitCCFn fn,
  void *params
) {
  if (!inst || !config) return;

  // Enable Timer Clock
  CMU_Clock_TypeDef timerClock = _getTimerClock(inst->h.timer);
  CMU_ClockEnable(timerClock, true);

  // Set PWM pin as output
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet((GPIO_Port_TypeDef)inst->h.port,
                  inst->h.pin,
                  gpioModePushPull,
                  config->polarity);

  // Set CC channel parameters
  TIMER_InitCC_TypeDef initCC = TIMER_INITCC_DEFAULT;
  initCC.mode = timerCCModePWM;
  if (config->outInvert) initCC.outInvert = true;
  if (fn) fn(&initCC, params);

  TIMER_InitCC(inst->h.timer, inst->h.channel, &initCC);

  volatile uint32_t *routeRegister = &GPIO->TIMERROUTE[TIMER_NUM(inst->h.timer)].CC0ROUTE;
  routeRegister += inst->h.channel;
  *routeRegister = (inst->h.port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                    | (inst->h.pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

  // Configure TIMER frequency
  uint32_t top = (CMU_ClockFreqGet(timerClock) / (config->frequency)) - 1U;
  TIMER_TopSet(inst->h.timer, top);

  // Set initial duty cycle to 0%
  TIMER_CompareSet(inst->h.timer, inst->h.channel, 0U);
}

void pwmHiDeinit(
  EFR32PwmInstance *inst
) {
  if (!inst) return;

  pwmHiOff(inst);

  volatile uint32_t *routeRegister = &GPIO->TIMERROUTE[TIMER_NUM(inst->h.timer)].CC0ROUTE;
  routeRegister += inst->h.channel;
  *routeRegister = 0;

  TIMER_Reset(inst->h.timer);
  GPIO_PinModeSet((GPIO_Port_TypeDef)inst->h.port,
                  inst->h.pin,
                  gpioModeDisabled,
                  0);
  CMU_Clock_TypeDef timerClock = _getTimerClock(inst->h.timer);
  CMU_ClockEnable(timerClock, false);
}

void pwmHiOn(
  EFR32PwmInstance *inst
) {
  if (!inst) return;
  GPIO->TIMERROUTE_SET[TIMER_NUM(inst->h.timer)].ROUTEEN = 1 << (inst->h.channel + _GPIO_TIMER_ROUTEEN_CC0PEN_SHIFT);
}

void pwmHiOff(
  EFR32PwmInstance *inst
) {
  if (!inst) return;
  GPIO->TIMERROUTE_CLR[TIMER_NUM(inst->h.timer)].ROUTEEN = 1 << (inst->h.channel + _GPIO_TIMER_ROUTEEN_CC0PEN_SHIFT);
}

void pwmHiSetDutyCycle(
  EFR32PwmInstance *inst,
  float percent
) {
  if (!inst || (percent > 100.0f)) return;
  uint32_t top = TIMER_TopGet(inst->h.timer);
  volatile bool outInvert = inst->h.timer->CC[inst->h.channel].CTRL & TIMER_CC_CTRL_OUTINV;
  if (outInvert) percent = 100 - percent;
  TIMER_CompareBufSet(inst->h.timer, inst->h.channel, (uint32_t) (top * percent) / 100);
}

float pwmHiGetDutyCycle(
  EFR32PwmInstance *inst
) {
  if (!inst) return 0;
  uint32_t top = TIMER_TopGet(inst->h.timer);
  uint32_t compare = TIMER_CaptureGet(inst->h.timer, inst->h.channel);
  volatile bool outInvert = inst->h.timer->CC[inst->h.channel].CTRL & TIMER_CC_CTRL_OUTINV;
  float percent = (float)((compare * 100) / top);
  return outInvert ? (100 - percent) : percent;
}

void pwmLoConfig(
  EFR32PwmInstance *inst,
  const int pin
) {
  if (!inst) return;
  inst->l.port = getSilabsPortFromArduinoPin(pinToPinName(pin));
  inst->l.pin = getSilabsPinFromArduinoPin(pinToPinName(pin));
}

void pwmLoInit(
  EFR32PwmInstance *inst
) {
  if (!inst) return;

  // Low side PWM
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet((GPIO_Port_TypeDef)inst->l.port,
                  inst->l.pin,
                  gpioModePushPull,
                  0);

  volatile uint32_t *routeRegister = &GPIO->TIMERROUTE[TIMER_NUM(inst->h.timer)].CDTI0ROUTE;
  routeRegister += inst->h.channel;
  *routeRegister = (inst->l.port << _GPIO_TIMER_CDTI0ROUTE_PORT_SHIFT)
                   | (inst->l.pin << _GPIO_TIMER_CDTI0ROUTE_PIN_SHIFT);
}

void pwmLoDeinit(
  EFR32PwmInstance *inst
) {
  if (!inst) return;

  pwmLoOff(inst);

  volatile uint32_t *routeRegister = &GPIO->TIMERROUTE[TIMER_NUM(inst->h.timer)].CDTI0ROUTE;
  routeRegister += inst->h.channel;
  *routeRegister = 0;

  GPIO_PinModeSet((GPIO_Port_TypeDef)inst->l.port,
                  inst->l.pin,
                  gpioModeDisabled,
                  0);
}

void pwmLoOn(
  EFR32PwmInstance *inst
) {
  if (!inst) return;
  GPIO->TIMERROUTE_SET[TIMER_NUM(inst->h.timer)].ROUTEEN |= 1 << (inst->h.channel + _GPIO_TIMER_ROUTEEN_CCC0PEN_SHIFT);
}

void pwmLoOff(
  EFR32PwmInstance *inst
) {
  if (!inst) return;
  GPIO->TIMERROUTE_CLR[TIMER_NUM(inst->h.timer)].ROUTEEN |= 1 << (inst->h.channel + _GPIO_TIMER_ROUTEEN_CCC0PEN_SHIFT);
}

void pwmInit(
  EFR32PwmInstance *inst,
  EFR32PwmConfig *config,
  prevTimerInitCCFn fn,
  void *params
) {
  if (!inst || !config) return;
  pwmHiInit(inst, config, fn, params);
  pwmLoInit(inst);
}

void pwmDeinit(
  EFR32PwmInstance *inst
) {
  if (!inst) return;
  pwmHiDeinit(inst);
  pwmLoDeinit(inst);
}

void pwmOff(EFR32PwmInstance *inst) {
  if (!inst) return;
  pwmHiOff(inst);
  pwmLoOff(inst);
}

void pwmOn(EFR32PwmInstance *inst) {
  if (!inst) return;
  pwmHiOn(inst);
  pwmLoOn(inst);
}

void pwmStart(EFR32PwmInstance *inst, prevTimerInitFn fn, void *params) {
  if (!inst) return;

  // Initialize TIMER
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  if (fn) fn(&timerInit, params);
  TIMER_Init(inst->h.timer, &timerInit);
}

void pwmDeadTimeInit(
  EFR32PwmInstance *inst,
  EFR32PwmDeadTimeConfig *config
) {
  if (!inst || !config) return;

  // Enable Timer Clock
  CMU_Clock_TypeDef timerClock = _getTimerClock(inst->h.timer);
  CMU_ClockEnable(timerClock, true);

  unsigned int dtiTime = (CMU_ClockFreqGet(timerClock) / 1e3f) * config->deadTimeNs / 1e6f;
  if (dtiTime > 64) dtiTime = SILABBS_DEFAULT_DEAD_TIME;

  TIMER_InitDTI_TypeDef initDTI = TIMER_INITDTI_DEFAULT;
  initDTI.riseTime = dtiTime;
  initDTI.fallTime = dtiTime;
  initDTI.outputsEnableMask = config->outputMask;

  TIMER_InitDTI(inst->h.timer, &initDTI);
}

#endif