
#include "./silabs_mcu.h"


#if defined(ARDUINO_ARCH_SILABS)

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"

#pragma message("")
#pragma message("SimpleFOC: compiling for SiliconLabs")
#pragma message("")


#ifndef SIMPLEFOC_SILABS_MAX_MOTORS
#define SIMPLEFOC_SILABS_MAX_MOTORS 5
#endif

SilabsDriverParams* configured_motors[SIMPLEFOC_SILABS_MAX_MOTORS] = {NULL};
uint8_t num_configured_motors = 0;

void printPortLetter(GPIO_Port_TypeDef port);
int8_t getTimerNumber(TIMER_TypeDef* timer);
CMU_Clock_TypeDef getTimerClock(TIMER_TypeDef* timer);


void setupPWM(int pin_nr, long pwm_frequency, bool active_high, SilabsDriverParams* params, uint8_t index, TIMER_TypeDef* timer, uint8_t channel) {
	PinName pin_n = pinToPinName(pin_nr);
	GPIO_Port_TypeDef port = getSilabsPortFromArduinoPin(pin_n);
	uint32_t pin = getSilabsPinFromArduinoPin(pin_n);
	int8_t timer_nr = getTimerNumber(timer);
  	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModePWM;
	timerCCInit.outInvert = !active_high;
	switch(channel) {
		case 0:
			GPIO->TIMERROUTE[timer_nr].ROUTEEN  |= GPIO_TIMER_ROUTEEN_CC0PEN;
			GPIO->TIMERROUTE[timer_nr].CC0ROUTE = (port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
												| (pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
			break;
		case 1:
			GPIO->TIMERROUTE[timer_nr].ROUTEEN  |= GPIO_TIMER_ROUTEEN_CC1PEN;
			GPIO->TIMERROUTE[timer_nr].CC1ROUTE = (port << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
												| (pin << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
			break;
		case 2:
			GPIO->TIMERROUTE[timer_nr].ROUTEEN  |= GPIO_TIMER_ROUTEEN_CC2PEN;
			GPIO->TIMERROUTE[timer_nr].CC2ROUTE = (port << _GPIO_TIMER_CC2ROUTE_PORT_SHIFT)
												| (pin << _GPIO_TIMER_CC2ROUTE_PIN_SHIFT);
			break;
	}
  	TIMER_InitCC(timer, channel, &timerCCInit);

	params->pins[index] = pin_nr;
	params->timer[index] = timer;
	params->channel[index] = channel;

	SimpleFOCDebug::print("DRV (Silabs): Pin ");
	SimpleFOCDebug::print(pin_nr);
	SimpleFOCDebug::print(" (P");
	printPortLetter(port);
	SimpleFOCDebug::print((int)pin);
	SimpleFOCDebug::print(") on TIMER");
	SimpleFOCDebug::print(getTimerNumber(timer));
	SimpleFOCDebug::print(" CH");
	SimpleFOCDebug::print(channel);
	SimpleFOCDebug::print(" top ");
	SimpleFOCDebug::println((int)params->resolution);
}



bool isTimerUsed(TIMER_TypeDef* timer) {
	for (int i=0;i<num_configured_motors;i++) {
		for (int j=0;j<6;j++) {
			if (configured_motors[i]->timer[j] == timer) return true;
		}
	}
	return false;
}



TIMER_TypeDef* findFreeTimer(int* pins, uint8_t num_pins)  {
	TIMER_TypeDef* max_timer = NULL;
	for (int i=0;i<num_pins;i++) {
		PinName pin_n = pinToPinName(pins[i]);
		GPIO_Port_TypeDef port = getSilabsPortFromArduinoPin(pin_n);
		if(port == gpioPortC || port == gpioPortD) {
			if (max_timer == TIMER4)
				max_timer = TIMER1;
			else if (max_timer == NULL)
				max_timer = TIMER3;
		}
		else {
			if (max_timer == TIMER3)
				max_timer = TIMER1;
			else if (max_timer == NULL)
				max_timer = TIMER4;
		}
	}
	if (max_timer==TIMER4 && isTimerUsed(TIMER4)) max_timer = TIMER2;
	if (max_timer==TIMER3 && isTimerUsed(TIMER3)) max_timer = TIMER1;
	if (max_timer==TIMER2 && isTimerUsed(TIMER2)) max_timer = TIMER1;
	if (max_timer==TIMER1 && isTimerUsed(TIMER1)) max_timer = TIMER0;
	if (max_timer==TIMER0 && isTimerUsed(TIMER0)) max_timer = NULL;
	return max_timer;	
}



void initTimer(TIMER_TypeDef* timer, long pwm_frequency, SilabsDriverParams* params) {
	if (pwm_frequency == NOT_SET || pwm_frequency <= 0) pwm_frequency = SIMPLEFOC_SILABS_DEFAULT_PWM_FREQUENCY;
	CMU_Clock_TypeDef timer_clock = getTimerClock(timer);
	//CMU_ClockEnable(cmuClock_GPIO, true); assume this is done by Arduino core
	CMU_ClockEnable(timer_clock, true); // enable timer clock
  	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.enable = false;
	timerInit.mode = timerModeUpDown;
	unsigned long long max = TIMER_MaxCount(timer) + 1;
	// adjust pre-scaler as needed to get the desired frequency
	timerInit.prescale = (TIMER_Prescale_TypeDef)(CMU_ClockFreqGet(timer_clock) / pwm_frequency / 2 / max);
	TIMER_Init(timer, &timerInit);
	uint32_t timerFreq = CMU_ClockFreqGet(timer_clock) / (timerInit.prescale + 1);
	uint32_t topValue = (timerFreq / pwm_frequency / 2) - 1;
	// TIMER_Enable(timer, true);
	// TIMER_TopSet(timer, topValue); //timer must be enabled to call this
	timer->TOP = topValue;
	params->pwm_frequency = pwm_frequency;
	params->resolution = topValue;
	params->timer[0] = timer;
	configured_motors[num_configured_motors++] = params;
}




	// if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY;
	// else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX);
	// params->pwm_frequency = pwm_frequency;

void* _configure1PWM(long pwm_frequency, const int pinA) {
	SilabsDriverParams* params = new SilabsDriverParams();
	int pins[1] = {pinA};
	TIMER_TypeDef* timer = findFreeTimer(pins, 1);
	initTimer(timer, pwm_frequency, params);
	setupPWM(pinA, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0, timer, 0);
	TIMER_Enable(timer, true);
	return params;
}



void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
	SilabsDriverParams* params = new SilabsDriverParams();
	int pins[2] = {pinA, pinB};
	TIMER_TypeDef* timer = findFreeTimer(pins, 2);
	initTimer(timer, pwm_frequency, params);
	setupPWM(pinA, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0, timer, 0);
	setupPWM(pinB, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1, timer, 1);
	TIMER_Enable(timer, true);
	return params;
}



void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
	SilabsDriverParams* params = new SilabsDriverParams();
	int pins[3] = {pinA, pinB, pinC};
	TIMER_TypeDef* timer = findFreeTimer(pins, 3);
	initTimer(timer, pwm_frequency, params);
	setupPWM(pinA, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0, timer, 0);
	setupPWM(pinB, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1, timer, 1);
	setupPWM(pinC, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 2, timer, 2);
	TIMER_Enable(timer, true);
	return params;
}




void* _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
	SilabsDriverParams* params = new SilabsDriverParams();
	int pins[2] = { pin1A, pin1B };
	TIMER_TypeDef* timer0 = findFreeTimer(pins, 2);
	initTimer(timer0, pwm_frequency, params);
	pins[0] = pin2A; pins[1] = pin2B;
	TIMER_TypeDef* timer1 = findFreeTimer(pins, 2);
	initTimer(timer1, pwm_frequency, params);
	setupPWM(pin1A, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0, timer0, 0);
	setupPWM(pin1B, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1, timer0, 1);
	setupPWM(pin2A, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 2, timer1, 0);
	setupPWM(pin2B, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 3, timer1, 1);
	TIMER_Enable(timer0, true);
	TIMER_Enable(timer1, true);
	return params;
}


void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
	SilabsDriverParams* params = new SilabsDriverParams();
	params->dead_zone = dead_zone;
	// TODO init using DTI if posssible
	// setupPWM(pinA_h, pwm_frequency, SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH, params, 0);
	// setupPWM(pinB_h, pwm_frequency, SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH, params, 2);
	// setupPWM(pinC_h, pwm_frequency, SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH, params, 4);
	// setupPWM(pinA_l, pwm_frequency, SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH, params, 1);
	// setupPWM(pinB_l, pwm_frequency, SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH, params, 3);
	// setupPWM(pinC_l, pwm_frequency, SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH, params, 5);
	return params;
}





void writeDutyCycle(float val, TIMER_TypeDef* timer, uint8_t chan) {
	timer->CC[chan].OC = val * timer->TOP;
	//TIMER_CompareSet(timer, chan, val * timer->TOP);
}




void _writeDutyCycle1PWM(float dc_a, void* params) {
	writeDutyCycle(dc_a, ((SilabsDriverParams*)params)->timer[0], ((SilabsDriverParams*)params)->channel[0]);
}




void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params) {
	writeDutyCycle(dc_a, ((SilabsDriverParams*)params)->timer[0], ((SilabsDriverParams*)params)->channel[0]);
	writeDutyCycle(dc_b, ((SilabsDriverParams*)params)->timer[1], ((SilabsDriverParams*)params)->channel[1]);
}



void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params) {
	writeDutyCycle(dc_a, ((SilabsDriverParams*)params)->timer[0], ((SilabsDriverParams*)params)->channel[0]);
	writeDutyCycle(dc_b, ((SilabsDriverParams*)params)->timer[1], ((SilabsDriverParams*)params)->channel[1]);
	writeDutyCycle(dc_c, ((SilabsDriverParams*)params)->timer[2], ((SilabsDriverParams*)params)->channel[2]);
}



void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params) {
	writeDutyCycle(dc_1a, ((SilabsDriverParams*)params)->timer[0], ((SilabsDriverParams*)params)->channel[0]);
	writeDutyCycle(dc_1b, ((SilabsDriverParams*)params)->timer[1], ((SilabsDriverParams*)params)->channel[1]);
	writeDutyCycle(dc_2a, ((SilabsDriverParams*)params)->timer[2], ((SilabsDriverParams*)params)->channel[2]);
	writeDutyCycle(dc_2b, ((SilabsDriverParams*)params)->timer[3], ((SilabsDriverParams*)params)->channel[3]);
}



CMU_Clock_TypeDef getTimerClock(TIMER_TypeDef* timer) {
	if(timer == TIMER0) return cmuClock_TIMER0;
	if(timer == TIMER1) return cmuClock_TIMER1;
	if(timer == TIMER2) return cmuClock_TIMER2;
	if(timer == TIMER3) return cmuClock_TIMER3;
	if(timer == TIMER4) return cmuClock_TIMER4;
	return cmuClock_TIMER0;
}



int8_t getTimerNumber(TIMER_TypeDef* timer) {
	if(timer == TIMER0) return 0;
	if(timer == TIMER1) return 1;
	if(timer == TIMER2) return 2;
	if(timer == TIMER3) return 3;
	if(timer == TIMER4) return 4;
	return -1;
}

void printPortLetter(GPIO_Port_TypeDef port) {
	if(port == gpioPortA) SimpleFOCDebug::print("A");
	else if(port == gpioPortB) SimpleFOCDebug::print("B");
	else if(port == gpioPortC) SimpleFOCDebug::print("C");
	else if(port == gpioPortD) SimpleFOCDebug::print("D");
	else SimpleFOCDebug::print("?");
}


#endif

