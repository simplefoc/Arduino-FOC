
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

void setupPWM(int pin_nr, long pwm_frequency, bool active_high, SilabsDriverParams* params, uint8_t index) {
	GPIO_Port_TypeDef port = getSilabsPortFromArduinoPin(pin_nr);
	uint32_t pin = getSilabsPinFromArduinoPin(pin_nr);
	TIMER_TypeDef* timer = TIMER0; // TODO determine correct timer
	uint32_t timerFreq = 0;
	uint32_t topValue = 0;
	//CMU_ClockEnable(cmuClock_GPIO, true); assume this is done by Arduino core
	CMU_ClockEnable(cmuClock_TIMER0, true); // enable timer clock

  	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.enable = false;
	timerInit.mode = timerModeUpDown;
	// TODO adjust pre-scaler as needed to get the desired frequency
	uint32_t max = TIMER_MaxCount(timer);
	timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
	topValue = (timerFreq / pwm_frequency);
	TIMER_TopSet(timer, topValue);
  
  	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModePWM;
	timerCCInit.outInvert = !active_high;
	GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  	GPIO->TIMERROUTE[0].CC0ROUTE = (port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                    			 | (pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  	TIMER_InitCC(timer, 0, &timerCCInit);

	// TODO enable all the timers at the same time?
	TIMER_Enable(timer, true);
}


	// if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY;
	// else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX);
	// params->pwm_frequency = pwm_frequency;

void* _configure1PWM(long pwm_frequency, const int pinA) {
	SilabsDriverParams* params = new SilabsDriverParams();
	setupPWM(pinA, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0);
	return params;
}



void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
	SilabsDriverParams* params = new SilabsDriverParams();
	setupPWM(pinA, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0);
	setupPWM(pinB, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1);
	return params;
}



void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
	SilabsDriverParams* params = new SilabsDriverParams();
	setupPWM(pinA, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0);
	setupPWM(pinB, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1);
	setupPWM(pinC, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 2);
	return params;
}




void* _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
	SilabsDriverParams* params = new SilabsDriverParams();
	setupPWM(pin1A, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0);
	setupPWM(pin1B, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1);
	setupPWM(pin2A, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 2);
	setupPWM(pin2B, pwm_frequency, SIMPLEFOC_PWM_ACTIVE_HIGH, params, 3);
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
	//pwm_set_chan_level(slice, chan, (wrapvalues[slice]+1) * val);
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





#endif

