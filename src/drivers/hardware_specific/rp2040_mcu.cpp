
/**
 * Support for the RP2040 MCU, as found on the Raspberry Pi Pico.
 */
#if defined(TARGET_RP2040)

#define SIMPLEFOC_DEBUG_RP2040


#ifdef SIMPLEFOC_DEBUG_RP2040

#ifndef SIMPLEFOC_RP2040_DEBUG_SERIAL
#define SIMPLEFOC_RP2040_DEBUG_SERIAL Serial
#endif

#endif

#include "Arduino.h"




// until I can figure out if this can be quickly read from some register, keep it here.
// it also serves as a marker for what slices are already used.
uint16_t wrapvalues[NUM_PWM_SLICES];


// TODO add checks which channels are already used...

void setupPWM(int pin, long pwm_frequency, bool invert = false) {
	gpio_set_function(pin, GPIO_FUNC_PWM);
	uint slice = pwm_gpio_to_slice_num(pin);
	uint chan = pwm_gpio_to_channel(pin);
	pwm_set_clkdiv_int_frac(slice, 1, 0); // fastest pwm we can get
	pwm_set_phase_correct(slice, true);
	uint16_t wrapvalue = ((125L * 1000L * 1000L) / pwm_frequency) / 2L - 1L;
	if (wrapvalue < 999) wrapvalue = 999; // 66kHz, resolution 1000
	if (wrapvalue > 3299) wrapvalue = 3299; // 20kHz, resolution 3300
#ifdef SIMPLEFOC_DEBUG_RP2040
	SIMPLEFOC_RP2040_DEBUG_SERIAL.print("Configuring pin ");
	SIMPLEFOC_RP2040_DEBUG_SERIAL.print(pin);
	SIMPLEFOC_RP2040_DEBUG_SERIAL.print(" slice ");
	SIMPLEFOC_RP2040_DEBUG_SERIAL.print(slice);
	SIMPLEFOC_RP2040_DEBUG_SERIAL.print(" channel ");
	SIMPLEFOC_RP2040_DEBUG_SERIAL.print(chan);
	SIMPLEFOC_RP2040_DEBUG_SERIAL.print(" frequency ");
	SIMPLEFOC_RP2040_DEBUG_SERIAL.print(pwm_frequency);
	SIMPLEFOC_RP2040_DEBUG_SERIAL.print(" top value ");
	SIMPLEFOC_RP2040_DEBUG_SERIAL.println(wrapvalue);
#endif
	pwm_set_wrap(slice, wrapvalue);
	wrapvalues[slice] = wrapvalue;
	if (invert) {
		if (chan==0)
			hw_write_masked(&pwm_hw->slice[slice].csr, 0x1 << PWM_CH0_CSR_A_INV_LSB, PWM_CH0_CSR_A_INV_BITS);
		else
			hw_write_masked(&pwm_hw->slice[slice].csr, 0x1 << PWM_CH0_CSR_B_INV_LSB, PWM_CH0_CSR_B_INV_BITS);
	}
	pwm_set_chan_level(slice, chan, 0); // switch off initially
}


void syncSlices() {
	for (int i=0;i<NUM_PWM_SLICES;i++) {
		pwm_set_enabled(i, false);
		pwm_set_counter(i, 0);
	}
	// enable all slices
	pwm_set_mask_enabled(0x7F);
}


void _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
	setupPWM(pinA, pwm_frequency);
	setupPWM(pinB, pwm_frequency);
	syncSlices();
}



void _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
	setupPWM(pinA, pwm_frequency);
	setupPWM(pinB, pwm_frequency);
	setupPWM(pinC, pwm_frequency);
	syncSlices();
}




void _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
	setupPWM(pin1A, pwm_frequency);
	setupPWM(pin1B, pwm_frequency);
	setupPWM(pin2A, pwm_frequency);
	setupPWM(pin2B, pwm_frequency);
	syncSlices();
}


int _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
	// non-PIO solution...
	setupPWM(pinA_h, pwm_frequency);
	setupPWM(pinB_h, pwm_frequency);
	setupPWM(pinC_h, pwm_frequency);
	setupPWM(pinA_l, pwm_frequency, true);
	setupPWM(pinB_l, pwm_frequency, true);
	setupPWM(pinC_l, pwm_frequency, true);
	syncSlices();
	return 0;
}





void writeDutyCycle(float val, int pin) {
	uint slice = pwm_gpio_to_slice_num(pin);
	uint chan = pwm_gpio_to_channel(pin);
	pwm_set_chan_level(slice, chan, (wrapvalues[slice]+1) * val);
}





void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB) {
	writeDutyCycle(dc_a, pinA);
	writeDutyCycle(dc_b, pinB);
}



void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC) {
	writeDutyCycle(dc_a, pinA);
	writeDutyCycle(dc_b, pinB);
	writeDutyCycle(dc_c, pinC);
}



void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B) {
	writeDutyCycle(dc_1a, pin1A);
	writeDutyCycle(dc_1b, pin1B);
	writeDutyCycle(dc_2a, pin2A);
	writeDutyCycle(dc_2b, pin2B);
}

inline float swDti(float val, float dt) {
	float ret = dt+val;
	if (ret>1.0) ret = 1.0f;
	return ret;
}

void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l) {
	writeDutyCycle(dc_a, pinA_h);
	writeDutyCycle(swDti(dc_a, dead_zone), pinA_l);
	writeDutyCycle(dc_b, pinB_h);
	writeDutyCycle(swDti(dc_b,dead_zone), pinB_l);
	writeDutyCycle(dc_c, pinC_h);
	writeDutyCycle(swDti(dc_c,dead_zone), pinC_l);
}

#endif
