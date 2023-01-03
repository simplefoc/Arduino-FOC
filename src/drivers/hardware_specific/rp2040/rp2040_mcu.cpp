
/**
 * Support for the RP2040 MCU, as found on the Raspberry Pi Pico.
 */
#if defined(TARGET_RP2040)

#define SIMPLEFOC_DEBUG_RP2040

#include "../../hardware_api.h"
#include "./rp2040_mcu.h"
#include "hardware/pwm.h"

#define _PWM_FREQUENCY 24000
#define _PWM_FREQUENCY_MAX 66000
#define _PWM_FREQUENCY_MIN 5000



// until I can figure out if this can be quickly read from some register, keep it here.
// it also serves as a marker for what slices are already used.
uint16_t wrapvalues[NUM_PWM_SLICES];


// TODO add checks which channels are already used...

void setupPWM(int pin, long pwm_frequency, bool invert, RP2040DriverParams* params, uint8_t index) {
	gpio_set_function(pin, GPIO_FUNC_PWM);
	uint slice = pwm_gpio_to_slice_num(pin);
	uint chan = pwm_gpio_to_channel(pin);
	params->pins[index] = pin;
	params->slice[index] = slice;
	params->chan[index] = chan;
	pwm_set_clkdiv_int_frac(slice, 1, 0); // fastest pwm we can get
	pwm_set_phase_correct(slice, true);
	uint16_t wrapvalue = ((125L * 1000L * 1000L) / pwm_frequency) / 2L - 1L;
	if (wrapvalue < 999) wrapvalue = 999; // 66kHz, resolution 1000
	if (wrapvalue > 12499) wrapvalue = 12499; // 20kHz, resolution 12500
#ifdef SIMPLEFOC_DEBUG_RP2040
	SimpleFOCDebug::print("Configuring pin ");
	SimpleFOCDebug::print(pin);
	SimpleFOCDebug::print(" slice ");
	SimpleFOCDebug::print((int)slice);
	SimpleFOCDebug::print(" channel ");
	SimpleFOCDebug::print((int)chan);
	SimpleFOCDebug::print(" frequency ");
	SimpleFOCDebug::print((int)pwm_frequency);
	SimpleFOCDebug::print(" top value ");
	SimpleFOCDebug::println(wrapvalue);
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
	for (uint i=0;i<NUM_PWM_SLICES;i++) {
		pwm_set_enabled(i, false);
		pwm_set_counter(i, 0);
	}
	// enable all slices
	pwm_set_mask_enabled(0xFF);
}



void* _configure1PWM(long pwm_frequency, const int pinA) {
	RP2040DriverParams* params = new RP2040DriverParams();
	if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY;
	else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX);
	params->pwm_frequency = pwm_frequency;
	setupPWM(pinA, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0);
	syncSlices();
	return params;
}



void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
	RP2040DriverParams* params = new RP2040DriverParams();
	if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY;
	else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX);
	params->pwm_frequency = pwm_frequency;
	setupPWM(pinA, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0);
	setupPWM(pinB, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1);
	syncSlices();
	return params;
}



void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
	RP2040DriverParams* params = new RP2040DriverParams();
	if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY;
	else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX);
	params->pwm_frequency = pwm_frequency;
	setupPWM(pinA, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0);
	setupPWM(pinB, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1);
	setupPWM(pinC, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 2);
	syncSlices();
	return params;
}




void* _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
	RP2040DriverParams* params = new RP2040DriverParams();
	if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY;
	else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX);
	params->pwm_frequency = pwm_frequency;
	setupPWM(pin1A, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0);
	setupPWM(pin1B, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1);
	setupPWM(pin2A, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 2);
	setupPWM(pin2B, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 3);
	syncSlices();
	return params;
}


void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
	// non-PIO solution...
	RP2040DriverParams* params = new RP2040DriverParams();
	if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY;
	else pwm_frequency = _constrain(pwm_frequency, _PWM_FREQUENCY_MIN, _PWM_FREQUENCY_MAX);
	params->pwm_frequency = pwm_frequency;
	params->dead_zone = dead_zone;
	setupPWM(pinA_h, pwm_frequency, !SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH, params, 0);
	setupPWM(pinB_h, pwm_frequency, !SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH, params, 2);
	setupPWM(pinC_h, pwm_frequency, !SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH, params, 4);
	setupPWM(pinA_l, pwm_frequency, SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH, params, 1);
	setupPWM(pinB_l, pwm_frequency, SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH, params, 3);
	setupPWM(pinC_l, pwm_frequency, SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH, params, 5);
	syncSlices();
	return params;
}





void writeDutyCycle(float val, uint slice, uint chan) {
	pwm_set_chan_level(slice, chan, (wrapvalues[slice]+1) * val);
}




void _writeDutyCycle1PWM(float dc_a, void* params) {
	writeDutyCycle(dc_a, ((RP2040DriverParams*)params)->slice[0], ((RP2040DriverParams*)params)->chan[0]);
}




void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params) {
	writeDutyCycle(dc_a, ((RP2040DriverParams*)params)->slice[0], ((RP2040DriverParams*)params)->chan[0]);
	writeDutyCycle(dc_b, ((RP2040DriverParams*)params)->slice[1], ((RP2040DriverParams*)params)->chan[1]);
}



void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params) {
	writeDutyCycle(dc_a, ((RP2040DriverParams*)params)->slice[0], ((RP2040DriverParams*)params)->chan[0]);
	writeDutyCycle(dc_b, ((RP2040DriverParams*)params)->slice[1], ((RP2040DriverParams*)params)->chan[1]);
	writeDutyCycle(dc_c, ((RP2040DriverParams*)params)->slice[2], ((RP2040DriverParams*)params)->chan[2]);
}



void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params) {
	writeDutyCycle(dc_1a, ((RP2040DriverParams*)params)->slice[0], ((RP2040DriverParams*)params)->chan[0]);
	writeDutyCycle(dc_1b, ((RP2040DriverParams*)params)->slice[1], ((RP2040DriverParams*)params)->chan[1]);
	writeDutyCycle(dc_2a, ((RP2040DriverParams*)params)->slice[2], ((RP2040DriverParams*)params)->chan[2]);
	writeDutyCycle(dc_2b, ((RP2040DriverParams*)params)->slice[3], ((RP2040DriverParams*)params)->chan[3]);
}

inline float swDti(float val, float dt) {
	float ret = dt+val;
	if (ret>1.0) ret = 1.0f;
	return ret;
}

void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params) {
	if (phase_state[0]==PhaseState::PHASE_ON || phase_state[0]==PhaseState::PHASE_HI)
		writeDutyCycle(dc_a, ((RP2040DriverParams*)params)->slice[0], ((RP2040DriverParams*)params)->chan[0]);
	else
		writeDutyCycle(0.0f, ((RP2040DriverParams*)params)->slice[0], ((RP2040DriverParams*)params)->chan[0]);
	if (phase_state[0]==PhaseState::PHASE_ON || phase_state[0]==PhaseState::PHASE_LO)
		writeDutyCycle(swDti(dc_a, ((RP2040DriverParams*)params)->dead_zone), ((RP2040DriverParams*)params)->slice[1], ((RP2040DriverParams*)params)->chan[1]);
	else
		writeDutyCycle(0.0f, ((RP2040DriverParams*)params)->slice[1], ((RP2040DriverParams*)params)->chan[1]);

	if (phase_state[1]==PhaseState::PHASE_ON || phase_state[1]==PhaseState::PHASE_HI)
		writeDutyCycle(dc_b, ((RP2040DriverParams*)params)->slice[2], ((RP2040DriverParams*)params)->chan[2]);
	else
		writeDutyCycle(0.0f, ((RP2040DriverParams*)params)->slice[2], ((RP2040DriverParams*)params)->chan[2]);
	if (phase_state[1]==PhaseState::PHASE_ON || phase_state[1]==PhaseState::PHASE_LO)
		writeDutyCycle(swDti(dc_b, ((RP2040DriverParams*)params)->dead_zone), ((RP2040DriverParams*)params)->slice[3], ((RP2040DriverParams*)params)->chan[3]);
	else
		writeDutyCycle(0.0f, ((RP2040DriverParams*)params)->slice[3], ((RP2040DriverParams*)params)->chan[3]);

	if (phase_state[2]==PhaseState::PHASE_ON || phase_state[2]==PhaseState::PHASE_HI)
		writeDutyCycle(dc_c, ((RP2040DriverParams*)params)->slice[4], ((RP2040DriverParams*)params)->chan[4]);
	else
		writeDutyCycle(0.0f, ((RP2040DriverParams*)params)->slice[4], ((RP2040DriverParams*)params)->chan[4]);
	if (phase_state[2]==PhaseState::PHASE_ON || phase_state[2]==PhaseState::PHASE_LO)
		writeDutyCycle(swDti(dc_c, ((RP2040DriverParams*)params)->dead_zone), ((RP2040DriverParams*)params)->slice[5], ((RP2040DriverParams*)params)->chan[5]);
	else
		writeDutyCycle(0.0f, ((RP2040DriverParams*)params)->slice[5], ((RP2040DriverParams*)params)->chan[5]);

	_UNUSED(phase_state);
}

#endif
