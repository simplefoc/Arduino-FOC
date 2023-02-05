

#include "./samd_mcu.h"


#if defined(_SAMD51_)||defined(_SAME51_)



// expected frequency on DPLL, since we don't configure it ourselves. Typically this is the CPU frequency.
// for custom boards or overclockers you can override it using this define.
#ifndef SIMPLEFOC_SAMD51_DPLL_FREQ
#define SIMPLEFOC_SAMD51_DPLL_FREQ 120000000
#endif


#ifndef TCC3_CH0
#define TCC3_CH0 NOT_ON_TIMER
#define TCC3_CH1 NOT_ON_TIMER
#endif

#ifndef TCC4_CH0
#define TCC4_CH0 NOT_ON_TIMER
#define TCC4_CH1 NOT_ON_TIMER
#endif


#ifndef TC4_CH0
#define TC4_CH0 NOT_ON_TIMER
#define TC4_CH1 NOT_ON_TIMER
#endif

#ifndef TC5_CH0
#define TC5_CH0 NOT_ON_TIMER
#define TC5_CH1 NOT_ON_TIMER
#endif

#ifndef TC6_CH0
#define TC6_CH0 NOT_ON_TIMER
#define TC6_CH1 NOT_ON_TIMER
#endif

#ifndef TC7_CH0
#define TC7_CH0 NOT_ON_TIMER
#define TC7_CH1 NOT_ON_TIMER
#endif



//	TCC#   Channels   WO_NUM   Counter size   Fault   Dithering   Output matrix   DTI   SWAP   Pattern generation
//	0         6         8         24-bit       Yes       Yes       Yes            Yes   Yes    Yes
//	1         4         8         24-bit       Yes       Yes       Yes            Yes   Yes    Yes
//	2         3         3         16-bit       Yes       -         Yes            -     -      -
//	3         2         2         16-bit       Yes       -         -              -     -      -
//	4         2         2         16-bit       Yes       -         -              -     -      -


#define NUM_WO_ASSOCIATIONS 72

struct wo_association WO_associations[] = {

		{ PORTB,   9, TC4_CH1, 		1, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0},
		{ PORTA,   4, TC0_CH0, 		0, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0},
		{ PORTA,   5, TC0_CH1, 		1, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0},
		{ PORTA,   6, TC1_CH0, 		0, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0},
		{ PORTA,   7, TC1_CH1, 		1, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0},
		{ PORTC,   4, NOT_ON_TIMER, 0, TCC0_CH0, 	    0, NOT_ON_TIMER, 	0},
		// PC05, PC06, PC07 -> no timers
		{ PORTA,   8, TC0_CH0, 		0, TCC0_CH0,     	0, TCC1_CH0, 		4},
		{ PORTA,   9, TC0_CH1, 		1, TCC0_CH1,     	1, TCC1_CH1, 		5},
		{ PORTA,  10, TC1_CH0, 		0, TCC0_CH2,     	2, TCC1_CH2, 		6},
		{ PORTA,  11, TC1_CH1, 		1, TCC0_CH3,     	3, TCC1_CH3, 		7},
		{ PORTB,  10, TC5_CH0, 		0, TCC0_CH4, 		4, TCC1_CH0, 		0},
		{ PORTB,  11, TC5_CH1, 		1, TCC0_CH5,  		5, TCC1_CH1, 		1},
		{ PORTB,  12, TC4_CH0, 		0, TCC3_CH0, 		0, TCC0_CH0, 		0},
		{ PORTB,  13, TC4_CH1, 		1, TCC3_CH1, 		1, TCC0_CH1, 		1},
		{ PORTB,  14, TC5_CH0, 		0, TCC4_CH0, 		0, TCC0_CH2, 		2},
		{ PORTB,  15, TC5_CH1, 		1, TCC4_CH1, 		1, TCC0_CH3, 		3},
		{ PORTD,   8, NOT_ON_TIMER,	0, TCC0_CH1,     	1, NOT_ON_TIMER, 	0},
		{ PORTD,   9, NOT_ON_TIMER,	0, TCC0_CH2,     	2, NOT_ON_TIMER, 	0},
		{ PORTD,  10, NOT_ON_TIMER,	0, TCC0_CH3,     	3, NOT_ON_TIMER, 	0},
		{ PORTD,  11, NOT_ON_TIMER,	0, TCC0_CH4, 		4, NOT_ON_TIMER, 	0},
		{ PORTD,  12, NOT_ON_TIMER,	0, TCC0_CH5,  		5, NOT_ON_TIMER, 	0},
		{ PORTC,  10, NOT_ON_TIMER,	0, TCC0_CH0,     	0, TCC1_CH0, 		4},
		{ PORTC,  11, NOT_ON_TIMER,	0, TCC0_CH1,     	1, TCC1_CH1, 		5},
		{ PORTC,  12, NOT_ON_TIMER,	0, TCC0_CH2, 		2, TCC1_CH2, 		6},
		{ PORTC,  13, NOT_ON_TIMER,	0, TCC0_CH3, 		3, TCC1_CH3, 		7},
		{ PORTC,  14, NOT_ON_TIMER,	0, TCC0_CH4, 		4, TCC1_CH0, 		0},
		{ PORTC,  15, NOT_ON_TIMER,	0, TCC0_CH5, 		5, TCC1_CH1, 		1},
		{ PORTA,  12, TC2_CH0,		0, TCC0_CH0, 		6, TCC1_CH2, 		2},
		{ PORTA,  13, TC2_CH1,		1, TCC0_CH1, 		7, TCC1_CH3, 		3},
		{ PORTA,  14, TC3_CH0,		0, TCC2_CH0, 		0, TCC1_CH2, 		2},
		{ PORTA,  15, TC3_CH1,		1, TCC2_CH1, 		1, TCC1_CH3, 		3},
		{ PORTA,  16, TC2_CH0,		0, TCC1_CH0, 		0, TCC0_CH4, 		4},
		{ PORTA,  17, TC2_CH1,		1, TCC1_CH1, 		1, TCC0_CH5, 		5},
		{ PORTA,  18, TC3_CH0,		0, TCC1_CH2, 		2, TCC0_CH0, 		6},
		{ PORTA,  19, TC3_CH1,		1, TCC1_CH3, 		3, TCC0_CH1, 		7},
		{ PORTC,  16, NOT_ON_TIMER,	0, TCC0_CH0, 		0, NOT_ON_TIMER, 	0}, // PDEC0
		{ PORTC,  17, NOT_ON_TIMER,	0, TCC0_CH1, 		1, NOT_ON_TIMER, 	0}, // PDEC1
		{ PORTC,  18, NOT_ON_TIMER,	0, TCC0_CH2, 		2, NOT_ON_TIMER, 	0}, // PDEC2
		{ PORTC,  19, NOT_ON_TIMER,	0, TCC0_CH3, 		3, NOT_ON_TIMER, 	0},
		{ PORTC,  20, NOT_ON_TIMER,	0, TCC0_CH4, 		4, NOT_ON_TIMER, 	0},
		{ PORTC,  21, NOT_ON_TIMER,	0, TCC0_CH5, 		5, NOT_ON_TIMER, 	0},
		{ PORTC,  22, NOT_ON_TIMER,	0, TCC0_CH0, 		6, NOT_ON_TIMER, 	0},
		{ PORTC,  23, NOT_ON_TIMER,	0, TCC0_CH1, 		7, NOT_ON_TIMER, 	0},
		{ PORTD,  20, NOT_ON_TIMER,	0, TCC1_CH0, 		0, NOT_ON_TIMER, 	0},
		{ PORTD,  21, NOT_ON_TIMER,	0, TCC1_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTB,  16, TC6_CH0,		0, TCC3_CH0, 		0, TCC0_CH4, 		4},
		{ PORTB,  17, TC6_CH1,		1, TCC3_CH1, 		1, TCC0_CH5,	 	5},
		{ PORTB,  18, NOT_ON_TIMER,	0, TCC1_CH0, 		0, NOT_ON_TIMER, 	0}, // PDEC0
		{ PORTB,  19, NOT_ON_TIMER,	0, TCC1_CH1, 		1, NOT_ON_TIMER, 	0}, // PDEC1
		{ PORTB,  20, NOT_ON_TIMER,	0, TCC1_CH2, 		2, NOT_ON_TIMER, 	0}, // PDEC2
		{ PORTB,  21, NOT_ON_TIMER,	0, TCC1_CH3, 		3, NOT_ON_TIMER, 	0},
		{ PORTA,  20, TC7_CH0,		0, TCC1_CH0, 		4, TCC0_CH0, 		0},
		{ PORTA,  21, TC7_CH1,		1, TCC1_CH1, 		5, TCC0_CH1, 		1},
		{ PORTA,  22, TC4_CH0,		0, TCC1_CH2, 		6, TCC0_CH2, 		2},
		{ PORTA,  23, TC4_CH1,		1, TCC1_CH3, 		7, TCC0_CH3, 		3},
		{ PORTA,  24, TC5_CH0,		0, TCC2_CH2, 		2, NOT_ON_TIMER, 	0}, // PDEC0
		{ PORTA,  25, TC5_CH1,		1, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0}, // PDEC1
		{ PORTB,  22, TC7_CH0,		0, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0}, // PDEC2
		{ PORTB,  23, TC7_CH1,		1, NOT_ON_TIMER, 	0, NOT_ON_TIMER,	0}, // PDEC0
		{ PORTB,  24, NOT_ON_TIMER,	0, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0}, // PDEC1
		{ PORTB,  25, NOT_ON_TIMER,	0, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0}, // PDEC2
		{ PORTB,  26, NOT_ON_TIMER,	0, TCC1_CH2, 		2, NOT_ON_TIMER, 	0},
		{ PORTB,  27, NOT_ON_TIMER,	0, TCC1_CH3, 		3, NOT_ON_TIMER, 	0},
		{ PORTB,  28, NOT_ON_TIMER,	0, TCC1_CH0, 		4, NOT_ON_TIMER, 	0},
		{ PORTB,  29, NOT_ON_TIMER,	0, TCC1_CH1, 		5, NOT_ON_TIMER,	0},
		// PC24-PC28, PA27, RESET -> no TC/TCC peripherals
		{ PORTA,  30, TC6_CH0,		0, TCC2_CH0, 		0, NOT_ON_TIMER, 	0},
		{ PORTA,  31, TC6_CH1,		1, TCC2_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTB,  30, TC0_CH0,		0, TCC4_CH0, 		0, TCC0_CH0, 		6},
		{ PORTB,  31, TC0_CH1,		1, TCC4_CH1, 		1, TCC0_CH1, 		7},
		// PC30, PC31 -> no TC/TCC peripherals
		{ PORTB,   0, TC7_CH0,		0, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0},
		{ PORTB,   1, TC7_CH1,		1, NOT_ON_TIMER, 	0, NOT_ON_TIMER,	0},
		{ PORTB,   2, TC6_CH0,		0, TCC2_CH2, 		2, NOT_ON_TIMER,	0},

};

wo_association ASSOCIATION_NOT_FOUND = { NOT_A_PORT, 0, NOT_ON_TIMER, 0, NOT_ON_TIMER, 0, NOT_ON_TIMER, 0};

#ifndef TCC3_CC_NUM
uint8_t TCC_CHANNEL_COUNT[] = { TCC0_CC_NUM, TCC1_CC_NUM, TCC2_CC_NUM };
#else
uint8_t TCC_CHANNEL_COUNT[] = { TCC0_CC_NUM, TCC1_CC_NUM, TCC2_CC_NUM, TCC3_CC_NUM, TCC4_CC_NUM };
#endif


struct wo_association& getWOAssociation(EPortType port, uint32_t pin) {
	for (int i=0;i<NUM_WO_ASSOCIATIONS;i++) {
		if (WO_associations[i].port==port && WO_associations[i].pin==pin)
			return WO_associations[i];
	}
	return ASSOCIATION_NOT_FOUND;
};


EPioType getPeripheralOfPermutation(int permutation, int pin_position) {
	return ((permutation>>pin_position)&0x01)==0x1?PIO_TCC_PDEC:PIO_TIMER_ALT;
}



void syncTCC(Tcc* TCCx) {
	while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}




void writeSAMDDutyCycle(tccConfiguration* info, float dc) {
	uint8_t tccn = GetTCNumber(info->tcc.chaninfo);
	uint8_t chan = GetTCChannelNumber(info->tcc.chaninfo);
	if (tccn<TCC_INST_NUM) {
		Tcc* tcc = (Tcc*)GetTC(info->tcc.chaninfo);
		// set via CCBUF
//		while ( (tcc->SYNCBUSY.vec.CC & (0x1<<chan)) > 0 );
		tcc->CCBUF[chan].reg = (uint32_t)((info->pwm_res-1) * dc); // TODO pwm frequency!
	}
	else { 
		// we don't support the TC channels on SAMD51, isn't worth it.
	}
}


#define DPLL_CLOCK_NUM 2  	// use GCLK2
#define PWM_CLOCK_NUM 3  	// use GCLK3


/**
 * Configure Clock 4 - we want all simplefoc PWMs to use the same clock. This ensures that
 * any compatible pin combination can be used without having to worry about configuring different
 * clocks.
 */
void configureSAMDClock() {
	if (!SAMDClockConfigured) {
		SAMDClockConfigured = true;						// mark clock as configured
		for (int i=0;i<TCC_INST_NUM;i++)				// mark all the TCCs as not yet configured
			tccConfigured[i] = false;

		// GCLK->GENCTRL[DPLL_CLOCK_NUM].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIV(1)
		// 								  | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val);
		// while (GCLK->SYNCBUSY.vec.GENCTRL&(0x1<<DPLL_CLOCK_NUM));

		// GCLK->PCHCTRL[1].reg = GCLK_PCHCTRL_GEN(DPLL_CLOCK_NUM)|GCLK_PCHCTRL_CHEN;
		// while (GCLK->SYNCBUSY.vec.GENCTRL&(0x1<<DPLL_CLOCK_NUM));

		// OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 0;
		// while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.reg!=0x0);

		// OSCCTRL->Dpll[0].DPLLCTRLB.bit.REFCLK = OSCCTRL_DPLLCTRLB_REFCLK_GCLK_Val;
		// while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.reg!=0x0);
		// OSCCTRL->Dpll[0].DPLLRATIO.reg = 3;
		// while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.reg!=0x0);

		// OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 1;
		// while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.reg!=0x0);

		GCLK->GENCTRL[PWM_CLOCK_NUM].bit.GENEN = 0;
		while (GCLK->SYNCBUSY.vec.GENCTRL&(0x1<<PWM_CLOCK_NUM));

		GCLK->GENCTRL[PWM_CLOCK_NUM].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_IDC
										 //| GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val);
	 	 	 	 	 	 	 	 	 	 | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL0_Val);
		while (GCLK->SYNCBUSY.vec.GENCTRL&(0x1<<PWM_CLOCK_NUM));

#ifdef SIMPLEFOC_SAMD_DEBUG
		SIMPLEFOC_DEBUG("SAMD: Configured clock...");
#endif
	}
}





/**
 * Configure a TCC unit
 * pwm_frequency is fixed at 24kHz for now. We could go slower, but going
 * faster won't be possible without sacrificing resolution.
 */
void configureTCC(tccConfiguration& tccConfig, long pwm_frequency, bool negate, float hw6pwm) {
	// TODO for the moment we ignore the frequency...
	if (!tccConfigured[tccConfig.tcc.tccn]) {
		uint32_t GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_IDs[tccConfig.tcc.tccn];
		GCLK->PCHCTRL[GCLK_CLKCTRL_ID_ofthistcc].reg = GCLK_PCHCTRL_GEN(PWM_CLOCK_NUM)|GCLK_PCHCTRL_CHEN;
		while (GCLK->SYNCBUSY.vec.GENCTRL&(0x1<<PWM_CLOCK_NUM));
	}

	if (tccConfig.tcc.tccn<TCC_INST_NUM) {
		Tcc* tcc = (Tcc*)GetTC(tccConfig.tcc.chaninfo);

		tcc->CTRLA.bit.ENABLE = 0; //switch off tcc
		while ( tcc->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync

		uint8_t invenMask = ~(1<<tccConfig.tcc.chan);	// negate (invert) the signal if needed
		uint8_t invenVal = negate?(1<<tccConfig.tcc.chan):0;
		tcc->DRVCTRL.vec.INVEN = (tcc->DRVCTRL.vec.INVEN&invenMask)|invenVal;
		syncTCC(tcc); // wait for sync

		// work out pwm resolution for desired frequency and constrain to max/min values
		long pwm_resolution = (SIMPLEFOC_SAMD51_DPLL_FREQ/2) / pwm_frequency;
		if (pwm_resolution>SIMPLEFOC_SAMD_MAX_PWM_RESOLUTION) 
			pwm_resolution = SIMPLEFOC_SAMD_MAX_PWM_RESOLUTION;
		if (pwm_resolution<SIMPLEFOC_SAMD_MIN_PWM_RESOLUTION) 
			pwm_resolution = SIMPLEFOC_SAMD_MIN_PWM_RESOLUTION;
		// store for later use
		tccConfig.pwm_res = pwm_resolution;

		if (hw6pwm>0.0) {
			tcc->WEXCTRL.vec.DTIEN |= (1<<tccConfig.tcc.chan);
			tcc->WEXCTRL.bit.DTLS = hw6pwm*(pwm_resolution-1);
			tcc->WEXCTRL.bit.DTHS = hw6pwm*(pwm_resolution-1);
			syncTCC(tcc); // wait for sync
		}

		if (!tccConfigured[tccConfig.tcc.tccn]) {
			tcc->WAVE.reg |= TCC_WAVE_POL(0xF)|TCC_WAVE_WAVEGEN_DSTOP;   // Set wave form configuration - TODO check this... why set like this?
			while ( tcc->SYNCBUSY.bit.WAVE == 1 ); // wait for sync

			tcc->PER.reg = pwm_resolution - 1;                 // Set counter Top using the PER register
			while ( tcc->SYNCBUSY.bit.PER == 1 ); // wait for sync

			// set all channels to 0%
			for (int i=0;i<TCC_CHANNEL_COUNT[tccConfig.tcc.tccn];i++) {
				tcc->CC[i].reg = 0;					// start off at 0% duty cycle
				uint32_t chanbit = 0x1<<(TCC_SYNCBUSY_CC0_Pos+i);
				while ( (tcc->SYNCBUSY.reg & chanbit) > 0 );
			}
		}

		// Enable TCC
		tcc->CTRLA.reg |= TCC_CTRLA_ENABLE | TCC_CTRLA_PRESCALER_DIV1; //48Mhz/1=48Mhz/2(up/down)=24MHz/1024=24KHz
		while ( tcc->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync

#if defined(SIMPLEFOC_SAMD_DEBUG) && !defined(SIMPLEFOC_DISABLE_DEBUG)
		SimpleFOCDebug::print("SAMD: (Re-)Initialized TCC ");
		SimpleFOCDebug::print(tccConfig.tcc.tccn);
		SimpleFOCDebug::print("-");
		SimpleFOCDebug::print(tccConfig.tcc.chan);
		SimpleFOCDebug::print("[");
		SimpleFOCDebug::print(tccConfig.wo);
		SimpleFOCDebug::print("]  pwm res ");
		SimpleFOCDebug::print((int)pwm_resolution);
		SimpleFOCDebug::println();	
#endif
	}
	else if (tccConfig.tcc.tccn>=TCC_INST_NUM) {
		//Tc* tc = (Tc*)GetTC(tccConfig.tcc.chaninfo);

		// disable
		// tc->COUNT8.CTRLA.bit.ENABLE = 0;
		// while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
		// // unfortunately we need the 8-bit counter mode to use the PER register...
		// tc->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_WAVEGEN_NPWM ;
		// while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
		// // meaning prescaler of 8, since TC-Unit has no up/down mode, and has resolution of 250 rather than 1000...
		// tc->COUNT8.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV8_Val ;
		// while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
		// // period is 250, period cannot be higher than 256!
		// tc->COUNT8.PER.reg = SIMPLEFOC_SAMD_PWM_TC_RESOLUTION-1;
		// while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
		// // initial duty cycle is 0
		// tc->COUNT8.CC[tccConfig.tcc.chan].reg = 0;
		// while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
		// // enable
		// tc->COUNT8.CTRLA.bit.ENABLE = 1;
		// while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );

	#ifdef SIMPLEFOC_SAMD_DEBUG
		SIMPLEFOC_DEBUG("SAMD: Not initialized: TC ", tccConfig.tcc.tccn);
		SIMPLEFOC_DEBUG("SAMD: TC units not supported on SAMD51");
	#endif
	}

	// set as configured
	tccConfigured[tccConfig.tcc.tccn] = true;


}





#endif
