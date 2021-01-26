
#include "../hardware_api.h"
#include "wiring_private.h"

#if defined(ARDUINO_ARCH_SAMD)


// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}




struct tccChanInfo { int pin; int tccn; int chan; };

// get the TCC channel associated with that pin
tccChanInfo getTCCChannelNr(int pin, bool alternate) {
	tccChanInfo result;
	result.pin = pin;
	result.tccn = -2;
	result.chan = -2;
	PinDescription pinDesc = g_APinDescription[pin];
	//uint32_t attr = pinDesc.ulPinAttribute;
	if (!alternate) { // && (attr & PIN_ATTR_PWM) == PIN_ATTR_PWM) {
		result.tccn = GetTCNumber(pinDesc.ulPWMChannel);
		result.chan = GetTCChannelNumber(pinDesc.ulPWMChannel);
	}
	else { // && (attr & PIN_ATTR_TIMER_ALT) == PIN_ATTR_TIMER_ALT) {
		result.tccn = GetTCNumber(pinDesc.ulTCChannel);
		result.chan = GetTCChannelNumber(pinDesc.ulTCChannel);
	}
	return result;
}




void printAllPinInfos() {
	Serial.println();
	for (int pin=0;pin<PINCOUNT_fn();pin++) {
		PinDescription pinDesc = g_APinDescription[pin];
		uint32_t attr = pinDesc.ulPinAttribute;
		Serial.print("Pin ");
		Serial.print(pin);
		Serial.print("  PWM=");
		Serial.print(((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM));
		Serial.print(" TCC");
		Serial.print(GetTCNumber(pinDesc.ulPWMChannel));
		Serial.print("-");
		Serial.print(GetTCChannelNumber(pinDesc.ulPWMChannel));
		Serial.print("  ALT=");
		Serial.print(((attr & PIN_ATTR_TIMER_ALT) == PIN_ATTR_TIMER_ALT));
		Serial.print(" TCC");
		Serial.print(GetTCNumber(pinDesc.ulTCChannel));
		Serial.print("-");
		Serial.println(GetTCChannelNumber(pinDesc.ulTCChannel));
	}
	Serial.println();
}




#ifndef SIMPLEFOC_SAMD_ALLOW_DIFFERENT_TCCS
#define SIMPLEFOC_SAMD_ALLOW_DIFFERENT_TCCS false
#endif



void printTCCInfo(tccChanInfo& info, bool alternate) {
	Serial.print(info.pin);
	Serial.print(alternate?" alternate TCC":" normal    TCC");
	if (info.tccn>=0) {
		Serial.print(info.tccn);
		Serial.print("-");
		Serial.println(info.chan);
	}
	else
		Serial.println(" None");
}



bool checkPeripheralPermutation(int pin1, int pin2, int pin3, bool p1, bool p2, bool p3) {
	tccChanInfo info1 = getTCCChannelNr(pin1, p1);
	tccChanInfo info2 = getTCCChannelNr(pin2, p2);
	tccChanInfo info3 = getTCCChannelNr(pin3, p3);

	printTCCInfo(info1, p1);
	printTCCInfo(info2, p2);
	printTCCInfo(info3, p3);
	Serial.println();

	if (info1.tccn!=-2
		&& info1.tccn==info2.tccn && info1.tccn==info3.tccn && info2.tccn==info3.tccn
		&& info1.chan!=info2.chan && info1.chan!=info3.chan && info2.chan!=info3.chan
			)
		return true;

	// TODO allow more permissive setups for 3-pwm?

	return false;
}


int checkCompatible(int pin1, int pin2, int pin3) {
	if (checkPeripheralPermutation(pin1, pin2, pin3, false, false, false)) {
		return 0;
	}
	if (checkPeripheralPermutation(pin1, pin2, pin3, false, false, true)) {
		return 1;
	}
	if (checkPeripheralPermutation(pin1, pin2, pin3, false, true, false)) {
		return 2;
	}
	if (checkPeripheralPermutation(pin1, pin2, pin3, false, true, true)) {
		return 3;
	}
	if (checkPeripheralPermutation(pin1, pin2, pin3, true, false, false)) {
		return 4;
	}
	if (checkPeripheralPermutation(pin1, pin2, pin3, true, false, true)) {
		return 5;
	}
	if (checkPeripheralPermutation(pin1, pin2, pin3, true, true, false)) {
		return 6;
	}
	if (checkPeripheralPermutation(pin1, pin2, pin3, true, true, true)) {
		return 7;
	}
	return -1;
}



void writeSAMDDutyCycle(int chaninfo, float dc) {
	Tcc* tcc = (Tcc*)GetTC(chaninfo);
	int chan = GetTCChannelNumber(chaninfo);
	// TODO should we set this in a different way to ensure we don't have over/underflows?
	tcc->CC[chan].reg = tcc->PER.reg * dc;
	// TODO do we need to wait for sync?
	uint32_t chanbit = 0x1<<(TCC_SYNCBUSY_CC0_Pos+chan);
	while ( (tcc->SYNCBUSY.reg & chanbit) > 0 );
}



bool SAMDClockConfigured = false;
bool tccConfigured[TCC_INST_NUM];

void configureSAMDClock() {
	if (!SAMDClockConfigured) {
		SAMDClockConfigured = true;
		for (int i=0;i<TCC_INST_NUM;i++)
			tccConfigured[i] = false;
		REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor N=1: 48MHz/1=48MHz
						GCLK_GENDIV_ID(4);            	// Select Generic Clock (GCLK) 4
		while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

		REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
						 GCLK_GENCTRL_GENEN |         	// Enable GCLK4
						 GCLK_GENCTRL_SRC_DFLL48M |   	// Set the 48MHz clock source
						 GCLK_GENCTRL_ID(4);          	// Select GCLK4
		while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	}
}



void configureTCC(int pin, uint32_t chaninfo, long pwm_frequency) {
	// TODO for the moment we ignore the frequency...
	int tccn = GetTCNumber(chaninfo);
	Tcc* tcc = (Tcc*)GetTC(chaninfo);
	if (!tccConfigured[tccn]) {
		uint32_t GCLK_CLKCTRL_ID_ofthistcc = -1;
		switch (tccn>>1) {
		case 0:
			GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID_TCC0_TCC1;
			break;
		case 1:
			GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID_TCC2_TC3;
			break;
		// TODO support SAMDs with more TCCs!
		default:
			return;
		}
		tccConfigured[tccn] = true;

		// Feed GCLK4 to TCC
	    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4
	                       GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
						   GCLK_CLKCTRL_ID_ofthistcc;   // Feed GCLK4 to tcc
	    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

	    tcc->WAVE.reg |= TCC_WAVE_POL(0xF)|TCC_WAVEB_WAVEGENB_DSBOTH;   // Set wave form configuration
		while ( tcc->SYNCBUSY.bit.WAVE == 1 ); // wait for sync

		tcc->PER.reg = 1024;                 // Set counter Top using the PER register
		while ( tcc->SYNCBUSY.bit.PER == 1 ); // wait for sync

		// set all channels to 0%
		for (int i=0;i<4;i++) { // TODO some of the TCCs don't have 4 channels... can one set them anyway?
			tcc->CC[i].reg = 0;					// start off at 0% duty cycle
			uint32_t chanbit = 0x1<<(TCC_SYNCBUSY_CC0_Pos+i);
		    while ( (tcc->SYNCBUSY.reg & chanbit) > 0 );
		}

	    // Enable TC
		tcc->CTRLA.reg |= TCC_CTRLA_ENABLE | TCC_CTRLA_PRESCALER_DIV1; //48Mhz/1=48Mhz/2(up/down)=24MHz/1024=24KHz
	    while ( tcc->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync
	}


}


void attachTCC(int pin, bool alternate) {
    pinMode(pin, OUTPUT);
    pinPeripheral(pin, alternate?EPioType::PIO_TIMER_ALT:EPioType::PIO_TIMER);
}




/**
 * Configuring PWM frequency, resolution and alignment
 * - Stepper driver - 2PWM setting
 * - hardware specific
 *
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pinA pinA bldc driver
 * @param pinB pinB bldc driver
 */
void _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
	return; // I don't know much about stepper motors... maybe someone can fill in these based on the BLDC methods? Someone with a stepper-setup who can test it?
}












/**
 * Configuring PWM frequency, resolution and alignment
 * - BLDC driver - 3PWM setting
 * - hardware specific
 *
 * SAMD21 will support up to 2 BLDC motors in 3-PWM:
 *  one on TCC0 using 3 of the channels 0,1,2 or 3
 *  one on TCC3 using 3 of the channels 0,1,2 or 3
 * i.e. 8 different pins can be used, but only 4 different signals (WO[x]) on those 8 pins
 *  WO[0] and WO[4] are the same
 *  WO[1] and WO[5] are the same
 *  WO[2] and WO[6] are the same
 *  WO[3] and WO[7] are the same
 *
 * If you're on the Arduino Nano33 IoT, please see the Nano33 IoT pinout diagram to see which TCC0/WO[x]
 * signal is on which pin of the Nano. You can drive one motor on TCC0. For other boards, consult their documentation.
 *
 * Note:
 * That's if we want to keep the signals strictly in sync.
 *
 * If we can accept out-of-sync PWMs on the different phases, we could drive up to 4 BLDCs in 3-PWM mode,
 * using all the TCC channels. (TCC0 & TCC3 - 4 channels each, TCC1 & TCC2 - 2 channels each)
 *
 * All channels will use the same resolution, prescaler and clock, but they will have different start-times leading
 * to misaligned signals.
 *
 *
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pinA pinA bldc driver
 * @param pinB pinB bldc driver
 * @param pinC pinC bldc driver
 */
void _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
	// basically, we just need 3 pins on different channels from the same timer to ensure sync
	// so check that the given pins are on the same timer, and not on the same channel...
	uint32_t chaninfo1 = 0; // TCC0 Channel 0  - more generally:    tccn<<8 | (chan&0xFF)
	uint32_t chaninfo2 = 1; // TCC0 Channel 1
	uint32_t chaninfo3 = 2; // TCC0 Channel 2
	printAllPinInfos();
	int compatibility = checkCompatible(pinA, pinB, pinC);
	if (true) { //(compatibility>=0) {
		// attach pin to timer peripheral
//		attachTCC(pinA, (compatibility&0x1)==1);
//		attachTCC(pinB, (compatibility&0x2)==2);
//		attachTCC(pinC, (compatibility&0x4)==4);
		attachTCC(6, false);
		attachTCC(5, false);
		attachTCC(8, true);

		// set up clock - TODO if we did this right it should be possible to get all TCC units synchronized?
		// e.g. attach all the timers, start them, and then start the clock...
		configureSAMDClock();

		// configure the TCC (waveform, top-value, pre-scaler = frequency)
		configureTCC(6, chaninfo1, pwm_frequency);
		configureTCC(5, chaninfo2, pwm_frequency);
		configureTCC(8, chaninfo3, pwm_frequency);

		printAllPinInfos();
	}
	else
		Serial.println("Bad combination!");
	// TODO what to do if not compatible?
}








/**
 * Configuring PWM frequency, resolution and alignment
 * - Stepper driver - 4PWM setting
 * - hardware specific
 *
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pin1A pin1A stepper driver
 * @param pin1B pin1B stepper driver
 * @param pin2A pin2A stepper driver
 * @param pin2B pin2B stepper driver
 */
void _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
	return; // stepper-code?
}









/**
 * Configuring PWM frequency, resolution and alignment
 * - BLDC driver - 6PWM setting
 * - hardware specific
 *
 * SAMD21 will support up to 2 BLDC motors in 6-PWM:
 *  one on TCC0 using 3 of the channels 0,1,2 or 3
 *  one on TCC3 using 3 of the channels 0,1,2 or 3
 * i.e. 6 out of 8 pins must be used, in the following high/low side pairs:
 *  WO[0] & WO[4]  (high side & low side)
 *  WO[1] & WO[5]
 *  WO[2] & WO[6]
 *  WO[3] & WO[7]
 *
 * If you're on the Arduino Nano33 IoT, please see the Nano33 IoT pinout diagram to see which TCC0/WO[x]
 * signal is on which pin of the Nano. You can drive 1 BLDC on TCC0. For other boards, consult their documentation.
 *
 *
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param dead_zone  duty cycle protection zone [0, 1] - both low and high side low - if applicable
 * @param pinA_h pinA high-side bldc driver
 * @param pinA_l pinA low-side bldc driver
 * @param pinB_h pinA high-side bldc driver
 * @param pinB_l pinA low-side bldc driver
 * @param pinC_h pinA high-side bldc driver
 * @param pinC_l pinA low-side bldc driver
 *
 * @return 0 if config good, -1 if failed
 */
int _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
	// we want to use a TCC channel with 1 non-inverted and 1 inverted output for each phase, with dead-time insertion

}


/**
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - Stepper driver - 2PWM setting
 * - hardware specific
 *
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param pinA  phase A hardware pin number
 * @param pinB  phase B hardware pin number
 */
void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB) {
	return;
}

/**
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - BLDC driver - 3PWM setting
 * - hardware specific
 *
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param dc_c  duty cycle phase C [0, 1]
 * @param pinA  phase A hardware pin number
 * @param pinB  phase B hardware pin number
 * @param pinC  phase C hardware pin number
 */
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC) {
//	int pin = dc_a;
//	bool alternate;
//	if ( g_APinDescription[pin].ulPin & 1 )
//		alternate = ( PORT->Group[g_APinDescription[pin].ulPort].PMUX[g_APinDescription[pin].ulPin >> 1].reg & PORT_PMUX_PMUXO( 0xF ) ) == PORT_PMUX_PMUXO( PIO_TIMER_ALT );
//	else
//		alternate = ( PORT->Group[g_APinDescription[pin].ulPort].PMUX[g_APinDescription[pin].ulPin >> 1].reg & PORT_PMUX_PMUXE( 0xF ) ) == PORT_PMUX_PMUXO( PIO_TIMER_ALT );
//	Tcc* tcc;
//	int chan;
//	if (alternate) {
//		tcc = (Tcc*)GetTC(g_APinDescription[pin].ulTCChannel);
//		chan = GetTCChannelNumber(g_APinDescription[pin].ulTCChannel);
//	}
//	else {
//		tcc = (Tcc*)GetTC(g_APinDescription[pin].ulPWMChannel);
//		chan = GetTCChannelNumber(g_APinDescription[pin].ulPWMChannel);
//	}


//	writeSAMDDutyCycle(pinA, dc_a);
//	writeSAMDDutyCycle(pinB, dc_b);
//	writeSAMDDutyCycle(pinC, dc_c);
	writeSAMDDutyCycle(0, dc_a);
	writeSAMDDutyCycle(1, dc_b);
	writeSAMDDutyCycle(2, dc_c);
	return;
}


/**
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - Stepper driver - 4PWM setting
 * - hardware specific
 *
 * @param dc_1a  duty cycle phase 1A [0, 1]
 * @param dc_1b  duty cycle phase 1B [0, 1]
 * @param dc_2a  duty cycle phase 2A [0, 1]
 * @param dc_2b  duty cycle phase 2B [0, 1]
 * @param pin1A  phase 1A hardware pin number
 * @param pin1B  phase 1B hardware pin number
 * @param pin2A  phase 2A hardware pin number
 * @param pin2B  phase 2B hardware pin number
 */
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
	return;
}

/**
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - BLDC driver - 6PWM setting
 * - hardware specific
 *
 * Note: dead-time must be setup in advance, so parameter "dead_zone" is ignored
 *       the low side pins are automatically driven by the SAMD DTI module, so it is enough to set the high-side
 *       duty cycle.
 *       No sanity checks are perfomed to ensure the pinA, pinB, pinC are the same pins you used in configure method...
 *       so use appropriately.
 *
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param dc_c  duty cycle phase C [0, 1]
 * @param dead_zone  duty cycle protection zone [0, 1] - both low and high side low
 * @param pinA_h  phase A high-side hardware pin number
 * @param pinA_l  phase A low-side hardware pin number
 * @param pinB_h  phase B high-side hardware pin number
 * @param pinB_l  phase B low-side hardware pin number
 * @param pinC_h  phase C high-side hardware pin number
 * @param pinC_l  phase C low-side hardware pin number
 *
 */
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l){
	writeSAMDDutyCycle(pinA_h, dc_a);
	writeSAMDDutyCycle(pinB_h, dc_b);
	writeSAMDDutyCycle(pinC_h, dc_c);
	return;
}







#endif
