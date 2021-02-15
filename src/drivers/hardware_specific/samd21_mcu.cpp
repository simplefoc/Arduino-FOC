

#if defined(ARDUINO_ARCH_SAMD)

#include "../hardware_api.h"
#include "wiring_private.h"

#include "./samd21_wo_associations.h"


#ifndef SIMPLEFOC_SAMD_ALLOW_DIFFERENT_TCCS
#define SIMPLEFOC_SAMD_ALLOW_DIFFERENT_TCCS false
#endif

#ifndef SIMPLEFOC_SAMD_PWM_RESOLUTION
#define SIMPLEFOC_SAMD_PWM_RESOLUTION 1000
#define SIMPLEFOC_SAMD_PWM_TC_RESOLUTION 250
#endif

#ifndef SIMPLEFOC_SAMD_MAX_TCC_PINCONFIGURATIONS
#define SIMPLEFOC_SAMD_MAX_TCC_PINCONFIGURATIONS 12
#endif

#ifndef SIMPLEFOC_SAMD_DEAD_TIME
#define SIMPLEFOC_SAMD_DEAD_TIME 50
#endif
#define SIMPLEFOC_SAMD_DEAD_TIME_VAL (SIMPLEFOC_SAMD_DEAD_TIME*1.0/SIMPLEFOC_SAMD_PWM_RESOLUTION)


// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}



struct tccConfiguration {
	uint8_t pin;
	uint8_t alternate; // 1=true, 0=false
	uint8_t wo;
	union tccChanInfo {
		struct {
			int8_t chan;
			int8_t tccn;
		};
		uint16_t chaninfo;
	} tcc;
};




// Read count
//TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
//while (TCC0->SYNCBUSY.bit.CTRLB); // or while (TCC0->SYNCBUSY.reg);
//int count = TCC0->COUNT.reg;



void printAllPinInfos() {
	Serial.println();
	for (int pin=0;pin<PINS_COUNT;pin++) {
		const PinDescription& pinDesc = g_APinDescription[pin];
		wo_association& association = getWOAssociation(pinDesc.ulPort, pinDesc.ulPin);
		uint32_t attr = pinDesc.ulPinAttribute;
		Serial.print("Pin ");
		if (pin<10) Serial.print("0");
		Serial.print(pin);
#ifdef PIN_ATTR_PWM //SAMD21
		Serial.print("  PWM=");
		Serial.print(((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM));
#endif
#ifdef PIN_ATTR_PWM_E //SAMD51
		Serial.print("  PWM_E=");
		Serial.print(((attr & PIN_ATTR_PWM_E) == PIN_ATTR_PWM_E));
		Serial.print("  PWM_F=");
		Serial.print(((attr & PIN_ATTR_PWM_F) == PIN_ATTR_PWM_F));
		Serial.print("  PWM_G=");
		Serial.print(((attr & PIN_ATTR_PWM_G) == PIN_ATTR_PWM_G));
#endif
		if (association.tccE>=0) {
			int tcn = GetTCNumber(association.tccE);
			if (tcn>=TCC_INST_NUM)
				Serial.print("  TC");
			else
				Serial.print(" TCC");
			Serial.print(tcn);
			Serial.print("-");
			Serial.print(GetTCChannelNumber(association.tccE));
			Serial.print("[");
			Serial.print(GetTCChannelNumber(association.woE));
			Serial.print("]");
		}
		else
			Serial.print("  None    ");
		Serial.print("  TIM=");
		Serial.print(((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER));
		Serial.print("  ALT=");
		Serial.print(((attr & PIN_ATTR_TIMER_ALT) == PIN_ATTR_TIMER_ALT));
		if (association.tccF>=0) {
			int tcn = GetTCNumber(association.tccF);
			if (tcn>=TCC_INST_NUM)
				Serial.print("  TC");
			else
				Serial.print(" TCC");
			Serial.print(tcn);
			Serial.print("-");
			Serial.print(GetTCChannelNumber(association.tccF));
			Serial.print("[");
			Serial.print(GetTCChannelNumber(association.woF));
			Serial.println("]");
		}
		else
			Serial.println("  None ");

	}
	Serial.println();
}

void printTCCConfiguration(tccConfiguration& info) {
	Serial.print(info.pin);
	Serial.print((info.alternate==1)?" alternate TCC":" normal    TCC");
	if (info.tcc.tccn>=0) {
		Serial.print(info.tcc.tccn);
		Serial.print("-");
		Serial.println(info.tcc.chan);
	}
	else
		Serial.println(" None");
}









/**
 * Global state
 */
tccConfiguration tccPinConfigurations[SIMPLEFOC_SAMD_MAX_TCC_PINCONFIGURATIONS];
uint8_t numTccPinConfigurations = 0;
bool SAMDClockConfigured = false;
bool tccConfigured[TCC_INST_NUM+TC_INST_NUM];

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
		REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor N=1: 48MHz/1=48MHz
						GCLK_GENDIV_ID(4);            	// Select Generic Clock (GCLK) 4
		while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

		REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
						 GCLK_GENCTRL_GENEN |         	// Enable GCLK4
						 GCLK_GENCTRL_SRC_DFLL48M |   	// Set the 48MHz clock source
						 GCLK_GENCTRL_ID(4);          	// Select GCLK4
		while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

		Serial.println("Configured clock...");

	}
}




/**
 * Configure a TCC unit
 * pwm_frequency is fixed at 24kHz for now. We could go slower, but going
 * faster won't be possible without sacrificing resolution.
 */
void configureTCC(tccConfiguration& tccConfig, long pwm_frequency, bool negate=false, bool hw6pwm=false) {
	// TODO for the moment we ignore the frequency...
	if (!tccConfigured[tccConfig.tcc.tccn]) {
		uint32_t GCLK_CLKCTRL_ID_ofthistcc = -1;
		switch (tccConfig.tcc.tccn>>1) {
		case 0:
			GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID(GCM_TCC0_TCC1);//GCLK_CLKCTRL_ID_TCC0_TCC1;
			break;
		case 1:
			GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID(GCM_TCC2_TC3);//GCLK_CLKCTRL_ID_TCC2_TC3;
			break;
		case 2:
			GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID(GCM_TC4_TC5);//GCLK_CLKCTRL_ID_TC4_TC5;
			break;
		case 3:
			GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID(GCM_TC6_TC7);
			break;
		default:
			return;
		}

		// Feed GCLK4 to TCC
		REG_GCLK_CLKCTRL = (uint16_t)  GCLK_CLKCTRL_CLKEN |         // Enable GCLK4
									   GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
									   GCLK_CLKCTRL_ID_ofthistcc;   // Feed GCLK4 to tcc
		while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

		tccConfigured[tccConfig.tcc.tccn] = true;

		if (tccConfig.tcc.tccn>=TCC_INST_NUM) {
			Tc* tc = (Tc*)GetTC(tccConfig.tcc.chaninfo);

			// disable
			tc->COUNT8.CTRLA.bit.ENABLE = 0;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// unfortunately we need the 8-bit counter mode to use the PER register...
			tc->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_WAVEGEN_NPWM ;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// meaning prescaler of 8, since TC-Unit has no up/down mode, and has resolution of 250 rather than 1000...
			tc->COUNT8.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV8_Val ;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// period is 250, period cannot be higher than 256!
			tc->COUNT8.PER.reg = SIMPLEFOC_SAMD_PWM_TC_RESOLUTION-1;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// initial duty cycle is 0
			tc->COUNT8.CC[tccConfig.tcc.chan].reg = 0;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// enable
			tc->COUNT8.CTRLA.bit.ENABLE = 1;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );

			Serial.print("Initialized TC ");
			Serial.println(tccConfig.tcc.tccn);
		}
		else {
			Tcc* tcc = (Tcc*)GetTC(tccConfig.tcc.chaninfo);

			uint8_t invenMask = ~(1<<tccConfig.tcc.chan);	// negate (invert) the signal if needed
			uint8_t invenVal = negate?(1<<tccConfig.tcc.chan):0;
			tcc->DRVCTRL.vec.INVEN = (tcc->DRVCTRL.vec.INVEN&invenMask)|invenVal;
			syncTCC(tcc); // wait for sync

			tcc->WAVE.reg |= TCC_WAVE_POL(0xF)|TCC_WAVEB_WAVEGENB_DSBOTH;   // Set wave form configuration
			while ( tcc->SYNCBUSY.bit.WAVE == 1 ); // wait for sync

			if (hw6pwm) {
				tcc->WEXCTRL.vec.DTIEN |= (1<<tccConfig.tcc.chan);
				tcc->WEXCTRL.bit.DTLS = SIMPLEFOC_SAMD_DEAD_TIME;
				tcc->WEXCTRL.bit.DTHS = SIMPLEFOC_SAMD_DEAD_TIME;
				syncTCC(tcc); // wait for sync
			}

			tcc->PER.reg = SIMPLEFOC_SAMD_PWM_RESOLUTION - 1;                 // Set counter Top using the PER register
			while ( tcc->SYNCBUSY.bit.PER == 1 ); // wait for sync

			// set all channels to 0%
			uint8_t chanCount = (tccConfig.tcc.tccn==1||tccConfig.tcc.tccn==2)?2:4;
			for (int i=0;i<chanCount;i++) { // TODO some of the TCCs don't have 4 channels... can one set them anyway?
				tcc->CC[i].reg = 0;					// start off at 0% duty cycle
				uint32_t chanbit = 0x1<<(TCC_SYNCBUSY_CC0_Pos+i);
				while ( (tcc->SYNCBUSY.reg & chanbit) > 0 );
			}

			// enable double buffering
			//tcc->CTRLBCLR.bit.LUPD = 1;
			//while ( tcc->SYNCBUSY.bit.CTRLB == 1 );

			// Enable TC
			tcc->CTRLA.reg |= TCC_CTRLA_ENABLE | TCC_CTRLA_PRESCALER_DIV1; //48Mhz/1=48Mhz/2(up/down)=24MHz/1024=24KHz
			while ( tcc->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync

			Serial.print("    Initialized TCC ");
			Serial.print(tccConfig.tcc.tccn);
			Serial.print("-");
			Serial.print(tccConfig.tcc.chan);
			Serial.print("[");
			Serial.print(tccConfig.wo);
			Serial.println("]");
		}
	}
	else if (tccConfig.tcc.tccn<TCC_INST_NUM) {
		// set invert bit for TCC channels in SW 6-PWM...
		Tcc* tcc = (Tcc*)GetTC(tccConfig.tcc.chaninfo);

		tcc->CTRLA.bit.ENABLE = 0;
		while ( tcc->SYNCBUSY.bit.ENABLE == 1 );

		uint8_t invenMask = ~(1<<tccConfig.tcc.chan);	// negate (invert) the signal if needed
		uint8_t invenVal = negate?(1<<tccConfig.tcc.chan):0;
		tcc->DRVCTRL.vec.INVEN = (tcc->DRVCTRL.vec.INVEN&invenMask)|invenVal;
		syncTCC(tcc); // wait for sync

		if (hw6pwm) {
			tcc->WEXCTRL.vec.DTIEN |= (1<<tccConfig.tcc.chan);
			tcc->WEXCTRL.bit.DTLS = SIMPLEFOC_SAMD_DEAD_TIME;
			tcc->WEXCTRL.bit.DTHS = SIMPLEFOC_SAMD_DEAD_TIME;
			syncTCC(tcc); // wait for sync
		}

		tcc->CTRLA.bit.ENABLE = 1;
		while ( tcc->SYNCBUSY.bit.ENABLE == 1 );

		Serial.print("(Re-)Initialized TCC ");
		Serial.print(tccConfig.tcc.tccn);
		Serial.print("-");
		Serial.print(tccConfig.tcc.chan);
		Serial.print("[");
		Serial.print(tccConfig.wo);
		Serial.println("]");
	}


}





/**
 * Attach the TCC to the pin
 */
bool attachTCC(tccConfiguration& tccConfig) {
	if (numTccPinConfigurations>=SIMPLEFOC_SAMD_MAX_TCC_PINCONFIGURATIONS)
		return false;
    pinMode(tccConfig.pin, OUTPUT);
    pinPeripheral(tccConfig.pin, (tccConfig.alternate==1)?EPioType::PIO_TIMER_ALT:EPioType::PIO_TIMER);
    tccPinConfigurations[numTccPinConfigurations++] = tccConfig;
    return true;
}






/**
 * Check if the configuration is in use already.
 */
bool inUse(tccConfiguration& tccConfig) {
	for (int i=0;i<numTccPinConfigurations;i++) {
		if (tccPinConfigurations[i].tcc.chaninfo==tccConfig.tcc.chaninfo)
			return true;
	}
	return false;
}


tccConfiguration* getTccPinConfiguration(uint8_t pin) {
	for (int i=0;i<numTccPinConfigurations;i++)
		if (tccPinConfigurations[i].pin==pin)
			return &tccPinConfigurations[i];
	return NULL;
}





/**
 * Get the TCC channel associated with that pin
 */
tccConfiguration getTCCChannelNr(int pin, bool alternate) {
	tccConfiguration result;
	result.pin = pin;
	result.alternate = alternate?1:0;
	result.tcc.tccn = -2;
	result.tcc.chan = -2;
	const PinDescription& pinDesc = g_APinDescription[pin];
	struct wo_association& association = getWOAssociation(pinDesc.ulPort, pinDesc.ulPin);
	if (association.port==NOT_A_PORT)
		return result; // could not find the port/pin
	if (!alternate) { // && (attr & PIN_ATTR_PWM) == PIN_ATTR_PWM) {
		result.tcc.chaninfo = association.tccE;
		result.wo = association.woE;
	}
	else { // && (attr & PIN_ATTR_TIMER_ALT) == PIN_ATTR_TIMER_ALT) {
		result.tcc.chaninfo = association.tccF;
		result.wo = association.woF;
	}
	return result;
}





bool checkPeripheralPermutationSameTCC3(tccConfiguration& pin1, tccConfiguration& pin2, tccConfiguration& pin3) {
	printTCCConfiguration(pin1);
	printTCCConfiguration(pin2);
	printTCCConfiguration(pin3);
	Serial.println();

	if (inUse(pin3) || inUse(pin2) || inUse(pin3))
		return false;

	if (pin1.tcc.tccn!=-2
		&& pin1.tcc.tccn==pin2.tcc.tccn && pin1.tcc.tccn==pin3.tcc.tccn && pin2.tcc.tccn==pin3.tcc.tccn
		&& pin1.tcc.chan!=pin2.tcc.chan && pin1.tcc.chan!=pin3.tcc.chan && pin2.tcc.chan!=pin3.tcc.chan
			)
		return true;

	return false;
}

bool checkPeripheralPermutationSameTCC6(tccConfiguration& pinAh, tccConfiguration& pinAl, tccConfiguration& pinBh, tccConfiguration& pinBl, tccConfiguration& pinCh, tccConfiguration& pinCl) {
	printTCCConfiguration(pinAh);
	printTCCConfiguration(pinAl);
	printTCCConfiguration(pinBh);
	printTCCConfiguration(pinBl);
	printTCCConfiguration(pinCh);
	printTCCConfiguration(pinCl);
	Serial.println();

	// TODO consider WOs
	if (inUse(pinAh) || inUse(pinAl) || inUse(pinBh) || inUse(pinBl) || inUse(pinCh) || inUse(pinCl))
		return false;

	if (pinAh.tcc.tccn<0 || pinAh.tcc.tccn!=pinAl.tcc.tccn || pinAh.tcc.tccn!=pinBh.tcc.tccn || pinAh.tcc.tccn!=pinBl.tcc.tccn
		|| pinAh.tcc.tccn!=pinCh.tcc.tccn || pinAh.tcc.tccn!=pinCl.tcc.tccn || pinAh.tcc.tccn>=TCC_INST_NUM)
		return false;

	if (pinAh.tcc.chan==pinBh.tcc.chan || pinAh.tcc.chan==pinBl.tcc.chan || pinAh.tcc.chan==pinCh.tcc.chan || pinAh.tcc.chan==pinCl.tcc.chan)
		return false;
	if (pinBh.tcc.chan==pinCh.tcc.chan || pinBh.tcc.chan==pinCl.tcc.chan)
		return false;
	if (pinAl.tcc.chan==pinBh.tcc.chan || pinAl.tcc.chan==pinBl.tcc.chan || pinAl.tcc.chan==pinCh.tcc.chan || pinAl.tcc.chan==pinCl.tcc.chan)
		return false;
	if (pinBl.tcc.chan==pinCh.tcc.chan || pinBl.tcc.chan==pinCl.tcc.chan)
		return false;

	if (pinAh.tcc.chan!=pinAl.tcc.chan || pinBh.tcc.chan!=pinBl.tcc.chan || pinCh.tcc.chan!=pinCl.tcc.chan)
		return false;
	if (pinAh.wo==pinAl.wo || pinBh.wo==pinBl.wo || pinCh.wo!=pinCl.wo)
		return false;

	return true;
}




bool checkPeripheralPermutationCompatible3(tccConfiguration& pin1, tccConfiguration& pin2, tccConfiguration& pin3) {
	if (pin1.tcc.tccn<0 || pin2.tcc.tccn<0 || pin3.tcc.tccn<0) // must have a timer
		return false;
	if (pin1.tcc.tccn==pin2.tcc.tccn && pin1.tcc.chan==pin2.tcc.chan) // can't be on same timer and channel
		return false;
	if (pin1.tcc.tccn==pin3.tcc.tccn && pin1.tcc.chan==pin3.tcc.chan)
		return false;
	if (pin2.tcc.tccn==pin3.tcc.tccn && pin2.tcc.chan==pin3.tcc.chan)
		return false;
	if (inUse(pin3) || inUse(pin2) || inUse(pin3))
		return false;
	return true;
}





bool checkPeripheralPermutationCompatible6(tccConfiguration& pinAh, tccConfiguration& pinAl, tccConfiguration& pinBh, tccConfiguration& pinBl, tccConfiguration& pinCh, tccConfiguration& pinCl) {
	printTCCConfiguration(pinAh);
	printTCCConfiguration(pinAl);
	printTCCConfiguration(pinBh);
	printTCCConfiguration(pinBl);
	printTCCConfiguration(pinCh);
	printTCCConfiguration(pinCl);
	Serial.println();

	// check we're valid PWM pins
	if (pinAh.tcc.tccn<0 || pinAl.tcc.tccn<0 || pinBh.tcc.tccn<0 || pinBl.tcc.tccn<0 || pinCh.tcc.tccn<0 || pinCl.tcc.tccn<0)
		return false;
	// only TCC units for 6-PWM
	if (pinAh.tcc.tccn>=TCC_INST_NUM || pinAl.tcc.tccn>=TCC_INST_NUM || pinBh.tcc.tccn>=TCC_INST_NUM
			|| pinBl.tcc.tccn>=TCC_INST_NUM || pinCh.tcc.tccn>=TCC_INST_NUM || pinCl.tcc.tccn>=TCC_INST_NUM)
		return false;

	// check we're not in use
	if (inUse(pinAh) || inUse(pinAl) || inUse(pinBh) || inUse(pinBl) || inUse(pinCh) || inUse(pinCl))
		return false;

	// check pins are all different tccs/channels
	if (pinAh.tcc.chaninfo==pinBh.tcc.chaninfo || pinAh.tcc.chaninfo==pinBl.tcc.chaninfo || pinAh.tcc.chaninfo==pinCh.tcc.chaninfo || pinAh.tcc.chaninfo==pinCl.tcc.chaninfo)
		return false;
	if (pinAl.tcc.chaninfo==pinBh.tcc.chaninfo || pinAl.tcc.chaninfo==pinBl.tcc.chaninfo || pinAl.tcc.chaninfo==pinCh.tcc.chaninfo || pinAl.tcc.chaninfo==pinCl.tcc.chaninfo)
		return false;
	if (pinBh.tcc.chaninfo==pinCh.tcc.chaninfo || pinBh.tcc.chaninfo==pinCl.tcc.chaninfo)
		return false;
	if (pinBl.tcc.chaninfo==pinCh.tcc.chaninfo || pinBl.tcc.chaninfo==pinCl.tcc.chaninfo)
		return false;

	// check H/L pins are on same timer
	if (pinAh.tcc.tccn!=pinAl.tcc.tccn || pinBh.tcc.tccn!=pinBl.tcc.tccn || pinCh.tcc.tccn!=pinCl.tcc.tccn)
		return false;

	// check H/L pins aren't on both the same timer and wo
	if (pinAh.wo==pinAl.wo || pinBh.wo==pinBl.wo || pinCh.wo==pinCl.wo)
		return false;

	return true;
}





int checkSameTCC3(int pin1, int pin2, int pin3) {
	for (int i=0;i<8;i++) {
		tccConfiguration pinA = getTCCChannelNr(pin1, (i>>0&0x01)==0x1);
		tccConfiguration pinB = getTCCChannelNr(pin2, (i>>1&0x01)==0x1);
		tccConfiguration pinC = getTCCChannelNr(pin3, (i>>2&0x01)==0x1);
		if (checkPeripheralPermutationSameTCC3(pinA, pinB, pinC))
			return i;
	}
	return -1;
}


int checkCompatible3(int pin1, int pin2, int pin3) {
	for (int i=0;i<8;i++) {
		tccConfiguration pinA = getTCCChannelNr(pin1, (i>>0&0x01)==0x1);
		tccConfiguration pinB = getTCCChannelNr(pin2, (i>>1&0x01)==0x1);
		tccConfiguration pinC = getTCCChannelNr(pin3, (i>>2&0x01)==0x1);
		if (checkPeripheralPermutationCompatible3(pinA, pinB, pinC))
			return i;
	}
	return -1;
}




int checkHardware6PWM(const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
	for (int i=0;i<64;i++) {
		tccConfiguration pinAh = getTCCChannelNr(pinA_h, (i>>0&0x01)==0x1);
		tccConfiguration pinAl = getTCCChannelNr(pinA_l, (i>>1&0x01)==0x1);
		tccConfiguration pinBh = getTCCChannelNr(pinB_h, (i>>2&0x01)==0x1);
		tccConfiguration pinBl = getTCCChannelNr(pinB_l, (i>>3&0x01)==0x1);
		tccConfiguration pinCh = getTCCChannelNr(pinC_h, (i>>4&0x01)==0x1);
		tccConfiguration pinCl = getTCCChannelNr(pinC_l, (i>>5&0x01)==0x1);
		if (checkPeripheralPermutationSameTCC6(pinAh, pinAl, pinBh, pinBl, pinCh, pinCl))
			return i;
	}
	return -1;
}




int checkSoftware6PWM(const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
	for (int i=0;i<64;i++) {
		tccConfiguration pinAh = getTCCChannelNr(pinA_h, (i>>0&0x01)==0x1);
		tccConfiguration pinAl = getTCCChannelNr(pinA_l, (i>>1&0x01)==0x1);
		tccConfiguration pinBh = getTCCChannelNr(pinB_h, (i>>2&0x01)==0x1);
		tccConfiguration pinBl = getTCCChannelNr(pinB_l, (i>>3&0x01)==0x1);
		tccConfiguration pinCh = getTCCChannelNr(pinC_h, (i>>4&0x01)==0x1);
		tccConfiguration pinCl = getTCCChannelNr(pinC_l, (i>>5&0x01)==0x1);
		if (checkPeripheralPermutationCompatible6(pinAh, pinAl, pinBh, pinBl, pinCh, pinCl))
			return i;
	}
	return -1;
}







void writeSAMDDutyCycle(int chaninfo, float dc) {
	uint8_t tccn = GetTCNumber(chaninfo);
	uint8_t chan = GetTCChannelNumber(chaninfo);
	if (tccn<TCC_INST_NUM) {
		Tcc* tcc = (Tcc*)GetTC(chaninfo);
		// set via CC
		tcc->CC[chan].reg = (uint32_t)((SIMPLEFOC_SAMD_PWM_RESOLUTION-1) * dc);
		uint32_t chanbit = 0x1<<(TCC_SYNCBUSY_CC0_Pos+chan);
		while ( (tcc->SYNCBUSY.reg & chanbit) > 0 );
		// set via CCB
//		tcc->CCB[chan].reg = (uint32_t)((SIMPLEFOC_SAMD_PWM_RESOLUTION-1) * dc);
//		tcc->STATUS.vec.CCBV = tcc->STATUS.vec.CCBV | (1<<chan);
		// TODO do we need to wait for sync?
//		uint32_t chanbit = 0x1<<(TCC_SYNCBUSY_CCB0_Pos+chan);
//		while ( (tcc->SYNCBUSY.reg & chanbit) > 0 );
	}
	else {
		Tc* tc = (Tc*)GetTC(chaninfo);
		//tc->COUNT16.CC[chan].reg = (uint32_t)((SIMPLEFOC_SAMD_PWM_RESOLUTION-1) * dc);
		tc->COUNT8.CC[chan].reg = (uint8_t)((SIMPLEFOC_SAMD_PWM_TC_RESOLUTION-1) * dc);;
		while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
	}
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
	printAllPinInfos();
	int compatibility = checkSameTCC3(pinA, pinB, pinC);
	if (compatibility<0) {
		compatibility = checkCompatible3(pinA, pinB, pinC);
		if (compatibility<0) {
			// no result!
			Serial.println("Bad combination!");
			return;
		}
	}

	tccConfiguration pin1 = getTCCChannelNr(pinA, (compatibility>>0&0x01)==0x1);
	tccConfiguration pin2 = getTCCChannelNr(pinB, (compatibility>>1&0x01)==0x1);
	tccConfiguration pin3 = getTCCChannelNr(pinC, (compatibility>>2&0x01)==0x1);

	Serial.println("Found configuration: ");
	printTCCConfiguration(pin1);
	printTCCConfiguration(pin2);
	printTCCConfiguration(pin3);

	// attach pins to timer peripherals
	attachTCC(pin1); // in theory this can fail, but there is no way to signal it...
	attachTCC(pin2);
	attachTCC(pin3);
	Serial.println("Attached pins...");

	// set up clock - TODO if we did this right it should be possible to get all TCC units synchronized?
	// e.g. attach all the timers, start them, and then start the clock...
	configureSAMDClock();

	// configure the TCC (waveform, top-value, pre-scaler = frequency)
	configureTCC(pin1, pwm_frequency);
	configureTCC(pin2, pwm_frequency);
	configureTCC(pin3, pwm_frequency);

	Serial.println("Configured TCCs...");
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
	printAllPinInfos();
	int compatibility = checkHardware6PWM(pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l);
	if (compatibility<0) {
		compatibility = checkSoftware6PWM(pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l);
		if (compatibility<0) {
			// no result!
			Serial.println("Bad combination!");
			return -1;
		}
	}

	tccConfiguration pinAh = getTCCChannelNr(pinA_h, (compatibility>>0&0x01)==0x1);
	tccConfiguration pinAl = getTCCChannelNr(pinA_l, (compatibility>>1&0x01)==0x1);
	tccConfiguration pinBh = getTCCChannelNr(pinB_h, (compatibility>>2&0x01)==0x1);
	tccConfiguration pinBl = getTCCChannelNr(pinB_l, (compatibility>>3&0x01)==0x1);
	tccConfiguration pinCh = getTCCChannelNr(pinC_h, (compatibility>>4&0x01)==0x1);
	tccConfiguration pinCl = getTCCChannelNr(pinC_l, (compatibility>>5&0x01)==0x1);

	Serial.println("Found configuration: ");
	printTCCConfiguration(pinAh);
	printTCCConfiguration(pinAl);
	printTCCConfiguration(pinBh);
	printTCCConfiguration(pinBl);
	printTCCConfiguration(pinCh);
	printTCCConfiguration(pinCl);

	// attach pins to timer peripherals
	bool allAttached = true;
	allAttached |= attachTCC(pinAh); // in theory this can fail, but there is no way to signal it...
	allAttached |= attachTCC(pinAl);
	allAttached |= attachTCC(pinBh);
	allAttached |= attachTCC(pinBl);
	allAttached |= attachTCC(pinCh);
	allAttached |= attachTCC(pinCl);
	if (!allAttached)
		return -1;
	// set up clock - TODO if we did this right it should be possible to get all TCC units synchronized?
	// e.g. attach all the timers, start them, and then start the clock...
	configureSAMDClock();

	// configure the TCC(s)
	configureTCC(pinAh, pwm_frequency, false, (pinAh.tcc.chaninfo==pinAl.tcc.chaninfo));
	if ((pinAh.tcc.chaninfo!=pinAl.tcc.chaninfo))
		configureTCC(pinAl, pwm_frequency, true, false);
	configureTCC(pinBh, pwm_frequency, false, (pinBh.tcc.chaninfo==pinBl.tcc.chaninfo));
	if ((pinBh.tcc.chaninfo!=pinBl.tcc.chaninfo))
		configureTCC(pinBl, pwm_frequency, false, true);
	configureTCC(pinCh, pwm_frequency, false, (pinCh.tcc.chaninfo==pinCl.tcc.chaninfo));
	if ((pinCh.tcc.chaninfo!=pinCl.tcc.chaninfo))
		configureTCC(pinCl, pwm_frequency, false, true);

	return 0;
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
	tccConfiguration* tccI = getTccPinConfiguration(pinA);
	writeSAMDDutyCycle(tccI->tcc.chaninfo, dc_a);
	tccI = getTccPinConfiguration(pinB);
	writeSAMDDutyCycle(tccI->tcc.chaninfo, dc_b);
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
	tccConfiguration* tccI = getTccPinConfiguration(pinA);
	writeSAMDDutyCycle(tccI->tcc.chaninfo, dc_a);
	tccI = getTccPinConfiguration(pinB);
	writeSAMDDutyCycle(tccI->tcc.chaninfo, dc_b);
	tccI = getTccPinConfiguration(pinC);
	writeSAMDDutyCycle(tccI->tcc.chaninfo, dc_c);
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
	tccConfiguration* tcc1 = getTccPinConfiguration(pinA_h);
	tccConfiguration* tcc2 = getTccPinConfiguration(pinA_l);
	if (tcc1->tcc.chaninfo!=tcc2->tcc.chaninfo) {
		// low-side on a different pin of same TCC - do dead-time in software...
		float ls = dc_a+SIMPLEFOC_SAMD_DEAD_TIME_VAL;
		if (ls>1.0) ls = 1.0; // no off-time is better than too-short dead-time
		writeSAMDDutyCycle(tcc1->tcc.chaninfo, dc_a);
		writeSAMDDutyCycle(tcc2->tcc.chaninfo, ls);
	}
	else
		writeSAMDDutyCycle(tcc1->tcc.chaninfo, dc_a); // dead-time is done is hardware, no need to set low side pin explicitly

	tcc1 = getTccPinConfiguration(pinB_h);
	tcc2 = getTccPinConfiguration(pinB_l);
	if (tcc1->tcc.chaninfo!=tcc2->tcc.chaninfo) {
		float ls = dc_b+SIMPLEFOC_SAMD_DEAD_TIME_VAL;
		if (ls>(1.0-SIMPLEFOC_SAMD_DEAD_TIME_VAL)) ls = 1.0;
		writeSAMDDutyCycle(tcc1->tcc.chaninfo, dc_b);
		writeSAMDDutyCycle(tcc2->tcc.chaninfo, ls);
	}
	else
		writeSAMDDutyCycle(tcc1->tcc.chaninfo, dc_b);

	tcc1 = getTccPinConfiguration(pinC_h);
	tcc2 = getTccPinConfiguration(pinC_l);
	if (tcc1->tcc.chaninfo!=tcc2->tcc.chaninfo) {
		float ls = dc_b+SIMPLEFOC_SAMD_DEAD_TIME_VAL;
		if (ls>(1.0-SIMPLEFOC_SAMD_DEAD_TIME_VAL)) ls = 1.0;
		writeSAMDDutyCycle(tcc1->tcc.chaninfo, dc_c);
		writeSAMDDutyCycle(tcc2->tcc.chaninfo, ls);
	}
	else
		writeSAMDDutyCycle(tcc1->tcc.chaninfo, dc_c);
	return;
}







#endif
