


#include "./samd_mcu.h"

#if defined(_SAMD21_)||defined(_SAMD51_)||defined(_SAME51_)



/**
 * Global state
 */
tccConfiguration tccPinConfigurations[SIMPLEFOC_SAMD_MAX_TCC_PINCONFIGURATIONS];
uint8_t numTccPinConfigurations = 0;
bool SAMDClockConfigured = false;
bool tccConfigured[TCC_INST_NUM+TC_INST_NUM];





/**
 * Attach the TCC to the pin
 */
bool attachTCC(tccConfiguration& tccConfig) {
	if (numTccPinConfigurations>=SIMPLEFOC_SAMD_MAX_TCC_PINCONFIGURATIONS)
		return false;
    pinMode(tccConfig.pin, OUTPUT);

    pinPeripheral(tccConfig.pin, tccConfig.peripheral);
    tccPinConfigurations[numTccPinConfigurations++] = tccConfig;
    return true;
}





int getPermutationNumber(int pins) {
	int num = 1;
	for (int i=0;i<pins;i++)
		num *= NUM_PIO_TIMER_PERIPHERALS;
	return num;
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
tccConfiguration getTCCChannelNr(int pin, EPioType peripheral) {
	tccConfiguration result;
	result.pin = pin;
	result.peripheral  = peripheral;
	result.tcc.tccn = -2;
	result.tcc.chan = -2;
	const PinDescription& pinDesc = g_APinDescription[pin];
	struct wo_association& association = getWOAssociation(pinDesc.ulPort, pinDesc.ulPin);
	if (association.port==NOT_A_PORT)
		return result; // could not find the port/pin
	if (peripheral==PIO_TIMER) {
		result.tcc.chaninfo = association.tccE;
		result.wo = association.woE;
	}
	else if (peripheral==PIO_TIMER_ALT) {
		result.tcc.chaninfo = association.tccF;
		result.wo = association.woF;
	}
#if defined(_SAMD51_)||defined(_SAME51_)
	else if (peripheral==PIO_TCC_PDEC) {
		result.tcc.chaninfo = association.tccG;
		result.wo = association.woG;
	}
#endif
	return result;
}





bool checkPeripheralPermutationSameTCC6(tccConfiguration& pinAh, tccConfiguration& pinAl, tccConfiguration& pinBh, tccConfiguration& pinBl, tccConfiguration& pinCh, tccConfiguration& pinCl) {
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




bool checkPeripheralPermutationCompatible(tccConfiguration pins[], uint8_t num) {
	for (int i=0;i<num;i++)
		if (pins[i].tcc.tccn<0)
			return false;
	for (int i=0;i<num-1;i++)
		for (int j=i+1;j<num;j++)
			if (pins[i].tcc.chaninfo==pins[j].tcc.chaninfo)
				return false;
	for (int i=0;i<num;i++)
		if (inUse(pins[i]))
			return false;
	return true;
}





bool checkPeripheralPermutationCompatible6(tccConfiguration& pinAh, tccConfiguration& pinAl, tccConfiguration& pinBh, tccConfiguration& pinBl, tccConfiguration& pinCh, tccConfiguration& pinCl) {
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





int checkHardware6PWM(const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
	for (int i=0;i<64;i++) {
		tccConfiguration pinAh = getTCCChannelNr(pinA_h, getPeripheralOfPermutation(i, 0));
		tccConfiguration pinAl = getTCCChannelNr(pinA_l, getPeripheralOfPermutation(i, 1));
		tccConfiguration pinBh = getTCCChannelNr(pinB_h, getPeripheralOfPermutation(i, 2));
		tccConfiguration pinBl = getTCCChannelNr(pinB_l, getPeripheralOfPermutation(i, 3));
		tccConfiguration pinCh = getTCCChannelNr(pinC_h, getPeripheralOfPermutation(i, 4));
		tccConfiguration pinCl = getTCCChannelNr(pinC_l, getPeripheralOfPermutation(i, 5));
		if (checkPeripheralPermutationSameTCC6(pinAh, pinAl, pinBh, pinBl, pinCh, pinCl))
			return i;
	}
	return -1;
}




int checkSoftware6PWM(const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
	for (int i=0;i<64;i++) {
		tccConfiguration pinAh = getTCCChannelNr(pinA_h, getPeripheralOfPermutation(i, 0));
		tccConfiguration pinAl = getTCCChannelNr(pinA_l, getPeripheralOfPermutation(i, 1));
		tccConfiguration pinBh = getTCCChannelNr(pinB_h, getPeripheralOfPermutation(i, 2));
		tccConfiguration pinBl = getTCCChannelNr(pinB_l, getPeripheralOfPermutation(i, 3));
		tccConfiguration pinCh = getTCCChannelNr(pinC_h, getPeripheralOfPermutation(i, 4));
		tccConfiguration pinCl = getTCCChannelNr(pinC_l, getPeripheralOfPermutation(i, 5));
		if (checkPeripheralPermutationCompatible6(pinAh, pinAl, pinBh, pinBl, pinCh, pinCl))
			return i;
	}
	return -1;
}



int scorePermutation(tccConfiguration pins[], uint8_t num) {
	uint32_t usedtccs = 0;
	for (int i=0;i<num;i++)
		usedtccs |= (1<<pins[i].tcc.tccn);
	int score = 0;
	for (int i=0;i<TCC_INST_NUM;i++){
		if (usedtccs&0x1)
			score+=1;
		usedtccs = usedtccs>>1;
	}
	for (int i=0;i<TC_INST_NUM;i++){
		if (usedtccs&0x1)
			score+=(num+1);
		usedtccs = usedtccs>>1;
	}
	return score;
}








int checkPermutations(uint8_t num, int pins[], bool (*checkFunc)(tccConfiguration[], uint8_t) ) {
	tccConfiguration tccConfs[num];
	int best = -1;
	int bestscore = 1000000;
	for (int i=0;i<(0x1<<num);i++) {
		for (int j=0;j<num;j++)
			tccConfs[j] = getTCCChannelNr(pins[j], getPeripheralOfPermutation(i, j));
		if (checkFunc(tccConfs, num)) {
			int score = scorePermutation(tccConfs, num);
			if (score<bestscore) {
				bestscore = score;
				best = i;
			}
		}
	}
	return best;
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
#ifdef SIMPLEFOC_SAMD_DEBUG
	printAllPinInfos();
#endif
	int pins[2] = { pinA, pinB };
	int compatibility = checkPermutations(2, pins, checkPeripheralPermutationCompatible);
	if (compatibility<0) {
		// no result!
#ifdef SIMPLEFOC_SAMD_DEBUG
		SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Bad combination!");
#endif
		return;
	}

	tccConfiguration tccConfs[2] = { getTCCChannelNr(pinA, getPeripheralOfPermutation(compatibility, 0)),
								     getTCCChannelNr(pinB, getPeripheralOfPermutation(compatibility, 1)) };


#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.print("Found configuration: (score=");
	SIMPLEFOC_SAMD_DEBUG_SERIAL.print(scorePermutation(tccConfs, 2));
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println(")");
	printTCCConfiguration(tccConfs[0]);
	printTCCConfiguration(tccConfs[1]);
#endif

	// attach pins to timer peripherals
	attachTCC(tccConfs[0]); // in theory this can fail, but there is no way to signal it...
	attachTCC(tccConfs[1]);
#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Attached pins...");
#endif

	// set up clock - Note: if we did this right it should be possible to get all TCC units synchronized?
	// e.g. attach all the timers, start them, and then start the clock... but this would require API-changes in SimpleFOC...
	configureSAMDClock();

	if (pwm_frequency==NOT_SET) {
		// use default frequency
		pwm_frequency = SIMPLEFOC_SAMD_DEFAULT_PWM_FREQUENCY_HZ;
	}

	// configure the TCC (waveform, top-value, pre-scaler = frequency)
	configureTCC(tccConfs[0], pwm_frequency);
	configureTCC(tccConfs[1], pwm_frequency);
	getTccPinConfiguration(pinA)->pwm_res = tccConfs[0].pwm_res;
	getTccPinConfiguration(pinB)->pwm_res = tccConfs[1].pwm_res;
#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Configured TCCs...");
#endif

	return; // Someone with a stepper-setup who can test it?
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
#ifdef SIMPLEFOC_SAMD_DEBUG
	printAllPinInfos();
#endif
	int pins[3] = { pinA, pinB, pinC };
	int compatibility = checkPermutations(3, pins, checkPeripheralPermutationCompatible);
	if (compatibility<0) {
		// no result!
#ifdef SIMPLEFOC_SAMD_DEBUG
		SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Bad combination!");
#endif
		return;
	}

	tccConfiguration tccConfs[3] = { getTCCChannelNr(pinA, getPeripheralOfPermutation(compatibility, 0)),
									 getTCCChannelNr(pinB, getPeripheralOfPermutation(compatibility, 1)),
								     getTCCChannelNr(pinC, getPeripheralOfPermutation(compatibility, 2)) };


#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.print("Found configuration: (score=");
	SIMPLEFOC_SAMD_DEBUG_SERIAL.print(scorePermutation(tccConfs, 3));
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println(")");
	printTCCConfiguration(tccConfs[0]);
	printTCCConfiguration(tccConfs[1]);
	printTCCConfiguration(tccConfs[2]);
#endif

	// attach pins to timer peripherals
	attachTCC(tccConfs[0]); // in theory this can fail, but there is no way to signal it...
	attachTCC(tccConfs[1]);
	attachTCC(tccConfs[2]);
#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Attached pins...");
#endif

	// set up clock - Note: if we did this right it should be possible to get all TCC units synchronized?
	// e.g. attach all the timers, start them, and then start the clock... but this would require API-changes in SimpleFOC...
	configureSAMDClock();

	if (pwm_frequency==NOT_SET) {
		// use default frequency
		pwm_frequency = SIMPLEFOC_SAMD_DEFAULT_PWM_FREQUENCY_HZ;
	}

	// configure the TCC (waveform, top-value, pre-scaler = frequency)
	configureTCC(tccConfs[0], pwm_frequency);
	configureTCC(tccConfs[1], pwm_frequency);
	configureTCC(tccConfs[2], pwm_frequency);
	getTccPinConfiguration(pinA)->pwm_res = tccConfs[0].pwm_res;
	getTccPinConfiguration(pinB)->pwm_res = tccConfs[1].pwm_res;
	getTccPinConfiguration(pinC)->pwm_res = tccConfs[2].pwm_res;
#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Configured TCCs...");
#endif

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
#ifdef SIMPLEFOC_SAMD_DEBUG
	printAllPinInfos();
#endif
	int pins[4] = { pin1A, pin1B, pin2A, pin2B };
	int compatibility = checkPermutations(4, pins, checkPeripheralPermutationCompatible);
	if (compatibility<0) {
		// no result!
#ifdef SIMPLEFOC_SAMD_DEBUG
		SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Bad combination!");
#endif
		return;
	}

	tccConfiguration tccConfs[4] = { getTCCChannelNr(pin1A, getPeripheralOfPermutation(compatibility, 0)),
									 getTCCChannelNr(pin1B, getPeripheralOfPermutation(compatibility, 1)),
									 getTCCChannelNr(pin2A, getPeripheralOfPermutation(compatibility, 2)),
									 getTCCChannelNr(pin2B, getPeripheralOfPermutation(compatibility, 3)) };


#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.print("Found configuration: (score=");
	SIMPLEFOC_SAMD_DEBUG_SERIAL.print(scorePermutation(tccConfs, 4));
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println(")");
	printTCCConfiguration(tccConfs[0]);
	printTCCConfiguration(tccConfs[1]);
	printTCCConfiguration(tccConfs[2]);
	printTCCConfiguration(tccConfs[3]);
#endif

	// attach pins to timer peripherals
	attachTCC(tccConfs[0]); // in theory this can fail, but there is no way to signal it...
	attachTCC(tccConfs[1]);
	attachTCC(tccConfs[2]);
	attachTCC(tccConfs[3]);
#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Attached pins...");
#endif

	// set up clock - Note: if we did this right it should be possible to get all TCC units synchronized?
	// e.g. attach all the timers, start them, and then start the clock... but this would require API-changes in SimpleFOC...
	configureSAMDClock();

	if (pwm_frequency==NOT_SET) {
		// use default frequency
		pwm_frequency = SIMPLEFOC_SAMD_DEFAULT_PWM_FREQUENCY_HZ;
	}

	// configure the TCC (waveform, top-value, pre-scaler = frequency)
	configureTCC(tccConfs[0], pwm_frequency);
	configureTCC(tccConfs[1], pwm_frequency);
	configureTCC(tccConfs[2], pwm_frequency);
	configureTCC(tccConfs[3], pwm_frequency);
	getTccPinConfiguration(pin1A)->pwm_res = tccConfs[0].pwm_res;
	getTccPinConfiguration(pin2A)->pwm_res = tccConfs[1].pwm_res;
	getTccPinConfiguration(pin1B)->pwm_res = tccConfs[2].pwm_res;
	getTccPinConfiguration(pin2B)->pwm_res = tccConfs[3].pwm_res;
#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Configured TCCs...");
#endif

	return; // Someone with a stepper-setup who can test it?
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
#ifdef SIMPLEFOC_SAMD_DEBUG
	printAllPinInfos();
#endif
	int compatibility = checkHardware6PWM(pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l);
	if (compatibility<0) {
		compatibility = checkSoftware6PWM(pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l);
		if (compatibility<0) {
			// no result!
#ifdef SIMPLEFOC_SAMD_DEBUG
			SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Bad combination!");
#endif
			return -1;
		}
	}

	tccConfiguration pinAh = getTCCChannelNr(pinA_h, getPeripheralOfPermutation(compatibility, 0));
	tccConfiguration pinAl = getTCCChannelNr(pinA_l, getPeripheralOfPermutation(compatibility, 1));
	tccConfiguration pinBh = getTCCChannelNr(pinB_h, getPeripheralOfPermutation(compatibility, 2));
	tccConfiguration pinBl = getTCCChannelNr(pinB_l, getPeripheralOfPermutation(compatibility, 3));
	tccConfiguration pinCh = getTCCChannelNr(pinC_h, getPeripheralOfPermutation(compatibility, 4));
	tccConfiguration pinCl = getTCCChannelNr(pinC_l, getPeripheralOfPermutation(compatibility, 5));

#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Found configuration: ");
	printTCCConfiguration(pinAh);
	printTCCConfiguration(pinAl);
	printTCCConfiguration(pinBh);
	printTCCConfiguration(pinBl);
	printTCCConfiguration(pinCh);
	printTCCConfiguration(pinCl);
#endif

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
#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Attached pins...");
#endif
	// set up clock - if we did this right it should be possible to get all TCC units synchronized?
	// e.g. attach all the timers, start them, and then start the clock... but this would require API changes in SimpleFOC driver API
	configureSAMDClock();

	if (pwm_frequency==NOT_SET) {
		// use default frequency
		pwm_frequency = SIMPLEFOC_SAMD_DEFAULT_PWM_FREQUENCY_HZ;
	}

	// configure the TCC(s)
	configureTCC(pinAh, pwm_frequency, false, (pinAh.tcc.chaninfo==pinAl.tcc.chaninfo)?dead_zone:-1);
	if ((pinAh.tcc.chaninfo!=pinAl.tcc.chaninfo))
		configureTCC(pinAl, pwm_frequency, true, -1.0);
	configureTCC(pinBh, pwm_frequency, false, (pinBh.tcc.chaninfo==pinBl.tcc.chaninfo)?dead_zone:-1);
	if ((pinBh.tcc.chaninfo!=pinBl.tcc.chaninfo))
		configureTCC(pinBl, pwm_frequency, true, -1.0);
	configureTCC(pinCh, pwm_frequency, false, (pinCh.tcc.chaninfo==pinCl.tcc.chaninfo)?dead_zone:-1);
	if ((pinCh.tcc.chaninfo!=pinCl.tcc.chaninfo))
		configureTCC(pinCl, pwm_frequency, true, -1.0);
	getTccPinConfiguration(pinA_h)->pwm_res = pinAh.pwm_res;
	getTccPinConfiguration(pinA_l)->pwm_res = pinAh.pwm_res; // use the high phase resolution, in case we didn't set it
	getTccPinConfiguration(pinB_h)->pwm_res = pinBh.pwm_res;
	getTccPinConfiguration(pinB_l)->pwm_res = pinBh.pwm_res;
	getTccPinConfiguration(pinC_h)->pwm_res = pinCh.pwm_res;
	getTccPinConfiguration(pinC_l)->pwm_res = pinCh.pwm_res;
#ifdef SIMPLEFOC_SAMD_DEBUG
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Configured TCCs...");
#endif

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
	writeSAMDDutyCycle(tccI, dc_a);
	tccI = getTccPinConfiguration(pinB);
	writeSAMDDutyCycle(tccI, dc_b);
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
	writeSAMDDutyCycle(tccI, dc_a);
	tccI = getTccPinConfiguration(pinB);
	writeSAMDDutyCycle(tccI, dc_b);
	tccI = getTccPinConfiguration(pinC);
	writeSAMDDutyCycle(tccI, dc_c);
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
	tccConfiguration* tccI = getTccPinConfiguration(pin1A);
	writeSAMDDutyCycle(tccI, dc_1a);
	tccI = getTccPinConfiguration(pin2A);
	writeSAMDDutyCycle(tccI, dc_2a);
	tccI = getTccPinConfiguration(pin1B);
	writeSAMDDutyCycle(tccI, dc_1b);
	tccI = getTccPinConfiguration(pin2B);
	writeSAMDDutyCycle(tccI, dc_2b);
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
		float ls = dc_a+(dead_zone*(SIMPLEFOC_SAMD_PWM_RESOLUTION-1));
		if (ls>1.0) ls = 1.0f; // no off-time is better than too-short dead-time
		writeSAMDDutyCycle(tcc1, dc_a);
		writeSAMDDutyCycle(tcc2, ls);
	}
	else
		writeSAMDDutyCycle(tcc1, dc_a); // dead-time is done is hardware, no need to set low side pin explicitly

	tcc1 = getTccPinConfiguration(pinB_h);
	tcc2 = getTccPinConfiguration(pinB_l);
	if (tcc1->tcc.chaninfo!=tcc2->tcc.chaninfo) {
		float ls = dc_b+(dead_zone*(SIMPLEFOC_SAMD_PWM_RESOLUTION-1));
		if (ls>1.0) ls = 1.0f; // no off-time is better than too-short dead-time
		writeSAMDDutyCycle(tcc1, dc_b);
		writeSAMDDutyCycle(tcc2, ls);
	}
	else
		writeSAMDDutyCycle(tcc1, dc_b);

	tcc1 = getTccPinConfiguration(pinC_h);
	tcc2 = getTccPinConfiguration(pinC_l);
	if (tcc1->tcc.chaninfo!=tcc2->tcc.chaninfo) {
		float ls = dc_c+(dead_zone*(SIMPLEFOC_SAMD_PWM_RESOLUTION-1));
		if (ls>1.0) ls = 1.0f; // no off-time is better than too-short dead-time
		writeSAMDDutyCycle(tcc1, dc_c);
		writeSAMDDutyCycle(tcc2, ls);
	}
	else
		writeSAMDDutyCycle(tcc1, dc_c);
	return;
}




#ifdef SIMPLEFOC_SAMD_DEBUG

/**
 * Prints a table of pin assignments for your SAMD MCU. Very useful since the
 * board pinout descriptions and variant.cpp are usually quite wrong, and this
 * saves you hours of cross-referencing with the datasheet.
 */
void printAllPinInfos() {
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println();
	for (uint8_t pin=0;pin<PINS_COUNT;pin++) {
		const PinDescription& pinDesc = g_APinDescription[pin];
		wo_association& association = getWOAssociation(pinDesc.ulPort, pinDesc.ulPin);
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print("Pin ");
		if (pin<10) SIMPLEFOC_SAMD_DEBUG_SERIAL.print("0");
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(pin);
		switch (pinDesc.ulPort) {
			case NOT_A_PORT: SIMPLEFOC_SAMD_DEBUG_SERIAL.print("    "); break;
			case PORTA: SIMPLEFOC_SAMD_DEBUG_SERIAL.print("  PA"); break;
			case PORTB: SIMPLEFOC_SAMD_DEBUG_SERIAL.print("  PB"); break;
			case PORTC: SIMPLEFOC_SAMD_DEBUG_SERIAL.print("  PC"); break;
#if defined(_SAMD51_)||defined(_SAME51_)
			case PORTD: SIMPLEFOC_SAMD_DEBUG_SERIAL.print("  PD"); break;
#endif
		}
		if (pinDesc.ulPin <10) SIMPLEFOC_SAMD_DEBUG_SERIAL.print("0");
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(pinDesc.ulPin);

		SIMPLEFOC_SAMD_DEBUG_SERIAL.print("  E=");
		if (association.tccE>=0) {
			int tcn = GetTCNumber(association.tccE);
			if (tcn>=TCC_INST_NUM)
				SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" TC");
			else
				SIMPLEFOC_SAMD_DEBUG_SERIAL.print("TCC");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(tcn);
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("-");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(GetTCChannelNumber(association.tccE));
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("[");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(GetTCChannelNumber(association.woE));
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("]");
			if (tcn<10)
				SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" ");
		}
		else
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("  None    ");

		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" F=");
		if (association.tccF>=0) {
			int tcn = GetTCNumber(association.tccF);
			if (tcn>=TCC_INST_NUM)
				SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" TC");
			else
				SIMPLEFOC_SAMD_DEBUG_SERIAL.print("TCC");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(tcn);
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("-");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(GetTCChannelNumber(association.tccF));
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("[");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(GetTCChannelNumber(association.woF));
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("]");
			if (tcn<10)
				SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" ");
		}
		else
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("  None    ");

#if defined(_SAMD51_)||defined(_SAME51_)
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" G=");
		if (association.tccG>=0) {
			int tcn = GetTCNumber(association.tccG);
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("TCC");
			if (tcn<10)
				SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" ");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(tcn);
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("-");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(GetTCChannelNumber(association.tccG));
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("[");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(GetTCChannelNumber(association.woG));
			SIMPLEFOC_SAMD_DEBUG_SERIAL.println("]");
		}
		else
			SIMPLEFOC_SAMD_DEBUG_SERIAL.println("  None ");
#else
		SIMPLEFOC_SAMD_DEBUG_SERIAL.println("");
#endif

	}
	SIMPLEFOC_SAMD_DEBUG_SERIAL.println();
}





void printTCCConfiguration(tccConfiguration& info) {
	SIMPLEFOC_SAMD_DEBUG_SERIAL.print(info.pin);
	if (info.tcc.tccn>=TCC_INST_NUM)
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(":  TC Peripheral");
	else
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(": TCC Peripheral");
	switch (info.peripheral) {
	case PIO_TIMER:
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" E "); break;
	case PIO_TIMER_ALT:
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" F "); break;
#if defined(_SAMD51_)||defined(_SAME51_)
	case PIO_TCC_PDEC:
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" G "); break;
#endif
	default:
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(" ? "); break;
	}
	if (info.tcc.tccn>=0) {
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(info.tcc.tccn);
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print("-");
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(info.tcc.chan);
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print("[");
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(info.wo);
		SIMPLEFOC_SAMD_DEBUG_SERIAL.println("]");
	}
	else
		SIMPLEFOC_SAMD_DEBUG_SERIAL.println(" None");
}



#endif

#endif
