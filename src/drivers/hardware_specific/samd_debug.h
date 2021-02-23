


#include "../hardware_api.h"
#include "wiring_private.h"

// Read count
//TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
//while (TCC0->SYNCBUSY.bit.CTRLB); // or while (TCC0->SYNCBUSY.reg);
//int count = TCC0->COUNT.reg;


/**
 * Prints a table of pin assignments for your SAMD MCU. Very useful since the
 * board pinout descriptions and variant.cpp are usually quite wrong, and this
 * saves you hours of cross-referencing with the datasheet.
 */
void printAllPinInfos() {
	Serial.println();
	for (uint8_t pin=0;pin<PINS_COUNT;pin++) {
		const PinDescription& pinDesc = g_APinDescription[pin];
		wo_association& association = getWOAssociation(pinDesc.ulPort, pinDesc.ulPin);
		Serial.print("Pin ");
		if (pin<10) Serial.print("0");
		Serial.print(pin);
		switch (pinDesc.ulPort) {
			case NOT_A_PORT: Serial.print("    "); break;
			case PORTA: Serial.print("  PA"); break;
			case PORTB: Serial.print("  PB"); break;
			case PORTC: Serial.print("  PC"); break;
		}
		if (pinDesc.ulPin <10) Serial.print("0");
		Serial.print(pinDesc.ulPin);

		Serial.print("  E=");
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

		Serial.print(" F=");
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

	//		uint32_t attr = pinDesc.ulPinAttribute;
	//#ifdef PIN_ATTR_PWM //SAMD21
	//		Serial.print("  PWM=");
	//		Serial.print(((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM));
	//#endif
	//#ifdef PIN_ATTR_PWM_E //SAMD51
	//		Serial.print("  PWM_E=");
	//		Serial.print(((attr & PIN_ATTR_PWM_E) == PIN_ATTR_PWM_E));
	//		Serial.print("  PWM_F=");
	//		Serial.print(((attr & PIN_ATTR_PWM_F) == PIN_ATTR_PWM_F));
	//		Serial.print("  PWM_G=");
	//		Serial.print(((attr & PIN_ATTR_PWM_G) == PIN_ATTR_PWM_G));
	//#endif
	//		Serial.print("  TIM=");
	//		Serial.print(((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER));
	//		Serial.print("  ALT=");
	//		Serial.print(((attr & PIN_ATTR_TIMER_ALT) == PIN_ATTR_TIMER_ALT));
}

void printTCCConfiguration(tccConfiguration& info) {
	Serial.print(info.pin);
	Serial.print((info.alternate==1)?" alternate TCC":" normal    TCC");
	if (info.tcc.tccn>=0) {
		Serial.print(info.tcc.tccn);
		Serial.print("-");
		Serial.print(info.tcc.chan);
		Serial.print("[");
		Serial.print(info.wo);
		Serial.println("]");
	}
	else
		Serial.println(" None");
}



