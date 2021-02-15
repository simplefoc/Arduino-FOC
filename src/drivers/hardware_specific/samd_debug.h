



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



