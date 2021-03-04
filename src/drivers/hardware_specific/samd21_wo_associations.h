

struct wo_association {
	EPortType port;
	uint32_t pin;
	ETCChannel tccE;
	uint8_t woE;
	ETCChannel tccF;
	uint8_t woF;
};



#ifdef _SAMD21_



#ifndef TCC3_CH0
#define TCC3_CH0 NOT_ON_TIMER
#endif
#ifndef TCC3_CH1
#define TCC3_CH1 NOT_ON_TIMER
#endif
#ifndef TCC3_CH2
#define TCC3_CH2 NOT_ON_TIMER
#endif
#ifndef TCC3_CH3
#define TCC3_CH3 NOT_ON_TIMER
#endif
#ifndef TCC3_CH4
#define TCC3_CH4 NOT_ON_TIMER
#endif
#ifndef TCC3_CH5
#define TCC3_CH5 NOT_ON_TIMER
#endif
#ifndef TCC3_CH6
#define TCC3_CH6 NOT_ON_TIMER
#endif
#ifndef TCC3_CH7
#define TCC3_CH7 NOT_ON_TIMER
#endif
#ifndef TC6_CH0
#define TC6_CH0 NOT_ON_TIMER
#endif
#ifndef TC6_CH1
#define TC6_CH1 NOT_ON_TIMER
#endif
#ifndef TC7_CH0
#define TC7_CH0 NOT_ON_TIMER
#endif
#ifndef TC7_CH1
#define TC7_CH1 NOT_ON_TIMER
#endif


/*
 * For SAM D21 A/B/C/D Variant Devices and SAM DA1 A/B Variant Devices
 * Good for SAMD2xE, SAMD2xG and SAMD2xJ devices. Other SAMD21s currently not supported in arduino anyway?
 *
 * Note: only the pins which have timers associated are listed in this table.
 * You can use the values from g_APinDescription.ulPort and g_APinDescription.ulPin to find the correct row in the table.
 *
 * See Microchip Technology datasheet DS40001882F-page 30
 */
struct wo_association WO_associations[] = {

		{ PORTA,  0, TCC2_CH0, 		0, NOT_ON_TIMER, 	0},
		{ PORTA,  1, TCC2_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTA,  2, NOT_ON_TIMER, 	0, TCC3_CH0, 		0},
		{ PORTA,  3, NOT_ON_TIMER,	0, TCC3_CH1, 		1},
		// PB04, PB05, PB06, PB07 - no timers
		{ PORTB,  8, TC4_CH0, 		0, TCC3_CH6, 		6},
		{ PORTB,  9, TC4_CH1, 		1, TCC3_CH7, 		7},
		{ PORTA,  4, TCC0_CH0, 		0, TCC3_CH2, 		2},
		{ PORTA,  5, TCC0_CH1, 		1, TCC3_CH3, 		3},
		{ PORTA,  6, TCC1_CH0, 		0, TCC3_CH4, 		4},
		{ PORTA,  7, TCC1_CH1, 		1, TCC3_CH5, 		5},
		{ PORTA,  8, TCC0_CH0, 		0, TCC1_CH2, 		2},
		{ PORTA,  9, TCC0_CH1, 		1, TCC1_CH3, 		3},
		{ PORTA, 10, TCC1_CH0, 		0, TCC0_CH2, 		2},
		{ PORTA, 11, TCC1_CH1, 		1, TCC0_CH3, 		3},
		{ PORTB, 10, TC5_CH0, 		0, TCC0_CH4, 		4},
		{ PORTB, 11, TC5_CH1, 		1, TCC0_CH5, 		5},
		{ PORTB, 12, TC4_CH0, 		0, TCC0_CH6, 		6},
		{ PORTB, 13, TC4_CH1, 		1, TCC0_CH7, 		7},
		{ PORTB, 14, TC5_CH0, 		0, NOT_ON_TIMER,	0},
		{ PORTB, 15, TC5_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTA, 12, TCC2_CH0, 		0, TCC0_CH6, 		6},
		{ PORTA, 13, TCC2_CH1, 		1, TCC0_CH7, 		7},
		{ PORTA, 14, TC3_CH0, 		0, TCC0_CH4, 		4},
		{ PORTA, 15, TC3_CH1, 		1, TCC0_CH5, 		5},
		{ PORTA, 16, TCC2_CH0, 		0, TCC0_CH6, 		6},
		{ PORTA, 17, TCC2_CH1, 		1, TCC0_CH7, 		7},
		{ PORTA, 18, TC3_CH0, 		0, TCC0_CH2, 		2},
		{ PORTA, 19, TC3_CH1, 		1, TCC0_CH3, 		3},
		{ PORTB, 16, TC6_CH0, 		0, TCC0_CH4, 		4},
		{ PORTB, 17, TC6_CH1, 		1, TCC0_CH5, 		5},
		{ PORTA, 20, TC7_CH0, 		0, TCC0_CH6, 		6},
		{ PORTA, 21, TC7_CH1, 		1, TCC0_CH7, 		7},
		{ PORTA, 22, TC4_CH0, 		0, TCC0_CH4, 		4},
		{ PORTA, 23, TC4_CH1, 		1, TCC0_CH5, 		5},
		{ PORTA, 24, TC5_CH0, 		0, TCC1_CH2, 		2},
		{ PORTA, 25, TC5_CH1, 		1, TCC1_CH3, 		3},
		{ PORTB, 22, TC7_CH0, 		0, TCC3_CH0, 		0},
		{ PORTB, 23, TC7_CH1, 		1, TCC3_CH1, 		1},
		{ PORTA, 27, NOT_ON_TIMER, 	0, TCC3_CH6, 		6},
		{ PORTA, 28, NOT_ON_TIMER, 	0, TCC3_CH7, 		7},
		{ PORTA, 30, TCC1_CH0, 		0, TCC3_CH4, 		4},
		{ PORTA, 31, TCC1_CH1, 		1, TCC3_CH5, 		5},
		{ PORTB, 30, TCC0_CH0, 		0, TCC1_CH2, 		2},
		{ PORTB, 31, TCC0_CH1, 		1, TCC1_CH3, 		3},
		{ PORTB,  0, TC7_CH0, 		0, NOT_ON_TIMER, 	0},
		{ PORTB,  1, TC7_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTB,  2, TC6_CH0, 		0, TCC3_CH2, 		2},
		{ PORTB,  3, TC6_CH1, 		1, TCC3_CH3, 		3}
};
#define NUM_WO_ASSOCIATIONS 48

wo_association ASSOCIATION_NOT_FOUND = { NOT_A_PORT, 0, NOT_ON_TIMER, 0, NOT_ON_TIMER, 0};


struct wo_association& getWOAssociation(EPortType port, uint32_t pin) {
	for (int i=0;i<NUM_WO_ASSOCIATIONS;i++) {
		if (WO_associations[i].port==port && WO_associations[i].pin==pin)
			return WO_associations[i];
	}
	return ASSOCIATION_NOT_FOUND;
};


#endif
