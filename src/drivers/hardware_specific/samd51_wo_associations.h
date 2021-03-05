


//	TCC#   Channels   WO_NUM   Counter size   Fault   Dithering   Output matrix   DTI   SWAP   Pattern generation
//	0         6         8         24-bit       Yes       Yes       Yes            Yes   Yes    Yes
//	1         4         8         24-bit       Yes       Yes       Yes            Yes   Yes    Yes
//	2         3         3         16-bit       Yes       -         Yes            -     -      -
//	3         2         2         16-bit       Yes       -         -              -     -      -
//	4         2         2         16-bit       Yes       -         -              -     -      -


#include "variant.h"


#ifdef _SAMD51_



struct wo_association {
	EPortType port;
	uint32_t pin;
	ETCChannel tccE;
	uint8_t woE;
	ETCChannel tccF;
	uint8_t woF;
	ETCChannel tccG;
	uint8_t woG;
};


struct wo_association WO_associations[] = {

		{ PORTB,  9, TC4_CH1, 		1, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0},
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
		{ PORTB,  10, TC5_CH0, 		0, TCC0_CH0, 		4, TCC1_CH0, 		0}, //?
		{ PORTB,  11, TC5_CH1, 		1, TCC0_CH1,  		5, TCC1_CH1, 		1}, //?
		{ PORTB,  12, TC4_CH0, 		0, TCC3_CH0, 		0, TCC0_CH0, 		0},
		{ PORTB,  13, TC4_CH1, 		1, TCC3_CH1, 		1, TCC0_CH1, 		1},
		{ PORTB,  14, TC5_CH0, 		0, TCC4_CH0, 		0, TCC0_CH2, 		2},
		{ PORTB,  15, TC5_CH1, 		1, TCC4_CH1, 		1, TCC0_CH3, 		3},
		{ PORTD,   8, NOT_ON_TIMER,	0, TCC0_CH1,     	1, NOT_ON_TIMER, 	0},
		{ PORTD,   9, NOT_ON_TIMER,	0, TCC0_CH2,     	2, NOT_ON_TIMER, 	0},
		{ PORTD,  10, NOT_ON_TIMER,	0, TCC0_CH3,     	3, NOT_ON_TIMER, 	0},
		{ PORTD,  11, NOT_ON_TIMER,	0, TCC0_CH0, 		4, NOT_ON_TIMER, 	0}, //?
		{ PORTD,  12, NOT_ON_TIMER,	0, TCC0_CH1,  		5, NOT_ON_TIMER, 	0}, //?
		{ PORTC,  10, NOT_ON_TIMER,	0, TCC0_CH0,     	0, TCC1_CH0, 		4},
		{ PORTC,  11, NOT_ON_TIMER,	0, TCC0_CH1,     	1, TCC1_CH1, 		5},
		{ PORTC,  12, NOT_ON_TIMER,	0, TCC0_CH2, 		2, TCC1_CH2, 		6},
		{ PORTC,  13, NOT_ON_TIMER,	0, TCC0_CH3, 		3, TCC1_CH3, 		7},
		{ PORTC,  14, NOT_ON_TIMER,	0, TCC0_CH0, 		4, TCC1_CH0, 		0}, //?
		{ PORTC,  15, NOT_ON_TIMER,	0, TCC0_CH1, 		5, TCC1_CH1, 		1}, //?
		{ PORTA,  12, TC2_CH0,		0, TCC0_CH2, 		6, TCC1_CH2, 		2},
		{ PORTA,  13, TC2_CH1,		1, TCC0_CH3, 		7, TCC1_CH3, 		3},
		{ PORTA,  14, TC3_CH0,		0, TCC2_CH0, 		0, TCC1_CH2, 		2}, //?
		{ PORTA,  15, TC3_CH1,		1, TCC1_CH1, 		1, TCC1_CH3, 		3}, //?
		{ PORTA,  16, TC2_CH0,		0, TCC1_CH0, 		0, TCC0_CH0, 		4},
		{ PORTA,  17, TC2_CH1,		1, TCC1_CH1, 		1, TCC0_CH1, 		5},
		{ PORTA,  18, TC3_CH0,		0, TCC1_CH2, 		2, TCC0_CH2, 		6},
		{ PORTA,  19, TC3_CH1,		1, TCC1_CH3, 		3, TCC0_CH3, 		7},
		{ PORTC,  16, NOT_ON_TIMER,	0, TCC0_CH0, 		0, NOT_ON_TIMER, 	0}, // PDEC0
		{ PORTC,  17, NOT_ON_TIMER,	0, TCC0_CH1, 		1, NOT_ON_TIMER, 	0}, // PDEC1
		{ PORTC,  18, NOT_ON_TIMER,	0, TCC0_CH2, 		2, NOT_ON_TIMER, 	0}, // PDEC2
		{ PORTC,  19, NOT_ON_TIMER,	0, TCC0_CH3, 		3, NOT_ON_TIMER, 	0},
		{ PORTC,  20, NOT_ON_TIMER,	0, TCC0_CH0, 		4, NOT_ON_TIMER, 	0},
		{ PORTC,  21, NOT_ON_TIMER,	0, TCC0_CH1, 		5, NOT_ON_TIMER, 	0},
		{ PORTC,  22, NOT_ON_TIMER,	0, TCC0_CH2, 		6, NOT_ON_TIMER, 	0},
		{ PORTC,  23, NOT_ON_TIMER,	0, TCC0_CH3, 		7, NOT_ON_TIMER, 	0},
		{ PORTD,  20, NOT_ON_TIMER,	0, TCC1_CH0, 		0, NOT_ON_TIMER, 	0},
		{ PORTD,  21, NOT_ON_TIMER,	0, TCC1_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTB,  16, TC6_CH0,		0, TCC3_CH0, 		0, TCC0_CH0, 		4},
		{ PORTB,  17, TC6_CH1,		1, TCC3_CH1, 		1, TCC0_CH1,	 	5},
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
		{ PORTB,  29, NOT_ON_TIMER,	1, TCC1_CH1, 		5, NOT_ON_TIMER,	0},
		// PC24-PC28, PA27, RESET -> no TC/TCC peripherals
		{ PORTA,  30, TC6_CH0,		0, TCC2_CH0, 		0, NOT_ON_TIMER, 	0},
		{ PORTA,  31, TC6_CH1,		1, TCC2_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTB,  30, TC0_CH0,		0, TCC4_CH0, 		0, TCC0_CH2, 		6},
		{ PORTB,  31, TC0_CH1,		1, TCC4_CH1, 		1, TCC0_CH3, 		7},
		// PC30, PC31 -> no TC/TCC peripherals
		{ PORTB,   0, TC7_CH0,		0, NOT_ON_TIMER, 	0, NOT_ON_TIMER, 	0},
		{ PORTB,   1, TC7_CH1,		1, NOT_ON_TIMER, 	0, NOT_ON_TIMER,	0},
		{ PORTB,   2, TC6_CH0,		0, TCC2_CH2, 		2, NOT_ON_TIMER,	0},

};
#define NUM_WO_ASSOCIATIONS 72

wo_association ASSOCIATION_NOT_FOUND = { NOT_A_PORT, 0, NOT_ON_TIMER, 0, NOT_ON_TIMER, 0};


struct wo_association& getWOAssociation(EPortType port, uint32_t pin) {
	for (int i=0;i<NUM_WO_ASSOCIATIONS;i++) {
		if (WO_associations[i].port==port && WO_associations[i].pin==pin)
			return WO_associations[i];
	}
	return ASSOCIATION_NOT_FOUND;
};


#endif
