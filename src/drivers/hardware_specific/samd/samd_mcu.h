
#ifndef SAMD_MCU_H
#define SAMD_MCU_H


// uncomment to enable debug output from SAMD driver
// can set this as build-flag in Arduino IDE or PlatformIO
// #define SIMPLEFOC_SAMD_DEBUG

#include "../../hardware_api.h"


#if defined(__SAME51J19A__) || defined(__ATSAME51J19A__)
#ifndef _SAME51_
#define _SAME51_
#endif
#endif


#if defined(_SAMD21_)||defined(_SAMD51_)||defined(_SAME51_)


#include "Arduino.h"
#include "variant.h"
#include "wiring_private.h"


#ifndef SIMPLEFOC_SAMD_PWM_RESOLUTION
#define SIMPLEFOC_SAMD_PWM_RESOLUTION 1000
#endif

#define SIMPLEFOC_SAMD_DEFAULT_PWM_FREQUENCY_HZ 24000
// arbitrary maximum. On SAMD51 with 120MHz clock this means 2kHz minimum pwm frequency
#define SIMPLEFOC_SAMD_MAX_PWM_RESOLUTION 30000
// lets not go too low - 400 with clock speed of 120MHz on SAMD51 means 150kHz maximum PWM frequency...
//						 400 with 48MHz clock on SAMD21 means 60kHz maximum PWM frequency...
#define SIMPLEFOC_SAMD_MIN_PWM_RESOLUTION 400 
// this is the most we can support on the TC units
#define SIMPLEFOC_SAMD_PWM_TC_RESOLUTION 250

#ifndef SIMPLEFOC_SAMD_MAX_TCC_PINCONFIGURATIONS
#define SIMPLEFOC_SAMD_MAX_TCC_PINCONFIGURATIONS 24
#endif



struct tccConfiguration {
	uint8_t pin;
	EPioType peripheral; // 1=true, 0=false
	uint8_t wo;
	union tccChanInfo {
		struct {
			int8_t chan;
			int8_t tccn;
		};
		uint16_t chaninfo;
	} tcc;
	uint16_t pwm_res;
};






struct wo_association {
	EPortType port;
	uint32_t pin;
	ETCChannel tccE;
	uint8_t woE;
	ETCChannel tccF;
	uint8_t woF;
#if defined(_SAMD51_)||defined(_SAME51_)
	ETCChannel tccG;
	uint8_t woG;
#endif
};



typedef struct SAMDHardwareDriverParams {
	tccConfiguration* tccPinConfigurations[6];
	uint32_t pwm_frequency;
	float dead_zone;
} SAMDHardwareDriverParams;




#if defined(_SAMD21_)
#define NUM_PIO_TIMER_PERIPHERALS 2
#elif defined(_SAMD51_)||defined(_SAME51_)
#define NUM_PIO_TIMER_PERIPHERALS 3
#endif



/**
 * Global state
 */
extern struct wo_association WO_associations[];
extern uint8_t TCC_CHANNEL_COUNT[];
extern tccConfiguration tccPinConfigurations[SIMPLEFOC_SAMD_MAX_TCC_PINCONFIGURATIONS];
extern uint8_t numTccPinConfigurations;
extern bool SAMDClockConfigured;
extern bool tccConfigured[TCC_INST_NUM+TC_INST_NUM];



struct wo_association& getWOAssociation(EPortType port, uint32_t pin);
void writeSAMDDutyCycle(tccConfiguration* info, float dc);
void configureSAMDClock();
void configureTCC(tccConfiguration& tccConfig, long pwm_frequency, bool negate=false, float hw6pwm=-1);
__inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
EPioType getPeripheralOfPermutation(int permutation, int pin_position);

#ifdef SIMPLEFOC_SAMD_DEBUG
void printTCCConfiguration(tccConfiguration& info);
void printAllPinInfos();
#endif



#endif


#endif
