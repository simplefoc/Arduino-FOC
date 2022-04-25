#include "../hardware_api.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) 

#define _ADC_VOLTAGE 5.0f
#define _ADC_RESOLUTION 1024.0f

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// function reading an ADC value and returning the read voltage
void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  
  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };

  // set hight frequency adc - ADPS2,ADPS1,ADPS0 | 001 (16mhz/2) | 010 ( 16mhz/4 ) | 011 (16mhz/8) | 100(16mhz/16) | 101 (16mhz/32) | 110 (16mhz/64) | 111 (16mhz/128 default)  
  // set divisor to 8 - adc frequency 16mhz/8 = 2 mhz 
  // arduino takes 25 conversions per sample so - 2mhz/25 = 80k samples per second - 12.5us per sample 
  cbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  return params;
}


#endif