#include "../hardware_api.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) 

#define _ADC_VOLTAGE 5.0f
#define _ADC_RESOLUTION 1024.0f

// adc counts to voltage conversion ratio
// some optimizing for faster execution
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// adc counts to voltage conversion ratio
// some optimizing for faster execution
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * _ADC_CONV;
}
// function reading an ADC value and returning the read voltage
void _configureADCInline(const int pinA,const int pinB,const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  // set hight frequency adc - ADPS2,ADPS1,ADPS0 | 001 (16mhz/2) | 010 ( 16mhz/4 ) | 011 (16mhz/8) | 100(16mhz/16) | 101 (16mhz/32) | 110 (16mhz/64) | 111 (16mhz/128 default)  
  // set divisor to 8 - adc frequency 16mhz/8 = 2 mhz 
  // arduino takes 25 conversions per sample so - 2mhz/25 = 80k samples per second - 12.5us per sample 
  cbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);
}


// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pinA){
  return _readADCVoltageInline(pinA);
}
// Configure low side for generic mcu
// cannot do much but 
void _configureADCLowSide(const int pinA,const int pinB,const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}

#endif