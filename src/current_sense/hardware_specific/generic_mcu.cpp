#include "../hardware_api.h"



#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) //  if mcu is atmega328 or atmega2560
  #define _ADC_VOLTAGE 5.0
  #define _ADC_RESOLUTION 1024.0
  #ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
  #endif
  #ifndef sbi
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
  #endif
#elif defined(__arm__) && defined(CORE_TEENSY)  // or teensy
  #define _ADC_VOLTAGE 3.3
  #define _ADC_RESOLUTION 1024.0
#elif defined(__arm__) && defined(__SAM3X8E__)  // or due
  #define _ADC_VOLTAGE 3.3
  #define _ADC_RESOLUTION 1024.0
#elif defined(ESP_H)  // or esp32
  #define _ADC_VOLTAGE 3.3 
  #define _ADC_RESOLUTION 4095.0
#elif defined(_STM32_DEF_) // or stm32
  #define _ADC_VOLTAGE 3.3
  #define _ADC_RESOLUTION 1024.0
#else
  #define _ADC_VOLTAGE 5.0
  #define _ADC_RESOLUTION 1024.0
#endif

// adc counts to voltage conversion ratio
// some optimizing for faster execution
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

// function reading an ADC value and returning the read voltage
float _readADCVoltage(const int pinA){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * _ADC_CONV;
}


// function reading an ADC value and returning the read voltage
void _configureADC(const int pinA,const int pinB,const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) //  if mcu is atmega328 or atmega2560
    // set hight frequency adc - ADPS2,ADPS1,ADPS0 | 001 (16mhz/2) | 010 ( 16mhz/4 ) | 011 (16mhz/8) | 100(16mhz/16) | 101 (16mhz/32) | 110 (16mhz/64) | 111 (16mhz/128 default)  
    // set divisor to 8 - adc frequency 16mhz/8 = 2 mhz 
    // arduino takes 25 conversions per sample so - 2mhz/25 = 80k samples per second - 12.5us per sample 
    cbi(ADCSRA, ADPS2);
    sbi(ADCSRA, ADPS1);
    sbi(ADCSRA, ADPS0);
  #endif
}