#include "../hardware_api.h"


#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  // if mcu is not atmega328
  #define _ADC_VOLTAGE 5.0
  #define _ADC_RESOLUTION 1024.0
#elif defined(__AVR_ATmega2560__) // if mcu is not atmega2560
  #define _ADC_VOLTAGE 5.0
  #define _ADC_RESOLUTION 1024.0
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
  voltage = raw_adc * 5.0/1024.0;
#endif

float _readADCVoltage(const int pinA){
  int raw_adc = analogRead(pinA);
  return raw_adc * (float)_ADC_VOLTAGE / (float)_ADC_RESOLUTION;
}