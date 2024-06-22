#ifndef ESP32_MCU_CURRENT_SENSING_H
#define ESP32_MCU_CURRENT_SENSING_H

#include "../../hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) 


#include "../../../drivers/hardware_api.h"
#include "esp32_adc_driver.h"


// esp32 current sense parameters
typedef struct ESP32CurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
  int adc_buffer[3] = {};
  int buffer_index = 0;
  int no_adc_channels = 0;
} ESP32CurrentSenseParams;

// macros for debugging wuing the simplefoc debug system
#define SIMPLEFOC_ESP32_CS_DEBUG(str)\
   SimpleFOCDebug::println( "ESP32-CS: "+ String(str));\
   
#define CHECK_CS_ERR(func_call, message) \
  if ((func_call) != ESP_OK) { \
    SIMPLEFOC_ESP32_CS_DEBUG("ERROR - " + String(message)); \
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED; \
  }

  
#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f

#endif // ESP_H && ARDUINO_ARCH_ESP32
#endif