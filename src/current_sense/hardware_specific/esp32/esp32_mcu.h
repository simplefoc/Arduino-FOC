#ifndef ESP32_MCU_CURRENT_SENSING_H
#define ESP32_MCU_CURRENT_SENSING_H

#include "../../hardware_api.h"
#include <stdint.h>

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) 


#include "../../../drivers/hardware_api.h"
#include "esp32_adc_driver.h"


/*
 * Low-side ADC backend (set during LowsideCurrentSense::init):
 *   ADC_READ — default; MCPWM ISR + adcRead(), one phase per interrupt.
 *   DIGI_SW  — digi controller + DMA; ISR only starts conversion (ESP32, S2, …).
 *   DIGI_ETM — digi + DMA; MCPWM TEZ starts ADC via ETM (S3, C6, …).
 */
enum ESP32AdcLowsidePath : uint8_t {
  ESP32_ADC_LOWSIDE_ADC_READ = 0,
  ESP32_ADC_LOWSIDE_DIGI_SW,
  ESP32_ADC_LOWSIDE_DIGI_ETM,
};

// esp32 current sense parameters
typedef struct ESP32CurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
  /* ADC_READ: plain counts; DIGI_*: DMA lands adc_digi_output_data_t[0..N] here (zero-copy) */
  int adc_buffer[3] = {};
  int buffer_index = 0;
  int no_adc_channels = 0;
  void* pretrig_comparator = nullptr;
  ESP32AdcLowsidePath adc_lowside_path = ESP32_ADC_LOWSIDE_ADC_READ;
} ESP32CurrentSenseParams;

// macros for debugging wuing the simplefoc debug system
#ifndef SIMPLEFOC_DISABLE_DEBUG  
#define SIMPLEFOC_ESP32_CS_DEBUG(str)\
  SimpleFOCDebug::println( "ESP32-CS: "+ String(str));  
#else
#define SIMPLEFOC_ESP32_CS_DEBUG(str)
#endif   

  
#define CHECK_CS_ERR(func_call, message) \
  if ((func_call) != ESP_OK) { \
    SIMPLEFOC_ESP32_CS_DEBUG("ERROR - " + String(message)); \
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED; \
  }

  
#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f

#endif // ESP_H && ARDUINO_ARCH_ESP32
#endif