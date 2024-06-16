#include "../../hardware_api.h"
#include "../../../drivers/hardware_api.h"
#include "../../../drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC) 

#include "esp32_adc_driver.h"

#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include "esp_idf_version.h"  

// version check - this mcpwm driver is specific for ESP-IDF 5.x and arduino-esp32 3.x
#if ESP_IDF_VERSION_MAJOR < 5 
#error SimpleFOC: ESP-IDF version 4 or lower detected. Please update to ESP-IDF 5.x and Arduino-esp32 3.0 (or higher)
#endif

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f

// set the pin 19 in high during the adc interrupt
// #define SIMPLEFOC_ESP32_INTERRUPT_DEBUG

typedef struct ESP32MCPWMCurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
  int adc_buffer[3] = {};
  int buffer_index = 0;
  int no_adc_channels = 0;
} ESP32MCPWMCurrentSenseParams;


/**
 *  Inline adc reading implementation 
*/
// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA, const void* cs_params){
  uint32_t raw_adc = adcRead(pinA);
  return raw_adc * ((ESP32MCPWMCurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function reading an ADC value and returning the read voltage
void* _configureADCInline(const void* driver_params, const int pinA, const int pinB, const int pinC){

  SIMPLEFOC_DEBUG("ESP32-CS: Configuring ADC inline");
  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  ESP32MCPWMCurrentSenseParams* params = new ESP32MCPWMCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };

  return params;
}


// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pin, const void* cs_params){
  ESP32MCPWMCurrentSenseParams* p = (ESP32MCPWMCurrentSenseParams*)cs_params;
  int no_channel = 0;
  for(int i=0; i < 3; i++){
    if(!_isset(p->pins[i])) continue;
    if(pin == p->pins[i]) // found in the buffer
      return p->adc_buffer[no_channel] * p->adc_voltage_conv;
    else no_channel++;
  }
  // not found
  return  0;
}

// function configuring low-side current sensing 
void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
  
  SIMPLEFOC_DEBUG("ESP32-CS: Configuring ADC low-side");
  // check if driver timer is already running 
  // fail if it is
  // the easiest way that I've found to check if timer is running
  // is to start it and stop it 
  ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams*)driver_params;
  if(mcpwm_timer_start_stop(p->timers[0], MCPWM_TIMER_START_NO_STOP) != ESP_ERR_INVALID_STATE){
    // if we get the invalid state error it means that the timer is not enabled
    // that means that we can configure it for low-side current sensing
    SIMPLEFOC_DEBUG("ESP32-CS: ERR - The timer is already enabled. Cannot be configured for low-side current sensing.");
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  }


  ESP32MCPWMCurrentSenseParams* params = new ESP32MCPWMCurrentSenseParams{};
  int no_adc_channels = 0;
  if( _isset(pinA) ){
    pinMode(pinA, INPUT);
    params->pins[no_adc_channels++] = pinA;
  }
  if( _isset(pinB) ){
    pinMode(pinB, INPUT);
    params->pins[no_adc_channels++] = pinB;
  }
  if( _isset(pinC) ){
    pinMode(pinC, INPUT);
    params->pins[no_adc_channels++] = pinC;
  }

  params->adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION);
  params->no_adc_channels = no_adc_channels;
  return params;
}


void _driverSyncLowSide(void* driver_params, void* cs_params){
#ifdef  SIMPLEFOC_ESP32_INTERRUPT_DEBUG
  pinMode(19, OUTPUT);
#endif
  ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams*)driver_params;
  
  mcpwm_timer_event_callbacks_t cbs_timer = {
    .on_full = [](mcpwm_timer_handle_t tim, const mcpwm_timer_event_data_t* edata, void* user_data){ 
      ESP32MCPWMCurrentSenseParams *p = (ESP32MCPWMCurrentSenseParams*)user_data;
      #ifdef  SIMPLEFOC_ESP32_INTERRUPT_DEBUG
      digitalWrite(19, HIGH);
      #endif
      // increment buffer index
      p->buffer_index = (p->buffer_index + 1) % p->no_adc_channels;
      // sample the phase currents one at a time
      p->adc_buffer[p->buffer_index] = adcRead(p->pins[p->buffer_index]);
      #ifdef  SIMPLEFOC_ESP32_INTERRUPT_DEBUG
      digitalWrite(19, LOW);
      #endif
      return true; 
    }
  };
  if(mcpwm_timer_register_event_callbacks(p->timers[0], &cbs_timer, cs_params) != ESP_OK){
    SIMPLEFOC_DEBUG("ESP32-CS: ERR - Failed to sync ADC and driver");
  } 
}



#endif
