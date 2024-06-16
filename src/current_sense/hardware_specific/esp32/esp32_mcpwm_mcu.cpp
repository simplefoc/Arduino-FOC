#include "../../hardware_api.h"
#include "../../../drivers/hardware_api.h"
#include "../../../drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"
#include "../../../drivers/hardware_specific/esp32/mcpwm_private.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

// check the version of the ESP-IDF
#include "esp_idf_version.h"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
#error SimpleFOC: ESP-IDF version 4 or lower detected. Please update to ESP-IDF 5.x and Arduino-esp32 3.0 (or higher)
#endif


#include "esp32_adc_driver.h"

#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
#include "driver/gpio.h"
#endif

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f



#define SIMPLEFOC_ESP32_CS_DEBUG(str)\
   SIMPLEFOC_ESP32_DEBUG("CS", str);\
   
#define CHECK_CS_ERR(func_call, message) \
  if ((func_call) != ESP_OK) { \
    SIMPLEFOC_ESP32_CS_DEBUG("ERROR - " + String(message)); \
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED; \
  }

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


  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  ESP32MCPWMCurrentSenseParams* params = new ESP32MCPWMCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };

  return params;
}


/**
 *  Low side adc reading implementation 
*/


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
  // check if driver timer is already running 
  // fail if it is
  // the easiest way that I've found to check if timer is running
  // is to start it and stop it 
  ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams*)driver_params;
  mcpwm_timer_t* t = (mcpwm_timer_t*) p->timers[0];

  // check if low side callback is already set
  // if it is, return error
  if(t->on_full != nullptr){
    SIMPLEFOC_ESP32_CS_DEBUG("Low side callback is already set. Cannot set it again for timer: "+String(t->timer_id)+", group: "+String(t->group->group_id));
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
  
  t->user_data = params;
  params->adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION);
  params->no_adc_channels = no_adc_channels;
  return params;
}

void* _driverSyncLowSide(void* driver_params, void* cs_params){
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
  pinMode(19, OUTPUT);
#endif
  ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams*)driver_params;
  mcpwm_timer_t* t = (mcpwm_timer_t*) p->timers[0];

  // check if low side callback is already set
  // if it is, return error
  if(t->on_full != nullptr){
    SIMPLEFOC_ESP32_CS_DEBUG("Low side callback is already set. Cannot set it again for timer: "+String(t->timer_id)+", group: "+String(t->group->group_id));
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  }

  // set the callback for the low side current sensing
  // mcpwm_timer_event_callbacks_t can be used to set the callback
  // for three timer events 
  // - on_full  - low-side
  // - on_empty - high-side
  // - on_sync  - sync event (not used with simplefoc)
  auto cbs = mcpwm_timer_event_callbacks_t{
    .on_full = [](mcpwm_timer_handle_t tim, const mcpwm_timer_event_data_t* edata, void* user_data){ 
      ESP32MCPWMCurrentSenseParams *p = (ESP32MCPWMCurrentSenseParams*)user_data;
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
      gpio_set_level(GPIO_NUM_19,1); //cca 250ns for on+off
#endif
      // increment buffer index
      p->buffer_index = (p->buffer_index + 1) % p->no_adc_channels;
      // sample the phase currents one at a time
      // adc read takes around 10us which is very long
      // so we are sampling one phase per call
      p->adc_buffer[p->buffer_index] = adcRead(p->pins[p->buffer_index]); 
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
      gpio_set_level(GPIO_NUM_19,0); //cca 250ns for on+off
#endif
      return true; 
    },
  };
  SIMPLEFOC_ESP32_CS_DEBUG("Timer "+String(t->timer_id)+" enable interrupt callback.");
  // set the timer state to init (so that we can call the `mcpwm_timer_register_event_callbacks` )
  // this is a hack, as this function is not supposed to be called when the timer is running
  // the timer does not really go to the init state!
  t->fsm = MCPWM_TIMER_FSM_INIT;
  // set the callback
  CHECK_CS_ERR(mcpwm_timer_register_event_callbacks(t, &cbs, cs_params), "Failed to set low side callback");
  // set the timer state to enabled again
  t->fsm = MCPWM_TIMER_FSM_ENABLE;
  SIMPLEFOC_ESP32_CS_DEBUG("Timer "+String(t->timer_id)+" enable interrupts.");
  CHECK_CS_ERR(esp_intr_enable(t->intr), "Failed to enable low-side interrupts!");

  return cs_params;
}


#endif
