#include "esp32_mcu.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

// check the version of the ESP-IDF
#include "esp_idf_version.h"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
#error SimpleFOC: ESP-IDF version 4 or lower detected. Please update to ESP-IDF 5.x and Arduino-esp32 3.0 (or higher)
#endif

#include "../../../drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"
#include "../../../drivers/hardware_specific/esp32/mcpwm_private.h"

#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"



// adding a debug toggle pin to measure the time of the interrupt with oscilloscope
// #define SIMPLEFOC_ESP32_INTERRUPT_DEBUG

#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
#include "driver/gpio.h"

#ifndef DEBUGPIN
#ifdef CONFIG_IDF_TARGET_ESP32S3 
#define DEBUGPIN 16
#else
#define DEBUGPIN 19
#endif
#endif

#define GPIO_NUM (gpio_num_t)((int)GPIO_NUM_0 + DEBUGPIN)
#endif

#ifndef SIMPLEFOC_CS_PRETRIGGER_US
#define SIMPLEFOC_CS_PRETRIGGER_US 5 // 5 us because ADC read takes around 10us
#endif



/**
 *  Low side adc reading implementation
*/


// function reading an ADC value and returning the read voltage
float IRAM_ATTR _readADCVoltageLowSide(const int pin, const void* cs_params){
  ESP32CurrentSenseParams* p = (ESP32CurrentSenseParams*)cs_params;
  int no_channel = 0;
  for(int i=0; i < 3; i++){
    if(!_isset(p->pins[i])) continue;
    if(pin == p->pins[i]) // found in the buffer
      return p->adc_buffer[no_channel] * p->adc_voltage_conv;
    else no_channel++;
  }
  SIMPLEFOC_DEBUG("ERROR: ADC pin not found in the buffer!");
  // not found
  return  0;
}


// function configuring low-side current sensing
void* IRAM_ATTR _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
  // check if driver timer is already running
  // fail if it is
  // the easiest way that I've found to check if timer is running
  // is to start it and stop it
  ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams*)driver_params;
  mcpwm_timer_t* t = (mcpwm_timer_t*) p->timers[0];

  // check if low side callback is already set
  // if it is, return error
  if(t->on_full != nullptr){
    SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Low side callback is already set. Cannot set it again for timer: "+String(t->timer_id)+", group: "+String(t->group->group_id));
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  }


  ESP32CurrentSenseParams* params = new ESP32CurrentSenseParams{};
  int no_adc_channels = 0;

  // initialize the ADC pins
  // fail if the pin is not an ADC pin
  int adc_pins[3] = {pinA, pinB, pinC};
  for (int i = 0; i < 3; i++){
    if(_isset(adc_pins[i])){
      if(!adcInit(adc_pins[i])){
       SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Failed to initialise ADC pin: "+String(adc_pins[i]) + String(", maybe not an ADC pin?"));
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
      }
      params->pins[no_adc_channels++] = adc_pins[i];
    }
  }

  t->user_data = params;
  params->adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION);
  params->no_adc_channels = no_adc_channels;
  return params;
}

static bool IRAM_ATTR _mcpwmTriggerADCCallback(mcpwm_timer_handle_t tim, const mcpwm_timer_event_data_t* edata, void* user_data){
  ESP32CurrentSenseParams *p = (ESP32CurrentSenseParams*)user_data;
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG // debugging toggle pin to measure the time of the interrupt with oscilloscope
  gpio_set_level(GPIO_NUM,1); //cca 250ns for on+off
#endif

  // sample the phase currents one at a time
  // ESP's adc read takes around 10us which is very long
  // so we are sampling one phase per call
  p->adc_buffer[p->buffer_index] = adcRead(p->pins[p->buffer_index]);

  // increment buffer index
  p->buffer_index++;
  if(p->buffer_index >= p->no_adc_channels){
    p->buffer_index = 0;
  }

#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG // debugging toggle pin to measure the time of the interrupt with oscilloscope
  gpio_set_level(GPIO_NUM,0); //cca 250ns for on+off
#endif
  return true;
}

// Comparator on_reach callback: sample ADC pre-trigger
// In center-aligned mode, comparator fires twice per cycle (up and down)
// Only sample on down-count to get one sample per PWM period
static bool IRAM_ATTR _mcpwmComparatorADCCallback(mcpwm_cmpr_handle_t cmpr, const mcpwm_compare_event_data_t* edata, void* user_data){
  // Only trigger on down-count direction
  if(edata->direction != MCPWM_TIMER_DIRECTION_UP){
    return true;
  }

  ESP32CurrentSenseParams *p = (ESP32CurrentSenseParams*)user_data;
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
  gpio_set_level(GPIO_NUM,1);
#endif

  // sample the phase currents one at a time
  // ESP's adc read takes around 10us which is very long
  // so we are sampling one phase per call
  p->adc_buffer[p->buffer_index] = adcRead(p->pins[p->buffer_index]);

  // increment buffer index
  p->buffer_index++;
  if(p->buffer_index >= p->no_adc_channels){
    p->buffer_index = 0;
  }

#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
  gpio_set_level(GPIO_NUM,0);
#endif
  return true;
}

void* IRAM_ATTR _driverSyncLowSide(void* driver_params, void* cs_params){
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
  pinMode(DEBUGPIN, OUTPUT);
#endif
  ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams*)driver_params;
  mcpwm_timer_t* t = (mcpwm_timer_t*) p->timers[0];
  int group_id = p->group_id;

  
  ESP32CurrentSenseParams *cs = (ESP32CurrentSenseParams*)cs_params;
  if(!cs){
    SIMPLEFOC_ESP32_CS_DEBUG("ERROR: cs_params is null");
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  }

  SIMPLEFOC_ESP32_CS_DEBUG("Configuring sync with comparator...");

  // Create a spare comparator on the first operator for ADC pre-trigger
  // This comparator will fire ~5 µs before on_reach (peak)
  mcpwm_comparator_config_t cmp_config = {0};
  cmp_config.flags.update_cmp_on_tez = true;
  for(int i=2; i>=0; i--){ // start from the end as first operators are more likely to be used fully
    if(p->oper[i] == nullptr) continue;
    if(mcpwm_new_comparator(p->oper[i], &cmp_config, (mcpwm_cmpr_handle_t*)&cs->pretrig_comparator) == ESP_OK){
      break;
    }
  }

  // if comparator creation failed, fall back to on_full callback
  if (cs->pretrig_comparator){
    // Calculate pwm duty cycle ticks for pre-trigger channel
    // TODO: verify the timing it seems to be correct between 15 and 20kHz (but needs better testing)
    uint32_t pwm_duty_cycle  = p->mcpwm_period * (0.75 - ((float)p->pwm_frequency*SIMPLEFOC_CS_PRETRIGGER_US)/1e6/2.0); 
    // set up the comparator duty cycle
    CHECK_CS_ERR(mcpwm_comparator_set_compare_value((mcpwm_cmpr_handle_t)cs->pretrig_comparator, pwm_duty_cycle),
                "Failed to set pretrigger compare value");

    // Register comparator on_reach callback for ADC sampling
    mcpwm_comparator_event_callbacks_t cmp_cbs = {
      .on_reach = _mcpwmComparatorADCCallback
    };
    // register the callback
    CHECK_CS_ERR(mcpwm_comparator_register_event_callbacks((mcpwm_cmpr_handle_t)cs->pretrig_comparator, &cmp_cbs, cs_params),
                "Failed to register comparator callback");

    SIMPLEFOC_ESP32_CS_DEBUG("MCPWM"+String(group_id)+" Timer "+String(t->timer_id)+" pretrigger comparator configured.");
    // notify the driver code that low side is uses one comparator
    _notifyLowSideUsingComparator(group_id);

  }else{

    SIMPLEFOC_ESP32_CS_DEBUG("WARN: Failed to create comparator!");
    SIMPLEFOC_ESP32_CS_DEBUG("Configuring sync with on_full callback (less accurate)...");

    // check if low side callback is already set
    // if it is, return error
    if(t->on_full != nullptr){
      SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Low side callback is already set. Cannot set it again for timer: "+String(t->timer_id)+", group: "+String(t->group->group_id));
      return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }
     // set the callback for the low side current sensing
    // mcpwm_timer_event_callbacks_t can be used to set the callback
    // for three timer events
    // - on_full  - low-side
    // - on_empty - high-side
    // - on_sync  - sync event (not used with simplefoc)
    auto cbs = mcpwm_timer_event_callbacks_t{
      .on_full = _mcpwmTriggerADCCallback,
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
    CHECK_CS_ERR(esp_intr_enable(t->intr), "Failed to enable low-side interrupts!");

    SIMPLEFOC_ESP32_CS_DEBUG("MCPWM"+String(group_id)+" Timer "+String(t->timer_id)+" on_full callback configured.");
  }


  return cs_params;
}


#endif
