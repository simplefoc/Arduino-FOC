#include "../../hardware_api.h"
#include "../../../drivers/hardware_api.h"
#include "../../../drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) 

#include "esp32_adc_driver.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f


typedef struct ESP32MCPWMCurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
  mcpwm_unit_t mcpwm_unit;
  int buffer_index;
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
  _UNUSED(driver_params);

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

static void IRAM_ATTR mcpwm0_isr_handler(void*);
static void IRAM_ATTR mcpwm1_isr_handler(void*);
byte currentState = 1;
// two mcpwm units 
// - max 2 motors per mcpwm unit (6 adc channels)
int adc_pins[2][6]={0};
int adc_pin_count[2]={0};
uint32_t adc_buffer[2][6]={0};
int adc_read_index[2]={0};

// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pin, const void* cs_params){
  mcpwm_unit_t unit = ((ESP32MCPWMCurrentSenseParams*)cs_params)->mcpwm_unit;
  int buffer_index = ((ESP32MCPWMCurrentSenseParams*)cs_params)->buffer_index;
  float adc_voltage_conv = ((ESP32MCPWMCurrentSenseParams*)cs_params)->adc_voltage_conv;

  for(int i=0; i < adc_pin_count[unit]; i++){
    if( pin == ((ESP32MCPWMCurrentSenseParams*)cs_params)->pins[i]) // found in the buffer
      return adc_buffer[unit][buffer_index + i] * adc_voltage_conv;
  }
  // not found
  return  0;
}

// function configuring low-side current sensing 
void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
  
  mcpwm_unit_t unit = ((ESP32MCPWMDriverParams*)driver_params)->mcpwm_unit;
  int index_start = adc_pin_count[unit];
  if( _isset(pinA) ) adc_pins[unit][adc_pin_count[unit]++] = pinA;
  if( _isset(pinB) ) adc_pins[unit][adc_pin_count[unit]++] = pinB;
  if( _isset(pinC) ) adc_pins[unit][adc_pin_count[unit]++] = pinC;

  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  ESP32MCPWMCurrentSenseParams* params = new ESP32MCPWMCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION),
    .mcpwm_unit = unit,
    .buffer_index = index_start
  };

  return params;
}


void _driverSyncLowSide(void* driver_params, void* cs_params){

  mcpwm_dev_t* mcpwm_dev = ((ESP32MCPWMDriverParams*)driver_params)->mcpwm_dev;
  mcpwm_unit_t mcpwm_unit = ((ESP32MCPWMDriverParams*)driver_params)->mcpwm_unit;

  // low-side register enable interrupt
  mcpwm_dev->int_ena.timer0_tep_int_ena = true;//A PWM timer 0 TEP event will trigger this interrupt
  // high side registers enable interrupt 
  //mcpwm_dev->int_ena.timer0_tep_int_ena = true;//A PWM timer 0 TEZ event will trigger this interrupt 

  // register interrupts (mcpwm number, interrupt handler, handler argument = NULL, interrupt signal/flag, return handler = NULL)
  if(mcpwm_unit == MCPWM_UNIT_0)
    mcpwm_isr_register(mcpwm_unit, mcpwm0_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
  else
    mcpwm_isr_register(mcpwm_unit, mcpwm1_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
}

// Read currents when interrupt is triggered
static void IRAM_ATTR mcpwm0_isr_handler(void*){
  // // high side
  // uint32_t mcpwm_intr_status = MCPWM0.int_st.timer0_tez_int_st;
  
  // low side
  uint32_t mcpwm_intr_status = MCPWM0.int_st.timer0_tep_int_st;
  if(mcpwm_intr_status){
    adc_buffer[0][adc_read_index[0]] = adcRead(adc_pins[0][adc_read_index[0]]);
    adc_read_index[0]++;
    if(adc_read_index[0] == adc_pin_count[0]) adc_read_index[0] = 0;
  }
  // low side
  MCPWM0.int_clr.timer0_tep_int_clr = mcpwm_intr_status;
  // high side
  // MCPWM0.int_clr.timer0_tez_int_clr = mcpwm_intr_status_0;
}


// Read currents when interrupt is triggered
static void IRAM_ATTR mcpwm1_isr_handler(void*){
  // // high side
  // uint32_t mcpwm_intr_status = MCPWM1.int_st.timer0_tez_int_st;
  
  // low side
  uint32_t mcpwm_intr_status = MCPWM1.int_st.timer0_tep_int_st;
  if(mcpwm_intr_status){
    adc_buffer[1][adc_read_index[1]] = adcRead(adc_pins[1][adc_read_index[1]]);
    adc_read_index[1]++;
    if(adc_read_index[1] == adc_pin_count[1]) adc_read_index[1] = 0;
  }
  // low side
  MCPWM1.int_clr.timer0_tep_int_clr = mcpwm_intr_status;
  // high side
  // MCPWM1.int_clr.timer0_tez_int_clr = mcpwm_intr_status_0;
}


#endif
