#include "../hardware_api.h"
#include "../../drivers/hardware_api.h"
#include "../../drivers/hardware_specific/esp32_driver_mcpwm.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) 

#include "esp32_adc_driver.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f

/**
 *  Inline adc reading implementation 
*/
// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA, const void* cs_params){
  uint32_t raw_adc = adcRead(pinA);
  return raw_adc * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function reading an ADC value and returning the read voltage
void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };

  return params;
}



/**
 *  Low side adc reading implementation 
*/
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
int a1, a2, a3;         // Current readings from internal current sensor amplifiers
int _pinA, _pinB, _pinC;

static void IRAM_ATTR mcpwm0_isr_handler(void*);
static void IRAM_ATTR mcpwm1_isr_handler(void*);
byte currentState = 1;

int* adc_pins;
int* adc_buffer;
int adc_pin_count;
int adc_pin_read_index;


// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pin, const void* cs_params){
  uint32_t raw_adc;

  if (pin == _pinA) raw_adc = a1;
  else if (pin == _pinB) raw_adc = a2;
  else if (pin == _pinC) raw_adc = a3;

  return  raw_adc * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function reading an ADC value and returning the read voltage
void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

  _pinA= pinA;
  _pinB= pinB;
  if( _isset(pinC) ) _pinC= pinC;

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };

  return params;
}


void _driverSyncLowSide(void* driver_params, void* cs_params){
  // high side registers enable interrupt 
  // MCPWM[MCPWM_UNIT_0]->int_ena.timer0_tez_int_ena = true;//A PWM timer 0 TEP event will trigger this interrupt

  // low-side register enable interrupt
  mcpwm_dev_t* mcpwm_dev = ((ESP32MCPWMDriverParams*)driver_params)->mcpwm_dev;
  mcpwm_unit_t mcpwm_unit = ((ESP32MCPWMDriverParams*)driver_params)->mcpwm_unit;
  mcpwm_dev->int_ena.timer0_tep_int_ena = true;//A PWM timer 0 TEP event will trigger this interrupt
  if(mcpwm_unit == MCPWM_UNIT_0)
    mcpwm_isr_register(mcpwm_unit, mcpwm0_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
  else
    mcpwm_isr_register(mcpwm_unit, mcpwm1_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
}

// Read currents when interrupt is triggered
static void IRAM_ATTR mcpwm0_isr_handler(void*){
  // // high side
  // uint32_t mcpwm_intr_status_0 = MCPWM0.int_st.timer0_tez_int_st;
  
  // low side
  uint32_t mcpwm_intr_status = MCPWM0.int_st.timer0_tep_int_st;
  if(mcpwm_intr_status){
    switch (currentState)
    {
    case 1 :
      a1 = adcRead(_pinA);
      currentState = 2;
      break;
    case 2 :
      a2 = adcRead(_pinB);
      currentState = _isset(_pinC) ?  3 : 1;
      break;
    case 3 :
      a3 = adcRead(_pinC);
      currentState = 1;
      break;
    }
  }
  // high side
  // MCPWM0.int_clr.timer0_tez_int_clr = mcpwm_intr_status_0;

  // low side
  MCPWM0.int_clr.timer0_tep_int_clr = mcpwm_intr_status;
}

// Read currents when interrupt is triggered
static void IRAM_ATTR mcpwm1_isr_handler(void*){
  // // high side
  // uint32_t mcpwm_intr_status_0 = MCPWM1.int_st.timer0_tez_int_st;
  
  // low side
  uint32_t mcpwm_intr_status = MCPWM1.int_st.timer0_tep_int_st;
  if(mcpwm_intr_status){
    switch (currentState)
    {
    case 1 :
      a1 = adcRead(_pinA);
      currentState = 2;
      break;
    case 2 :
      a2 = adcRead(_pinB);
      currentState = _isset(_pinC) ?  3 : 1;
      break;
    case 3 :
      a3 = adcRead(_pinC);
      currentState = 1;
      break;
    }
  }
  // high side
  // MCPWM1.int_clr.timer0_tez_int_clr = mcpwm_intr_status_0;

  // low side
  MCPWM1.int_clr.timer0_tep_int_clr = mcpwm_intr_status;
}


#endif
