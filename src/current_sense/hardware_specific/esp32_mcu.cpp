#include "../hardware_api.h"
#include "../../drivers/hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) 

#include "esp32_adc_driver.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f

// adc counts to voltage conversion ratio
// some optimizing for faster execution
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )


/**
 *  Inline adc reading implementation 
*/
// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA){
  uint32_t raw_adc = adcRead(pinA);
  // uint32_t raw_adc = analogRead(pinA);
  return raw_adc * _ADC_CONV;
}

// function reading an ADC value and returning the read voltage
void _configureADCInline(const int pinA,const int pinB, const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}


/**
 *  Low side adc reading implementation 
*/
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
int a1, a2, a3;         // Current readings from internal current sensor amplifiers
int _pinA, _pinB, _pinC;
static void IRAM_ATTR isr_handler(void*);
byte currentState = 1;



// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pin){
  uint32_t raw_adc;

  if (pin == _pinA) raw_adc = a1;
  else if (pin == _pinB) raw_adc = a2;
  else if (pin == _pinC) raw_adc = a3;

  return raw_adc * _ADC_CONV;
}

// function reading an ADC value and returning the read voltage
void _configureADCLowSide(const int pinA,const int pinB,const int pinC){
  _pinA = pinA;
  _pinB = pinB;
  _pinC = pinC;
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}

void _driverSyncLowSide(){
  // high side registers enable interrupt 
  // MCPWM[MCPWM_UNIT_0]->int_ena.timer0_tez_int_ena = true;//A PWM timer 0 TEP event will trigger this interrupt
  // MCPWM[MCPWM_UNIT_0]->int_ena.timer1_tez_int_ena = true;//A PWM timer 1 TEP event will trigger this interrupt
  // if( _isset(_pinC) ) MCPWM[MCPWM_UNIT_0]->int_ena.timer2_tez_int_ena = true;//A PWM timer 2 TEP event will trigger this interrupt

  // low-side register enable interrupt
  MCPWM[MCPWM_UNIT_0]->int_ena.timer0_tep_int_ena = true;//A PWM timer 0 TEP event will trigger this interrupt
  MCPWM[MCPWM_UNIT_0]->int_ena.timer1_tep_int_ena = true;//A PWM timer 1 TEP event will trigger this interrupt
  if( _isset(_pinC) ) MCPWM[MCPWM_UNIT_0]->int_ena.timer2_tep_int_ena = true;//A PWM timer 2 TEP event will trigger this interrupt
  mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
}

// Read currents when interrupt is triggered
static void IRAM_ATTR isr_handler(void*){
  // // high side
  // uint32_t mcpwm_intr_status_0 = MCPWM[MCPWM_UNIT_0]->int_st.timer0_tez_int_st;
  // uint32_t mcpwm_intr_status_1 = MCPWM[MCPWM_UNIT_0]->int_st.timer1_tez_int_st;
  // uint32_t mcpwm_intr_status_2 = _isset(_pinC) ? MCPWM[MCPWM_UNIT_0]->int_st.timer2_tez_int_st : 0;

  // low side
  uint32_t mcpwm_intr_status_0 = MCPWM[MCPWM_UNIT_0]->int_st.timer0_tep_int_st;
  uint32_t mcpwm_intr_status_1 = MCPWM[MCPWM_UNIT_0]->int_st.timer1_tep_int_st;
  uint32_t mcpwm_intr_status_2 = _isset(_pinC) ? MCPWM[MCPWM_UNIT_0]->int_st.timer2_tep_int_st : 0;

  switch (currentState)
  {
  case 1 :
    if (mcpwm_intr_status_0 > 0) a1 = adcRead(_pinA);
    currentState = 2;
    break;
  case 2 :
    if (mcpwm_intr_status_1 > 0) a2 = adcRead(_pinB);
    currentState = _isset(_pinC) ?  3 : 1;
    break;
  case 3 :
    if (mcpwm_intr_status_2 > 0) a3 = adcRead(_pinC);
    currentState = 1;
    break;
  }

  // high side
  // MCPWM[MCPWM_UNIT_0]->int_clr.timer0_tez_int_clr = mcpwm_intr_status_0;
  // MCPWM[MCPWM_UNIT_0]->int_clr.timer1_tez_int_clr = mcpwm_intr_status_1;
  // if( _isset(_pinC) ) MCPWM[MCPWM_UNIT_0]->int_clr.timer2_tez_int_clr = mcpwm_intr_status_2;
  // low side
  MCPWM[MCPWM_UNIT_0]->int_clr.timer0_tep_int_clr = mcpwm_intr_status_0;
  MCPWM[MCPWM_UNIT_0]->int_clr.timer1_tep_int_clr = mcpwm_intr_status_1;
  if( _isset(_pinC) ) MCPWM[MCPWM_UNIT_0]->int_clr.timer2_tep_int_clr = mcpwm_intr_status_2;
}

#endif
