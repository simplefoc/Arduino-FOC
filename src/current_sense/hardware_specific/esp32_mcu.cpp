#include "../hardware_api.h"

#if defined(ESP_H)

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define _ADC_VOLTAGE 3.3
#define _ADC_RESOLUTION 4095.0

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
int a1, a2, a3;         //Current readings from internal current sensor amplifiers
int _pinA, _pinB, _pinC = NOT_SET;
static void IRAM_ATTR isr_handler(void*);
byte currentState = 1;

// adc counts to voltage conversion ratio
// some optimizing for faster execution
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

bool _readADCVoltagesLowSide( float & a, float & b, float & c)
{
  a = a1 * _ADC_CONV;
  b = a2 * _ADC_CONV;
  if( _isset(pinC) )
    c = a3 * _ADC_CONV;
  return true;
}

// function reading an ADC value and returning the read voltage
void _configureADCLowSide(const int pinA,const int pinB,const int pinC){
  _pinA = pinA;
  _pinB = pinB;
  if( _isset(pinC) ) _pinC = pinC;
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

}

void _driverSyncLowSide(){
  MCPWM[MCPWM_UNIT_0]->int_ena.timer0_tep_int_ena = true;//A PWM timer 0 TEP event will trigger this interrupt
  MCPWM[MCPWM_UNIT_0]->int_ena.timer1_tep_int_ena = true;//A PWM timer 1 TEP event will trigger this interrupt
  //MCPWM[MCPWM_UNIT_0]->int_ena.timer2_tep_int_ena = true;//A PWM timer 2 TEP event will trigger this interrupt
  mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
}

// Read currents when interrupt is triggered
static void IRAM_ATTR isr_handler(void*){
  uint32_t mcpwm_intr_status_0 = MCPWM[MCPWM_UNIT_0]->int_st.timer0_tep_int_st;
  uint32_t mcpwm_intr_status_1 = MCPWM[MCPWM_UNIT_0]->int_st.timer1_tep_int_st;
  //uint32_t mcpwm_intr_status_2 = MCPWM[MCPWM_UNIT_0]->int_st.timer2_tep_int_st;

  if(mcpwm_intr_status_0 > 0 && currentState == 1){
    a1 = analogRead(_pinA);
    //a2 = analogRead(_pinB);
    currentState = 2;
  }
  else if(mcpwm_intr_status_1 > 0 && currentState == 2){
    a2 = analogRead(_pinB);
    //a3 = analogRead(_pinC);
    currentState = 1;
  }
  /*
  else if(mcpwm_intr_status_2 > 0 && currentState == 3){
    a3 = analogRead(_pinC);
    //a1 = analogRead(_pinA);
    currentState = 1;
  }*/

  MCPWM[MCPWM_UNIT_0]->int_clr.timer0_tep_int_clr = mcpwm_intr_status_0;
  MCPWM[MCPWM_UNIT_0]->int_clr.timer1_tep_int_clr = mcpwm_intr_status_1;
  //MCPWM[MCPWM_UNIT_0]->int_clr.timer2_tep_int_clr = mcpwm_intr_status_2;
}

#endif
