#include "../../hardware_api.h"
#include "../../../drivers/hardware_api.h"
#include "../../../drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

#include "esp32_adc_driver.h"
#include "esp32_i2s_driver.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <freertos/portmacro.h>

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f
#define _I2S_ADC true

typedef struct ESP32MCPWMCurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
  mcpwm_unit_t mcpwm_unit;
  int buffer_index;
} ESP32MCPWMCurrentSenseParams;

#ifdef HFI
__attribute__((weak)) void IRAM_ATTR process_hfi(){};
#endif

/**
 *  Inline adc reading implementation 
*/
// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA, const void *cs_params)
{
#if _I2S_ADC == true
  return _readADCVoltageI2S(pinA, cs_params);
#else
  uint32_t raw_adc = adcRead(pinA);
  return raw_adc * ((ESP32MCPWMCurrentSenseParams *)cs_params)->adc_voltage_conv;
#endif
}

// function reading an ADC value and returning the read voltage
void* _configureADCInline(const void* driver_params, const int pinA, const int pinB, const int pinC){
#if _I2S_ADC == true
  return _configureI2S(false, driver_params, pinA, pinB, pinC);
#else
  _UNUSED(driver_params);

  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  ESP32MCPWMCurrentSenseParams* params = new ESP32MCPWMCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };
  return params;
#endif
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
float IRAM_ATTR _readADCVoltageLowSide(const int pin, const void* cs_params){
#if _I2S_ADC == true
  return _readADCVoltageI2S(pin, cs_params);
#else
  mcpwm_unit_t unit = ((ESP32MCPWMCurrentSenseParams*)cs_params)->mcpwm_unit;
  int buffer_index = ((ESP32MCPWMCurrentSenseParams*)cs_params)->buffer_index;
  float adc_voltage_conv = ((ESP32MCPWMCurrentSenseParams*)cs_params)->adc_voltage_conv;

  for(int i=0; i < adc_pin_count[unit]; i++){
    if( pin == ((ESP32MCPWMCurrentSenseParams*)cs_params)->pins[i]) // found in the buffer
      return adc_buffer[unit][buffer_index + i] * adc_voltage_conv;
  }
  // not found
  return  0;
#endif
}

// function configuring low-side current sensing 
void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
#if _I2S_ADC == true
  return _configureI2S(true, driver_params, pinA, pinB, pinC);
#else
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
#endif
}


void _driverSyncLowSide(void* driver_params, void* cs_params){

  mcpwm_dev_t* mcpwm_dev = ((ESP32MCPWMDriverParams*)driver_params)->mcpwm_dev;
  mcpwm_unit_t mcpwm_unit = ((ESP32MCPWMDriverParams*)driver_params)->mcpwm_unit;
  #ifndef HFI
    // low-side register enable interrupt
    mcpwm_dev->int_ena.timer0_tep_int_ena = true;//A PWM timer 0 TEP event will trigger this interrupt
    // high side registers enable interrupt 
    // mcpwm_dev->int_ena.timer0_tep_int_ena = true;//A PWM timer 0 TEZ event will trigger this interrupt 
  #else
    // low-side & high side register enable interrupt
    mcpwm_dev->int_ena.timer0_tep_int_ena = true;//A PWM timer 0 TEP event will trigger this interrupt
    mcpwm_dev->int_ena.timer0_tez_int_ena = true;//A PWM timer 0 TEZ event will trigger this interrupt 
  #endif
  // register interrupts (mcpwm number, interrupt handler, handler argument = NULL, interrupt signal/flag, return handler = NULL)
  if(mcpwm_unit == MCPWM_UNIT_0)
    mcpwm_isr_register(mcpwm_unit, mcpwm0_isr_handler, NULL, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, NULL);  //Set ISR Handler
  else
    mcpwm_isr_register(mcpwm_unit, mcpwm1_isr_handler, NULL, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, NULL);  //Set ISR Handler
}

static uint32_t DRAM_ATTR cp0_regs[65];
static portMUX_TYPE fpuInISRMutex = portMUX_INITIALIZER_UNLOCKED;;

static void IRAM_ATTR mcpwm0_isr_handler(void*) __attribute__ ((unused));

// Read currents when interrupt is triggered
static void IRAM_ATTR mcpwm0_isr_handler(void*){
  // GPIO.out_w1ts = ((uint32_t)1 << 19);
  
  // // high side
  uint32_t mcpwm_intr_status_high = MCPWM0.int_st.timer0_tez_int_st;

  // low side
  uint32_t mcpwm_intr_status_low = MCPWM0.int_st.timer0_tep_int_st;

  // uint32_t mcpwm_intr_status = (MCPWM0.int_st.timer0_tep_int_st << 5) | (MCPWM0.int_st.timer0_tez_int_st << 2);
  #ifndef HFI_2XPWM
    bool runadc = mcpwm_intr_status_low;
  #else
    bool runadc = mcpwm_intr_status_high || mcpwm_intr_status_low;
  #endif

  if(mcpwm_intr_status_high || mcpwm_intr_status_low){
      GPIO.out_w1ts = ((uint32_t)1 << 19);

    if(runadc){
      #if _I2S_ADC == true
        delayMicroseconds(2);
        readFiFo();
      #else
        adc_buffer[0][adc_read_index[0]] = adcRead(adc_pins[0][adc_read_index[0]]);
        adc_read_index[0]++;
        if(adc_read_index[0] == adc_pin_count[0]) adc_read_index[0] = 0;
      #endif
    }
      GPIO.out_w1tc = ((uint32_t)1 << 19);

    #ifdef HFI
      // // enable FPU
      // xthal_set_cpenable(1);
      // // Save FPU registers
      // xthal_save_cp0(cp0_regs);
      
      portENTER_CRITICAL_ISR(&fpuInISRMutex);
      uint32_t cp_state = xthal_get_cpenable();
      if(!cp_state) {
          xthal_set_cpenable(1);
      }

      xthal_save_cp0(cp0_regs);   // Save FPU registers

      process_hfi();

      
      xthal_restore_cp0(cp0_regs);

      if(cp_state) {
          // Restore FPU registers
      } else {
          // turn it back off
          xthal_set_cpenable(0);
      }
      portEXIT_CRITICAL_ISR(&fpuInISRMutex);

      // // Restore FPU
      // xthal_restore_cp0(cp0_regs);
      // // and turn it back off
      // xthal_set_cpenable(0);
    #endif
  }
  // low side
  MCPWM0.int_clr.timer0_tep_int_clr = mcpwm_intr_status_low;//(mcpwm_intr_status >> 5) & 1;
  // high side
  MCPWM0.int_clr.timer0_tez_int_clr = mcpwm_intr_status_high;//(mcpwm_intr_status >> 2) & 1;
  // GPIO.out_w1tc = ((uint32_t)1 << 19);
}

static void IRAM_ATTR mcpwm1_isr_handler(void*) __attribute__ ((unused));

// Read currents when interrupt is triggered
static void IRAM_ATTR mcpwm1_isr_handler(void*){

  // // high side
  uint32_t mcpwm_intr_status_high = MCPWM1.int_st.timer0_tez_int_st;

  // low side
  uint32_t mcpwm_intr_status_low = MCPWM1.int_st.timer0_tep_int_st;

  // uint32_t mcpwm_intr_status = (MCPWM1.int_st.timer0_tep_int_st << 5) | (MCPWM1.int_st.timer0_tez_int_st << 2);
  #ifndef HFI_2XPWM
    bool runadc = mcpwm_intr_status_low;
  #else
    bool runadc = mcpwm_intr_status_high || mcpwm_intr_status_low;
  #endif

  if(mcpwm_intr_status_high || mcpwm_intr_status_low){
    if(runadc){
      #if _I2S_ADC == true
        delayMicroseconds(2);
        readFiFo();
      #else
        adc_buffer[0][adc_read_index[0]] = adcRead(adc_pins[0][adc_read_index[0]]);
        adc_read_index[0]++;
        if(adc_read_index[0] == adc_pin_count[0]) adc_read_index[0] = 0;
      #endif
    }
    #ifdef HFI
      // // enable FPU
      // xthal_set_cpenable(1);
      // // Save FPU registers
      // xthal_save_cp0(cp0_regs);
      
      portENTER_CRITICAL_ISR(&fpuInISRMutex);
      uint32_t cp_state = xthal_get_cpenable();
      if(!cp_state) {
          xthal_set_cpenable(1);
      }

      xthal_save_cp0(cp0_regs);   // Save FPU registers

      process_hfi();

      
      xthal_restore_cp0(cp0_regs);

      if(cp_state) {
          // Restore FPU registers
      } else {
          // turn it back off
          xthal_set_cpenable(0);
      }
      portEXIT_CRITICAL_ISR(&fpuInISRMutex);

      // // Restore FPU
      // xthal_restore_cp0(cp0_regs);
      // // and turn it back off
      // xthal_set_cpenable(0);
    #endif
  }
  // low side
  MCPWM1.int_clr.timer0_tep_int_clr = mcpwm_intr_status_low;//(mcpwm_intr_status >> 5) & 1;
  // high side
  MCPWM1.int_clr.timer0_tez_int_clr = mcpwm_intr_status_high;//(mcpwm_intr_status >> 2) & 1;
}


#endif
