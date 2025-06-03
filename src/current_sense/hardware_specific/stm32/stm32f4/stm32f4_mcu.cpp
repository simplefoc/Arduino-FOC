#include "../../../hardware_api.h"

#if defined(STM32F4xx)
#include "../../../../common/foc_utils.h"
#include "../../../../drivers/hardware_api.h"
#include "../../../../drivers/hardware_specific/stm32/stm32_mcu.h"
#include "../../../hardware_api.h"
#include "../stm32_mcu.h"
#include "../stm32_adc_utils.h"
#include "stm32f4_hal.h"
#include "Arduino.h"


#define _ADC_VOLTAGE_F4 3.3f
#define _ADC_RESOLUTION_F4 4096.0f

#ifdef SIMPLEFOC_STM32_ADC_INTERRUPT
#define USE_ADC_INTERRUPT 1
#else
#define USE_ADC_INTERRUPT 0
#endif

// array of values of 4 injected channels per adc instance (5)
uint32_t adc_val[5][4]={0};

// structure containing the configuration of the adc interrupt
Stm32AdcInterruptConfig adc_interrupt_config[5] = {
  {0, 0, USE_ADC_INTERRUPT}, // ADC1
  {0, 0, USE_ADC_INTERRUPT}, // ADC2
  {0, 0, USE_ADC_INTERRUPT}, // ADC3
  {0, 0, USE_ADC_INTERRUPT}, // ADC4
  {0, 0, USE_ADC_INTERRUPT}  // ADC5
};

void* _configureADCLowSide(const void* driver_params, const int pinA, const int pinB, const int pinC){

  Stm32CurrentSenseParams* cs_params= new Stm32CurrentSenseParams {
    .pins={(int)NOT_SET,(int)NOT_SET,(int)NOT_SET},
    .adc_voltage_conv = (_ADC_VOLTAGE_F4) / (_ADC_RESOLUTION_F4)
  };
  if(_adc_gpio_init(cs_params, pinA,pinB,pinC) != 0) return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  if(_adc_init(cs_params, (STM32DriverParams*)driver_params) != 0) return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  return cs_params;
}


void* _driverSyncLowSide(void* _driver_params, void* _cs_params){
  STM32DriverParams* driver_params = (STM32DriverParams*)_driver_params;
  Stm32CurrentSenseParams* cs_params = (Stm32CurrentSenseParams*)_cs_params;
 
  // if compatible timer has not been found
  if (cs_params->timer_handle == NULL) return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  
  // stop all the timers for the driver
  stm32_pause(driver_params);

  // get the index of the adc
  int adc_index = _adcToIndex(cs_params->adc_handle);

  bool tim_interrupt = _initTimerInterruptDownsampling(cs_params, driver_params, adc_interrupt_config[adc_index]);
  if(tim_interrupt) {
  // error in the timer interrupt initialization
    SIMPLEFOC_DEBUG("STM32-CS: timer has no repetition counter, ADC interrupt has to be used");
  }

  // set the trigger output event
  LL_TIM_SetTriggerOutput(cs_params->timer_handle->Instance, LL_TIM_TRGO_UPDATE);

  // start the adc
  if (adc_interrupt_config[adc_index].use_adc_interrupt){
    // enable interrupt
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    
    HAL_ADCEx_InjectedStart_IT(cs_params->adc_handle);
  }else{
    HAL_ADCEx_InjectedStart(cs_params->adc_handle);
  }

  // restart all the timers of the driver
  stm32_resume(driver_params);
  
  // return the cs parameters 
  // successfully initialized
  // TODO verify if success in future
  return _cs_params;
}
  

// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pin, const void* cs_params){
  uint8_t adc_index = (uint8_t)_adcToIndex(((Stm32CurrentSenseParams*)cs_params)->adc_handle);
  return _readADCInjectedChannelVoltage(pin, (void*)cs_params, adc_interrupt_config[adc_index], adc_val[adc_index]); 
}

extern "C" {
  void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *AdcHandle){
    uint8_t adc_index = (uint8_t)_adcToIndex(AdcHandle);
    _handleInjectedConvCpltCallback(AdcHandle, adc_interrupt_config[adc_index], adc_val[adc_index]);
  }
}

#endif