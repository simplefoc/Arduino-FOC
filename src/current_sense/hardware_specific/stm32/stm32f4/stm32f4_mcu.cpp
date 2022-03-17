#include "../../../hardware_api.h"

#if defined(STM32F4xx)
#include "../../../../common/foc_utils.h"
#include "../../../../drivers/hardware_api.h"
#include "../../../../drivers/hardware_specific/stm32_mcu.h"
#include "../../../hardware_api.h"
#include "../stm32_mcu.h"
#include "stm32f4_hal.h"
#include "Arduino.h"


#define _ADC_VOLTAGE_F4 3.3f
#define _ADC_RESOLUTION_F4 4096.0f

uint32_t adc_val[3][4]={0};
uint16_t adc_channel_number[3]={0};

int _adcToIndex(ADC_HandleTypeDef *AdcHandle){
  if(AdcHandle->Instance == ADC1) return 0;
#ifdef ADC2 // if ADC2 exists
  else if(AdcHandle->Instance == ADC2) return 1;
#endif
#ifdef ADC3 // if ADC3 exists
  else if(AdcHandle->Instance == ADC3) return 2;
#endif
  return 0;
}

void* _configureADCLowSide(const void* driver_params, const int pinA, const int pinB, const int pinC){

  Stm32CurrentSenseParams* cs_params= new Stm32CurrentSenseParams {
    .pins={0},
    .adc_voltage_conv = (_ADC_VOLTAGE_F4) / (_ADC_RESOLUTION_F4)
  };
  
  _adc_gpio_init(cs_params, pinA,pinB,pinC);
  _adc_init(cs_params, (STM32DriverParams*)driver_params);
  return cs_params;
}


void _driverSyncLowSide(void* driver_params, void* cs_params){
  _UNUSED(driver_params);
  Stm32CurrentSenseParams* params = (Stm32CurrentSenseParams*)cs_params;

  // if compatible timer has been found
  if (params->timer_handle)   
    LL_TIM_SetTriggerOutput(params->timer_handle->getHandle()->Instance, LL_TIM_TRGO_UPDATE);
  
  HAL_ADCEx_InjectedStart_IT(params->adc_handle);
}
  

// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pin, const void* cs_params){
  for(int i=0; i < 3; i++){
    if( pin == ((Stm32CurrentSenseParams*)cs_params)->pins[i]) // found in the buffer
      return adc_val[_adcToIndex(((Stm32CurrentSenseParams*)cs_params)->adc_handle)][i] * ((Stm32CurrentSenseParams*)cs_params)->adc_voltage_conv;
  } 
  return 0;
}



extern "C" {
  void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *AdcHandle){
    adc_val[_adcToIndex(AdcHandle)][0]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_1);
    adc_val[_adcToIndex(AdcHandle)][1]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_2);
    adc_val[_adcToIndex(AdcHandle)][2]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_3);
  }
}

#endif