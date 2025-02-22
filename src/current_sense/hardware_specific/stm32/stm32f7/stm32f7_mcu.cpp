#include "../../../hardware_api.h"

#if defined(STM32F7xx)
#include "../../../../common/foc_utils.h"
#include "../../../../drivers/hardware_api.h"
#include "../../../../drivers/hardware_specific/stm32/stm32_mcu.h"
#include "../../../hardware_api.h"
#include "../stm32_mcu.h"
#include "stm32f7_hal.h"
#include "stm32f7_utils.h"
#include "Arduino.h"


#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4096.0f


// array of values of 4 injected channels per adc instance (3)
uint32_t adc_val[3][4]={0};
// does adc interrupt need a downsample - per adc (3)
bool needs_downsample[3] = {1};
// downsampling variable - per adc (3)
uint8_t tim_downsample[3] = {1};

void* _configureADCLowSide(const void* driver_params, const int pinA, const int pinB, const int pinC){

  Stm32CurrentSenseParams* cs_params= new Stm32CurrentSenseParams {
    .pins={(int)NOT_SET,(int)NOT_SET,(int)NOT_SET},
    .adc_voltage_conv = (_ADC_VOLTAGE) / (_ADC_RESOLUTION)
  };
  _adc_gpio_init(cs_params, pinA,pinB,pinC);
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

  // if timer has repetition counter - it will downsample using it
  // and it does not need the software downsample
  if( IS_TIM_REPETITION_COUNTER_INSTANCE(cs_params->timer_handle->Instance) ){
    // adjust the initial timer state such that the trigger 
    //   - for DMA transfer aligns with the pwm peaks instead of throughs.
    //   - for interrupt based ADC transfer 
    //   - only necessary for the timers that have repetition counters

    cs_params->timer_handle->Instance->CR1 |= TIM_CR1_DIR;
    cs_params->timer_handle->Instance->CNT = cs_params->timer_handle->Instance->ARR;
    // remember that this timer has repetition counter - no need to downasmple
    needs_downsample[_adcToIndex(cs_params->adc_handle)] = 0;
  }
  // set the trigger output event
  LL_TIM_SetTriggerOutput(cs_params->timer_handle->Instance, LL_TIM_TRGO_UPDATE);

  // start the adc
  #ifdef SIMPLEFOC_STM32_ADC_INTERRUPT 
  HAL_ADCEx_InjectedStart_IT(cs_params->adc_handle);
  #else
  HAL_ADCEx_InjectedStart(cs_params->adc_handle);
  #endif

  // restart all the timers of the driver
  stm32_resume(driver_params);
  
  // return the cs parameters 
  // successfully initialized
  // TODO verify if success in future
  return _cs_params;
}
  

// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pin, const void* cs_params){
  for(int i=0; i < 3; i++){
    if( pin == ((Stm32CurrentSenseParams*)cs_params)->pins[i]){ // found in the buffer
      #ifdef SIMPLEFOC_STM32_ADC_INTERRUPT
        return adc_val[_adcToIndex(((Stm32CurrentSenseParams*)cs_params)->adc_handle)][i] * ((Stm32CurrentSenseParams*)cs_params)->adc_voltage_conv;
      #else
        // an optimized way to go from i to the channel i=0 -> channel 1, i=1 -> channel 2, i=2 -> channel 3
        uint32_t channel = (i == 0) ? ADC_INJECTED_RANK_1 : (i == 1) ? ADC_INJECTED_RANK_2 : ADC_INJECTED_RANK_3;
        return HAL_ADCEx_InjectedGetValue(((Stm32CurrentSenseParams*)cs_params)->adc_handle, channel) * ((Stm32CurrentSenseParams*)cs_params)->adc_voltage_conv;
      #endif
    }
  } 
  return 0;
}

#ifdef SIMPLEFOC_STM32_ADC_INTERRUPT
extern "C" {
  void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *AdcHandle){

    // calculate the instance
    int adc_index = _adcToIndex(AdcHandle);

    // if the timer han't repetition counter - downsample two times
    if( needs_downsample[adc_index] && tim_downsample[adc_index]++ > 0) {
      tim_downsample[adc_index] = 0;
      return;
    }
    
    adc_val[adc_index][0]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_1);
    adc_val[adc_index][1]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_2);
    adc_val[adc_index][2]=HAL_ADCEx_InjectedGetValue(AdcHandle, ADC_INJECTED_RANK_3);    
  }
}
#endif

#endif