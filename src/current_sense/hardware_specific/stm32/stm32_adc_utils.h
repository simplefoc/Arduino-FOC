#ifndef STM32_ADC_UTILS_HAL
#define STM32_ADC_UTILS_HAL

#include "Arduino.h"

#if defined(_STM32_DEF_) 

#define _TRGO_NOT_AVAILABLE 12345

#include "../../../common/foc_utils.h"
#include "../../../communication/SimpleFOCDebug.h"

/* Exported Functions */
/**
  * @brief  Return ADC HAL channel linked to a PinName
  * @param  pin: PinName
  * @param  adc: ADC_TypeDef a pointer to the ADC handle
  * @retval Valid HAL channel
  */
uint32_t _getADCChannel(PinName pin, ADC_TypeDef* adc = NP);
uint32_t _getADCInjectedRank(uint8_t ind);

// timer to injected TRGO - architecure specific
uint32_t _timerToInjectedTRGO(TIM_HandleTypeDef* timer);

// timer to regular TRGO - architecure specific
uint32_t _timerToRegularTRGO(TIM_HandleTypeDef* timer);

// function returning index of the ADC instance
int _adcToIndex(ADC_HandleTypeDef *AdcHandle);
int _adcToIndex(ADC_TypeDef *AdcHandle);

// functions helping to find the best ADC channel
int _findIndexOfFirstPinMapADCEntry(int pin);
int _findIndexOfLastPinMapADCEntry(int pin);
ADC_TypeDef* _findBestADCForPins(int num_pins, int pins[]);
#endif
#endif
