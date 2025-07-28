#ifndef STM32_ADC_UTILS_HAL
#define STM32_ADC_UTILS_HAL

#include "Arduino.h"

#if defined(_STM32_DEF_) 

#define _TRGO_NOT_AVAILABLE 12345

// for searching the best ADCs, we need to know the number of ADCs 
// it might be better to use some HAL variable for example ADC_COUNT
// here I've assumed the maximum number of ADCs is 5
#define ADC_COUNT 5


#include "../../../common/foc_utils.h"
#include "../../../communication/SimpleFOCDebug.h"
#include "../../../drivers/hardware_specific/stm32/stm32_mcu.h"
#include "stm32_mcu.h"



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


// Structure to hold ADC interrupt configuration per ADC instance
struct Stm32AdcInterruptConfig {
  bool needs_downsample = 0;
  uint8_t tim_downsample = 0;
  bool use_adc_interrupt = 0;
};

// returns 0 if no interrupt is needed, 1 if interrupt is needed
uint32_t _initTimerInterruptDownsampling(Stm32CurrentSenseParams* cs_params, STM32DriverParams* driver_params, Stm32AdcInterruptConfig& adc_interrupt_config);
// returns 0 if no downsampling is needed, 1 if downsampling is needed, 2 if error
uint8_t _handleInjectedConvCpltCallback(ADC_HandleTypeDef *AdcHandle, Stm32AdcInterruptConfig& adc_interrupt_config, uint32_t adc_val[4]);
// reads the ADC injected voltage for the given pin
// returns the voltage 
// if the pin is not found in the current sense parameters, returns 0
float _readADCInjectedChannelVoltage(int pin, void* cs_params, Stm32AdcInterruptConfig& adc_interrupt_config, uint32_t adc_val[4]);
#endif
#endif
