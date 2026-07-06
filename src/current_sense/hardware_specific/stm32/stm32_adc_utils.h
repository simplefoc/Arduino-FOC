#ifndef STM32_ADC_UTILS_HAL
#define STM32_ADC_UTILS_HAL

#include "Arduino.h"

#if defined(_STM32_DEF_) 

#define _TRGO_NOT_AVAILABLE 12345


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

constexpr uint32_t _getADCChannel(uint8_t ind)
{
  switch (ind)
  {
#ifdef ADC_CHANNEL_0
    case 0:
      return ADC_CHANNEL_0;
#endif
#ifdef ADC_CHANNEL_1
    case 1:
      return ADC_CHANNEL_1;
#endif
#ifdef ADC_CHANNEL_2
    case 2:
      return ADC_CHANNEL_2;
#endif
#ifdef ADC_CHANNEL_3
    case 3:
      return ADC_CHANNEL_3;
#endif
#ifdef ADC_CHANNEL_4
    case 4:
      return ADC_CHANNEL_4;
#endif
#ifdef ADC_CHANNEL_5
    case 5:
      return ADC_CHANNEL_5;
#endif
#ifdef ADC_CHANNEL_6
    case 6:
      return ADC_CHANNEL_6;
#endif
#ifdef ADC_CHANNEL_7
    case 7:
      return ADC_CHANNEL_7;
#endif
#ifdef ADC_CHANNEL_8
    case 8:
      return ADC_CHANNEL_8;
#endif
#ifdef ADC_CHANNEL_9
    case 9:
      return ADC_CHANNEL_9;
#endif
#ifdef ADC_CHANNEL_10
    case 10:
      return ADC_CHANNEL_10;
#endif
#ifdef ADC_CHANNEL_11
    case 11:
      return ADC_CHANNEL_11;
#endif
#ifdef ADC_CHANNEL_12
    case 12:
      return ADC_CHANNEL_12;
#endif
#ifdef ADC_CHANNEL_13
    case 13:
      return ADC_CHANNEL_13;
#endif
#ifdef ADC_CHANNEL_14
    case 14:
      return ADC_CHANNEL_14;
#endif
#ifdef ADC_CHANNEL_15
    case 15:
      return ADC_CHANNEL_15;
#endif
#ifdef ADC_CHANNEL_16
    case 16:
      return ADC_CHANNEL_16;
#endif
#ifdef ADC_CHANNEL_17
    case 17:
      return ADC_CHANNEL_17;
#endif
#ifdef ADC_CHANNEL_18
    case 18:
      return ADC_CHANNEL_18;
#endif
#ifdef ADC_CHANNEL_19
    case 19:
      return ADC_CHANNEL_19;
#endif
#ifdef ADC_CHANNEL_20
    case 20:
      return ADC_CHANNEL_20;
#endif
#ifdef ADC_CHANNEL_21
    case 21:
      return ADC_CHANNEL_21;
#endif
#ifdef ADC_CHANNEL_22
    case 22:
      return ADC_CHANNEL_22;
#endif
#ifdef ADC_CHANNEL_23
    case 23:
      return ADC_CHANNEL_23;
#endif
#ifdef ADC_CHANNEL_24
    case 24:
      return ADC_CHANNEL_24;
#endif
#ifdef ADC_CHANNEL_25
    case 25:
      return ADC_CHANNEL_25;
#endif
#ifdef ADC_CHANNEL_26
    case 26:
      return ADC_CHANNEL_26;
#endif
#ifdef ADC_CHANNEL_27
    case 27:
      return ADC_CHANNEL_27;
#endif
#ifdef ADC_CHANNEL_28
    case 28:
      return ADC_CHANNEL_28;
#endif
#ifdef ADC_CHANNEL_29
    case 29:
      return ADC_CHANNEL_29;
#endif
#ifdef ADC_CHANNEL_30
    case 30:
      return ADC_CHANNEL_30;
#endif
#ifdef ADC_CHANNEL_31
    case 31:
      return ADC_CHANNEL_31;
#endif
    default:
      return 0;
  }
}

constexpr uint32_t _OPAMP_internal_channel_to_ADC(uint8_t opamp_idx, ADC_TypeDef *ADC)
{
  #if defined(STM32G4xx)
  switch (opamp_idx)
  {
  case 1:
    return ADC == ADC1 ? ADC_CHANNEL_13 : 0; //adc1
  case 2:
    return ADC == ADC2 ? ADC_CHANNEL_16 : 0; //adc2
  #ifdef OPAMP3
  case 3:
    switch ((uintptr_t)ADC)
    {
    case ADC2_BASE:
      return ADC_CHANNEL_18;
    #ifdef ADC3
    case ADC3_BASE:
      return ADC_CHANNEL_13;
    #endif
    default:
      return 0;
    }
  #endif
  #ifdef OPAMP4
    return ADC == ADC5 ? ADC_CHANNEL_5 : 0; //adc5
  #endif
  #ifdef OPAMP5
    return ADC == ADC5 ? ADC_CHANNEL_3 : 0; //adc5
  #endif
  #ifdef OPAMP6
    switch ((uintptr_t)ADC)
    {
    #ifdef ADC3
    case ADC3_BASE:
      return ADC_CHANNEL_17; //For stm32g3x4
    #endif
    #ifdef ADC4
    case ADC4_BASE:
      return ADC_CHANNEL_17; //For stm32g4x3
    #endif
    default:
      return 0;
    }
  #endif
  default:
    return 0;
  }
  #endif
  return 0;
}

constexpr uint32_t _getADCInjectedRank(uint8_t ind)
{
  switch (ind) {
#ifdef ADC_INJECTED_RANK_1
    case 0:
      return ADC_INJECTED_RANK_1;
#endif
#ifdef ADC_INJECTED_RANK_2
    case 1:
      return ADC_INJECTED_RANK_2;
#endif
#ifdef ADC_INJECTED_RANK_3
    case 2:
      return ADC_INJECTED_RANK_3;
#endif
#ifdef ADC_INJECTED_RANK_4
    case 3:
      return ADC_INJECTED_RANK_4;
#endif
    default:
      return 0;
  }
}

constexpr uint32_t _getADCRegularRank(uint8_t ind)
{
  switch (ind) {
#ifdef ADC_REGULAR_RANK_1
    case 0:
      return ADC_REGULAR_RANK_1;
#endif
#ifdef ADC_REGULAR_RANK_2
    case 1:
      return ADC_REGULAR_RANK_2;
#endif
#ifdef ADC_REGULAR_RANK_3
    case 2:
      return ADC_REGULAR_RANK_3;
#endif
#ifdef ADC_REGULAR_RANK_4
    case 3:
      return ADC_REGULAR_RANK_4;
#endif
#ifdef ADC_REGULAR_RANK_5
    case 4:
      return ADC_REGULAR_RANK_5;
#endif
#ifdef ADC_REGULAR_RANK_6
    case 5:
      return ADC_REGULAR_RANK_6;
#endif
#ifdef ADC_REGULAR_RANK_7
    case 6:
      return ADC_REGULAR_RANK_7;
#endif
#ifdef ADC_REGULAR_RANK_8
    case 7:
      return ADC_REGULAR_RANK_8;
#endif
#ifdef ADC_REGULAR_RANK_9
    case 8:
      return ADC_REGULAR_RANK_9;
#endif
#ifdef ADC_REGULAR_RANK_10
    case 9:
      return ADC_REGULAR_RANK_10;
#endif
#ifdef ADC_REGULAR_RANK_11
    case 10:
      return ADC_REGULAR_RANK_11;
#endif
#ifdef ADC_REGULAR_RANK_12
    case 11:
      return ADC_REGULAR_RANK_12;
#endif
#ifdef ADC_REGULAR_RANK_13
    case 12:
      return ADC_REGULAR_RANK_13;
#endif
#ifdef ADC_REGULAR_RANK_14
    case 13:
      return ADC_REGULAR_RANK_14;
#endif
#ifdef ADC_REGULAR_RANK_15
    case 14:
      return ADC_REGULAR_RANK_15;
#endif
#ifdef ADC_REGULAR_RANK_16
    case 15:
      return ADC_REGULAR_RANK_16;
#endif
    default:
      return 0;
  }
}

// timer to injected TRGO - architecure specific
uint32_t _timerToInjectedTRGO(TIM_HandleTypeDef* timer);

// timer to regular TRGO - architecure specific
uint32_t _timerToRegularTRGO(TIM_HandleTypeDef* timer);

constexpr int _adcToIndex(ADC_TypeDef *AdcHandle)
{
  if(AdcHandle == ADC1) return 0;
#ifdef ADC2 // if ADC2 exists
  else if(AdcHandle == ADC2) return 1;
#endif
#ifdef ADC3 // if ADC3 exists
  else if(AdcHandle == ADC3) return 2;
#endif
#ifdef ADC4 // if ADC4 exists
  else if(AdcHandle == ADC4) return 3;
#endif
#ifdef ADC5 // if ADC5 exists
  else if(AdcHandle == ADC5) return 4;
#endif
  return 0;
}

// function returning index of the ADC instance
constexpr int _adcToIndex(ADC_HandleTypeDef *AdcHandle)
{
  return _adcToIndex(AdcHandle->Instance);
}

constexpr ADC_TypeDef* _indexToADC(uint8_t index)
{
  switch (index) {
    case 0:
      return ADC1;
      break;
#ifdef ADC2 // if ADC2 exists
    case 1:
      return ADC2;
      break;
#endif
#ifdef ADC3 // if ADC3 exists
    case 2:
      return ADC3;
      break;
#endif
#ifdef ADC4 // if ADC4 exists
    case 3:
      return ADC4;
      break;
#endif
#ifdef ADC5 // if ADC5 exists
    case 4:
      return ADC5;
      break;
#endif
  }
  return nullptr;
}

// functions helping to find the best ADC channel
int _findIndexOfFirstPinMapADCEntry(int pin);
int _findIndexOfLastPinMapADCEntry(int pin);
ADC_TypeDef* _findBestADCForInjectedPins(int num_pins, int pins[], ADC_HandleTypeDef adc_handles[]);
ADC_TypeDef* _findBestADCForRegularPin(int pin, ADC_HandleTypeDef adc_handles[]);

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
