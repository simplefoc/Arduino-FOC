#ifndef EFR32_CURRENTSENSE_MCU_H
#define EFR32_CURRENTSENSE_MCU_H

#include "../../hardware_api.h"

#if defined(ARDUINO_ARCH_SILABS)
#include <em_iadc.h>
#include <em_timer.h>
#include <FreeRTOS.h>
#include <semphr.h>

#ifndef SILABS_DEFAULT_ADC_PERPHERAL
#define SILABS_DEFAULT_ADC_PERPHERAL  IADC0
#endif

#ifndef SILABS_ADC_VREF
#define SILABS_ADC_VREF 3300
#endif

#ifndef SILABS_ADC_PRS_CHANNEL
#define SILABS_ADC_PRS_CHANNEL 1
#endif

#ifndef SILABS_MAX_ANALOG
#define SILABS_MAX_ANALOG 3
#endif

typedef enum {
  CS_INLINE,
  CS_LO_SIDE,
  CS_HI_SIDE,
} EFR32CurrentSenseMode;

typedef struct {
  uint8_t port;
  uint8_t pin;
} EFR32AdcInstance;

typedef struct {
  int pins[SILABS_MAX_ANALOG];
  float adc_voltage_conv;
  EFR32AdcInstance inst[SILABS_MAX_ANALOG];
  uint8_t firstIndex;
  uint8_t noAdcChannels;
  volatile bool dataReady;
  uint32_t id;
  uint32_t buffer[SILABS_MAX_ANALOG];
  uint32_t vRef;
  unsigned int dmaChannel;
  unsigned int prsChannel;
  EFR32CurrentSenseMode mode;
  IADC_TypeDef *adc;
  LDMA_Descriptor_t descriptor;
} EFR32CurrentSenseParams;

#endif

#endif