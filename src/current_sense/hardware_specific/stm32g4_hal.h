#ifndef stm32g4_hal
#define stm32g4_hal

#if defined(STM32G4xx)

#include <stm32g4xx_hal_adc.h>
#include <stm32g4xx_hal_opamp.h>

void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(ADC_HandleTypeDef* hadc1);
void MX_ADC2_Init(ADC_HandleTypeDef* hadc2);
void MX_OPAMP1_Init(OPAMP_HandleTypeDef* hopamp);
void MX_OPAMP2_Init(OPAMP_HandleTypeDef* hopamp);
void MX_OPAMP3_Init(OPAMP_HandleTypeDef* hopamp);
#endif

#endif