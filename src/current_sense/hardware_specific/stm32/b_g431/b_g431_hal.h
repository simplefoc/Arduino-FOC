#ifndef B_G431_ESC1_HAL
#define B_G431_ESC1_HAL

#if defined(ARDUINO_B_G431B_ESC1)

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