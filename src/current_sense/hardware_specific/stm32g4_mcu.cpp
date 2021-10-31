#include "../hardware_api.h"
#include "stm32g4_hal.h"

#if defined(STM32G4xx) 
#define _ADC_VOLTAGE 3.3
#define _ADC_RESOLUTION 4096.0
#define ADC_BUF_LEN_1 2
#define ADC_BUF_LEN_2 1

static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static OPAMP_HandleTypeDef hopamp1;
static OPAMP_HandleTypeDef hopamp2;
static OPAMP_HandleTypeDef hopamp3;

static DMA_HandleTypeDef hdma_adc1;
static DMA_HandleTypeDef hdma_adc2;

uint16_t adcBuffer1[ADC_BUF_LEN_1] = {0}; // Buffer for store the results of the ADC conversion
uint16_t adcBuffer2[ADC_BUF_LEN_2] = {0}; // Buffer for store the results of the ADC conversion

#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

// function reading an ADC value and returning the read voltage
// As DMA is being used just return the DMA result
float _readADCVoltageInline(const int pin){
  uint32_t raw_adc = 0;
  if(pin == PA2)  // = ADC1_IN3 = phase U (OP1_OUT) on B-G431B-ESC1
    raw_adc = adcBuffer1[1];
  else if(pin == PA6) // = ADC2_IN3 = phase V (OP2_OUT) on B-G431B-ESC1
    raw_adc = adcBuffer2[0];
  else if(pin == PB1) // = ADC1_IN12 = phase W (OP3_OUT) on B-G431B-ESC1
    raw_adc = adcBuffer1[0];

  return raw_adc * _ADC_CONV;
}
// do the same for low side sensing
float _readADCVoltageLowSide(const int pin){
  return _readADCVoltageInline(pin);
}

void _configureOPAMP(OPAMP_HandleTypeDef *hopamp, OPAMP_TypeDef *OPAMPx_Def){
  // could this be replaced with LL_OPAMP calls??
  hopamp->Instance = OPAMPx_Def;
  hopamp->Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp->Init.Mode = OPAMP_PGA_MODE;
  hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp->Init.InternalOutput = DISABLE;
  hopamp->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp->Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(hopamp) != HAL_OK)
  {
    Error_Handler();
  }
}
void _configureOPAMPs(OPAMP_HandleTypeDef *OPAMPA, OPAMP_HandleTypeDef *OPAMPB, OPAMP_HandleTypeDef *OPAMPC){
  // Configure the opamps
  _configureOPAMP(OPAMPA, OPAMP1);
  _configureOPAMP(OPAMPB, OPAMP2);
  _configureOPAMP(OPAMPC, OPAMP3);
}

void MX_DMA1_Init(ADC_HandleTypeDef *hadc, DMA_HandleTypeDef *hdma_adc, DMA_Channel_TypeDef* channel, uint32_t request) {
  hdma_adc->Instance = channel;
  hdma_adc->Init.Request = request;
  hdma_adc->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc->Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc->Init.Mode = DMA_CIRCULAR;
  hdma_adc->Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_DeInit(hdma_adc);
  if (HAL_DMA_Init(hdma_adc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(hadc, DMA_Handle, *hdma_adc);
}

void _configureADCInline(const int pinA,const int pinB,const int pinC){
  HAL_Init();
  MX_GPIO_Init();
  MX_DMA_Init(); 
  _configureOPAMPs(&hopamp1, &hopamp3, &hopamp2);
  MX_ADC1_Init(&hadc1);
  MX_ADC2_Init(&hadc2);

  MX_DMA1_Init(&hadc1, &hdma_adc1, DMA1_Channel1, DMA_REQUEST_ADC1);
  MX_DMA1_Init(&hadc2, &hdma_adc2, DMA1_Channel2, DMA_REQUEST_ADC2);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer1, ADC_BUF_LEN_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcBuffer2, ADC_BUF_LEN_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3); 

  // Check if the ADC DMA is collecting any data.
  // If this fails, it likely means timer1 has not started. Verify that your application starts
  // the motor pwm (usually BLDCDriver6PWM::init()) before initializing the ADC engine.
  _delay(5);
  if (adcBuffer1[0] == 0 || adcBuffer1[1] == 0 || adcBuffer2[0] == 0) {
    Error_Handler();
  }
}
// do the same for low side
void _configureADCLowSide(const int pinA,const int pinB,const int pinC){
  _configureADCInline(pinA, pinB, pinC);
}

extern "C" {
void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel2_IRQHandler(void) {
   HAL_DMA_IRQHandler(&hdma_adc2);
}
}

#endif