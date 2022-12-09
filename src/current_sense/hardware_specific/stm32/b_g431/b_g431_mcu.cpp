#include "../../../hardware_api.h"

#if defined(ARDUINO_B_G431B_ESC1) 

#include "b_g431_hal.h"
#include "Arduino.h"
#include "../stm32_mcu.h"
#include "../../../../drivers/hardware_specific/stm32/stm32_mcu.h"
#include "communication/SimpleFOCDebug.h"

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4096.0f
#define ADC_BUF_LEN_1 5
#define ADC_BUF_LEN_2 1

static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static OPAMP_HandleTypeDef hopamp1;
static OPAMP_HandleTypeDef hopamp2;
static OPAMP_HandleTypeDef hopamp3;

static DMA_HandleTypeDef hdma_adc1;
static DMA_HandleTypeDef hdma_adc2;

volatile uint16_t adcBuffer1[ADC_BUF_LEN_1] = {0}; // Buffer for store the results of the ADC conversion
volatile uint16_t adcBuffer2[ADC_BUF_LEN_2] = {0}; // Buffer for store the results of the ADC conversion

// function reading an ADC value and returning the read voltage
// As DMA is being used just return the DMA result
float _readADCVoltageInline(const int pin, const void* cs_params){
  uint32_t raw_adc = 0;
  if(pin == PA2)  // = ADC1_IN3 = phase U (OP1_OUT) on B-G431B-ESC1
    raw_adc = adcBuffer1[1];
  else if(pin == PA6) // = ADC2_IN3 = phase V (OP2_OUT) on B-G431B-ESC1
    raw_adc = adcBuffer2[0];
#ifdef PB1
  else if(pin == PB1) // = ADC1_IN12 = phase W (OP3_OUT) on B-G431B-ESC1
    raw_adc = adcBuffer1[0];
#endif

  else if (pin == A_POTENTIOMETER)
    raw_adc = adcBuffer1[2];
  else if (pin == A_TEMPERATURE)
    raw_adc = adcBuffer1[3];
  else if (pin == A_VBUS)
    raw_adc = adcBuffer1[4];

  return raw_adc * ((Stm32CurrentSenseParams*)cs_params)->adc_voltage_conv;
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
    SIMPLEFOC_DEBUG("HAL_OPAMP_Init failed!");
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
    SIMPLEFOC_DEBUG("HAL_DMA_Init failed!");
  }
  __HAL_LINKDMA(hadc, DMA_Handle, *hdma_adc);
}

void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);
  _UNUSED(pinA);
  _UNUSED(pinB);
  _UNUSED(pinC);

  SIMPLEFOC_DEBUG("B-G431B does not implement inline current sense. Use low-side current sense instead.");
  return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
}


void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

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
    SIMPLEFOC_DEBUG("DMA read init failed");
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcBuffer2, ADC_BUF_LEN_2) != HAL_OK)
  {
    SIMPLEFOC_DEBUG("DMA read init failed");
  }

  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3); 
  
  Stm32CurrentSenseParams* params = new Stm32CurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE) / (_ADC_RESOLUTION),
    .timer_handle = (HardwareTimer *)(HardwareTimer_Handle[get_timer_index(TIM1)]->__this)
  };

  return params;
}

extern "C" {
void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel2_IRQHandler(void) {
   HAL_DMA_IRQHandler(&hdma_adc2);
}
}

void _driverSyncLowSide(void* _driver_params, void* _cs_params){
  STM32DriverParams* driver_params = (STM32DriverParams*)_driver_params;
  Stm32CurrentSenseParams* cs_params = (Stm32CurrentSenseParams*)_cs_params;
   
  // stop all the timers for the driver
  _stopTimers(driver_params->timers, 6);

  // if timer has repetition counter - it will downsample using it
  // and it does not need the software downsample
  if( IS_TIM_REPETITION_COUNTER_INSTANCE(cs_params->timer_handle->getHandle()->Instance) ){
    // adjust the initial timer state such that the trigger for DMA transfer aligns with the pwm peaks instead of throughs.
    // only necessary for the timers that have repetition counters
    cs_params->timer_handle->getHandle()->Instance->CR1 |= TIM_CR1_DIR;
    cs_params->timer_handle->getHandle()->Instance->CNT =  cs_params->timer_handle->getHandle()->Instance->ARR;
  }
  // set the trigger output event
  LL_TIM_SetTriggerOutput(cs_params->timer_handle->getHandle()->Instance, LL_TIM_TRGO_UPDATE);

  // restart all the timers of the driver
  _startTimers(driver_params->timers, 6);

}

#endif