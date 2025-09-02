#include <pinDefinitions.h>
#include <pins_arduino.h>

#if defined(ARDUINO_ARCH_SILABS)

#include <em_device.h>
#include <em_prs.h>
#include "efr32_mcu.h"
#include "../../../drivers/hardware_specific/silabs/efr32_mcu.h"

#ifndef _ADC_VOLTAGE
#define _ADC_VOLTAGE 3.3f
#endif

#ifndef _ADC_RESOLUTION
#define _ADC_RESOLUTION 4095.0f
#endif

#ifndef _CLK_SRC_ADC_FREQ
#define _CLK_SRC_ADC_FREQ 20000000
#endif

#ifndef _CLK_ADC_FREQ
#define _CLK_ADC_FREQ 10000000
#endif

extern void _getPrsSourceAndUnderflowSignal(
  TIMER_TypeDef *timer,
  uint32_t *source,
  uint32_t *signal);

static void _adcBusAllocate(
  uint8_t port,
  uint8_t pin
) {
  switch (port) {
#if (GPIO_PA_COUNT > 0)
    case gpioPortA:
      if (0 == pin % 2) {
        if ((GPIO->ABUSALLOC & _GPIO_ABUSALLOC_AEVEN0_MASK) == GPIO_ABUSALLOC_AEVEN0_TRISTATE) {
          GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN0_ADC0;
        } else if ((GPIO->ABUSALLOC & _GPIO_ABUSALLOC_AEVEN1_MASK) == GPIO_ABUSALLOC_AEVEN1_TRISTATE) {
          GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN1_ADC0;
        } else {}
      } else {
        if ((GPIO->ABUSALLOC & _GPIO_ABUSALLOC_AODD0_MASK) == GPIO_ABUSALLOC_AODD0_TRISTATE) {
          GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0;
        } else if ((GPIO->ABUSALLOC & _GPIO_ABUSALLOC_AODD1_MASK) == GPIO_ABUSALLOC_AODD1_TRISTATE) {
          GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD1_ADC0;
        } else {
          // MISRA
        }
      }
      break;
#endif

#if (GPIO_PB_COUNT > 0)
    case gpioPortB:
      if (0 == pin % 2) {
        if ((GPIO->BBUSALLOC & _GPIO_BBUSALLOC_BEVEN0_MASK) == GPIO_BBUSALLOC_BEVEN0_TRISTATE) {
          GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN0_ADC0;
        } else if ((GPIO->BBUSALLOC & _GPIO_BBUSALLOC_BEVEN1_MASK) == GPIO_BBUSALLOC_BEVEN1_TRISTATE) {
          GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN1_ADC0;
        } else {
          // MISRA
        }
      } else {
        if ((GPIO->BBUSALLOC & _GPIO_BBUSALLOC_BODD0_MASK) == GPIO_BBUSALLOC_BODD0_TRISTATE) {
          GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD0_ADC0;
        } else if ((GPIO->BBUSALLOC & _GPIO_BBUSALLOC_BODD1_MASK) == GPIO_BBUSALLOC_BODD1_TRISTATE) {
          GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD1_ADC0;
        } else {
          // MISRA
        }
      }
      break;
#endif

#if (GPIO_PC_COUNT > 0 || GPIO_PD_COUNT > 0)
    case gpioPortC:
    case gpioPortD:
      if (0 == pin % 2) {
        if ((GPIO->CDBUSALLOC & _GPIO_CDBUSALLOC_CDEVEN0_MASK) == GPIO_CDBUSALLOC_CDEVEN0_TRISTATE) {
          GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN0_ADC0;
        } else if ((GPIO->CDBUSALLOC & _GPIO_CDBUSALLOC_CDEVEN1_MASK) == GPIO_CDBUSALLOC_CDEVEN1_TRISTATE) {
          GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN1_ADC0;
        } else {
          // MISRA
        }
      } else {
        if ((GPIO->CDBUSALLOC & _GPIO_CDBUSALLOC_CDODD0_MASK) == GPIO_CDBUSALLOC_CDODD0_TRISTATE) {
          GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD0_ADC0;
        } else if ((GPIO->CDBUSALLOC & _GPIO_CDBUSALLOC_CDODD1_MASK) == GPIO_CDBUSALLOC_CDODD1_TRISTATE) {
          GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD1_ADC0;
        } else {
          // MISRA
        }
      }
      break;
#endif
  }
}

static void _adcConfig(
  EFR32AdcInstance *inst,
  const int pin
) {
  if (!inst) return;
  inst->port = getSilabsPortFromArduinoPin(pinToPinName(pin));
  inst->pin = getSilabsPinFromArduinoPin(pinToPinName(pin));
}

static float _readAdc(
  EFR32CurrentSenseParams *params,
  const int pin
) {
  if (!params) return 0.0f;

  for (uint8_t i = 0; i < SILABS_MAX_ANALOG; ++i) {
    if (!_isset(params->pins[i])) continue;
    if (pin == params->pins[i]) {
      return params->buffer[i] * params->adc_voltage_conv;
    }
  }
  return 0.0f;
}

static bool _dmaTransferFinishedCb(
  unsigned int channel,
  unsigned int sequenceNo,
  void *data
) {
  _UNUSED(sequenceNo);

  EFR32CurrentSenseParams *params = (EFR32CurrentSenseParams *) data;
  if (!params || !params->adc || (params->dmaChannel != channel))
    return false;

  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_ATOMIC();
  params->dataReady = true;
  CORE_EXIT_ATOMIC();

  return false;
}

static void _currentSenseInitDMA(
  EFR32CurrentSenseParams *params,
  DMADRV_Callback_t fn,
  void *data
) {
  if (!params) return;

  // Initialize DMA with default parameters
  DMADRV_Init();

  DMADRV_AllocateChannel(&params->dmaChannel, NULL);

  // Trigger LDMA transfer on IADC scan completion
  LDMA_TransferCfg_t transferCfg =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SCAN);

  params->descriptor =
    (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(params->adc->SCANFIFODATA),
                                                        params->buffer,
                                                        params->noAdcChannels,
                                                        0);

  DMADRV_LdmaStartTransfer(params->dmaChannel,
                           &transferCfg,
                           &params->descriptor,
                           fn,
                           params);
}

static void _currentSenseInitScan(
  EFR32CurrentSenseParams *params
) {
  if (!params || !params->adc) return;

  // Enable Clock
  CMU_ClockEnable(cmuClock_IADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Use the FSRC0 as the IADC clock so it can run in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

  for (uint8_t i = 0; i < params->noAdcChannels; ++i) {
    GPIO_PinModeSet(params->inst[i].port, params->inst[i].pin, gpioModeDisabled, 0);
  }

  IADC_Init_t init = IADC_INIT_DEFAULT;
  init.warmup = iadcWarmupKeepWarm;
  init.srcClkPrescale = IADC_calcSrcClkPrescale(params->adc, _CLK_SRC_ADC_FREQ, 0);

  IADC_CfgReference_t adcRef;
  switch (params->vRef) {
    case 1200: adcRef = iadcCfgReferenceInt1V2; break;
    case 1250: adcRef = iadcCfgReferenceExt1V25; break;
    case 3300: adcRef = iadcCfgReferenceVddx; break;
    case 2640: adcRef = iadcCfgReferenceVddX0P8Buf; break;
    default: return;
  }

  IADC_AllConfigs_t allConfigs = IADC_ALLCONFIGS_DEFAULT;
  allConfigs.configs[0].reference = adcRef;
  allConfigs.configs[0].vRef = params->vRef;
  allConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(params->adc,
                                                                 _CLK_ADC_FREQ,
                                                                 0,
                                                                 iadcCfgModeNormal,
                                                                 init.srcClkPrescale);

  // Reset the ADC
  IADC_reset(params->adc);

  // Only configure the ADC if it is not already running
  if (params->adc->CTRL == _IADC_CTRL_RESETVALUE) {
    IADC_init(params->adc, &init, &allConfigs);
  }

  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  if ((params->mode == CS_LO_SIDE) || (params->mode == CS_HI_SIDE)) {
    // Note: CS_HI_SIDE not implemented
    initScan.triggerSelect = iadcTriggerSelPrs0PosEdge;
  }
  initScan.fifoDmaWakeup = true;

  IADC_ScanTable_t scanTable = IADC_SCANTABLE_DEFAULT;
  for (uint8_t i = 0; i < params->noAdcChannels; ++i) {
    scanTable.entries[i].posInput = IADC_portPinToPosInput(params->inst[i].port, params->inst[i].pin);
    scanTable.entries[i].negInput = iadcNegInputGnd;
    scanTable.entries[i].includeInScan = true;
  }

  // Initialize IADC
  IADC_init(params->adc, &init, &allConfigs);

  // Initialize Scan
  IADC_initScan(params->adc, &initScan, &scanTable);

  // Allocate
  for (uint8_t i = 0; i < params->noAdcChannels; ++i) {
    _adcBusAllocate(params->inst[i].port, params->inst[i].pin);
  }
}

static void _currentSenseConfig(
  EFR32CurrentSenseParams *params,
  int adcPins[SILABS_MAX_ANALOG]
) {
  if (!params) return;

  uint8_t noAdcChannels = 0;
  for (int i = 0; i < SILABS_MAX_ANALOG; ++i) {
    if (!_isset(adcPins[i])) continue;
    if (params->firstIndex == 0xFF) params->firstIndex = i;
    _adcConfig(&params->inst[noAdcChannels], adcPins[i]);
    params->pins[noAdcChannels] = adcPins[i];
    ++noAdcChannels;
  }

  params->noAdcChannels = noAdcChannels;
}

static void _currentSenseDeinit(
  EFR32CurrentSenseParams *params
) {
  if (!params) return;

  DMADRV_StopTransfer(params->dmaChannel);
  DMADRV_FreeChannel(params->dmaChannel);

  IADC_reset(params->adc);
}

static void _currentSenseStartScan(
  EFR32CurrentSenseParams *params
) {
  if (!params || !params->adc) return;

  IADC_command(params->adc, iadcCmdStartScan);
}

static void _currentSenseStopScan(
  EFR32CurrentSenseParams *params
) {
  if (!params) return;

  IADC_command(params->adc, iadcCmdStopScan);
}

static void _currentSenseStopTranfer(
  EFR32CurrentSenseParams *params
) {
  if (!params || !params->adc) return;

  DMADRV_PauseTransfer(params->dmaChannel);
}

static void _currentSenseStartTranfer(
  EFR32CurrentSenseParams *params
) {
  if (!params || !params->adc) return;

  DMADRV_ResumeTransfer(params->dmaChannel);
}

////////////////////////////////////////////////////////////////////////////////
// Low Side Mode
////////////////////////////////////////////////////////////////////////////////

float _readADCVoltageLowSide(
  const int pin,
  const void *cs_params
) {
  EFR32CurrentSenseParams *params = (EFR32CurrentSenseParams *) cs_params;
  if (!params) return 0.0f;

  return _readAdc(params, pin);
}

void* _configureADCLowSide(
  const void* driver_params,
  const int pinA,
  const int pinB,
  const int pinC
) {
  _UNUSED(driver_params);

  EFR32CurrentSenseParams *params = new EFR32CurrentSenseParams {
    .adc_voltage_conv = (_ADC_VOLTAGE) / (_ADC_RESOLUTION),
    .firstIndex = 0xFF,
    .noAdcChannels = 0,
    .vRef = SILABS_ADC_VREF,
    .prsChannel = SILABS_ADC_PRS_CHANNEL,
    .mode = CS_LO_SIDE,
    .adc = SILABS_DEFAULT_ADC_PERPHERAL,
  };

  if (!params) return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;

  int adcPins[3] = { pinA, pinB, pinC };

  _currentSenseConfig(params, adcPins);
  _currentSenseInitScan(params);
  _currentSenseInitDMA(params, NULL, params);
  _currentSenseStartScan(params);

  return params;
}

void* _driverSyncLowSide(
  void *driver_params,
  void *cs_params
) {
  EFR32DriverParams *driver = (EFR32DriverParams *) driver_params;
  EFR32CurrentSenseParams *params = (EFR32CurrentSenseParams *) cs_params;

  if (!driver || !params) return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;

  uint32_t prsSource, prsSignal;

  CMU_ClockEnable(cmuClock_PRS, true);

  _getPrsSourceAndUnderflowSignal(driver->inst[0].h.timer, &prsSource, &prsSignal);
  PRS_SourceAsyncSignalSet(params->prsChannel, prsSource, prsSignal);
  PRS_ConnectConsumer(params->prsChannel, prsTypeAsync, prsConsumerIADC0_SCANTRIGGER);

  return cs_params;
}

////////////////////////////////////////////////////////////////////////////////
// Inline Mode
////////////////////////////////////////////////////////////////////////////////

float _readADCVoltageInline(
  const int pin,
  const void *cs_params
) {
  EFR32CurrentSenseParams *params = (EFR32CurrentSenseParams *) cs_params;
  if (!params || !_isset(pin) || (params->firstIndex == 0xFF)) return 0.0f;

  if (params->pins[params->firstIndex] == pin) {
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_ATOMIC();
    params->dataReady = false;
    CORE_EXIT_ATOMIC();

    _currentSenseStartScan(params);

    while (!params->dataReady) {}
  }

  return _readAdc(params, pin);
}

void* _configureADCInline(
  const void* driver_params,
  const int pinA,
  const int pinB,
  const int pinC
) {
  _UNUSED(driver_params);

  EFR32CurrentSenseParams *params = new EFR32CurrentSenseParams {
    .adc_voltage_conv = (_ADC_VOLTAGE) / (_ADC_RESOLUTION),
    .firstIndex = 0xFF,
    .noAdcChannels = 0,
    .vRef = SILABS_ADC_VREF,
    .prsChannel = SILABS_ADC_PRS_CHANNEL,
    .mode = CS_INLINE,
    .adc = SILABS_DEFAULT_ADC_PERPHERAL,
  };

  if (!params) return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;

  int adcPins[3] = { pinA, pinB, pinC };

  _currentSenseConfig(params, adcPins);
  _currentSenseInitScan(params);
  _currentSenseInitDMA(params, _dmaTransferFinishedCb, params);

  return params;
}

#endif