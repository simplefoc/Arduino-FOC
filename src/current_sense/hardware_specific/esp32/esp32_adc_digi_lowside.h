/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Glue between SimpleFOC LowsideCurrentSense (ESP32 MCPWM) and esp32_adc_digi_*.
 */
#pragma once

#include "esp32_adc_digi_internal.h"

#if SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

#include "esp32_mcu.h"

ESP32AdcLowsidePath esp32_adc_lowside_configure(ESP32CurrentSenseParams *params);
bool esp32_adc_lowside_uses_mcpwm_isr(const ESP32CurrentSenseParams *params);
void *esp32_adc_lowside_sync_mcpwm(void *driver_params, ESP32CurrentSenseParams *cs);
void esp32_adc_lowside_start_conversion(void);

#endif
