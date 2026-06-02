/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Digital ADC + DMA driver for SimpleFOC ESP32 low-side current sense.
 * See esp32_adc_digi_internal.h for chip-specific trigger/DMA behaviour.
 */
#pragma once

#include "sdkconfig.h"
#include "esp_err.h"
#include "hal/adc_types.h"
#include "esp32_adc_digi_internal.h"

#if SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    adc_channel_t channels[SIMPLEFOC_ESP32_ADC_NUM_CHANNELS];
    adc_unit_t unit;
    int *adc_buffer;
    int no_adc_channels;
} esp32_adc_digi_config_t;

typedef enum {
    SIMPLEFOC_ESP32_ADC_TRIG_SOFTWARE = 0,
#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
    SIMPLEFOC_ESP32_ADC_TRIG_ETM,
#endif
} esp32_adc_digi_trigger_t;

esp_err_t esp32_adc_digi_init(const esp32_adc_digi_config_t *cfg);
esp_err_t esp32_adc_digi_deinit(void);
esp_err_t esp32_adc_digi_set_trigger(esp32_adc_digi_trigger_t mode);
esp_err_t esp32_adc_digi_trigger_software(void);

int esp32_adc_digi_read_raw(const void *adc_buffer, int index);

bool esp32_adc_digi_supported(void);

#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
esp_err_t esp32_adc_digi_set_etm_source(const esp32_adc_digi_etm_config_t *cfg);
bool esp32_adc_digi_etm_supported(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED */
