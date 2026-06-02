/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "hal/dma_types.h"

/*
 * Low-side current sense on ESP32 uses the SAR **digital** controller (pattern
 * sequencer) plus a chip-specific DMA shim — same approach as espFoC isensor_adc.
 *
 * Triggering (when to start one pattern conversion):
 *   - ESP32 / ESP32-S2: **software** only. MCPWM ISR (comparator or timer event)
 *     must call esp32_adc_digi_trigger_software() at the desired PWM phase.
 *   - ESP32-S3 and newer (C3, C6, …): optional **ETM** wires MCPWM timer TEZ/TEP
 *     to ADC_TASK_START0 with no CPU in the trigger path.
 *
 * DMA backend (where SOC moves ADC results):
 *   - ESP32:     I2S peripheral (see esp32_adc_dma_esp32.c)
 *   - ESP32-S2:  SPI3 (esp32_adc_dma_esp32s2.c)
 *   - ESP32-S3+: GDMA  (esp32_adc_dma_gdma.c)
 */

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)

#define SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED 1

/* ETM is not available on original ESP32 or ESP32-S2. */
#if SOC_ETM_SUPPORTED && !CONFIG_IDF_TARGET_ESP32 && !CONFIG_IDF_TARGET_ESP32S2
#define SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED 1
#else
#define SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED 0
#endif

#else

#define SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED 0
#define SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED 0

#endif

/* Pattern / DMA sizing — kept aligned with espFoC isensor_adc_internal.h */
#define SIMPLEFOC_ESP32_ADC_PATTERN_HZ      80000
#define SIMPLEFOC_ESP32_ADC_NUM_CHANNELS    2
#define SIMPLEFOC_ESP32_ADC_CONVERT_LIMIT   2

typedef struct esp32_adc_digi_dma_ctx esp32_adc_digi_dma_ctx_t;

typedef void (*esp32_adc_digi_dma_done_fn_t)(void *user);

struct esp32_adc_digi_dma_ctx {
    esp32_adc_digi_dma_done_fn_t on_done;
    void *on_done_arg;
    volatile intptr_t eof_desc_addr;
};

esp_err_t esp32_adc_digi_dma_init(esp32_adc_digi_dma_ctx_t *ctx,
                                  esp32_adc_digi_dma_done_fn_t on_done,
                                  void *user);
esp_err_t esp32_adc_digi_dma_deinit(esp32_adc_digi_dma_ctx_t *ctx);
esp_err_t esp32_adc_digi_dma_start(esp32_adc_digi_dma_ctx_t *ctx, dma_descriptor_t *desc);
esp_err_t esp32_adc_digi_dma_stop(esp32_adc_digi_dma_ctx_t *ctx);
esp_err_t esp32_adc_digi_dma_reset(esp32_adc_digi_dma_ctx_t *ctx);

#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED

typedef enum {
    SIMPLEFOC_ESP32_ADC_MCPWM_EVT_TIMER_TEZ = 0,
    SIMPLEFOC_ESP32_ADC_MCPWM_EVT_TIMER_TEP,
} esp32_adc_digi_mcpwm_event_t;

typedef struct {
    uint8_t mcpwm_timer;
    esp32_adc_digi_mcpwm_event_t event;
} esp32_adc_digi_etm_config_t;

esp_err_t esp32_adc_digi_etm_connect(const esp32_adc_digi_etm_config_t *cfg);
esp_err_t esp32_adc_digi_etm_enable(bool enable);
void esp32_adc_digi_etm_disconnect(void);

#endif /* SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED */
