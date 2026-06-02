/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * I2S DMA backend for ADC digi on ESP32 (i2s_ll only).
 */

#if defined(ARDUINO_ARCH_ESP32)

#include "esp32_adc_digi_internal.h"

#if defined(SIMPLEFOC_ESP32_ADC_USE_I2S_DMA) && SIMPLEFOC_ESP32_ADC_USE_I2S_DMA

#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "hal/i2s_ll.h"
#include "hal/adc_hal.h"
#include "soc/i2s_periph.h"
#include "esp_private/i2s_platform.h"

#define ADC_DMA_I2S_HOST    ADC_HAL_DMA_I2S_HOST
#define ADC_DMA_INTR_MASK   BIT(9)

static i2s_dev_t *s_i2s_dev;
static esp32_adc_digi_dma_ctx_t *s_ctx;
static intr_handle_t s_intr;
static bool s_inited;

static void IRAM_ATTR esp32_adc_i2s_isr(void *arg)
{
    (void)arg;
    if ((i2s_ll_get_intr_status(s_i2s_dev) & ADC_DMA_INTR_MASK) == 0) {
        return;
    }
    i2s_ll_clear_intr_status(s_i2s_dev, ADC_DMA_INTR_MASK);

    if (s_ctx != NULL) {
        uint32_t desc_addr = 0;
        i2s_ll_rx_get_eof_des_addr(s_i2s_dev, &desc_addr);
        s_ctx->eof_desc_addr = (intptr_t)desc_addr;
        if (s_ctx->on_done != NULL) {
            s_ctx->on_done(s_ctx->on_done_arg);
        }
    }
}

esp_err_t esp32_adc_digi_dma_init(esp32_adc_digi_dma_ctx_t *ctx,
                                  esp32_adc_digi_dma_done_fn_t on_done,
                                  void *user)
{
    if (ctx == NULL || on_done == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_inited) {
        ctx->on_done = on_done;
        ctx->on_done_arg = user;
        s_ctx = ctx;
        return ESP_OK;
    }

    esp_err_t err = i2s_platform_acquire_occupation(I2S_CTLR_HP, ADC_DMA_I2S_HOST, "simplefoc_adc_digi");
    if (err != ESP_OK) {
        return err;
    }
    s_i2s_dev = I2S_LL_GET_HW(ADC_DMA_I2S_HOST);

    err = esp_intr_alloc(i2s_periph_signal[ADC_DMA_I2S_HOST].irq, ESP_INTR_FLAG_IRAM,
                         esp32_adc_i2s_isr, NULL, &s_intr);
    if (err != ESP_OK) {
        i2s_platform_release_occupation(I2S_CTLR_HP, ADC_DMA_I2S_HOST);
        return err;
    }

    ctx->on_done = on_done;
    ctx->on_done_arg = user;
    s_ctx = ctx;
    s_inited = true;
    return ESP_OK;
}

esp_err_t esp32_adc_digi_dma_deinit(esp32_adc_digi_dma_ctx_t *ctx)
{
    (void)ctx;
    if (!s_inited) {
        return ESP_OK;
    }
    esp32_adc_digi_dma_stop(ctx);
    if (s_intr != NULL) {
        esp_intr_free(s_intr);
        s_intr = NULL;
    }
    i2s_platform_release_occupation(I2S_CTLR_HP, ADC_DMA_I2S_HOST);
    s_ctx = NULL;
    s_inited = false;
    return ESP_OK;
}

esp_err_t esp32_adc_digi_dma_start(esp32_adc_digi_dma_ctx_t *ctx, dma_descriptor_t *desc)
{
    (void)ctx;
    i2s_ll_clear_intr_status(s_i2s_dev, ADC_DMA_INTR_MASK);
    i2s_ll_enable_intr(s_i2s_dev, ADC_DMA_INTR_MASK, true);
    i2s_ll_enable_dma(s_i2s_dev, true);
    i2s_ll_rx_start_link(s_i2s_dev, (uint32_t)desc);
    return ESP_OK;
}

esp_err_t esp32_adc_digi_dma_stop(esp32_adc_digi_dma_ctx_t *ctx)
{
    (void)ctx;
    if (!s_inited) {
        return ESP_OK;
    }
    i2s_ll_enable_intr(s_i2s_dev, ADC_DMA_INTR_MASK, false);
    i2s_ll_clear_intr_status(s_i2s_dev, ADC_DMA_INTR_MASK);
    i2s_ll_rx_stop_link(s_i2s_dev);
    return ESP_OK;
}

esp_err_t esp32_adc_digi_dma_reset(esp32_adc_digi_dma_ctx_t *ctx)
{
    (void)ctx;
    if (!s_inited) {
        return ESP_OK;
    }
    i2s_ll_rx_reset_dma(s_i2s_dev);
    return ESP_OK;
}

#endif /* SIMPLEFOC_ESP32_ADC_USE_I2S_DMA */
#endif /* ARDUINO_ARCH_ESP32 */
