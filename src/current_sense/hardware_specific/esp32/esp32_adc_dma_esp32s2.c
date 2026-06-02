/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * SPI3 DMA backend for ADC digi on ESP32-S2 (spi_ll + spicommon).
 */

#if defined(ARDUINO_ARCH_ESP32)

#include "esp32_adc_digi_internal.h"

#if defined(SIMPLEFOC_ESP32_ADC_USE_SPI3_DMA) && SIMPLEFOC_ESP32_ADC_USE_SPI3_DMA

#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "hal/spi_ll.h"
#include "esp_private/spi_common_internal.h"

#define ADC_DMA_SPI_HOST    SPI3_HOST
#define ADC_DMA_INTR_MASK   SPI_LL_INTR_IN_SUC_EOF

static spi_dev_t *s_spi_dev;
static spi_dma_ctx_t *s_spi_dma;
static esp32_adc_digi_dma_ctx_t *s_ctx;
static intr_handle_t s_intr;
static bool s_inited;

static void IRAM_ATTR esp32_adc_spi_isr(void *arg)
{
    (void)arg;
    if (!spi_ll_get_intr(s_spi_dev, ADC_DMA_INTR_MASK)) {
        return;
    }
    spi_ll_clear_intr(s_spi_dev, ADC_DMA_INTR_MASK);

    if (s_ctx != NULL) {
        s_ctx->eof_desc_addr = spi_dma_ll_get_in_suc_eof_desc_addr(s_spi_dev,
                                                                    s_spi_dma->rx_dma_chan.chan_id);
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

    if (!spicommon_periph_claim(ADC_DMA_SPI_HOST, "simplefoc_adc_digi")) {
        return ESP_FAIL;
    }

    esp_err_t err = spicommon_dma_chan_alloc(ADC_DMA_SPI_HOST, SPI_DMA_CH_AUTO, &s_spi_dma);
    if (err != ESP_OK) {
        spicommon_periph_free(ADC_DMA_SPI_HOST);
        return err;
    }

    s_spi_dev = SPI_LL_GET_HW(ADC_DMA_SPI_HOST);
    err = esp_intr_alloc(spicommon_irqdma_source_for_host(ADC_DMA_SPI_HOST), ESP_INTR_FLAG_IRAM,
                         esp32_adc_spi_isr, NULL, &s_intr);
    if (err != ESP_OK) {
        spicommon_dma_chan_free(s_spi_dma);
        spicommon_periph_free(ADC_DMA_SPI_HOST);
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
    spicommon_dma_chan_free(s_spi_dma);
    spicommon_periph_free(ADC_DMA_SPI_HOST);
    s_spi_dma = NULL;
    s_ctx = NULL;
    s_inited = false;
    return ESP_OK;
}

esp_err_t esp32_adc_digi_dma_start(esp32_adc_digi_dma_ctx_t *ctx, dma_descriptor_t *desc)
{
    (void)ctx;
    spi_ll_clear_intr(s_spi_dev, ADC_DMA_INTR_MASK);
    spi_ll_enable_intr(s_spi_dev, ADC_DMA_INTR_MASK);
    spi_dma_ll_rx_start(s_spi_dev, s_spi_dma->rx_dma_chan.chan_id, (lldesc_t *)desc);
    return ESP_OK;
}

esp_err_t esp32_adc_digi_dma_stop(esp32_adc_digi_dma_ctx_t *ctx)
{
    (void)ctx;
    if (!s_inited) {
        return ESP_OK;
    }
    spi_ll_disable_intr(s_spi_dev, ADC_DMA_INTR_MASK);
    spi_ll_clear_intr(s_spi_dev, ADC_DMA_INTR_MASK);
    spi_dma_ll_rx_stop(s_spi_dev, s_spi_dma->rx_dma_chan.chan_id);
    return ESP_OK;
}

esp_err_t esp32_adc_digi_dma_reset(esp32_adc_digi_dma_ctx_t *ctx)
{
    (void)ctx;
    if (!s_inited) {
        return ESP_OK;
    }
    spi_dma_ll_rx_reset(s_spi_dev, s_spi_dma->rx_dma_chan.chan_id);
    return ESP_OK;
}

#endif /* SIMPLEFOC_ESP32_ADC_USE_SPI3_DMA */
#endif /* ARDUINO_ARCH_ESP32 */
