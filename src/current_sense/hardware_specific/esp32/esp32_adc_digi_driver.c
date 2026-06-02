/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * ADC digi + DMA for SimpleFOC low-side current sense.
 *
 * Triggering is selected with esp32_adc_digi_set_trigger():
 *   - SOFTWARE: always available (ESP32, S2, S3, …).
 *   - ETM:        ESP32-S3 and newer only; see esp32_adc_etm.c.
 */

#include "sdkconfig.h"
#include "esp32_adc_digi_internal.h"

#if SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_clk_tree.h"
#include "esp_private/regi2c_ctrl.h"
#include "esp_private/sar_periph_ctrl.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_private/gpio.h"
#include "soc/adc_periph.h"
#include "soc/soc_caps.h"
#include "soc/clk_tree_defs.h"
#include "hal/adc_hal.h"
#include "hal/adc_ll.h"
#include "esp32_adc_digi_driver.h"

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#include "esp_cache.h"
#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define ESP32_ADC_DMA_DESC_ALIGN  4
#define ESP32_ADC_FRAME_BYTES     (SIMPLEFOC_ESP32_ADC_NUM_CHANNELS * SOC_ADC_DIGI_RESULT_BYTES)

typedef enum {
    ESP32_ADC_STATE_IDLE = 0,
    ESP32_ADC_STATE_BUSY,
} esp32_adc_digi_state_t;

static const char *TAG = "SF_ESP32_ADC_DIGI";

typedef struct {
    adc_hal_dma_ctx_t hal;
    adc_hal_digi_ctrlr_cfg_t hal_cfg;
    adc_digi_pattern_config_t patterns[SIMPLEFOC_ESP32_ADC_NUM_CHANNELS];
    uint8_t *rx_buf;
    uint32_t rx_desc_size;
    esp32_adc_digi_dma_ctx_t dma_ctx;
    adc_channel_t channels[SIMPLEFOC_ESP32_ADC_NUM_CHANNELS];
    adc_unit_t unit;
    adc_atten_t atten;
    int *adc_buffer;
    int no_adc_channels;
    esp32_adc_digi_trigger_t trigger;
    esp32_adc_digi_state_t state;
    bool started;
#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
    esp32_adc_digi_etm_config_t etm_cfg;
#endif
} esp32_adc_digi_ctx_t;

DRAM_ATTR static esp32_adc_digi_ctx_t s_adc;
static bool s_adc_initialized;

static adc_bitwidth_t esp32_adc_digi_bitwidth(void)
{
#if CONFIG_IDF_TARGET_ESP32
    return ADC_BITWIDTH_12;
#else
    return SOC_ADC_DIGI_MAX_BITWIDTH;
#endif
}

static adc_atten_t esp32_adc_default_atten(void)
{
#if CONFIG_IDF_TARGET_ESP32
    return ADC_ATTEN_DB_11;
#else
    return ADC_ATTEN_DB_12;
#endif
}

static void esp32_adc_digi_apply_convert_limit(void)
{
    adc_ll_digi_set_convert_limit_num(SIMPLEFOC_ESP32_ADC_CONVERT_LIMIT);
    adc_ll_digi_convert_limit_enable(true);
}

static void esp32_adc_apply_max_clock(esp32_adc_digi_ctx_t *adc)
{
#if CONFIG_IDF_TARGET_ESP32
    adc_ll_set_sample_cycle(ADC_LL_SAMPLE_CYCLE_DEFAULT);
    adc_ll_digi_set_clk_div(2);
#elif CONFIG_IDF_TARGET_ESP32S2
    ESP_ERROR_CHECK(esp_clk_tree_enable_src(SOC_MOD_CLK_APLL, true));
    uint32_t clk_hz = 0;
    ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_APLL, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_hz));
    adc->hal_cfg.clk_src = ADC_DIGI_CLK_SRC_APLL;
    adc->hal_cfg.clk_src_freq_hz = clk_hz;
    adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_APLL);
    adc_ll_digi_controller_clk_div(0, 1, 0);
    adc_ll_digi_set_clk_div(1);
    adc_ll_set_sample_cycle(ADC_LL_SAMPLE_CYCLE_DEFAULT);
#elif CONFIG_IDF_TARGET_ESP32S3
    ESP_ERROR_CHECK(esp_clk_tree_enable_src(SOC_MOD_CLK_PLL_D2, true));
    uint32_t clk_hz = 0;
    ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_PLL_D2, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_hz));
    adc->hal_cfg.clk_src = ADC_DIGI_CLK_SRC_PLL_F240M;
    adc->hal_cfg.clk_src_freq_hz = clk_hz;
    adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_PLL_F240M);
    adc_ll_digi_controller_clk_div(0, 1, 0);
    adc_ll_digi_set_clk_div(1);
    adc_ll_set_sample_cycle(ADC_LL_SAMPLE_CYCLE_DEFAULT);
#else
    const soc_module_clk_t fast_src = SOC_MOD_CLK_PLL_F80M;
    ESP_ERROR_CHECK(esp_clk_tree_enable_src(fast_src, true));
    uint32_t clk_hz = 0;
    ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz(fast_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_hz));
    adc->hal_cfg.clk_src = ADC_DIGI_CLK_SRC_PLL_F80M;
    adc->hal_cfg.clk_src_freq_hz = clk_hz;
    adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_PLL_F80M);
    adc_ll_digi_controller_clk_div(0, 1, 0);
    adc_ll_digi_set_clk_div(1);
    adc_ll_set_sample_cycle(ADC_LL_SAMPLE_CYCLE_DEFAULT);
#endif
    ESP_LOGI(TAG, "ADC fast clock: src=%d freq=%" PRIu32 " Hz",
             (int)adc->hal_cfg.clk_src, adc->hal_cfg.clk_src_freq_hz);
}

static void esp32_adc_rearm(esp32_adc_digi_ctx_t *adc)
{
    esp32_adc_digi_dma_reset(&adc->dma_ctx);
    adc_hal_digi_reset();
    adc_hal_digi_dma_link(&adc->hal, adc->rx_buf);
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    esp_cache_msync(adc->hal.rx_desc, adc->rx_desc_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
#endif
    esp32_adc_digi_dma_start(&adc->dma_ctx, adc->hal.rx_desc);
    adc_hal_digi_connect(true);
    if (adc->trigger == SIMPLEFOC_ESP32_ADC_TRIG_SOFTWARE) {
        adc_hal_digi_enable(true);
    } else {
        adc_hal_digi_enable(false);
    }
    adc->state = ESP32_ADC_STATE_BUSY;
}

static esp_err_t esp32_adc_apply_trigger_mode(esp32_adc_digi_ctx_t *adc)
{
#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
    if (adc->trigger == SIMPLEFOC_ESP32_ADC_TRIG_ETM) {
        esp_err_t err = esp32_adc_digi_etm_connect(&adc->etm_cfg);
        if (err != ESP_OK) {
            return err;
        }
        err = esp32_adc_digi_etm_enable(true);
        if (err != ESP_OK) {
            return err;
        }
        adc_hal_digi_enable(false);
        return ESP_OK;
    }
    esp32_adc_digi_etm_enable(false);
    esp32_adc_digi_etm_disconnect();
#else
    if (adc->trigger != SIMPLEFOC_ESP32_ADC_TRIG_SOFTWARE) {
        return ESP_ERR_NOT_SUPPORTED;
    }
#endif
    return ESP_OK;
}

static esp_err_t esp32_adc_gpio_init(adc_unit_t unit, uint32_t chan_mask)
{
    while (chan_mask) {
        int ch = __builtin_ctz(chan_mask);
        int8_t io = adc_channel_io_map[unit][ch];
        if (io < 0) {
            return ESP_ERR_INVALID_ARG;
        }
        gpio_config_as_analog(io);
        chan_mask &= ~(1U << ch);
    }
    return ESP_OK;
}

static void IRAM_ATTR esp32_adc_process_frame(esp32_adc_digi_ctx_t *adc, const uint8_t *frame, uint32_t size)
{
    if (frame == NULL || size < ESP32_ADC_FRAME_BYTES || adc->adc_buffer == NULL) {
        return;
    }

    adc_digi_output_data_t *p = (adc_digi_output_data_t *)frame;
    const int n = adc->no_adc_channels < SIMPLEFOC_ESP32_ADC_NUM_CHANNELS
                      ? adc->no_adc_channels
                      : SIMPLEFOC_ESP32_ADC_NUM_CHANNELS;
    for (int i = 0; i < n; i++) {
        adc->adc_buffer[i] = (int)ADC_GET_DATA(p);
        p++;
    }
}

static void IRAM_ATTR esp32_adc_dma_done(void *arg)
{
    esp32_adc_digi_ctx_t *adc = (esp32_adc_digi_ctx_t *)arg;
    adc_hal_dma_desc_status_t status;
    uint8_t *finished_buffer = NULL;
    uint32_t finished_size = 0;

    while (1) {
        status = adc_hal_get_reading_result(&adc->hal, adc->dma_ctx.eof_desc_addr,
                                            &finished_buffer, &finished_size);
        if (status != ADC_HAL_DMA_DESC_VALID) {
            break;
        }
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
        esp_cache_msync(finished_buffer, finished_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
#endif
        esp32_adc_process_frame(adc, finished_buffer, finished_size);
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    esp_cache_msync(adc->hal.rx_desc, adc->rx_desc_size,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
#endif

#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
    if (adc->trigger == SIMPLEFOC_ESP32_ADC_TRIG_ETM) {
        esp32_adc_rearm(adc);
    } else
#endif
    {
        adc_hal_digi_enable(false);
        adc_hal_digi_connect(false);
        adc->state = ESP32_ADC_STATE_IDLE;
    }
}

static esp_err_t esp32_adc_hw_start(esp32_adc_digi_ctx_t *adc)
{
    ANALOG_CLOCK_ENABLE();

    ADC_BUS_CLK_ATOMIC() {
        adc_ll_reset_register();
    }

    sar_periph_ctrl_adc_continuous_power_acquire();
    adc_lock_acquire(adc->unit);

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    adc_hal_calibration_init(adc->unit);
    adc_set_hw_calibration_code(adc->unit, adc->atten);
#endif

    adc_hal_set_controller(adc->unit, ADC_HAL_CONTINUOUS_READ_MODE);

#if !CONFIG_IDF_TARGET_ESP32
    ESP_ERROR_CHECK(esp_clk_tree_enable_src((soc_module_clk_t)adc->hal_cfg.clk_src, true));
#endif

    adc_hal_digi_init(&adc->hal);
    adc_hal_digi_controller_config(&adc->hal, &adc->hal_cfg);
    esp32_adc_apply_max_clock(adc);
    esp32_adc_digi_apply_convert_limit();

    esp32_adc_digi_dma_stop(&adc->dma_ctx);
    adc->started = true;
    esp32_adc_rearm(adc);
    return ESP_OK;
}

static esp_err_t esp32_adc_setup_hal(esp32_adc_digi_ctx_t *adc)
{
    uint32_t clk_hz = 0;
    esp_clk_tree_src_get_freq_hz(ADC_DIGI_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_hz);

    adc->hal_cfg.adc_pattern = adc->patterns;
    adc->hal_cfg.adc_pattern_len = SIMPLEFOC_ESP32_ADC_NUM_CHANNELS;
    adc->hal_cfg.sample_freq_hz = SIMPLEFOC_ESP32_ADC_PATTERN_HZ;
    adc->hal_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adc->hal_cfg.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
    adc->hal_cfg.clk_src_freq_hz = clk_hz;

    uint32_t chan_mask = 0;
    for (int i = 0; i < SIMPLEFOC_ESP32_ADC_NUM_CHANNELS; i++) {
        adc->patterns[i].atten = adc->atten;
        adc->patterns[i].channel = adc->channels[i] & 0x7;
        adc->patterns[i].unit = adc->unit;
        adc->patterns[i].bit_width = esp32_adc_digi_bitwidth();
        chan_mask |= BIT(adc->patterns[i].channel);
        ESP_LOGI(TAG, "pattern[%d] unit=%d ch=%d atten=%d", i,
                 (int)adc->patterns[i].unit, (int)adc->patterns[i].channel,
                 (int)adc->patterns[i].atten);
    }

    adc_hal_dma_config_t dma_cfg = {
        .eof_desc_num = 1,
        .eof_step = 1,
        .eof_num = SIMPLEFOC_ESP32_ADC_NUM_CHANNELS,
    };
    adc_hal_dma_ctx_config(&adc->hal, &dma_cfg);

    adc->rx_buf = heap_caps_calloc(1, ESP32_ADC_FRAME_BYTES,
                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (adc->rx_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    adc->hal.rx_desc = heap_caps_aligned_calloc(ESP32_ADC_DMA_DESC_ALIGN, 1,
                                                sizeof(dma_descriptor_t),
                                                MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (adc->hal.rx_desc == NULL) {
        return ESP_ERR_NO_MEM;
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    uint32_t line = 4;
    adc->rx_desc_size = (sizeof(dma_descriptor_t) + line - 1) & ~(line - 1);
#else
    adc->rx_desc_size = sizeof(dma_descriptor_t);
#endif

    ESP_RETURN_ON_ERROR(esp32_adc_gpio_init(adc->unit, chan_mask), TAG, "gpio init failed");
    ESP_RETURN_ON_ERROR(esp32_adc_digi_dma_init(&adc->dma_ctx, esp32_adc_dma_done, adc),
                        TAG, "dma init failed");
    return ESP_OK;
}

bool esp32_adc_digi_supported(void)
{
    return true;
}

#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
bool esp32_adc_digi_etm_supported(void)
{
    return true;
}
#endif

esp_err_t esp32_adc_digi_init(const esp32_adc_digi_config_t *cfg)
{
    if (cfg == NULL || cfg->adc_buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "only ADC1 supported");
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (cfg->no_adc_channels < 1 || cfg->no_adc_channels > SIMPLEFOC_ESP32_ADC_NUM_CHANNELS) {
        ESP_LOGE(TAG, "need 1..%d ADC channels for hw trigger", SIMPLEFOC_ESP32_ADC_NUM_CHANNELS);
        return ESP_ERR_INVALID_ARG;
    }
    if (s_adc_initialized) {
        return ESP_OK;
    }

    memset(&s_adc, 0, sizeof(s_adc));
    s_adc.unit = cfg->unit;
    s_adc.atten = esp32_adc_default_atten();
    s_adc.channels[0] = cfg->channels[0];
    s_adc.channels[1] = cfg->channels[1];
    s_adc.adc_buffer = cfg->adc_buffer;
    s_adc.no_adc_channels = cfg->no_adc_channels;
    s_adc.trigger = SIMPLEFOC_ESP32_ADC_TRIG_SOFTWARE;
    s_adc.state = ESP32_ADC_STATE_IDLE;
#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
    s_adc.etm_cfg.mcpwm_timer = 0;
    s_adc.etm_cfg.event = SIMPLEFOC_ESP32_ADC_MCPWM_EVT_TIMER_TEZ;
#endif

    adc_apb_periph_claim();

    esp_err_t err = esp32_adc_setup_hal(&s_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC HAL setup failed: %d", err);
        return err;
    }

    err = esp32_adc_hw_start(&s_adc);
    if (err != ESP_OK) {
        return err;
    }

    s_adc_initialized = true;
    return ESP_OK;
}

esp_err_t esp32_adc_digi_deinit(void)
{
    if (!s_adc_initialized) {
        return ESP_OK;
    }
    esp32_adc_digi_set_trigger(SIMPLEFOC_ESP32_ADC_TRIG_SOFTWARE);
    esp32_adc_digi_dma_stop(&s_adc.dma_ctx);
    esp32_adc_digi_dma_deinit(&s_adc.dma_ctx);
    adc_hal_digi_deinit(&s_adc.hal);
    adc_lock_release(s_adc.unit);
    sar_periph_ctrl_adc_continuous_power_release();
    if (s_adc.rx_buf) {
        heap_caps_free(s_adc.rx_buf);
        s_adc.rx_buf = NULL;
    }
    if (s_adc.hal.rx_desc) {
        heap_caps_free(s_adc.hal.rx_desc);
        s_adc.hal.rx_desc = NULL;
    }
    s_adc_initialized = false;
    return ESP_OK;
}

esp_err_t esp32_adc_digi_set_trigger(esp32_adc_digi_trigger_t mode)
{
    if (!s_adc_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
#if !SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
    if (mode != SIMPLEFOC_ESP32_ADC_TRIG_SOFTWARE) {
        return ESP_ERR_NOT_SUPPORTED;
    }
#endif
    s_adc.trigger = mode;
    esp_err_t err = esp32_adc_apply_trigger_mode(&s_adc);
    if (err != ESP_OK) {
        s_adc.trigger = SIMPLEFOC_ESP32_ADC_TRIG_SOFTWARE;
        return err;
    }
    if (s_adc.started && s_adc.state != ESP32_ADC_STATE_BUSY) {
        esp32_adc_rearm(&s_adc);
    }
    return ESP_OK;
}

#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
esp_err_t esp32_adc_digi_set_etm_source(const esp32_adc_digi_etm_config_t *cfg)
{
    if (!s_adc_initialized || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->mcpwm_timer >= SOC_MCPWM_TIMERS_PER_GROUP) {
        return ESP_ERR_INVALID_ARG;
    }
    s_adc.etm_cfg = *cfg;
    if (s_adc.trigger == SIMPLEFOC_ESP32_ADC_TRIG_ETM) {
        return esp32_adc_apply_trigger_mode(&s_adc);
    }
    return ESP_OK;
}
#endif /* SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED */

esp_err_t esp32_adc_digi_trigger_software(void)
{
    if (!s_adc_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_adc.trigger != SIMPLEFOC_ESP32_ADC_TRIG_SOFTWARE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_adc.state == ESP32_ADC_STATE_BUSY) {
        return ESP_OK;
    }
    if (!s_adc.started) {
        return esp32_adc_hw_start(&s_adc);
    }
    esp32_adc_rearm(&s_adc);
    return ESP_OK;
}

#endif
