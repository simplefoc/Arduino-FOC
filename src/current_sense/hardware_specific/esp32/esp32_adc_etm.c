/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * ETM wiring: MCPWM timer event -> ADC digi START task.
 */

#include "sdkconfig.h"
#include "esp32_adc_digi_internal.h"

#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED

#include <stdlib.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_etm.h"
#include "esp_heap_caps.h"
#include "esp_private/etm_interface.h"
#include "soc/soc_caps.h"
#include "soc/soc_etm_source.h"
#include "esp32_adc_digi_internal.h"

static const char *TAG = "esp32_adc_etm";

typedef struct {
    esp_etm_event_t base;
} esp32_adc_etm_event_t;

typedef struct {
    esp_etm_task_t base;
} esp32_adc_etm_task_t;

typedef struct {
    esp_etm_channel_handle_t chan;
    esp_etm_event_handle_t event;
    esp_etm_task_handle_t task;
    bool connected;
    bool enabled;
} esp32_adc_etm_ctx_t;

static esp32_adc_etm_ctx_t s_etm;

static esp_err_t esp32_adc_etm_del_event(esp_etm_event_t *event)
{
    esp32_adc_etm_event_t *ev = __containerof(event, esp32_adc_etm_event_t, base);
    free(ev);
    return ESP_OK;
}

static esp_err_t esp32_adc_etm_del_task(esp_etm_task_t *task)
{
    esp32_adc_etm_task_t *tk = __containerof(task, esp32_adc_etm_task_t, base);
    free(tk);
    return ESP_OK;
}

static uint32_t esp32_adc_mcpwm_event_id(const esp32_adc_digi_etm_config_t *cfg)
{
    switch (cfg->event) {
    case SIMPLEFOC_ESP32_ADC_MCPWM_EVT_TIMER_TEZ:
        return (uint32_t)(MCPWM_EVT_TIMER0_TEZ + cfg->mcpwm_timer);
    case SIMPLEFOC_ESP32_ADC_MCPWM_EVT_TIMER_TEP:
        return (uint32_t)(MCPWM_EVT_TIMER0_TEP + cfg->mcpwm_timer);
    default:
        return 0;
    }
}

static esp_err_t esp32_adc_etm_create_event(uint32_t event_id, esp_etm_event_handle_t *out)
{
    esp32_adc_etm_event_t *ev = heap_caps_calloc(1, sizeof(*ev), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(ev, ESP_ERR_NO_MEM, TAG, "no mem for etm event");
    ev->base.event_id = event_id;
    ev->base.trig_periph = ETM_TRIG_PERIPH_MCPWM;
    ev->base.del = esp32_adc_etm_del_event;
    *out = &ev->base;
    return ESP_OK;
}

static esp_err_t esp32_adc_etm_create_task(uint32_t task_id, esp_etm_task_handle_t *out)
{
    esp32_adc_etm_task_t *tk = heap_caps_calloc(1, sizeof(*tk), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(tk, ESP_ERR_NO_MEM, TAG, "no mem for etm task");
    tk->base.task_id = task_id;
    tk->base.trig_periph = ETM_TRIG_PERIPH_MCPWM;
    tk->base.del = esp32_adc_etm_del_task;
    *out = &tk->base;
    return ESP_OK;
}

static void esp32_adc_etm_teardown(void)
{
    if (s_etm.enabled) {
        esp_etm_channel_disable(s_etm.chan);
        s_etm.enabled = false;
    }
    if (s_etm.connected) {
        esp_etm_channel_connect(s_etm.chan, NULL, NULL);
        s_etm.connected = false;
    }
    if (s_etm.event) {
        esp_etm_del_event(s_etm.event);
        s_etm.event = NULL;
    }
    if (s_etm.task) {
        esp_etm_del_task(s_etm.task);
        s_etm.task = NULL;
    }
    if (s_etm.chan) {
        esp_etm_del_channel(s_etm.chan);
        s_etm.chan = NULL;
    }
}

esp_err_t esp32_adc_digi_etm_connect(const esp32_adc_digi_etm_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");
    ESP_RETURN_ON_FALSE(cfg->mcpwm_timer < SOC_MCPWM_TIMERS_PER_GROUP, ESP_ERR_INVALID_ARG, TAG,
                        "invalid mcpwm timer %u", cfg->mcpwm_timer);

    uint32_t event_id = esp32_adc_mcpwm_event_id(cfg);
    ESP_RETURN_ON_FALSE(event_id != 0, ESP_ERR_INVALID_ARG, TAG, "invalid mcpwm event");

    esp32_adc_etm_teardown();

    ESP_RETURN_ON_ERROR(esp32_adc_etm_create_event(event_id, &s_etm.event), TAG, "event create failed");
    ESP_RETURN_ON_ERROR(esp32_adc_etm_create_task(ADC_TASK_START0, &s_etm.task), TAG, "task create failed");

    esp_etm_channel_config_t chan_cfg = {};
    ESP_RETURN_ON_ERROR(esp_etm_new_channel(&chan_cfg, &s_etm.chan), TAG, "channel alloc failed");
    ESP_RETURN_ON_ERROR(esp_etm_channel_connect(s_etm.chan, s_etm.event, s_etm.task), TAG, "connect failed");
    s_etm.connected = true;

    ESP_LOGI(TAG, "ETM MCPWM timer%u evt=%u -> ADC_TASK_START0", cfg->mcpwm_timer, (unsigned)cfg->event);
    return ESP_OK;
}

esp_err_t esp32_adc_digi_etm_enable(bool enable)
{
    if (!s_etm.chan || !s_etm.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    if (enable == s_etm.enabled) {
        return ESP_OK;
    }
    esp_err_t err = enable ? esp_etm_channel_enable(s_etm.chan) : esp_etm_channel_disable(s_etm.chan);
    if (err == ESP_OK) {
        s_etm.enabled = enable;
    }
    return err;
}

void esp32_adc_digi_etm_disconnect(void)
{
    esp32_adc_etm_teardown();
}

#endif /* SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED */
