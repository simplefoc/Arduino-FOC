#include "esp32_adc_digi_lowside.h"

#if SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

#include "esp32_adc_digi_driver.h"
#include "../../../drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"
#include "../../../drivers/hardware_specific/esp32/mcpwm_private.h"
#include "driver/mcpwm_prelude.h"

#ifndef SIMPLEFOC_CS_PRETRIGGER_US
#define SIMPLEFOC_CS_PRETRIGGER_US 5
#endif

static bool esp32_pin_to_adc1_channel(int pin, adc_channel_t *out_ch)
{
    int8_t ch = digitalPinToAnalogChannel(pin);
    if (ch < 0 || ch >= SOC_ADC_MAX_CHANNEL_NUM) {
        return false;
    }
    *out_ch = (adc_channel_t)ch;
    return true;
}

static esp_err_t esp32_adc_digi_bind_params(ESP32CurrentSenseParams *params)
{
    if (params->no_adc_channels < 1 || params->no_adc_channels > SIMPLEFOC_ESP32_ADC_NUM_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_channel_t channels[SIMPLEFOC_ESP32_ADC_NUM_CHANNELS] = {};
    for (int i = 0; i < params->no_adc_channels; i++) {
        if (!esp32_pin_to_adc1_channel(params->pins[i], &channels[i])) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    esp32_adc_digi_config_t cfg = {
        .channels = { channels[0], channels[1] },
        .unit = ADC_UNIT_1,
        .adc_buffer = params->adc_buffer,
        .no_adc_channels = params->no_adc_channels,
    };

    return esp32_adc_digi_init(&cfg);
}

ESP32AdcLowsidePath esp32_adc_lowside_configure(ESP32CurrentSenseParams *params)
{
    if (params == NULL || !esp32_adc_digi_supported()) {
        return ESP32_ADC_LOWSIDE_LEGACY;
    }

    if (esp32_adc_digi_bind_params(params) != ESP_OK) {
        SIMPLEFOC_ESP32_CS_DEBUG("WARN: ADC digi+DMA init failed, using legacy adcRead path");
        return ESP32_ADC_LOWSIDE_LEGACY;
    }

    params->adc_lowside_path = ESP32_ADC_LOWSIDE_DIGI_SW;
    SIMPLEFOC_ESP32_CS_DEBUG("ADC digi+DMA ready (software trigger via MCPWM ISR on ESP32/S2)");
    return ESP32_ADC_LOWSIDE_DIGI_SW;
}

bool esp32_adc_lowside_uses_mcpwm_isr(const ESP32CurrentSenseParams *params)
{
    if (params == NULL) {
        return false;
    }
    return params->adc_lowside_path == ESP32_ADC_LOWSIDE_DIGI_SW;
}

void esp32_adc_lowside_start_conversion(void)
{
    esp32_adc_digi_trigger_software();
}

#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
static void *esp32_adc_lowside_sync_etm(void *driver_params, ESP32CurrentSenseParams *cs)
{
    ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams *)driver_params;
    mcpwm_timer_t *t = (mcpwm_timer_t *)p->timers[0];
    int group_id = p->group_id;

    esp32_adc_digi_etm_config_t etm = {
        .mcpwm_timer = (uint8_t)t->timer_id,
        .event = SIMPLEFOC_ESP32_ADC_MCPWM_EVT_TIMER_TEZ,
    };

    if (esp32_adc_digi_set_etm_source(&etm) != ESP_OK) {
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: ETM source setup failed");
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }
    if (esp32_adc_digi_set_trigger(SIMPLEFOC_ESP32_ADC_TRIG_ETM) != ESP_OK) {
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: ETM ADC trigger enable failed");
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }

    cs->adc_lowside_path = ESP32_ADC_LOWSIDE_DIGI_ETM;
    SIMPLEFOC_ESP32_CS_DEBUG("MCPWM" + String(group_id) + " timer " + String(t->timer_id) +
                              " -> ETM -> ADC (TEZ), no MCPWM ADC ISR");
    return cs;
}
#endif

static bool IRAM_ATTR esp32_adc_mcpwm_sw_trigger_cb(mcpwm_cmpr_handle_t cmpr,
                                                    const mcpwm_compare_event_data_t *edata,
                                                    void *user_data)
{
    (void)cmpr;
    (void)user_data;
    if (edata->direction != MCPWM_TIMER_DIRECTION_UP) {
        return true;
    }
    esp32_adc_digi_trigger_software();
    return true;
}

static bool IRAM_ATTR esp32_adc_mcpwm_sw_trigger_timer_cb(mcpwm_timer_handle_t tim,
                                                          const mcpwm_timer_event_data_t *edata,
                                                          void *user_data)
{
    (void)tim;
    (void)edata;
    (void)user_data;
    esp32_adc_digi_trigger_software();
    return true;
}

static void *esp32_adc_lowside_sync_mcpwm_sw(void *driver_params, ESP32CurrentSenseParams *cs)
{
    ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams *)driver_params;
    mcpwm_timer_t *t = (mcpwm_timer_t *)p->timers[0];
    int group_id = p->group_id;

    SIMPLEFOC_ESP32_CS_DEBUG("MCPWM comparator -> software ADC digi trigger (ESP32/S2 style)");

    mcpwm_comparator_config_t cmp_config = {};
    cmp_config.flags.update_cmp_on_tez = true;
    for (int i = 2; i >= 0; i--) {
        if (p->oper[i] == nullptr) {
            continue;
        }
        if (mcpwm_new_comparator(p->oper[i], &cmp_config, (mcpwm_cmpr_handle_t *)&cs->pretrig_comparator) == ESP_OK) {
            break;
        }
    }

    if (cs->pretrig_comparator) {
        uint32_t pwm_duty_cycle = p->mcpwm_period * (0.75f - ((float)p->pwm_frequency * SIMPLEFOC_CS_PRETRIGGER_US) / 1e6f / 2.0f);
        if (mcpwm_comparator_set_compare_value((mcpwm_cmpr_handle_t)cs->pretrig_comparator, pwm_duty_cycle) != ESP_OK) {
            SIMPLEFOC_ESP32_CS_DEBUG("ERROR: comparator compare value");
            return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
        }
        mcpwm_comparator_event_callbacks_t cmp_cbs = {
            .on_reach = esp32_adc_mcpwm_sw_trigger_cb,
        };
        if (mcpwm_comparator_register_event_callbacks((mcpwm_cmpr_handle_t)cs->pretrig_comparator, &cmp_cbs, cs) != ESP_OK) {
            SIMPLEFOC_ESP32_CS_DEBUG("ERROR: comparator callback");
            return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
        }
        _notifyLowSideUsingComparator(group_id);
        SIMPLEFOC_ESP32_CS_DEBUG("Comparator pre-trigger -> esp32_adc_digi_trigger_software()");
        return cs;
    }

    SIMPLEFOC_ESP32_CS_DEBUG("WARN: no comparator; MCPWM on_full -> software ADC digi trigger");
    if (t->on_full != nullptr) {
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: timer on_full already in use");
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }

    auto cbs = mcpwm_timer_event_callbacks_t{
        .on_full = esp32_adc_mcpwm_sw_trigger_timer_cb,
    };
    t->fsm = MCPWM_TIMER_FSM_INIT;
    if (mcpwm_timer_register_event_callbacks(t, &cbs, cs) != ESP_OK) {
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: timer callback");
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }
    t->fsm = MCPWM_TIMER_FSM_ENABLE;
    if (esp_intr_enable(t->intr) != ESP_OK) {
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: enable timer intr");
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }
    return cs;
}

void *esp32_adc_lowside_sync_mcpwm(void *driver_params, ESP32CurrentSenseParams *cs)
{
    if (cs == NULL || driver_params == NULL) {
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }

#if SIMPLEFOC_ESP32_ADC_ETM_SUPPORTED
    if (esp32_adc_digi_etm_supported() && cs->adc_lowside_path != ESP32_ADC_LOWSIDE_LEGACY) {
        return esp32_adc_lowside_sync_etm(driver_params, cs);
    }
#endif

    if (cs->adc_lowside_path == ESP32_ADC_LOWSIDE_DIGI_SW) {
        return esp32_adc_lowside_sync_mcpwm_sw(driver_params, cs);
    }

    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
}

#endif
