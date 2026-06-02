#include "esp32_mcu.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

#include "esp_idf_version.h"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
#error SimpleFOC: ESP-IDF version 4 or lower detected. Please update to ESP-IDF 5.x and Arduino-esp32 3.0 (or higher)
#endif

#include "../../../drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"
#include "../../../drivers/hardware_specific/esp32/mcpwm_private.h"
#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#if SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED
#include "esp32_adc_digi_lowside.h"
#endif

// #define SIMPLEFOC_ESP32_INTERRUPT_DEBUG

#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
#include "driver/gpio.h"

#ifndef DEBUGPIN
#ifdef CONFIG_IDF_TARGET_ESP32S3
#define DEBUGPIN 16
#else
#define DEBUGPIN 19
#endif
#endif

#define GPIO_NUM (gpio_num_t)((int)GPIO_NUM_0 + DEBUGPIN)
#endif

/**
 * Low-side current sense on ESP32 MCPWM.
 *
 * ADC path (see esp32_adc_digi_internal.h):
 *   - LEGACY:     MCPWM ISR calls adcRead() one phase per interrupt (~10 us each).
 *   - DIGI_SW:    ESP32 / S2 — digi+DMA; ISR only calls esp32_adc_digi_trigger_software().
 *   - DIGI_ETM:   S3+ — same digi+DMA; MCPWM TEZ starts ADC via ETM (no ADC ISR).
 */

float IRAM_ATTR _readADCVoltageLowSide(const int pin, const void *cs_params)
{
    ESP32CurrentSenseParams *p = (ESP32CurrentSenseParams *)cs_params;
    int no_channel = 0;
    for (int i = 0; i < 3; i++) {
        if (!_isset(p->pins[i])) {
            continue;
        }
        if (pin == p->pins[i]) {
            return p->adc_buffer[no_channel] * p->adc_voltage_conv;
        }
        no_channel++;
    }
    SIMPLEFOC_DEBUG("ERROR: ADC pin not found in the buffer!");
    return 0;
}

void *IRAM_ATTR _configureADCLowSide(const void *driver_params, const int pinA, const int pinB, const int pinC)
{
    ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams *)driver_params;
    mcpwm_timer_t *t = (mcpwm_timer_t *)p->timers[0];

    if (t->on_full != nullptr) {
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Low side callback is already set. Cannot set it again for timer: " +
                                 String(t->timer_id) + ", group: " + String(t->group->group_id));
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }

    ESP32CurrentSenseParams *params = new ESP32CurrentSenseParams{};
    int no_adc_channels = 0;

    int adc_pins[3] = { pinA, pinB, pinC };
    for (int i = 0; i < 3; i++) {
        if (_isset(adc_pins[i])) {
            if (!adcInit(adc_pins[i])) {
                SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Failed to initialise ADC pin: " + String(adc_pins[i]) +
                                         ", maybe not an ADC pin?");
                return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
            }
            params->pins[no_adc_channels++] = adc_pins[i];
        }
    }

    params->adc_voltage_conv = (_ADC_VOLTAGE) / (_ADC_RESOLUTION);
    params->no_adc_channels = no_adc_channels;

#if SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED
    if (esp32_adc_lowside_configure(params) != ESP32_ADC_LOWSIDE_LEGACY) {
        t->user_data = params;
        return params;
    }
#endif

    t->user_data = params;
    return params;
}

static bool IRAM_ATTR _mcpwm_legacy_adc_callback(mcpwm_timer_handle_t tim, const mcpwm_timer_event_data_t *edata,
                                                 void *user_data)
{
    (void)tim;
    (void)edata;
    ESP32CurrentSenseParams *p = (ESP32CurrentSenseParams *)user_data;
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
    gpio_set_level(GPIO_NUM, 1);
#endif
    p->adc_buffer[p->buffer_index] = adcRead(p->pins[p->buffer_index]);
    p->buffer_index++;
    if (p->buffer_index >= p->no_adc_channels) {
        p->buffer_index = 0;
    }
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
    gpio_set_level(GPIO_NUM, 0);
#endif
    return true;
}

static bool IRAM_ATTR _mcpwm_legacy_comparator_adc_callback(mcpwm_cmpr_handle_t cmpr,
                                                            const mcpwm_compare_event_data_t *edata, void *user_data)
{
    if (edata->direction != MCPWM_TIMER_DIRECTION_UP) {
        return true;
    }
    ESP32CurrentSenseParams *p = (ESP32CurrentSenseParams *)user_data;
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
    gpio_set_level(GPIO_NUM, 1);
#endif
    p->adc_buffer[p->buffer_index] = adcRead(p->pins[p->buffer_index]);
    p->buffer_index++;
    if (p->buffer_index >= p->no_adc_channels) {
        p->buffer_index = 0;
    }
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
    gpio_set_level(GPIO_NUM, 0);
#endif
    return true;
}

void IRAM_ATTR _startADC3PinConversionLowSide()
{
#if SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED
    esp32_adc_lowside_start_conversion();
#endif
}

static void *esp32_mcpwm_sync_legacy_lowside(void *driver_params, ESP32CurrentSenseParams *cs)
{
    ESP32MCPWMDriverParams *p = (ESP32MCPWMDriverParams *)driver_params;
    mcpwm_timer_t *t = (mcpwm_timer_t *)p->timers[0];
    int group_id = p->group_id;

    SIMPLEFOC_ESP32_CS_DEBUG("Legacy path: MCPWM ISR + adcRead() (one phase per interrupt)");

#ifndef SIMPLEFOC_CS_PRETRIGGER_US
#define SIMPLEFOC_CS_PRETRIGGER_US 5
#endif

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
        uint32_t pwm_duty_cycle =
            p->mcpwm_period * (0.75f - ((float)p->pwm_frequency * SIMPLEFOC_CS_PRETRIGGER_US) / 1e6f / 2.0f);
        CHECK_CS_ERR(mcpwm_comparator_set_compare_value((mcpwm_cmpr_handle_t)cs->pretrig_comparator, pwm_duty_cycle),
                     "Failed to set pretrigger compare value");

        mcpwm_comparator_event_callbacks_t cmp_cbs = {
            .on_reach = _mcpwm_legacy_comparator_adc_callback,
        };
        CHECK_CS_ERR(mcpwm_comparator_register_event_callbacks((mcpwm_cmpr_handle_t)cs->pretrig_comparator, &cmp_cbs,
                                                               cs),
                     "Failed to register comparator callback");
        _notifyLowSideUsingComparator(group_id);
        SIMPLEFOC_ESP32_CS_DEBUG("MCPWM" + String(group_id) + " timer " + String(t->timer_id) + " comparator + adcRead");
        return cs;
    }

    SIMPLEFOC_ESP32_CS_DEBUG("WARN: comparator unavailable; MCPWM on_full + adcRead");
    if (t->on_full != nullptr) {
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: on_full already set");
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }

    auto cbs = mcpwm_timer_event_callbacks_t{
        .on_full = _mcpwm_legacy_adc_callback,
    };
    t->fsm = MCPWM_TIMER_FSM_INIT;
    CHECK_CS_ERR(mcpwm_timer_register_event_callbacks(t, &cbs, cs), "Failed to set low side callback");
    t->fsm = MCPWM_TIMER_FSM_ENABLE;
    CHECK_CS_ERR(esp_intr_enable(t->intr), "Failed to enable low-side interrupts");
    return cs;
}

void *IRAM_ATTR _driverSyncLowSide(void *driver_params, void *cs_params)
{
#ifdef SIMPLEFOC_ESP32_INTERRUPT_DEBUG
    pinMode(DEBUGPIN, OUTPUT);
#endif

    ESP32CurrentSenseParams *cs = (ESP32CurrentSenseParams *)cs_params;
    if (!cs) {
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: cs_params is null");
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }

#if SIMPLEFOC_ESP32_ADC_DIGI_SUPPORTED
    if (cs->adc_lowside_path != ESP32_ADC_LOWSIDE_LEGACY) {
        void *r = esp32_adc_lowside_sync_mcpwm(driver_params, cs);
        if (r == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) {
            return r;
        }
        return cs;
    }
#endif

    return esp32_mcpwm_sync_legacy_lowside(driver_params, cs);
}

#endif
