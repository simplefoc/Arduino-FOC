#include "esp32_adc_driver.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3))

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"

static uint8_t __analogAttenuation = 3;//11db
static uint8_t __analogWidth = 3;//12 bits
static uint8_t __analogCycles = 8;
static uint8_t __analogSamples = 0;//1 sample
static uint8_t __analogClockDiv = 1;

// Width of returned answer ()
static uint8_t __analogReturnedWidth = 12;

void __analogSetWidth(uint8_t bits){
    if(bits < 9){
        bits = 9;
    } else if(bits > 12){
        bits = 12;
    }
    __analogReturnedWidth = bits;
    __analogWidth = bits - 9;
    // SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR1_BIT_WIDTH, __analogWidth, SENS_SAR1_BIT_WIDTH_S);
    // SET_PERI_REG_BITS(SENS_SAR_READER1_CTRL_REG, SENS_SAR1_SAMPLE_BIT, __analogWidth, SENS_SAR1_SAMPLE_BIT_S);

    // SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR2_BIT_WIDTH, __analogWidth, SENS_SAR2_BIT_WIDTH_S);
    // SET_PERI_REG_BITS(SENS_SAR_READER2_CTRL_REG, SENS_SAR2_SAMPLE_BIT, __analogWidth, SENS_SAR2_SAMPLE_BIT_S);
}

void __analogSetCycles(uint8_t cycles){
    __analogCycles = cycles;
    // SET_PERI_REG_BITS(SENS_SAR_READER1_CTRL_REG, SENS_SAR1_SAMPLE_CYCLE, __analogCycles, SENS_SAR1_SAMPLE_CYCLE_S);
    // SET_PERI_REG_BITS(SENS_SAR_READER2_CTRL_REG, SENS_SAR2_SAMPLE_CYCLE, __analogCycles, SENS_SAR2_SAMPLE_CYCLE_S);
}

void __analogSetSamples(uint8_t samples){
    if(!samples){
        return;
    }
    __analogSamples = samples - 1;
    SET_PERI_REG_BITS(SENS_SAR_READER1_CTRL_REG, SENS_SAR1_SAMPLE_NUM, __analogSamples, SENS_SAR1_SAMPLE_NUM_S);
    SET_PERI_REG_BITS(SENS_SAR_READER2_CTRL_REG, SENS_SAR2_SAMPLE_NUM, __analogSamples, SENS_SAR2_SAMPLE_NUM_S);
}

void __analogSetClockDiv(uint8_t clockDiv){
    if(!clockDiv){
        return;
    }
    __analogClockDiv = clockDiv;
    SET_PERI_REG_BITS(SENS_SAR_READER1_CTRL_REG, SENS_SAR1_CLK_DIV, __analogClockDiv, SENS_SAR1_CLK_DIV_S);
    SET_PERI_REG_BITS(SENS_SAR_READER2_CTRL_REG, SENS_SAR2_CLK_DIV, __analogClockDiv, SENS_SAR2_CLK_DIV_S);
}

void __analogSetAttenuation(uint8_t attenuation)
{
    __analogAttenuation = attenuation & 3;
    uint32_t att_data = 0;
    int i = 10;
    while(i--){
        att_data |= __analogAttenuation << (i * 2);
    }
    WRITE_PERI_REG(SENS_SAR_ATTEN1_REG, att_data & 0xFFFF);//ADC1 has 8 channels
    WRITE_PERI_REG(SENS_SAR_ATTEN2_REG, att_data);
}

void IRAM_ATTR __analogInit(){
    static bool initialized = false;
    if(initialized){
        return;
    }

    __analogSetAttenuation(__analogAttenuation);
    __analogSetCycles(__analogCycles);
    __analogSetSamples(__analogSamples + 1);//in samples
    __analogSetClockDiv(__analogClockDiv);
    __analogSetWidth(__analogWidth + 9);//in bits

    SET_PERI_REG_MASK(SENS_SAR_READER1_CTRL_REG, SENS_SAR1_DATA_INV);
    SET_PERI_REG_MASK(SENS_SAR_READER2_CTRL_REG, SENS_SAR2_DATA_INV);

    SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_FORCE_M); //SAR ADC1 controller (in RTC) is started by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD_FORCE_M); //SAR ADC1 pad enable bitmap is controlled by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_FORCE_M); //SAR ADC2 controller (in RTC) is started by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_SAR2_EN_PAD_FORCE_M); //SAR ADC2 pad enable bitmap is controlled by SW

    CLEAR_PERI_REG_MASK(SENS_SAR_POWER_XPD_SAR_REG, SENS_FORCE_XPD_SAR_M); //force XPD_SAR=0, use XPD_FSM
    SET_PERI_REG_BITS(SENS_SAR_POWER_XPD_SAR_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); //force XPD_AMP=0

    CLEAR_PERI_REG_MASK(SENS_SAR_AMP_CTRL3_REG, 0xfff << SENS_AMP_RST_FB_FSM_S);  //clear FSM
    SET_PERI_REG_BITS(SENS_SAR_AMP_CTRL1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
    SET_PERI_REG_BITS(SENS_SAR_AMP_CTRL1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
    SET_PERI_REG_BITS(SENS_SAR_POWER_XPD_SAR_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
    while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_SARADC_MEAS_STATUS_S) != 0); //wait det_fsm==

    initialized = true;
}

void __analogSetPinAttenuation(uint8_t pin, uint8_t attenuation)
{
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0 || attenuation > 3){
        return ;
    }
    __analogInit();
    if(channel > 7){
        SET_PERI_REG_BITS(SENS_SAR_ATTEN2_REG, 3, attenuation, ((channel - 10) * 2));
    } else {
        SET_PERI_REG_BITS(SENS_SAR_ATTEN1_REG, 3, attenuation, (channel * 2));
    }
}

bool IRAM_ATTR __adcAttachPin(uint8_t pin){

    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        return false;//not adc pin
    }

    int8_t pad = digitalPinToTouchChannel(pin);
    if(pad >= 0){
        uint32_t touch = READ_PERI_REG(SENS_SAR_TOUCH_CONF_REG);
        if(touch & (1 << pad)){
            touch &= ~((1 << (pad + SENS_TOUCH_OUTEN_S)));
            WRITE_PERI_REG(SENS_SAR_TOUCH_CONF_REG, touch);
        }
    } else if(pin == 25){
        CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE); //stop dac1
    } else if(pin == 26){
        CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_XPD_DAC | RTC_IO_PDAC2_DAC_XPD_FORCE); //stop dac2
    }

    pinMode(pin, ANALOG);

    __analogInit();
    return true;
}

bool IRAM_ATTR __adcStart(uint8_t pin){

    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        return false;//not adc pin
    }

    if(channel > 9){
        channel -= 10;
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS2_CTRL2_REG, SENS_SAR2_EN_PAD, (1 << channel), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    }
    return true;
}

bool IRAM_ATTR __adcBusy(uint8_t pin){

    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        return false;//not adc pin
    }

    if(channel > 7){
        return (GET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DONE_SAR) == 0);
    }
    return (GET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DONE_SAR) == 0);
}

uint16_t IRAM_ATTR __adcEnd(uint8_t pin)
{

    uint16_t value = 0;
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        return 0;//not adc pin
    }
    if(channel > 7){
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DONE_SAR) == 0); //wait for conversion
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DONE_SAR) == 0); //wait for conversion
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    // Shift result if necessary
    uint8_t from = __analogWidth + 9;
    if (from == __analogReturnedWidth) {
        return value;
    }
    if (from > __analogReturnedWidth) {
        return value >> (from - __analogReturnedWidth);
    }
    return value << (__analogReturnedWidth - from);
}

void __analogReadResolution(uint8_t bits)
{
    if(!bits || bits > 16){
        return;
    }
    __analogSetWidth(bits);         // hadware from 9 to 12
    __analogReturnedWidth = bits;   // software from 1 to 16
}

uint16_t IRAM_ATTR adcRead(uint8_t pin)
{
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        return false;//not adc pin
    }

    __analogInit();

    if(channel > 9){
        channel -= 10;
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS2_CTRL2_REG, SENS_SAR2_EN_PAD, (1 << channel), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    }

    uint16_t value = 0;

    if(channel > 7){
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DONE_SAR) == 0); //wait for conversion
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DONE_SAR) == 0); //wait for conversion
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    // Shift result if necessary
    uint8_t from = __analogWidth + 9;
    if (from == __analogReturnedWidth) {
        return value;
    }
    if (from > __analogReturnedWidth) {
        return value >> (from - __analogReturnedWidth);
    }
    return value << (__analogReturnedWidth - from);
}


#endif