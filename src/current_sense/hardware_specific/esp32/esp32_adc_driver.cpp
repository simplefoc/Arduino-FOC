
#include "esp32_mcu.h"
#include "esp32_adc_driver.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) 
#define SIMPLEFOC_ADC_ATTEN ADC_11db
#define SIMPLEFOC_ADC_RES 12


#if CONFIG_IDF_TARGET_ESP32 // if esp32 variant

#include "soc/sens_reg.h"

// configure the ADCs in RTC mode
// saves about 3us per call 
// going from 12us to 9us
void __configFastADCs(){
    
    // configure both ADCs in RTC mode
    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DATA_INV);
    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DATA_INV);

    SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_FORCE_M); //SAR ADC1 controller (in RTC) is started by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE_M); //SAR ADC1 pad enable bitmap is controlled by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_FORCE_M); //SAR ADC2 controller (in RTC) is started by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD_FORCE_M); //SAR ADC2 pad enable bitmap is controlled by SW

    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_M); //force XPD_SAR=0, use XPD_FSM
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); //force XPD_AMP=0

    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_CTRL_REG, 0xfff << SENS_AMP_RST_FB_FSM_S);  //clear FSM
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
    while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_MEAS_STATUS_S) != 0); //wait det_fsm==
}


uint16_t IRAM_ATTR adcRead(uint8_t pin)
{
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Not ADC pin: "+String(pin));
        return false;//not adc pin
    }

    // start teh ADC conversion
    if(channel > 9){
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD, (1 << (channel - 10)), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
    }

    uint16_t value = 0;

    if(channel > 7){
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0); 
        // read the value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0); 
        // read the value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    // return value
    return value;
}

#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 // if esp32 s2 or s3 variants

#include "soc/sens_reg.h"


// configure the ADCs in RTC mode 
// no real gain - see if we do something with it later
// void __configFastADCs(){
    
//     SET_PERI_REG_MASK(SENS_SAR_READER1_CTRL_REG, SENS_SAR1_DATA_INV);
//     SET_PERI_REG_MASK(SENS_SAR_READER2_CTRL_REG, SENS_SAR2_DATA_INV);

//     SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_FORCE_M); //SAR ADC1 controller (in RTC) is started by SW
//     SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD_FORCE_M); //SAR ADC1 pad enable bitmap is controlled by SW
//     SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_FORCE_M); //SAR ADC2 controller (in RTC) is started by SW
//     SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_SAR2_EN_PAD_FORCE_M); //SAR ADC2 pad enable bitmap is controlled by SW

//     CLEAR_PERI_REG_MASK(SENS_SAR_POWER_XPD_SAR_REG, SENS_FORCE_XPD_SAR_M); //force XPD_SAR=0, use XPD_FSM
//     SET_PERI_REG_BITS(SENS_SAR_POWER_XPD_SAR_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); //force XPD_AMP=0

//     CLEAR_PERI_REG_MASK(SENS_SAR_AMP_CTRL3_REG, 0xfff << SENS_AMP_RST_FB_FSM_S);  //clear FSM
//     SET_PERI_REG_BITS(SENS_SAR_AMP_CTRL1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
//     SET_PERI_REG_BITS(SENS_SAR_AMP_CTRL1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
//     SET_PERI_REG_BITS(SENS_SAR_POWER_XPD_SAR_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
//     while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_SARADC_MEAS_STATUS_S) != 0); //wait det_fsm==
// }

uint16_t IRAM_ATTR adcRead(uint8_t pin)
{
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Not ADC pin: "+String(pin));
        return false;//not adc pin
    }

    // start the ADC conversion
    if(channel > 9){
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS2_CTRL2_REG, SENS_SAR2_EN_PAD, (1 << (channel - 10)), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    }

    uint16_t value = 0;

    if(channel > 9){
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DONE_SAR) == 0); 
        // read the value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DONE_SAR) == 0); 
        // read teh value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    return value;
}

#else // if others just use analogRead

#pragma message("SimpleFOC: Using analogRead for ADC reading, no fast ADC configuration available!")

uint16_t IRAM_ATTR adcRead(uint8_t pin){
    return analogRead(pin);
}

#endif


// configure the ADC for the pin
bool IRAM_ATTR adcInit(uint8_t pin){
    static bool initialized = false;
    
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Not ADC pin: "+String(pin));
        return false;//not adc pin
    }

    if(! initialized){
        analogSetAttenuation(SIMPLEFOC_ADC_ATTEN);
        analogReadResolution(SIMPLEFOC_ADC_RES);
    }   
    pinMode(pin, ANALOG);
    analogSetPinAttenuation(pin, SIMPLEFOC_ADC_ATTEN);
    analogRead(pin);

#if CONFIG_IDF_TARGET_ESP32 // if esp32 variant
    __configFastADCs();
#endif

    initialized = true;
    return true;
}

#endif
