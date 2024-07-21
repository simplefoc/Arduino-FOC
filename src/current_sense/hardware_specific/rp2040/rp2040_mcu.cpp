
#if defined(TARGET_RP2040)


#include "../../hardware_api.h"
#include "./rp2040_mcu.h"
#include "../../../drivers/hardware_specific/rp2040/rp2040_mcu.h"
#include "communication/SimpleFOCDebug.h"

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"


/* Singleton instance of the ADC engine */
RP2040ADCEngine engine;

alignas(32) const uint32_t trigger_value = ADC_CS_START_ONCE_BITS; // start once

/* Hardware API implementation */

float _readADCVoltageInline(const int pinA, const void* cs_params) {
    // not super-happy with this. Here we have to return 1 phase current at a time, when actually we want to
    // return readings from the same ADC conversion run. The ADC on RP2040 is anyway in round robin mode :-(
    // like this we either have to block interrupts, or of course have the chance of reading across
    // new ADC conversions, which probably won't improve the accuracy.
    _UNUSED(cs_params);

    if (pinA>=26 && pinA<=29 && engine.channelsEnabled[pinA-26]) {
        return engine.lastResults.raw[pinA-26]*engine.adc_conv;
    }

    // otherwise return NaN
    return NAN;
};


void* _configureADCInline(const void *driver_params, const int pinA, const int pinB, const int pinC) {
    _UNUSED(driver_params);

    if( _isset(pinA) )
        engine.addPin(pinA);
    if( _isset(pinB) )
        engine.addPin(pinB);
    if( _isset(pinC) )
        engine.addPin(pinC);
    engine.init(); // TODO this has to happen later if we want to support more than one motor...
    engine.start();
    return &engine;
};


// not supported at the moment
// void* _configureADCLowSide(const void *driver_params, const int pinA, const int pinB, const int pinC) {    
//     if( _isset(pinA) )
//         engine.addPin(pinA);
//     if( _isset(pinB) )
//         engine.addPin(pinB);
//     if( _isset(pinC) )
//         engine.addPin(pinC);
//     engine.setPWMTrigger(((RP2040DriverParams*)driver_params)->slice[0]);
//     engine.init();
//     engine.start();
//     return &engine;
// };


// void _startADC3PinConversionLowSide() {
//     // what is this for?
// };


// float _readADCVoltageLowSide(const int pinA, const void* cs_params) {
//     // not super-happy with this. Here we have to return 1 phase current at a time, when actually we want to
//     // return readings from the same ADC conversion run. The ADC on RP2040 is anyway in round robin mode :-(
//     // like this we have block interrupts 3x instead of just once, and of course have the chance of reading across
//     // new ADC conversions, which probably won't improve the accuracy.

//     if (pinA>=26 && pinA<=29 && engine.channelsEnabled[pinA-26]) {
//         return engine.lastResults[pinA-26]*engine.adc_conv;
//     }

//     // otherwise return NaN
//     return NAN;
// };


// void* _driverSyncLowSide(void* driver_params, void* cs_params) {
//     // nothing to do
// };



volatile int rp2040_intcount = 0;

void _adcConversionFinishedHandler() {
    // conversion of all channels finished. copy results.
    volatile uint8_t* from = engine.samples;
    if (engine.channelsEnabled[0])
        engine.lastResults.raw[0] = (*from++);
    if (engine.channelsEnabled[1])
        engine.lastResults.raw[1] = (*from++);
    if (engine.channelsEnabled[2])
        engine.lastResults.raw[2] = (*from++);
    if (engine.channelsEnabled[3])
        engine.lastResults.raw[3] = (*from++);
    //dma_channel_acknowledge_irq0(engine.readDMAChannel);
    dma_hw->ints0 = 1u << engine.readDMAChannel;
    //dma_start_channel_mask( (1u << engine.readDMAChannel) );
    dma_channel_set_write_addr(engine.readDMAChannel, engine.samples, true);
    // if (engine.triggerPWMSlice>=0)
    //     dma_channel_set_trans_count(engine.triggerDMAChannel, 1, true);
    rp2040_intcount++;
};



/* ADC engine implementation */


RP2040ADCEngine::RP2040ADCEngine() {
    channelsEnabled[0] = false;
    channelsEnabled[1] = false;
    channelsEnabled[2] = false;
    channelsEnabled[3] = false;
    initialized = false;
};



void RP2040ADCEngine::addPin(int pin) {
    if (pin>=26 && pin<=29)
        channelsEnabled[pin-26] = true;
    else
        SIMPLEFOC_DEBUG("RP2040-CUR: ERR: Not an ADC pin: ", pin);
};



// void RP2040ADCEngine::setPWMTrigger(uint slice){
//     triggerPWMSlice = slice;
// };




bool RP2040ADCEngine::init() {
    if (initialized)
        return true;
    
    adc_init();
    int enableMask = 0x00;
    int channelCount = 0;
    for (int i = 3; i>=0; i--) {
        if (channelsEnabled[i]){
            adc_gpio_init(i+26);
            enableMask |= (0x01<<i);
            channelCount++;
        }
    }
    adc_set_round_robin(enableMask);
    adc_fifo_setup(
     true,              // Write each completed conversion to the sample FIFO
     true,              // Enable DMA data request (DREQ)
     channelCount,      // DREQ (and IRQ) asserted when all samples present
     false,             // We won't see the ERR bit because of 8 bit reads; disable.
     true               // Shift each sample to 8 bits when pushing to FIFO
    );
    if (samples_per_second<1 || samples_per_second>=500000) {
        samples_per_second = 0;
        adc_set_clkdiv(0);
    }
    else
        adc_set_clkdiv(48000000/samples_per_second);
    SIMPLEFOC_DEBUG("RP2040-CUR: ADC init");

    readDMAChannel = dma_claim_unused_channel(true);
    dma_channel_config cc1 = dma_channel_get_default_config(readDMAChannel);
    channel_config_set_transfer_data_size(&cc1, DMA_SIZE_8);
    channel_config_set_read_increment(&cc1, false);
    channel_config_set_write_increment(&cc1, true);
    channel_config_set_dreq(&cc1, DREQ_ADC);
    channel_config_set_irq_quiet(&cc1, false);
    dma_channel_configure(readDMAChannel,
        &cc1,
        samples,        // dest
        &adc_hw->fifo,  // source
        channelCount,   // count
        false           // defer start
    );
    dma_channel_set_irq0_enabled(readDMAChannel, true);
    irq_add_shared_handler(DMA_IRQ_0, _adcConversionFinishedHandler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    SIMPLEFOC_DEBUG("RP2040-CUR: DMA init");

    // if (triggerPWMSlice>=0) { // if we have a trigger
    //     triggerDMAChannel = dma_claim_unused_channel(true);
    //     dma_channel_config cc3 = dma_channel_get_default_config(triggerDMAChannel);
    //     channel_config_set_transfer_data_size(&cc3, DMA_SIZE_32);
    //     channel_config_set_read_increment(&cc3, false);
    //     channel_config_set_write_increment(&cc3, false);
    //     channel_config_set_irq_quiet(&cc3, true);
    //     channel_config_set_dreq(&cc3, DREQ_PWM_WRAP0+triggerPWMSlice); //pwm_get_dreq(triggerPWMSlice));
    //     pwm_set_irq_enabled(triggerPWMSlice, true);
    //     dma_channel_configure(triggerDMAChannel,
    //         &cc3,
    //         hw_set_alias_untyped(&adc_hw->cs),    // dest
    //         &trigger_value, // source
    //         1,              // count
    //         true           // defer start
    //     );
    //     SIMPLEFOC_DEBUG("RP2040-CUR: PWM trigger init slice ", triggerPWMSlice);
    // }

    initialized = true;
    return initialized;
};




void RP2040ADCEngine::start() {
    SIMPLEFOC_DEBUG("RP2040-CUR: ADC engine starting");
    irq_set_enabled(DMA_IRQ_0, true);
    dma_start_channel_mask( (1u << readDMAChannel) );
    for (int i=0;i<4;i++) {
        if (channelsEnabled[i]) {
            adc_select_input(i); // set input to first enabled channel
            break;
        }
    }
    // if (triggerPWMSlice>=0) {
    //     dma_start_channel_mask( (1u << triggerDMAChannel) );
    //     //hw_set_bits(&adc_hw->cs, trigger_value);
    // }
    // else
    adc_run(true);
    SIMPLEFOC_DEBUG("RP2040-CUR: ADC engine started");
};




void RP2040ADCEngine::stop() {
    adc_run(false);
    irq_set_enabled(DMA_IRQ_0, false);
    dma_channel_abort(readDMAChannel);
    // if (triggerPWMSlice>=0)
    //     dma_channel_abort(triggerDMAChannel);
    adc_fifo_drain();
    SIMPLEFOC_DEBUG("RP2040-CUR: ADC engine stopped");
};



ADCResults RP2040ADCEngine::getLastResults() {
    ADCResults r;
    r.value = lastResults.value;
    return r;
};



#endif