

#pragma once

/*
 * RP2040 ADC features are very weak :-(
 *  - only 4 inputs
 *  - only 9 bit effective resolution
 *  - read only 1 input at a time
 *  - 2 microseconds conversion time!
 *  - no triggers from PWM / events, only DMA
 *
 * So to read 3 phases takes 6 microseconds. :-(
 * 
 * The RP2040 ADC engine takes over the control of the MCU's ADC. Parallel ADC access is not permitted, as this would
 * cause conflicts with the engine's DMA based access and cause crashes.
 * To use the other ADC channels, use them via this engine. Use addPin() to add them to the conversion, and getLastResult()
 * to retrieve their value at any time.
 * 
 * For motor current sensing, the engine supports both inline sensing and low-side sensing.
 * 
 * Inline sensing is supported by offering a user-selectable fixed ADC sampling rate, which can be set between 500kHz and 1Hz.
 * After starting the engine it will continuously sample and provide new values at the configured rate.
 * 
 * Low-side sensing is supported by configuring a trigger from the PWM signal. The trigger happens at the middle-point of the
 * up/down counting PWM, which is the mid-point of the on-period.
 * So in the case of low-side sensing, all ADC channels are converted at the rate of the PWM frequency.
 * 
 * The SimpleFOC PWM driver for RP2040 syncs all the slices, so the PWM trigger is applied to the first used slice. For current
 * sensing to work correctly, all PWM slices have to be set to the same PWM frequency.
 * In theory, two motors could be sensed using 2 shunts on each motor. In practice, due to the slow conversion rate of the RP2040's
 * ADC, this would mean 8us conversion time, which would have to fit in the low-side on-time even at low duty-cycles... It remains
 * to be seen how well this can work, or if it works at all, but presumably the PWM frequency would have to be quite low.
 * 
 * TODO we need the mid-point of the low-side, which is actually the beginning/end of the PWM cycle - hmmmm...
 * 
 * Note that if using other ADC channels along with the motor current sensing, those channels will be subject to the same conversion schedule as the motor's ADC channels, i.e. convert at the same fixed rate in case
 * of inline sensing, or based on the motor PWM in case of PWM-triggered low-side sensing.
 * 
 * Solution to trigger ADC conversion from PWM via DMA:
 * use the PWM wrap as a DREQ to a DMA channel, and have the DMA channel write to the ADC's CS register to trigger an ADC sample.
 * Solution for ADC conversion:
 * ADC converts all channels in round-robin mode, and writes to FIFO. FIFO is emptied by a DMA which triggers after N conversions,
 * where N is the number of ADC channels used. So this DMA copies all the values from one round-robin conversion. This first DMA
 * triggers a second DMA which does a 32bit copy of all converted values (up to 4 channels x 8bit) at once, and triggers an interrupt.
 * The interrupt routine copies the values to the output buffer.
 * 
 * TODO think about whether the second DMA is needed
 */


#define SIMPLEFOC_RP2040_ADC_RESOLUTION 256
#ifndef SIMPLEFOC_RP2040_ADC_VDDA 
#define SIMPLEFOC_RP2040_ADC_VDDA 3.3f
#endif

class RP2040ADCEngine {

public:
    RP2040ADCEngine();
    void addPin(int pin);
    void setPWMTrigger(uint slice);

    bool init();
    void start();
    void stop();

    void getLastResult();

    void handleADCUpdate();

    int samples_per_second = 0; // leave at 0 to convert in tight loop
    float adc_conv = (SIMPLEFOC_RP2040_ADC_VDDA / SIMPLEFOC_RP2040_ADC_RESOLUTION); // conversion from raw ADC to float

    int triggerPWMSlice = -1;
    bool initialized;
    uint readDMAChannel;
    //uint copyDMAChannel;
    uint triggerDMAChannel;

    bool channelsEnabled[4];
    volatile uint8_t samples[4];
    volatile uint8_t lastResults[4];
    //alignas(32) volatile uint8_t nextResults[4];
};