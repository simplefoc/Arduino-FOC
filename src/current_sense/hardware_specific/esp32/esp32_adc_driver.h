

#ifndef SIMPLEFOC_ESP32_HAL_ADC_DRIVER_H_ 
#define SIMPLEFOC_ESP32_HAL_ADC_DRIVER_H_

#include "Arduino.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) 
/*
 * Get ADC value for pin
 * */
uint16_t adcRead(uint8_t pin);

/*
 * Set the resolution of analogRead return values. Default is 12 bits (range from 0 to 4096).
 * If between 9 and 12, it will equal the set hardware resolution, else value will be shifted.
 * Range is 1 - 16
 *
 * Note: compatibility with Arduino SAM
 */
void __analogReadResolution(uint8_t bits);

/*
 * Sets the sample bits and read resolution
 * Default is 12bit (0 - 4095)
 * Range is 9 - 12
 * */
void __analogSetWidth(uint8_t bits);

/*
 * Set number of cycles per sample
 * Default is 8 and seems to do well
 * Range is 1 - 255
 * */
void __analogSetCycles(uint8_t cycles);

/*
 * Set number of samples in the range.
 * Default is 1
 * Range is 1 - 255
 * This setting splits the range into
 * "samples" pieces, which could look
 * like the sensitivity has been multiplied
 * that many times
 * */
void __analogSetSamples(uint8_t samples);

/*
 * Set the divider for the ADC clock.
 * Default is 1
 * Range is 1 - 255
 * */
void __analogSetClockDiv(uint8_t clockDiv);

/*
 * Set the attenuation for all channels
 * Default is 11db
 * */
void __analogSetAttenuation(uint8_t attenuation);

/*
 * Set the attenuation for particular pin
 * Default is 11db
 * */
void __analogSetPinAttenuation(uint8_t pin, uint8_t attenuation);

/*
 * Attach pin to ADC (will also clear any other analog mode that could be on)
 * */
bool __adcAttachPin(uint8_t pin);

/*
 * Start ADC conversion on attached pin's bus
 * */
bool __adcStart(uint8_t pin);

/*
 * Check if conversion on the pin's ADC bus is currently running
 * */
bool __adcBusy(uint8_t pin);

/*
 * Get the result of the conversion (will wait if it have not finished)
 * */
uint16_t __adcEnd(uint8_t pin);

#endif /* SIMPLEFOC_ESP32_HAL_ADC_DRIVER_H_ */
#endif /* ESP32 */