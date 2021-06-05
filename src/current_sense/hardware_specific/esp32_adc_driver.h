/*
 Arduino.h - Main include file for the Arduino SDK
 Copyright (c) 2005-2013 Arduino Team.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#if !defined(MAIN_ESP32_HAL_ADC_DRIVER_H_) && defined(ESP_H) 
#define MAIN_ESP32_HAL_ADC_DRIVER_H_

#include "esp32-hal.h"


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

#endif /* MAIN_ESP32_HAL_ADC_H_ */