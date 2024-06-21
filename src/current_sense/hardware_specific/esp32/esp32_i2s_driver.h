#ifndef SIMPLEFOC_ESP32_HAL_I2S_DRIVER_H_ 
#define SIMPLEFOC_ESP32_HAL_I2S_DRIVER_H_

#include "Arduino.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) 

// This function reads data from the I2S FIFO and processes it to obtain average readings for each channel.
// The ADC counts get saved in uint32_t i2s_adc_buffer[].
void readFiFo();

void printdbg();

// Contrary to its name (so it can be called by the library), this function reads the already converted values from fifo
// and prints optional debug information.
// When using interrupt driven sampling, it only prints debug information.
void _startADC3PinConversionLowSide();
void _startADC3PinConversionInline();

// Takes the buffered adc counts and returns the coresponding float voltage for a pin.
float _readADCVoltageI2S(const int pin, const void *cs_params);

// Sets up the I2S peripheral and ADC. Can be run multiple times to configure multiple motors.
void* _configureI2S(const bool lowside, const void* driver_params, const int pinA, const int pinB, const int pinC);

#endif /* SIMPLEFOC_ESP32_HAL_I2S_DRIVER_H_ */
#endif /* ESP32 */