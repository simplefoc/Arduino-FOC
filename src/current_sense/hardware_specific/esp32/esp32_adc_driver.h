#ifndef SIMPLEFOC_ESP32_HAL_ADC_DRIVER_H_ 
#define SIMPLEFOC_ESP32_HAL_ADC_DRIVER_H_

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)

/**
 * Get ADC value for pin
 * @param pin - pin number
 * @return ADC value (0 - 4095)
 * */
uint16_t adcRead(uint8_t pin);

/**
 * Initialize ADC pin
 * @param pin - pin number
 * 
 * @return true if success
 *          false if pin is not an ADC pin
 */
bool adcInit(uint8_t pin);


#endif /* SIMPLEFOC_ESP32_HAL_ADC_DRIVER_H_ */
#endif /* ESP32 */