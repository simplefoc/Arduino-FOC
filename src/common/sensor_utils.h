#ifndef HARDWARE_H
#define HARDWARE_H

struct MagneticSensorSPIConfig_s  {
  int spi_mode;
  long clock_speed;
  int bit_resolution;
  int angle_register;
  int data_start_bit;
  int command_rw_bit;
  int command_parity_bit;
};

struct MagneticSensorI2CConfig_s  {
  int chip_address;
  long clock_speed;
  int bit_resolution;
  int angle_register;
  int data_start_bit; 
};

/** Typical configuration for the 14bit AMS AS5147 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s AS5147_SPI = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0xCFFF,
  .data_start_bit = 13, 
  .command_rw_bit = 14,
  .command_parity_bit = 15
};

/** Typical configuration for the 14bit MonolithicPower MA730 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s MA730_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x0000,
  .data_start_bit = 15, 
  .command_rw_bit = 0,  // not required
  .command_parity_bit = 0 // parity not implemented
};

/** Typical configuration for the 12bit AMS AS5600 magnetic sensor over I2C interface */
MagneticSensorI2CConfig_s AS5600_I2C = {
  .chip_address = 0x36,
  .clock_speed = 1000000,
  .bit_resolution = 12,
  .angle_register = 0x0E,
  .data_start_bit = 11
};

/** Typical configuration for the 12bit AMS AS5048 magnetic sensor over I2C interface */
MagneticSensorI2CConfig_s AS5048_I2C = {
  .chip_address = 0x40,  // highly configurable.  if A1 and A2 are held low, this is probable value
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0xFE,
  .data_start_bit = 15
};

#endif
