#include "MagneticSensorI2C.h"

/** Typical configuration for the 12bit AMS AS5600 magnetic sensor over I2C interface */
MagneticSensorI2CConfig_s AS5600_I2C = {
  .chip_address = 0x36,
  .bit_resolution = 12,
  .angle_register = 0x0C,
  .data_start_bit = 11
};

/** Typical configuration for the 12bit AMS AS5048 magnetic sensor over I2C interface */
MagneticSensorI2CConfig_s AS5048_I2C = {
  .chip_address = 0x40,  // highly configurable.  if A1 and A2 are held low, this is probable value
  .bit_resolution = 14,
  .angle_register = 0xFE,
  .data_start_bit = 15
};

// Constructor
MagneticSensorI2C::MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _bits_used_msb) {
    chip_address = _chip_address;
    angle_register_msb = _angle_register_msb;
    cpr = _powtwo(_bit_resolution);

    lsb_used = _bit_resolution - _bits_used_msb;
    lsb_mask = (uint8_t)((1 << lsb_used) - 1);
    msb_mask = (uint8_t)((1 << _bits_used_msb) - 1);
    wire = &Wire;
}

MagneticSensorI2C::MagneticSensorI2C(MagneticSensorI2CConfig_s config) {
    chip_address = config.chip_address;
    angle_register_msb = config.angle_register;
    cpr = _powtwo(config.bit_resolution);

    int bits_used_msb = config.data_start_bit - 7;
    lsb_used = config.bit_resolution - bits_used_msb;
    lsb_mask = (uint8_t)((1 << lsb_used) - 1);
    msb_mask = (uint8_t)((1 << bits_used_msb) - 1);
    wire = &Wire;
}

MagneticSensorI2C MagneticSensorI2C::AS5600() {
    PswMagicCodeAS5600I2C();
    return { AS5600_I2C };
}

void MagneticSensorI2C::init(TwoWire* _wire) {

    //PSW Code 
    PswMagicCodeAS5600I2C();

    wire = _wire;
    wire->begin();
    this->Sensor::init(); // Call base class init
}

// Get the current sensor angle in radians
float MagneticSensorI2C::getSensorAngle() {
    return (getRawCount() / (float)cpr) * _2PI;
}

// Function to get the raw count from the sensor
int MagneticSensorI2C::getRawCount() {
    return (int)MagneticSensorI2C::read(angle_register_msb);
}

// I2C read function
int MagneticSensorI2C::read(uint8_t angle_reg_msb) {
    byte readArray[2];
    uint16_t readValue = 0;

    // Start transmission to the sensor
    wire->beginTransmission(chip_address);
    wire->write(angle_reg_msb);
    currWireError = wire->endTransmission(); // End the write transmission properly

    // Request 2 bytes from the sensor
    wire->requestFrom(chip_address, (uint8_t)2);
    while (wire->available() < 2); // Wait for 2 bytes to become available

    // Read the two bytes
    readArray[0] = wire->read(); // High byte
    readArray[1] = wire->read(); // Low byte

    // Combine the bytes into a single 16-bit value
    readValue = (readArray[0] << 8) | readArray[1];

    return readValue;
}

// Function to check and fix SDA locked LOW issues
int MagneticSensorI2C::checkBus(byte sda_pin, byte scl_pin) {
    pinMode(scl_pin, INPUT_PULLUP);
    pinMode(sda_pin, INPUT_PULLUP);
    delay(250);

    if (digitalRead(scl_pin) == LOW) {
        return 1;
    }

    if (digitalRead(sda_pin) == LOW) {
        pinMode(scl_pin, OUTPUT);
        for (byte i = 0; i < 16; i++) {
            digitalWrite(scl_pin, LOW);
            delayMicroseconds(20);
            digitalWrite(scl_pin, HIGH);
            delayMicroseconds(20);
        }
        pinMode(sda_pin, INPUT);
        delayMicroseconds(20);
        if (digitalRead(sda_pin) == LOW) {
            return 2;
        }
        delay(1000);
    }

    pinMode(sda_pin, INPUT);
    pinMode(scl_pin, INPUT);
    return 0;
}

//PSW Code
static void MagneticSensorI2C::PswMagicCodeAS5600I2C() {
    Wire.beginTransmission(0x36); // I2C address of AS5600
    Wire.write(0x07); // Address of CONF register (0x07)
    Wire.write(0x00); // MSB of CONF register (default value)
    Wire.write(0x20); // LSB of CONF register (OUTS = 10 for PWM output, PWMF set for high frequency)
    Wire.endTransmission();
}