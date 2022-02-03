/*!
 * @file SimpleFOC.h
 *
 * @mainpage Simple Field Oriented Control BLDC motor control library
 *
 * @section intro_sec Introduction
 *
 * Proper low-cost and low-power FOC supporting boards are very hard to find these days and even may not exist.<br> Even harder to find is a stable and simple FOC algorithm code capable of running on Arduino devices. Therefore this is an attempt to:
 * - Demystify FOC algorithm and make a robust but simple Arduino library: Arduino SimpleFOC library
 * - Develop a modular BLDC driver board: Arduino SimpleFOC shield.
 *
 * @section features Features
 *  - Arduino compatible: Arduino library code
 *  - Easy to setup and configure:
 *     - Easy hardware configuration
 *     - Easy tuning the control loops
 *  - Modular:
 *     - Supports as many sensors , BLDC motors and driver boards as possible
 *     - Supports as many application requirements as possible
 *  - Plug & play: Arduino SimpleFOC shield
 * 
 * @section dependencies Supported Hardware
 *  - Motors 
 *    - BLDC motors
 *    - Stepper motors
 * - Drivers 
 *    - BLDC drivers
 *    - Gimbal drivers
 *    - Stepper drivers
 * - Position sensors 
 *    - Encoders
 *    - Magnetic sensors
 *    - Hall sensors
 *    - Open-loop control
 * - Microcontrollers 
 *    - Arduino
 *    - STM32
 *    - ESP32
 *    - Teensy
 * 
 * @section example_code Example code
 * @code
#include <SimpleFOC.h>

//  BLDCMotor( pole_pairs )
BLDCMotor motor = BLDCMotor(11);
//  BLDCDriver( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional) )
BLDCDriver3PWM motor = BLDCDriver3PWM(9, 10, 11, 8);
//  Encoder(pin_A, pin_B, CPR)
Encoder encoder = Encoder(2, 3, 2048);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {  
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // initialise driver hardware
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.controller = MotionControlType::velocity;
  // initialize motor
  motor.init();
  
  // align encoder and start FOC
  motor.initFOC();
}

void loop() {
  // FOC algorithm function
  motor.loopFOC();

  // velocity control loop function
  // setting the target velocity or 2rad/s
  motor.move(2);
}
 * @endcode 
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef SIMPLEFOC_H
#define SIMPLEFOC_H

#include "BLDCMotor.h"
#include "StepperMotor.h"
#include "sensors/Encoder.h"
#include "sensors/MagneticSensorSPI.h"
#include "sensors/MagneticSensorI2C.h"
#include "sensors/MagneticSensorAnalog.h"
#include "sensors/MagneticSensorPWM.h"
#include "sensors/HallSensor.h"
#include "sensors/GenericSensor.h"
#include "drivers/BLDCDriver3PWM.h"
#include "drivers/BLDCDriver6PWM.h"
#include "drivers/StepperDriver4PWM.h"
#include "drivers/StepperDriver2PWM.h"
#include "current_sense/InlineCurrentSense.h"
#include "current_sense/LowsideCurrentSense.h"
#include "current_sense/GenericCurrentSense.h"
#include "communication/Commander.h"
#include "communication/StepDirListener.h"

#endif
