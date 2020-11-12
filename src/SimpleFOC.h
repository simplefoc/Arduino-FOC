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
 *
 * This library supports any arduino device and it is especially optimized for Arduino UNO boards and 
 * other Atmega328 boards. But it supports Arrduinio MEGA boards and similar.
 * 
 * From the version 1.3.0 it will support the STM32 boards such as Bluepill and Nucelo devices.<br>
 * The programming is done the same way as for the Arduino UNO but stm32 devices require STM32Duino package. <br>
 * You can download it directly from library manager.  
 * 
 * @section example_code Example code
 * @code
#include <SimpleFOC.h>

//  initialize the motor
BLDCMotor motor = BLDCMotor(9, 10, 11, 11, 8);
//  initialize the encoder
Encoder encoder = Encoder(2, 3, 2048);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {  
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);

  // set control loop type to be used
  motor.controller = ControlType::velocity;
  
  // use monitoring with the BLDCMotor
  Serial.begin(115200);
  // monitoring port
  motor.useMonitoring(Serial);

  // link the motor to the sensor
  motor.linkSensor(&encoder);

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

  // monitoring function outputting motor variables to the serial terminal 
  motor.monitor();
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
#include "Encoder.h"
#include "MagneticSensorSPI.h"
#include "MagneticSensorI2C.h"
#include "MagneticSensorAnalog.h"
#include "HallSensor.h"
#include "BLDCDriver3PWM.h"
#include "StepperDriver4PWM.h"

#endif
