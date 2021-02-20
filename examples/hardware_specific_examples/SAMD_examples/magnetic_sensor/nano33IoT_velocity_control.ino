
// show the infos for SAMD pin assignment on serial console
// set this in your build environment or modify in drivers/hardware_specific/samd21_mcu.h
#define SIMPLEFOC_SAMD_DEBUG

#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <Math.h>

// this is for an AS5048B absolute magnetic encoder on I2C address 0x41
MagneticSensorI2C sensor = MagneticSensorI2C(0x41, 14, 0xFE, 8);

// small BLDC gimbal motor, 7 pole-pairs
BLDCMotor motor = BLDCMotor(7);
// 3-PWM driving on pins 6, 5 and 8 (these are all on timer TCC0)
BLDCDriver3PWM driver =  BLDCDriver3PWM(6,5,8);


void setup() {
	Serial.begin(115200);
	delay(1000);
	Serial.println("Initializing...");

	sensor.init();
	Wire.setClock(400000);
	motor.linkSensor(&sensor);
	driver.voltage_power_supply = 9;
	driver.init();
	motor.linkDriver(&driver);
	motor.controller = ControlType::velocity;
	motor.PID_velocity.P = 0.2;
	motor.PID_velocity.I = 20;
	motor.PID_velocity.D = 0.001;
	motor.PID_velocity.output_ramp = 1000;
	motor.LPF_velocity.Tf = 0.01;
	motor.voltage_limit = 9;
	//motor.P_angle.P = 20;
	motor.init();
	motor.initFOC();
	Serial.println("Init complete...");
}


// velocity set point variable
float target_velocity = 2.0;


void loop() {
//	Serial.print("Sensor: ");
//	Serial.println(sensor.getAngle());
	motor.loopFOC();
	motor.move(target_velocity);
}
