
// show the infos for SAMD pin assignment on serial console
// set this #define SIMPLEFOC_SAMD_DEBUG in drivers/hardware_specific/samd21_mcu.h


#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>

// this is for an AS5048B absolute magnetic encoder on I2C address 0x41
MagneticSensorI2C sensor = MagneticSensorI2C(0x41, 14, 0xFE, 8);

// small BLDC gimbal motor, 7 pole-pairs
BLDCMotor motor = BLDCMotor(7);
// 3-PWM driving on pins 6, 5 and 8 - these are all on the same timer unit (TCC0), but different channels
BLDCDriver3PWM driver =  BLDCDriver3PWM(6,5,8);

// velocity set point variable
float target_velocity = 2.0f;
// instantiate the commander
Commander command = Commander(SerialUSB);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }


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
	motor.controller = MotionControlType::velocity;
	motor.PID_velocity.P = 0.2f;
	motor.PID_velocity.I = 20;
	motor.PID_velocity.D = 0.001f;
	motor.PID_velocity.output_ramp = 1000;
	motor.LPF_velocity.Tf = 0.01f;
	motor.voltage_limit = 9;
	//motor.P_angle.P = 20;
	motor.init();
	motor.initFOC();

	// add target command T
	command.add('T', doTarget, "target velocity");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target velocity using serial terminal:"));
	delay(100);
}



void loop() {
//	Serial.print("Sensor: ");
//	Serial.println(sensor.getAngle());
	motor.loopFOC();
	motor.move(target_velocity);
	// user communication
	command.run();
}
