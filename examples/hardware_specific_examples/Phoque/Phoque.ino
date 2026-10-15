
// This is the example setup for the Phoque-series of Board (phoque1 and phoque2)
// You need SimpleFOCDrivers (https://github.com/simplefoc/Arduino-FOC-drivers) as well as SimpleFOC for this to work, and STM32Duino 3.0.1+ for the board definitions (Phoque1, Phoque2)

// The builtin angle sensor requires a diametral magnet to be mounted to the rotor (spinning part) of the motor. A gap of about 2mm between the magnet and the sensor IC is optimal.
// See documentation for more information. I (moddingear) spent a lot of time on it.


//Change these settings !
#define UNDERVOLTAGE_THRESHOLD (INFINITY) // Voltage under which the driver will turn off (or not turn on and blink madly at you), for security reasons (short-circuit, overcurrent, etc...)
#define BRAKE_VOLTAGE (INFINITY) // Set to something above you supply voltage, but not too much above. The rest of you electronics must also be able to bear that voltage.

#define POLE_PAIRS 7
#define PHASE_RESISTANCE	(NOT_SET)
#define KV					(NOT_SET)
#define PHASE_INDUCTANCE_Q	(NOT_SET)
#define PHASE_INDUCTANCE_D	(NOT_SET)

#include <Arduino.h>

//Gate Drivers
#ifdef ARDUINO_PHOQUE1
#include <drivers/stspin32g4/STSPIN32G4.h>
#include <current_sense/phoque1/Phoque1_CurrentSense.hpp>
#endif

#ifdef ARDUINO_PHOQUE2
#include <drivers/drv832x/drv832x.h>
#include <current_sense/phoque2/Phoque2_CurrentSense.hpp>
#endif

//Encoder and brake
#ifdef ARDUINO_PHOQUE
#include <brake/phoque/phoque_brake.hpp>
#include <encoders/sc60228/MagneticSensorSC60228.h>
#endif

//CORDIC, accelerates computation of sin/cos, because we can
#include <utilities/stm32math/STM32G4CORDICTrigFunctions.h>

//Regular SimpleFOC motor
#include <BLDCMotor.h>
BLDCMotor motor(POLE_PAIRS, PHASE_RESISTANCE, KV, PHASE_INDUCTANCE_Q, PHASE_INDUCTANCE_D);

//Driver instancing
#ifdef ARDUINO_PHOQUE1
STSPIN32G4 driver;
Phoque1_CurrentSense current_sensor(Phoque1_CurrentSense::compute_equivalent_shunt(3e-3f, 1.5e3f, 100e3f), -32.f);
#endif
#ifdef ARDUINO_PHOQUE2
DRV832xDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL, PIN_SS_DRIVER, NOT_SET, NOT_SET, 2000000);
Phoque2_CurrentSense current_sensor(3e-3f, 40.f);
#endif

//Phoque2 has the angle sensor on a dedicated SPI
#ifdef ARDUINO_PHOQUE2
SPIClass position_sensor_spi(PIN_MOSI_MAG_ANGLE, PIN_MISO_MAG_ANGLE, PIN_CLK_MAG_ANGLE);
#define MAG_SPI (&position_sensor_spi)
#else
#define MAG_SPI (&SPI)
#endif

MagneticSensorSC60228 position_sensor(PIN_SS_MAG_ANGLE);
phoque_brake brake;

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(A_VBUS, INPUT_ANALOG);

	SimpleFOC_CORDIC_Config(); //Enable CORDIC for SimpleFOC usage
	brake.init(BRAKE_VOLTAGE); //Brake mosfet will turn on at the set voltage, allowing voltage spikes to be absorbed (if you have setup a brake resistor between the corresponding holes)
	// DO NOT SET THIS LOWER THAN YOUR SUPPLY VOLTAGE

	// use monitoring with serial 
	Serial.begin(115200);
	// enable more verbose output for debugging
	// comment out if not needed
	SimpleFOCDebug::enable(&Serial);

	// initialize encoder sensor hardware
	position_sensor.init(MAG_SPI);

	// link the motor to the sensor
	motor.linkSensor(&position_sensor);
	
	// driver config
	// power supply voltage [V]
	analogReadResolution(12);
	for(;;)
	{
		driver.voltage_power_supply = analogRead(A_VBUS) * VBUS_MUL_FACTOR * 3.3f / ((1<<12)-1);
		if (driver.voltage_power_supply < UNDERVOLTAGE_THRESHOLD)
		{
			digitalWrite(LED_BUILTIN, (millis() / 100)%2);
		}
		else
		{
			break;
		}
	}
	
	
	driver.init();
	// link the motor and the driver
	motor.linkDriver(&driver);

	#ifdef ARDUINO_PHOQUE2
	driver.setCurrentSenseGain(CSAGain_40);
	driver.setHighSideChargeCurrent(IDRIVEP_1000mA);
	driver.setHighSideDischargeCurrent(IDRIVEN_2000mA);
	driver.setLowSideChargeCurrent(IDRIVEP_1000mA);
	driver.setLowSideDischargeCurrent(IDRIVEN_2000mA);
	driver.setDriveTime(TDRIVE_4000ns);
	driver.calibrate_current_sense(); 
	#endif

	// link current sense and the driver
	current_sensor.linkDriver(&driver);
	// current sensing
	current_sensor.init();
	// no need for aligning
	current_sensor.skip_align = true;
	motor.linkCurrentSense(&current_sensor);

	// aligning voltage [V]
	motor.voltage_sensor_align = 3;
	// index search velocity [rad/s]
	motor.velocity_index_search = 3;

	// set motion control loop to be used
	motor.controller = MotionControlType::velocity;

	// contoller configuration 
	// default parameters in defaults.h

	// velocity PI controller parameters
	motor.PID_velocity.P = 0.2;
	motor.PID_velocity.I = 20;
	// default voltage_power_supply
	motor.voltage_limit = 6;
	// jerk control using voltage voltage ramp
	// default value is 300 volts per sec  ~ 0.3V per millisecond
	motor.PID_velocity.output_ramp = 1000;
	
	// velocity low pass filtering time constant
	motor.LPF_velocity.Tf = 0.01;

	// angle P controller
	motor.P_angle.P = 20;
	//  maximal velocity of the position control
	motor.velocity_limit = 4;

	// comment out if not needed
	motor.useMonitoring(Serial);
	
	// initialize motor
	motor.init();
	// align encoder and start FOC
	motor.initFOC();

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target angle using serial terminal:"));
	_delay(1000);
}

void loop()
{
	//Update power supply voltage with voltage measured from the ADC to compensate for voltage swings
	driver.voltage_power_supply = current_sensor.readMillivolt(A_VBUS) * VBUS_MUL_FACTOR;
	//Limit motor to 4V or supply/2, whichever is smallest
	motor.voltage_limit = min(4, driver.voltage_power_supply / 2);
	//Get temperature for overtemp security (in degrees C)
	int temp = current_sensor.read_temperature();
	bool motor_fault = false;

	//Driver error
	if (digitalReadFast(digitalPinToPinName(PIN_NFAULT)) == LOW)
	{
		motor_fault = true;
		if (motor.enabled)
		{
			Serial.printf("Driver reported a fault, disabling\n");
		}
	}
	//Over temperature safety (configured here for 100C)
	if (temp > 100)
	{
		motor_fault = true;
		if (motor.enabled)
		{
			Serial.printf("Over temp! %f\n", temp);
		}
	}
	//Undervoltage lockout (configured here for 8V)
	if (driver.voltage_power_supply < UNDERVOLTAGE_THRESHOLD)
	{
		motor_fault = true;
		if (motor.enabled)
		{
			Serial.printf("Voltage too low, disabling\n");
		}
	}

	if (motor_fault)
	{
		if (motor.enabled)
		{
			motor.disable();
		}
	}

	// main FOC algorithm function
	motor.loopFOC();

	// Motion control function
	motor.move();

	// function intended to be used with serial plotter to monitor motor variables
	// significantly slowing the execution down!!!!
	// motor.monitor();
}