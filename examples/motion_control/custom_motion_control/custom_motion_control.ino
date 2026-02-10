
#include <Arduino.h>
#include <SimpleFOC.h>
#include "current_sense/hardware_specific/stm32/stm32_mcu.h"


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(D6, D10, D5, D8);

// encoder instance
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, D4);

// inline current sensor instance
// INA240A1 (gain 20V/V) and 5mOhm shunt resistor
LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 20.0f, A0, _NC, A3);

// commander communication instance
Commander command = Commander(Serial);
// void doMotion(char* cmd){ command.motion(&motor, cmd); }
void doMotor(char* cmd){ command.motor(&motor, cmd); }


// custom PID controller instance for the custom control method
// P controller with gain of 1.0f, no integral or derivative gain
PIDController custom_PID = PIDController(1.0f, 0, 0);
// custom motion control method
float positionPControl(FOCMotor* motor, float target){
  // simple proportional position control
  float error = target - motor->shaft_angle;
  // set the PID output limit to the motor current limit
  custom_PID.limit = motor->current_limit; 
  return custom_PID(error); // return current command based on the error
}

// optional add the PID to command to be able to tune it in runtime
void doPID(char* cmd){ command.pid(&custom_PID, cmd); }

void setup() {
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialize sensor hardware
  sensor.init();
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 20;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  current_sense.linkDriver(&driver);

  // set the custom control method
  motor.linkCustomMotionControl(positionPControl); 
  // set control loop type to be used
  motor.controller = MotionControlType::custom;
  // set the torque control type to voltage control (default is voltage control)
  motor.torque_controller = TorqueControlType::foc_current; 

  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable intially
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // monitor target velocity and angle

  // subscribe motor to the commander
  //command.add('T', doMotion, "motion control"); // a bit less resouce intensive
  command.add('M', doMotor, "motor");
  command.add('C', doPID, "custom PID");

  // current sense init and linking
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  _delay(1000);
}

void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  motor.move();

  // // motor monitoring
  motor.monitor();

  // user communication
  command.run();
}