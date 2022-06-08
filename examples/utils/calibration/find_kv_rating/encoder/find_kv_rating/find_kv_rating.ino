/**
 * 
 * Find KV rating for motor with encoder
 *
 * Motor KV rating is defiend as the increase of the motor velocity expressed in rotations per minute [rpm] per each 1 Volt int voltage control mode.
 * 
 * This example will set your motor in the torque control mode using voltage and set 1 volt to the motor. By reading the velocity it will calculat the motors KV rating.
 * - To make this esimation more credible you can try increasing the target voltage (or decrease in some cases) 
 * - The KV rating should be realatively static number - it should not change considerably with the increase in the voltage
 *
 */
#include <SimpleFOC.h>


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// encoder instance
Encoder sensor = Encoder(2, 3, 8192);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}


// voltage set point variable
float target_voltage = 1;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }
void calcKV(char* cmd) { 
  // calculate the KV
  Serial.println(motor.shaft_velocity/motor.target*30.0f/_PI);

}

void setup() { 
  
  // initialize encoder sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB); 
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // IMPORTANT!
  // make sure to set the correct power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 3;
  
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");
  command.add('K', calcKV, "calculate KV rating");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage : - commnad T"));
  Serial.println(F("Calculate the motor KV : - command K"));
  _delay(1000);
}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

  // user communication
  command.run();
}