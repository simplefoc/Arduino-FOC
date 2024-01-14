/**
 * 
 * Torque control example using voltage control loop.
 * 
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers 
 * you a way to control motor torque by setting the voltage to the motor instead of the current. 
 * 
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way.
 */
#include <SimpleFOC.h>


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// hall sensor instance
LinearHall sensor = LinearHall(A0, A1, 11);


// voltage set point variable
float target_voltage = 2;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() { 

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 3;
  
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // initialize sensor hardware. This moves the motor to find the min/max sensor readings and 
  // averages them to get the center values. The motor can't move until motor.init is called, and 
  // motor.initFOC can't do its calibration until the sensor is intialized, so this must be done inbetween.
  // You can then take the values printed to the serial monitor and pass them to sensor.init to 
  // avoid having to move the motor every time. In that case it doesn't matter whether sensor.init 
  // is called before or after motor.init.
  sensor.init(&motor);
  Serial.print("LinearHall centerA: ");
  Serial.print(sensor.centerA);
  Serial.print(", centerB: ");
  Serial.println(sensor.centerB);
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
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