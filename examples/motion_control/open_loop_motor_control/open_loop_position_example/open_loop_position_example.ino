// Open loop motor control example
 #include <SimpleFOC.h>


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

//target variable
float target_position = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_position, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doVelocity(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }

void setup() {

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // currnet = resistance*voltage, so try to be well under 1Amp
  motor.voltage_limit = 3;   // [V]
  // limit/set the velocity of the transition in between 
  // target angles
  motor.velocity_limit = 5; // [rad/s] cca 50rpm
  // open loop control config
  motor.controller = MotionControlType::angle_openloop;

  // init motor hardware
  motor.init();

  // add target command T
  command.add('T', doTarget, "target angle");
  command.add('L', doLimit, "voltage limit");
  command.add('V', doLimit, "movement velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target position [rad]");
  _delay(1000);
}

void loop() {
  // open  loop angle movements
  // using motor.voltage_limit and motor.velocity_limit
  motor.move(target_position);
  
  // user communication
  command.run();
}