/**

   Torque control example using current control loop.

   This example is based on the Arduino SimpleFOCShield V2.0
   The current is measured inline with the shunt resistor or R= 0.01 Ohm, and amplification gain og G = 50
*/
#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 10, 11, 8);

// encoder instance
Encoder encoder = Encoder(2, 3, 500);
// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// current sensor
// InlineCurrentSense(shunt_resistor, gain, pinA, pinB, (optional pinC))
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, A0, A2);

void setup() {

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 50000;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set motion control loop to be used 
  // ControlType::torque, ControlType::velocity or ControlType::angle
  motor.controller = ControlType::velocity;

  // foc currnet control parameters (all default)
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I= 1000;
  motor.PID_current_d.P= 5;
  motor.PID_current_d.I = 1000;
  motor.LPF_current_q.Tf = 0.002; // 1ms default
  motor.LPF_current_d.Tf = 0.002; // 1ms default
  
  // motion control parameters
  motor.PID_velocity.P = 0.05;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.005;
  motor.current_limit = 0.5; // 0.5 Amps limit

  _delay(1000);
  // initialise the current sensing
  current_sense.init();
  // linking the current sensing
  motor.linkCurrentSense(&current_sense);
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // use monitoring with serial
  Serial.begin(250000);

  Serial.println("Motor ready.");
}


// target voltage to be set to the motor
float target_angle = 0;

// counting variable for user display
long t = 0;

  void loop() {

  // set the phase voltages Ud and Uq
  motor.loopFOC();
  motor.move(target_angle);
  
  // show user the currents
  if (!t) {
    Serial.print(motor.current_measured.q*1000); // mAmps
    Serial.print("\t");
    Serial.println(motor.current_measured.d*1000); // mAmps
  }
  t > 20 ? t = 0 : t++;

  // communicate with the user
  serialReceiveUserCommand();
}


// utility function enabling serial communication with the user to set the target values
// this function can be implemented in serialEvent function as well
void serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {

      // change the motor target
      target_angle = received_chars.toFloat();
      Serial.print("Target angle [radian]: ");
      Serial.println(target_angle);

      // reset the command buffer
      received_chars = "";
    }
  }
}