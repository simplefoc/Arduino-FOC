#include "BLDCMotor.h"
#include "Encoder.h"

// Only pins 2 and 3 are supported
#define arduinoInt1 2             // Arduino UNO interrupt 0
#define arduinoInt2 3             // Arduino UNO interrupt 1

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 10, 11, 11, 8);
//  Encoder(int encA, int encB , int cpr, int index)
//  - encA, encB    - encoder A and B pins
//  - ppr           - impulses per rotation  (cpr=ppr*4)
//  - index pin     - (optional input)
Encoder encoder = Encoder(arduinoInt1, arduinoInt2, 8192, 4);
// interrupt ruotine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() { 
  // debugging port
  Serial.begin(115200);

  // check if you need internal pullups
  //  Quadrature::ENABLE - CPR = 4xPPR  - default
  //  Quadrature::DISABLE - CPR = PPR
  encoder.quadrature = Quadrature::ENABLE;

  // check if you need internal pullups
  // Pullup::EXTERN - external pullup added - dafault
  // Pullup::INTERN - needs internal arduino pullup
  encoder.pullup = Pullup::EXTERN;
  
  // initialise encoder hardware
  encoder.init(doA, doB);

  // set driver type
  //  DriverType::half_bridge - default
  //  DriverType::full_bridge    
  motor.driver = DriverType::full_bridge;

  // power supply voltage
  // default 12V
  motor.power_supply_voltage = 12;

  // set FOC loop to be used
  // ControlType::voltage
  // ControlType::velocity
  // ControlType::velocity_ultra_slow
  // ControlType::angle
  motor.controller = ControlType::angle;

  // contoller configuration based on the controll type 
  // velocity PI controller parameters
  // default K=1.0 Ti = 0.003
  motor.PI_velocity.K = 0.9;
  motor.PI_velocity.Ti = 0.007;
  motor.PI_velocity.u_limit = 4;

  // angle loop setting
  motor.P_angle.K = 20;
  motor.P_angle.velocity_limit = 5;


  // index search - default 3V
  motor.index_search_voltage = 4;

  // link the motor to the sensor
  motor.linkEncoder(&encoder);
  // intialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();


  Serial.println("Motor ready.");
  Serial.println("Input the new target value:");
  delay(1000);
}

// target velocity variable
float target = 0;
int t = 0;

void loop() {
  // iterative state calculation calculating angle
  // and setting FOC pahse voltage
  // the faster you run this funciton the better
  // in arduino loop it should have ~1kHz
  // the best would be to be in ~10kHz range
  motor.loopFOC();

  // // direction chnaging logic (comment out if you dont need it)
  // // change direction each 1000 loop passes
  // target *= (t >= 1000) ? -1 : 1; 
  // // loop passes counter
  // t = (t >= 1000) ? 0 : t+1;


  // iterative function setting the outter loop target
  // velocity, position or voltage
  // this funciton can be run at much lower frequency than loopFOC funciton
  // it can go as low as ~50Hz
  motor.move(target);


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor_monitor();
}

// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void motor_monitor() {
  switch (motor.controller) {
    case ControlType::velocity_ultra_slow:
    case ControlType::velocity:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.print(motor.shaft_velocity_sp);
      Serial.print("\t");
      Serial.print(motor.shaft_velocity);
      Serial.print("\t");
      Serial.println(motor.shaft_angle);
//      Serial.print("\t");
//      Serial.print(motor.Ua);
//      Serial.print("\t");
//      Serial.print(motor.Ub);
//      Serial.print("\t");
//      Serial.println(motor.Uc);
      break;
    case ControlType::angle:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.print(motor.shaft_angle_sp);
      Serial.print("\t");
      Serial.println(motor.shaft_angle);
      break;
    case ControlType::voltage:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.print(motor.shaft_angle);
      Serial.print("\t");
      Serial.println(motor.shaft_velocity);
//      Serial.print("\t");
//      Serial.print(motor.Ua);
//      Serial.print("\t");
//      Serial.print(motor.Ub);
//      Serial.print("\t");
//      Serial.println(motor.Uc);
      break;
  }
}

// Serial communication callback
void serialEvent() {
  // a string to hold incoming data
  static String inputString;
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline
    // end of input
    if (inChar == '\n') {
      target = inputString.toFloat();
      Serial.print("Tagret: ");
      Serial.println(target);
      inputString = "";
    }
  }
}


