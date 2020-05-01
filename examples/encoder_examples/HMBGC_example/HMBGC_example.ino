#include <SimpleFOC.h>
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>


//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 10, 11, 11);
//  Encoder(int encA, int encB , int cpr, int index)
//  - encA, encB    - encoder A and B pins
//  - ppr           - impulses per rotation  (cpr=ppr*4)
//  - index pin     - (optional input)
Encoder encoder = Encoder(A0, A1, 8192);
// interrupt ruotine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// encoder interrupt init
PciListenerImp listenerA(encoder.pinA, doA);
PciListenerImp listenerB(encoder.pinB, doB);

void setup() {
  // debugging port
  Serial.begin(115200);

  // by default for HMBGC
  encoder.quadrature = Quadrature::ENABLE;

  // check if you need internal pullups
  // Pullup::EXTERN - external pullup added  - dafault
  // Pullup::INTERN - needs internal arduino pullup
  encoder.pullup = Pullup::EXTERN;

  // initialise encoder hardware
  encoder.init();

  // interrupt intitialisation
  PciManager.registerListener(&listenerA);
  PciManager.registerListener(&listenerB);

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;

  // set FOC loop to be used
  // ControlType::voltage
  // ControlType::velocity
  // ControlType::angle
  motor.controller = ControlType::velocity;

  // contoller configuration based on the controll type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
  motor.PI_velocity.P = 0.2;
  motor.PI_velocity.I = 20;
  //defualt voltage_power_supply/2
  motor.PI_velocity.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PI_velocity.voltage_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // use debugging with serial for motor init
  // comment out if not needed
  motor.useDebugging(Serial);

  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // intialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  Serial.println("Motor ready.");
  Serial.println("Set the target velocity using serial terminal:");
  _delay(1000);
}

float target_velocity=0;

void loop() {
  // iterative state calculation calculating angle
  // and setting FOC pahse voltage
  // the faster you run this funciton the better
  // in arduino loop it should have ~1kHz
  // the best would be to be in ~10kHz range
  motor.loopFOC();

  // 0.5 hertz sine weve
  //target_velocity = sin( micros()*1e-6 *2*M_PI * 0.5 );

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // this funciton can be run at much lower frequency than loopFOC funciton
  // it can go as low as ~50Hz
  motor.move(target_velocity);


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();
}

// Serial communication callback function
// gets the target value from the user
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
      target_velocity = inputString.toFloat();
      Serial.print("Tagret velocity: ");
      Serial.println(target_velocity);
      inputString = "";
    }
  }
}