#include <SimpleFOC.h>

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
Encoder encoder = Encoder(arduinoInt1, arduinoInt2, 8192);

// Interrupt rutine intialisation
// channel A and B callbacks
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
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;

  // set control loop type to be used
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

  Serial.println("\n\n");
  Serial.println("PI controller parameters change:");
  Serial.println("- P value : Prefix P (ex. P0.1)");
  Serial.println("- I value : Prefix I (ex. I0.1)\n");
  Serial.println("Velocity filter:");
  Serial.println("- Tf value : Prefix F (ex. F0.001)\n");
  Serial.println("Average loop execution time:");
  Serial.println("- Type T\n");
  
  Serial.println("Initial parameters:");
  Serial.print("PI velocity P: ");
  Serial.print(motor.PI_velocity.P);
  Serial.print(",\t I: ");
  Serial.print(motor.PI_velocity.I);
  Serial.print(",\t Low passs filter Tf: ");
  Serial.println(motor.LPF_velocity.Tf,4);
  _delay(1000);
}

// velocity set point variable
float target_velocity = 0;
// loop stats variables
unsigned long  t = 0;
long timestamp = _micros();

void loop() {
  // iterative state calculation calculating angle
  // and setting FOC pahse voltage
  // the faster you run this funciton the better
  // in arduino loop it should have ~1kHz
  // the best would be to be in ~10kHz range
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // this funciton can be run at much lower frequency than loopFOC funciton
  // it can go as low as ~50Hz
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // keep track of loop number
  t++;
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
       if(inputString.charAt(0) == 'P'){
        motor.PI_velocity.P = inputString.substring(1).toFloat();
        Serial.print("PI velocity P: ");
        Serial.print(motor.PI_velocity.P);
        Serial.print(",\t I: ");
        Serial.print(motor.PI_velocity.I);
        Serial.print(",\t Low passs filter Tf: ");
        Serial.println(motor.LPF_velocity.Tf,4);
      }else if(inputString.charAt(0) == 'I'){
        motor.PI_velocity.I = inputString.substring(1).toFloat();
        Serial.print("PI velocity P: ");
        Serial.print(motor.PI_velocity.P);
        Serial.print(",\t I: ");
        Serial.print(motor.PI_velocity.I);
        Serial.print(",\t Low passs filter Tf: ");
        Serial.println(motor.LPF_velocity.Tf,4);
      }else if(inputString.charAt(0) == 'F'){
        motor.LPF_velocity.Tf = inputString.substring(1).toFloat();
        Serial.print("PI velocity P: ");
        Serial.print(motor.PI_velocity.P);
        Serial.print(",\t I: ");
        Serial.print(motor.PI_velocity.I);
        Serial.print(",\t Low passs filter Tf: ");
        Serial.println(motor.LPF_velocity.Tf,4);
      }else if(inputString.charAt(0) == 'T'){
        Serial.print("Average loop time is (microseconds): ");
        Serial.println((_micros() - timestamp)/t);
        t = 0;
        timestamp = _micros();
      }else{
        target_velocity = inputString.toFloat();
        Serial.print("Target velocity: ");
        Serial.println(target_velocity);
        inputString = "";
      }
      inputString = "";
    }
  }
}
