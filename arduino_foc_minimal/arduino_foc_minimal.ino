#include "SimpleFOC.h"
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 10, 11, 14);

// // MagneticSensor(int cs, float _cpr, int _angle_register)
// //  cs              - SPI chip select pin 
// //  _cpr            - counts per revolution 
// // _angle_register  - (optional) angle read register - default 0x3FFF
// MagneticSensor AS5x4x = MagneticSensor(10, 16384, 0x3FFF);


Encoder encoder = Encoder(A0, A1, 2048);
// interrupt ruotine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// encoder interrupt init
PciListenerImp listenerA(encoder.pinA, doA);
PciListenerImp listenerB(encoder.pinB, doB);

void setup() { 
  // debugging port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  // AS5x4x.init();
  encoder.init();
  // encoder.enableInterrupts(doA,doB);

  // interrupt intitialisation
  PciManager.registerListener(&listenerA);
  PciManager.registerListener(&listenerB);

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;
  
  // set control loop type to be used
  // ControlType::voltage
  // ControlType::velocity
  // ControlType::angle
  motor.controller = ControlType::angle;

  // contoller configuration based on the controll type 
  // velocity PI controller parameters
  motor.PI_velocity.P = 0.2;
  motor.PI_velocity.I = 20;
  //defualt voltage_power_supply/2
  motor.PI_velocity.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PI_velocity.voltage_ramp = 1000;

  // velocity low pass filtering
  // default 10ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.0;

  // angle loop controller
  motor.P_angle.P = 3;
  motor.P_angle.velocity_limit = 10;


  // link the motor to the sensor
  // motor.linkSensor(&AS5x4x);
  motor.linkSensor(&encoder);

  // use debugging with serial for motor init
  // comment out if not needed
  motor.useDebugging(Serial);

  // intialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  Serial.println("Motor ready.\n");
  Serial.println("Update all the PI contorller paramters from the serial temrinal:");
  Serial.println("- Type P100.2 to you the PI_velocity.P in 100.2");
  Serial.println("- Type I72.32 to you the PI_velocity.I in 72.32\n");
  Serial.println("Update the time constant of the velocity filter:");
  Serial.println("- Type F0.03 to you the LPF_velocity.Tf in 0.03\n");
  Serial.println("Check the loop executoion time (average):");
  Serial.println("- Type T\n");
  _delay(1000);

}

// target velocity variable
float target_velocity = 0;
// loop stats variables
unsigned long  t = 0;
long timestamp = _micros();

void loop() {
  // iterative setting FOC pahse voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  motor.move(target_velocity);

  // keep track of loop number
  t++;
}

// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void motor_monitor() {
  switch (motor.controller) {
    case ControlType::velocity:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.print(motor.shaft_velocity_sp);
      Serial.print("\t");
      Serial.println(motor.shaft_velocity);
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
      if(inputString.charAt(0) == 'P'){
        motor.PI_velocity.P = inputString.substring(1).toFloat();
        Serial.print("PI velocity P: ");
        Serial.print(motor.PI_velocity.P);
        Serial.print(",\t I: ");
        Serial.print(motor.PI_velocity.I);
        Serial.print(",\t Low passs filter Tf: ");
        Serial.println(motor.LPF_velocity.Tf);
      }else if(inputString.charAt(0) == 'I'){
        motor.PI_velocity.I = inputString.substring(1).toFloat();
        Serial.print("PI velocity P: ");
        Serial.print(motor.PI_velocity.P);
        Serial.print(",\t I: ");
        Serial.print(motor.PI_velocity.I);
        Serial.print(",\t Low passs filter Tf: ");
        Serial.println(motor.LPF_velocity.Tf);
      }else if(inputString.charAt(0) == 'F'){
        motor.LPF_velocity.Tf = inputString.substring(1).toFloat();
        Serial.print("PI velocity P: ");
        Serial.print(motor.PI_velocity.P);
        Serial.print(",\t I: ");
        Serial.print(motor.PI_velocity.I);
        Serial.print(",\t Low passs filter Tf: ");
        Serial.println(motor.LPF_velocity.Tf);
      }else if(inputString.charAt(0) == 'T'){
        Serial.print("Average loop time is (microseconds): ");
        Serial.println((_micros() - timestamp)/t);
        t = 0;
        timestamp = _micros();
      }else if(inputString.charAt(0) == 'C'){
        Serial.print("Contoller type: ");
        int cnt = inputString.substring(1).toFloat();
        if(cnt == 0){
          Serial.println("angle!");
          motor.controller = ControlType::angle;
        }else if(cnt == 1){
          Serial.println("velocity!");
          motor.controller = ControlType::velocity;
        }else if(cnt == 2){
          Serial.println("volatge!");
          motor.controller = ControlType::voltage;
        }
        Serial.println();
        t = 0;
        timestamp = _micros();
      }else{
        target_velocity = inputString.toFloat();
        Serial.print("Tagret Velocity: ");
        Serial.println(target_velocity);
        inputString = "";
      }
      inputString = "";
    }
  }
}
