#include "SimpleFOC.h"
// // software interrupt library
// #include <PciManager.h>
// #include <PciListenerImp.h>

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 10, 11, 11);

//Encoder sensor = Encoder(2, 3, 2048);
// interrupt ruotine intialisation
// void doA(){encoder.handleA();}
// void doB(){encoder.handleB();}

// encoder interrupt init
// PciListenerImp listenerA(encoder.pinA, doA);
// PciListenerImp listenerB(encoder.pinB, doB);


MagneticSensor sensor = MagneticSensor(10,16384);

void setup() { 
  // debugging port
  Serial.begin(115200);

  //as5047.init();

  // initialise magnetic sensor hardware
  sensor.init();
  //encoder.enableInterrupts(doA,doB);

  // // interrupt intitialisation
  // PciManager.registerListener(&listenerA);
  // PciManager.registerListener(&listenerB);

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
  motor.linkSensor(&sensor);
  // motor.linkSensor(&as5047);


  // use debugging with serial for motor init
  // comment out if not needed
  motor.useDebugging(Serial);

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
  Serial.println("Control loop type:");
  Serial.println("- C0 - angle control");
  Serial.println("- C1 - velocity control");
  Serial.println("- C2 - voltage control\n");
  Serial.println("Initial parameters:");
  Serial.print("PI velocity P: ");
  Serial.print(motor.PI_velocity.P);
  Serial.print(",\t I: ");
  Serial.print(motor.PI_velocity.I);
  Serial.print(",\t Low passs filter Tf: ");
  Serial.println(motor.LPF_velocity.Tf,4);
  
  _delay(1000);
}

// target velocity variable
float target = 0;
// loop stats variables
unsigned long  t = 0;
long timestamp = _micros();

void loop() {
  // iterative setting FOC pahse voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  motor.move(target);

  // keep track of loop number
  t++;
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
        target = inputString.toFloat();
        Serial.print("Target : ");
        Serial.println(target);
        inputString = "";
      }
      inputString = "";
    }
  }
}
