#include <SimpleFOC.h>

// Only pins 2 and 3 are supported
#define arduinoInt1 2             // Arduino UNO interrupt 0
#define arduinoInt2 3             // Arduino UNO interrupt 1

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 5, 6, 11, 8);
//  Encoder(int encA, int encB , int cpr, int index)
//  - encA, encB    - encoder A and B pins
//  - ppr           - impulses per rotation  (cpr=ppr*4)
//  - index pin     - (optional input) 
Encoder encoder = Encoder(arduinoInt1, arduinoInt2, 8192);

// Interrupt routine intialisation
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
  // Pullup::EXTERN - external pullup added - default
  // Pullup::INTERN - needs internal arduino pullup
  encoder.pullup = Pullup::EXTERN;

  // initialise encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;
  
  // choose FOC algorithm to be used:
  // FOCModulationType::SinePWM  (default)
  // FOCModulationType::SpaceVectorPWM
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // index search velocity - default 1rad/s
  motor.index_search_velocity = 1;
  // index search PI contoller parameters
  // default P=1, I=10
  motor.PI_velocity_index_search.P = 1;
  motor.PI_velocity_index_search.I = 20;
  //motor.PI_velocity_index_search.voltage_limit = 3;
  // jerk control using voltage voltage ramp
  // default value is 100
  motor.PI_velocity_index_search.voltage_ramp = 100;

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
  //default voltage_power_supply/2
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
  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();


  Serial.println("Motor ready.");
  _delay(1000);
}

// target velocity variable
float target_velocity = 3;
int t = 0;

void loop() {
  // iterative state calculation calculating angle
  // and setting FOC phase voltage
  // the faster you run this function the better
  // in arduino loop it should have ~1kHz
  // the best would be to be in ~10kHz range
  motor.loopFOC();

  // direction changing logic
  // change direction each 1000 loop passes
  target_velocity *= (t >= 1000) ? -1 : 1; 
  // loop passes counter
  t = (t >= 1000) ? 0 : t+1;


  // iterative function setting the outter loop target
  // velocity, position or voltage
  // this function can be run at much lower frequency than loopFOC function
  // it can go as low as ~50Hz
  motor.move(target_velocity);


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();
}


