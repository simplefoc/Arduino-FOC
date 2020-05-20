#include <SimpleFOC.h>

// Only pins 2 and 3 are supported
#define arduinoInt1 2             // Arduino UNO interrupt 0
#define arduinoInt2 3             // Arduino UNO interrupt 1

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 5, 6, 11, 8);

// MagneticSensor(int cs, float _cpr, int _angle_register)
//  cs              - SPI chip select pin 
//  _cpr            - counts per revolution 
// _angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensor AS5x4x = MagneticSensor(10, 16384, 0x3FFF);

void setup() {
  // debugging port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  AS5x4x.init();

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;
  
  // choose FOC algorithm to be used:
  // FOCModulationType::SinePWM  (default)
  // FOCModulationType::SpaceVectorPWM
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
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
  //dfefault voltage_power_supply/2
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
  motor.linkSensor(&AS5x4x);
  // initialize motor
  motor.init();
  // align sensor and start FOC
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


