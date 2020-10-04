#include "StepperMotor.h"

// StepperMotor( int phA, int phB, int phC, int pp, int cpr, int en)
// - ph1A, ph1B    - motor phase 1 pwm pins
// - ph2A, ph2B    - motor phase 2 pwm pins
// - pp            - pole pair number
// - enable pin    - (optional input)
StepperMotor::StepperMotor(int ph1A, int ph1B, int ph2A, int ph2B, int pp, int en1, int en2)
{
  // Pin initialization
  pwm1A = ph1A;
  pwm1B = ph1B;
  pwm2A = ph2A;
  pwm2B = ph2B;
  pole_pairs = pp;

  // enable_pin pin
  enable_pin1 = en1;
  enable_pin2 = en2;

  // Power supply voltage
  voltage_power_supply = DEF_POWER_SUPPLY;

  // Velocity loop config
  // PI controller constant
  PID_velocity.P = DEF_PID_VEL_P;
  PID_velocity.I = DEF_PID_VEL_I;
  PID_velocity.D = DEF_PID_VEL_D;
  PID_velocity.output_ramp = DEF_PID_VEL_U_RAMP;
  PID_velocity.timestamp = _micros();
  PID_velocity.integral_prev = 0;
  PID_velocity.output_prev = 0;
  PID_velocity.tracking_error_prev = 0;

  // velocity low pass filter 
  LPF_velocity.Tf = DEF_VEL_FILTER_Tf;
  LPF_velocity.timestamp = _micros();
  LPF_velocity.prev = 0;  

  // position loop config
  // P controller constant
  P_angle.P = DEF_P_ANGLE_P;

  // maximum angular velocity to be used for positioning 
  velocity_limit = DEF_VEL_LIM;
  // maximum voltage to be set to the motor
  voltage_limit = voltage_power_supply;

  // index search velocity
  velocity_index_search = DEF_INDEX_SEARCH_TARGET_VELOCITY;
  // sensor and motor align voltage
  voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;

  // electric angle of the zero angle
  zero_electric_angle = 0;

  // default modulation is SinePWM
  foc_modulation = FOCModulationType::SinePWM;

  // default target value
  target = 0;
  
  //monitor_port 
  monitor_port = nullptr;
  //sensor 
  sensor = nullptr;
}

// init hardware pins   
void StepperMotor::init() {
  if(monitor_port) monitor_port->println("MOT: Init pins.");
  // PWM pins
  pinMode(pwm1A, OUTPUT);
  pinMode(pwm1B, OUTPUT);
  pinMode(pwm2A, OUTPUT);
  pinMode(pwm2B, OUTPUT);
  if ( enable_pin1 != NOT_SET ) pinMode(enable_pin1, OUTPUT);
  if ( enable_pin2 != NOT_SET ) pinMode(enable_pin2, OUTPUT);

  if(monitor_port) monitor_port->println("MOT: PWM config.");
  // Increase PWM frequency
  // make silent
  _setPwmFrequency(pwm1A, pwm1B, pwm2A, pwm2B);
  
  // sanity check for the voltage limit configuration
  if(voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;
  // constrain voltage for sensor alignment
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  _delay(500);
  // enable motor
  if(monitor_port) monitor_port->println("MOT: Enable.");
  enable();
  _delay(500);
  
}


// disable motor driver
void StepperMotor::disable()
{
  // disable the driver - if enable_pin pin available
  if ( enable_pin1 != NOT_SET ) digitalWrite(enable_pin1, LOW);
  if ( enable_pin2 != NOT_SET ) digitalWrite(enable_pin2, LOW);
  // set zero to PWM
  setPwm(0, 0);
}
// enable motor driver
void StepperMotor::enable()
{
  // set zero to PWM
  setPwm(0, 0);
  // enable_pin the driver - if enable_pin pin available
  if ( enable_pin1 != NOT_SET ) digitalWrite(enable_pin1, HIGH);
  if ( enable_pin2 != NOT_SET ) digitalWrite(enable_pin2, HIGH);

}

void StepperMotor::linkSensor(Sensor* _sensor) {
  sensor = _sensor;
}


// Encoder alignment to electrical 0 angle
int StepperMotor::alignSensor() {
  if(monitor_port) monitor_port->println("MOT: Align sensor.");
  // align the electrical phases of the motor and sensor
  // set angle -90 degrees 

  float start_angle = shaftAngle();
  for (int i = 0; i <=5; i++ ) {
    float angle = _3PI_2 + _2PI * i / 6.0;
    setPhaseVoltage(voltage_sensor_align,  angle);
    _delay(200);
  }
  float mid_angle = shaftAngle();
  for (int i = 5; i >=0; i-- ) {
    float angle = _3PI_2 + _2PI * i / 6.0;
    setPhaseVoltage(voltage_sensor_align,  angle);
    _delay(200);
  }
  if (mid_angle < start_angle) {
    if(monitor_port) monitor_port->println("MOT: natural_direction==CCW");
    sensor->natural_direction = Direction::CCW;
  } else if (mid_angle == start_angle) {
    if(monitor_port) monitor_port->println("MOT: Sensor failed to notice movement");
  }

  // let the motor stabilize for 2 sec
  _delay(2000);
  // set sensor to zero
  sensor->initRelativeZero();
  _delay(500);
  setPhaseVoltage(0,0);
  _delay(200);

  // find the index if available
  int exit_flag = absoluteZeroAlign();
  _delay(500);
  if(monitor_port){
    if(exit_flag< 0 ) monitor_port->println("MOT: Error: Not found!");
    if(exit_flag> 0 ) monitor_port->println("MOT: Success!");
    else  monitor_port->println("MOT: Not available!");
  }
  return exit_flag;
}


// Encoder alignment the absolute zero angle 
// - to the index
int StepperMotor::absoluteZeroAlign() {

  if(monitor_port) monitor_port->println("MOT: Absolute zero align.");
    // if no absolute zero return
  if(!sensor->hasAbsoluteZero()) return 0;
  

  if(monitor_port && sensor->needsAbsoluteZeroSearch()) monitor_port->println("MOT: Searching...");
  // search the absolute zero with small velocity
  while(sensor->needsAbsoluteZeroSearch() && shaft_angle < _2PI){
    loopFOC();   
    voltage_q = velocityPID(velocity_index_search - shaftVelocity());
  }
  voltage_q = 0;
  // disable motor
  setPhaseVoltage(0,0);

  // align absolute zero if it has been found
  if(!sensor->needsAbsoluteZeroSearch()){
    // align the sensor with the absolute zero
    float zero_offset = sensor->initAbsoluteZero();
    // remember zero electric angle
    zero_electric_angle = normalizeAngle(electricAngle(zero_offset));
  }
  // return bool if zero found
  return !sensor->needsAbsoluteZeroSearch() ? 1 : -1;
}

/**
	State calculation methods
*/
// shaft angle calculation
float StepperMotor::shaftAngle() {
  // if no sensor linked return 0
  if(!sensor) return 0;
  return sensor->getAngle();
}
// shaft velocity calculation
float StepperMotor::shaftVelocity() {
  // if no sensor linked return 0
  if(!sensor) return 0;
  return lowPassFilter(sensor->getVelocity(), LPF_velocity);
}
// Electrical angle calculation
float StepperMotor::electricAngle(float shaftAngle) {
  //return normalizeAngle(shaftAngle * pole_pairs);
  return (shaftAngle * pole_pairs);
}

/**
  FOC functions
*/
// FOC initialization function
int  StepperMotor::initFOC( float zero_electric_offset, Direction sensor_direction ) {
  int exit_flag = 1;
  // align motor if necessary
  // alignment necessary for encoders!
  if(zero_electric_offset != NOT_SET){
    // abosolute zero offset provided - no need to align
    zero_electric_angle = zero_electric_offset;
    // set the sensor direction - default CW
    sensor->natural_direction = sensor_direction;
  }else{
    // sensor and motor alignment
    _delay(500);
    exit_flag = alignSensor();
    _delay(500);
    }
  if(monitor_port) monitor_port->println("MOT: Motor ready.");

  return exit_flag;
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void StepperMotor::loopFOC() {
  // shaft angle 
  shaft_angle = shaftAngle();
  // set the phase voltage - FOC heart function :) 
  setPhaseVoltage(voltage_q, electricAngle(shaft_angle));
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or voltage loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void StepperMotor::move(float new_target) {
  // set internal target variable
  if( new_target != NOT_SET ) target = new_target;
  // get angular velocity
  shaft_velocity = shaftVelocity();
  // choose control loop
  switch (controller) {
    case ControlType::voltage:
      voltage_q =  target;
      break;
    case ControlType::angle:
      // angle set point
      // include angle loop
      shaft_angle_sp = target;
      shaft_velocity_sp = positionP( shaft_angle_sp - shaft_angle );
      voltage_q = velocityPID(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity:
      // velocity set point
      // include velocity loop
      shaft_velocity_sp = target;
      voltage_q = velocityPID(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity_openloop:
      // velocity control in open loop
      // loopFOC should not be called
      shaft_velocity_sp = target;
      velocityOpenloop(shaft_velocity_sp);
      break;
    case ControlType::angle_openloop:
      // angle control in open loop
      // loopFOC should not be called
      shaft_angle_sp = target;
      angleOpenloop(shaft_angle_sp);
      break;
  }
}


// Method using FOC to set Uq to the motor at the optimal angle
// Function implementingSine PWM algorithms
// - space vector not implemented yet
// 
// Function using sine approximation
// regular sin + cos ~300us    (no memory usaage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void StepperMotor::setPhaseVoltage(float Uq, float angle_el) {
  // Sinusoidal PWM modulation 
  // Inverse Park transformation

  // angle normalization in between 0 and 2pi
  // only necessary if using _sin and _cos - approximation functions
  angle_el = normalizeAngle(angle_el + zero_electric_angle);
  // Inverse park transform
  Ualpha =  -_sin(angle_el) * Uq;  // -sin(angle) * Uq;
  Ubeta =  _cos(angle_el) * Uq;    //  cos(angle) * Uq;
  // set the voltages in hardware
  setPwm(Ualpha,Ubeta);
}



// Set voltage to the pwm pin
void StepperMotor::setPwm(float Ualpha, float Ubeta) {  
  float duty_cycle1A = 0,duty_cycle1B,duty_cycle2A,duty_cycle2B;
  // hardware specific writing
  if( Ualpha > 0 ){
    duty_cycle1A = 0;
    duty_cycle1B = constrain(abs(Ualpha)/voltage_power_supply,0,1);
  } else {
    duty_cycle1B = 0;
    duty_cycle1A = constrain(abs(Ualpha)/voltage_power_supply,0,1);
  }
  
  if( Ubeta > 0 ){
    duty_cycle2A = 0;
    duty_cycle2B = constrain(abs(Ubeta)/voltage_power_supply,0,1);
  } else {
    duty_cycle2B = 0;
    duty_cycle2A = constrain(abs(Ubeta)/voltage_power_supply,0,1);
  }
  // write to hardware
  _writeDutyCycle(duty_cycle1A, duty_cycle1B, duty_cycle2A, duty_cycle2B, pwm1A, pwm1B, pwm2A, pwm2B);
}

/**
	Utility functions
*/
// normalizing radian angle to [0,2PI]
float StepperMotor::normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

// low pass filter function
// - input  -singal to be filtered
// - lpf    -LPF_s structure with filter parameters 
float StepperMotor::lowPassFilter(float input, LPF_s& lpf){
  unsigned long now_us = _micros();
  float Ts = (now_us - lpf.timestamp) * 1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

  // calculate the filtering 
  float alpha = lpf.Tf/(lpf.Tf + Ts);
  float out = alpha*lpf.prev + (1-alpha)*input;

  // save the variables
  lpf.prev = out;
  lpf.timestamp = now_us;
  return out;
}


/**
	Motor control functions
*/
// PID controller function
float StepperMotor::controllerPID(float tracking_error, PID_s& cont){
  // calculate the time from the last call
  unsigned long now_us = _micros();
  float Ts = (now_us - cont.timestamp) * 1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

  // u(s) = (P + I/s + Ds)e(s)
  // Discrete implementations
  // proportional part 
  // u_p  = P *e(k)
  float proportional = cont.P * tracking_error;
  // Tustin transform of the integral part
  // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
  float integral = cont.integral_prev + cont.I*Ts*0.5*(tracking_error + cont.tracking_error_prev);
  // antiwindup - limit the output voltage_q
  integral = constrain(integral, -voltage_limit, voltage_limit);
  // Discrete derivation
  // u_dk = D(ek - ek_1)/Ts
  float derivative = cont.D*(tracking_error - cont.tracking_error_prev)/Ts;
  // sum all the components
  float voltage = proportional + integral + derivative;

  // antiwindup - limit the output voltage_q
  voltage = constrain(voltage, -voltage_limit, voltage_limit);

  // limit the acceleration by ramping the the voltage
  float d_voltage = voltage - cont.output_prev;
  if (abs(d_voltage)/Ts > cont.output_ramp) voltage = d_voltage > 0 ? cont.output_prev + cont.output_ramp*Ts : cont.output_prev - cont.output_ramp*Ts;

  // saving for the next pass
  cont.integral_prev = integral;
  cont.output_prev = voltage;
  cont.tracking_error_prev = tracking_error;
  cont.timestamp = now_us;
  return voltage;
}
// velocity control loop PI controller
float StepperMotor::velocityPID(float tracking_error) {
  return controllerPID(tracking_error, PID_velocity);
}

// P controller for position control loop
float StepperMotor::positionP(float ek) {
  // calculate the target velocity from the position error
  float velocity_target = P_angle.P * ek;
  // constrain velocity target value
  velocity_target = constrain(velocity_target, -velocity_limit, velocity_limit);
  return velocity_target;
}

// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
void StepperMotor::velocityOpenloop(float target_velocity){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6;

  // calculate the necessary angle to achieve target velocity
  shaft_angle += target_velocity*Ts; 

  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(voltage_limit, electricAngle(shaft_angle));

  // save timestamp for next call
  open_loop_timestamp = now_us;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
void StepperMotor::angleOpenloop(float target_angle){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6;
  
  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts))
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts; 
  else
    shaft_angle = target_angle;
  
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(voltage_limit, electricAngle(shaft_angle));

  // save timestamp for next call
  open_loop_timestamp = now_us;
}

/**
 *  Monitoring functions
 */
// function implementing the monitor_port setter
void StepperMotor::useMonitoring(Print &print){
  monitor_port = &print; //operate on the address of print
  if(monitor_port ) monitor_port->println("MOT: Monitor enabled!");
}
// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void StepperMotor::monitor() {
  if(!monitor_port) return;
  switch (controller) {
    case ControlType::velocity_openloop:
    case ControlType::velocity:
      monitor_port->print(voltage_q);
      monitor_port->print("\t");
      monitor_port->print(shaft_velocity_sp);
      monitor_port->print("\t");
      monitor_port->println(shaft_velocity);
      break;
    case ControlType::angle_openloop:
    case ControlType::angle:
      monitor_port->print(voltage_q);
      monitor_port->print("\t");
      monitor_port->print(shaft_angle_sp);
      monitor_port->print("\t");
      monitor_port->println(shaft_angle);
      break;
    case ControlType::voltage:
      monitor_port->print(voltage_q);
      monitor_port->print("\t");
      monitor_port->print(shaft_angle);
      monitor_port->print("\t");
      monitor_port->println(shaft_velocity);
      break;
  }
}

int StepperMotor::command(String user_command) {
  // error flag
  int errorFlag = 1;
  // if empty string
  if(user_command.length() < 1) return errorFlag;

  // parse command letter
  char cmd = user_command.charAt(0);
  // check if get command
  char GET = user_command.charAt(1) == '\n';
  // parse command values
  float value = user_command.substring(1).toFloat();

  // a bit of optimisation of variable memory for Arduino UNO (atmega328)
  switch(cmd){
    case 'P':      // velocity P gain change
    case 'I':      // velocity I gain change
    case 'D':      // velocity D gain change
    case 'R':      // velocity voltage ramp change
      if(monitor_port) monitor_port->print(" PID velocity| ");
      break;
    case 'F':      // velocity Tf low pass filter change
      if(monitor_port) monitor_port->print(" LPF velocity| ");
      break;
    case 'K':      // angle loop gain P change
      if(monitor_port) monitor_port->print(" P angle| ");
      break;
    case 'L':      // velocity voltage limit change
    case 'N':      // angle loop gain velocity_limit change
      if(monitor_port) monitor_port->print(" Limits| ");
      break;

  }

  // apply the the command
  switch(cmd){
    case 'P':      // velocity P gain change
      if(monitor_port) monitor_port->print("P: ");
      if(!GET) PID_velocity.P = value;
      if(monitor_port) monitor_port->println(PID_velocity.P);
      break;
    case 'I':      // velocity I gain change
      if(monitor_port) monitor_port->print("I: ");
      if(!GET) PID_velocity.I = value;
      if(monitor_port) monitor_port->println(PID_velocity.I);
      break;
    case 'D':      // velocity D gain change
      if(monitor_port) monitor_port->print("D: ");
      if(!GET) PID_velocity.D = value;
      if(monitor_port) monitor_port->println(PID_velocity.D);
      break;
    case 'R':      // velocity voltage ramp change
      if(monitor_port) monitor_port->print("volt_ramp: ");
      if(!GET) PID_velocity.output_ramp = value;
      if(monitor_port) monitor_port->println(PID_velocity.output_ramp);
      break;
    case 'L':      // velocity voltage limit change
      if(monitor_port) monitor_port->print("volt_limit: ");
      if(!GET)voltage_limit = value;
      if(monitor_port) monitor_port->println(voltage_limit);
      break;
    case 'F':      // velocity Tf low pass filter change
      if(monitor_port) monitor_port->print("Tf: ");
      if(!GET) LPF_velocity.Tf = value;
      if(monitor_port) monitor_port->println(LPF_velocity.Tf);
      break;
    case 'K':      // angle loop gain P change
      if(monitor_port) monitor_port->print(" P: ");
      if(!GET) P_angle.P = value;
      if(monitor_port) monitor_port->println(P_angle.P);
      break;
    case 'N':      // angle loop gain velocity_limit change
      if(monitor_port) monitor_port->print("vel_limit: ");
      if(!GET) velocity_limit = value;
      if(monitor_port) monitor_port->println(velocity_limit);
      break;
    case 'C':
      // change control type
      if(monitor_port) monitor_port->print("Control: ");
      
      if(GET){ // if get command
        switch(controller){
          case ControlType::voltage:
            if(monitor_port) monitor_port->println("voltage");
            break;
          case ControlType::velocity:
            if(monitor_port) monitor_port->println("velocity");
            break;
          case ControlType::angle:
            if(monitor_port) monitor_port->println("angle");
            break;
          default:
            if(monitor_port) monitor_port->println("open loop");
        }
      }else{ // if set command
        switch((int)value){
          case 0:
            if(monitor_port) monitor_port->println("voltage");
            controller = ControlType::voltage;
            break;
          case 1:
            if(monitor_port) monitor_port->println("velocity");
            controller = ControlType::velocity;
            break;
          case 2:
            if(monitor_port) monitor_port->println("angle");
            controller = ControlType::angle;
            break;
          default: // not valid command
            errorFlag = 0;
        }
      }
      break;
    case 'V':     // get current values of the state variables
        switch((int)value){
          case 0: // get voltage
            if(monitor_port) monitor_port->print("Uq: ");
            if(monitor_port) monitor_port->println(voltage_q);
            break;
          case 1: // get velocity
            if(monitor_port) monitor_port->print("Velocity: ");
            if(monitor_port) monitor_port->println(shaft_velocity);
            break;
          case 2: // get angle
            if(monitor_port) monitor_port->print("Angle: ");
            if(monitor_port) monitor_port->println(shaft_angle);
            break;
          case 3: // get angle
            if(monitor_port) monitor_port->print("Target: ");
            if(monitor_port) monitor_port->println(target);
            break;
          default: // not valid command
            errorFlag = 0;
        }
      break;
    default:  // target change
      if(monitor_port) monitor_port->print("Target : ");
      target = user_command.toFloat();
      if(monitor_port) monitor_port->println(target);
  }
  // return 0 if error and 1 if ok
  return errorFlag;
}
