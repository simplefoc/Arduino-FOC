#include "BLDCMotor.h"

// BLDCMotor( int phA, int phB, int phC, int pp, int cpr, int en)
// - phA, phB, phC - motor A,B,C phase pwm pins
// - pp            - pole pair number
// - cpr           - counts per rotation number (cpm=ppm*4)
// - enable pin    - (optional input)
BLDCMotor::BLDCMotor(int phA, int phB, int phC, int pp, int en)
{
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;
  pole_pairs = pp;

  // enable_pin pin
  enable_pin = en;

  // Power supply voltage
  voltage_power_supply = DEF_POWER_SUPPLY;

  // Velocity loop config
  // PI controller constant
  PI_velocity.P = DEF_PI_VEL_P;
  PI_velocity.I = DEF_PI_VEL_I;
  PI_velocity.timestamp = _micros();
  PI_velocity.voltage_limit = voltage_power_supply;
  PI_velocity.voltage_ramp = DEF_PI_VEL_U_RAMP;
  PI_velocity.voltage_prev = 0;
  PI_velocity.tracking_error_prev = 0;

  // velocity low pass filter 
  LPF_velocity.Tf = DEF_VEL_FILTER_Tf;
  LPF_velocity.timestamp = _micros();
  LPF_velocity.prev = 0;  

  // position loop config
  // P controller constant
  P_angle.P = DEF_P_ANGLE_P;
  // maximum angular velocity to be used for positioning 
  P_angle.velocity_limit = DEF_P_ANGLE_VEL_LIM;

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
}

// init hardware pins   
void BLDCMotor::init() {
  if(monitor_port) monitor_port->println("MONITOR: Initialize the motor pins.");
  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  if(hasEnable()) pinMode(enable_pin, OUTPUT);

  if(monitor_port) monitor_port->println("MONITOR: Set high frequency PWM.");
  // Increase PWM frequency to 32 kHz
  // make silent
  setPwmFrequency(pwmA);
  setPwmFrequency(pwmB);
  setPwmFrequency(pwmC);

  // sanity check for the voltage limit configuration
  if(PI_velocity.voltage_limit > voltage_power_supply) PI_velocity.voltage_limit =  voltage_power_supply;

  _delay(500);
  // enable motor
  if(monitor_port) monitor_port->println("MONITOR: Enabling motor.");
  enable();
  _delay(500);
  
}


// disable motor driver
void BLDCMotor::disable()
{
  // disable the driver - if enable_pin pin available
  if (hasEnable()) digitalWrite(enable_pin, LOW);
  // set zero to PWM
  setPwm(pwmA, 0);
  setPwm(pwmB, 0);
  setPwm(pwmC, 0);
}
// enable motor driver
void BLDCMotor::enable()
{
  // set zero to PWM
  setPwm(pwmA, 0);
  setPwm(pwmB, 0);
  setPwm(pwmC, 0);
  // enable_pin the driver - if enable_pin pin available
  if (hasEnable()) digitalWrite(enable_pin, HIGH);

}

void BLDCMotor::linkSensor(Sensor* _sensor) {
  sensor = _sensor;
}


// Encoder alignment to electrical 0 angle
int BLDCMotor::alignSensor() {
  if(monitor_port) monitor_port->println("MONITOR: Align the sensor's and motor electrical 0 angle.");
  // align the electrical phases of the motor and sensor
  // set angle -90 degrees 
  setPhaseVoltage(voltage_sensor_align, _3PI_2);
  // let the motor stabilize for 3 sec
  _delay(3000);
  // set sensor to zero
  sensor->initRelativeZero();
  _delay(500);
  setPhaseVoltage(0,0);
  _delay(200);

  // find the index if available
  int exit_flag = absoluteZeroAlign();
  _delay(500);
  if(monitor_port){
    if(exit_flag< 0 ) monitor_port->println("MONITOR: Error: Absolute zero not found!");
    if(exit_flag> 0 ) monitor_port->println("MONITOR: Success: Absolute zero found!");
    else  monitor_port->println("MONITOR: Absolute zero not available!");
  }
  return exit_flag;
}


// Encoder alignment the absolute zero angle 
// - to the index
int BLDCMotor::absoluteZeroAlign() {
  // if no absolute zero return
  if(!sensor->hasAbsoluteZero()) return 0;
  
  if(monitor_port) monitor_port->println("MONITOR: Aligning the absolute zero.");

  if(monitor_port && sensor->needsAbsoluteZeroSearch()) monitor_port->println("MONITOR: Searching for absolute zero.");
  // search the absolute zero with small velocity
  while(sensor->needsAbsoluteZeroSearch() && shaft_angle < _2PI){
    loopFOC();   
    voltage_q = velocityPI(velocity_index_search - shaftVelocity());
  }
  voltage_q = 0;
  // disable motor
  setPhaseVoltage(0,0);

  // align absolute zero if it has been found
  if(!sensor->needsAbsoluteZeroSearch()){
    // align the sensor with the absolute zero
    float zero_offset = sensor->initAbsoluteZero();
    // remember zero electric angle
    zero_electric_angle = electricAngle(zero_offset);
  }
  // return bool if zero found
  return !sensor->needsAbsoluteZeroSearch() ? 1 : -1;
}

/**
	State calculation methods
*/
// shaft angle calculation
float BLDCMotor::shaftAngle() {
  return sensor->getAngle();
}
// shaft velocity calculation
float BLDCMotor::shaftVelocity() {
  float Ts = (_micros() - LPF_velocity.timestamp) * 1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 
  // calculate the filtering 
  float alpha = LPF_velocity.Tf/(LPF_velocity.Tf + Ts);
  float vel = alpha*LPF_velocity.prev + (1-alpha)*sensor->getVelocity();
  // save the variables
  LPF_velocity.prev = vel;
  LPF_velocity.timestamp = _micros();
  return vel;
}
// Electrical angle calculation
float BLDCMotor::electricAngle(float shaftAngle) {
  //return normalizeAngle(shaftAngle * pole_pairs);
  return (shaftAngle * pole_pairs);
}

/**
  FOC functions
*/
// FOC initialization function
int  BLDCMotor::initFOC() {
  // sensor and motor alignment
  _delay(500);
  int exit_flag = alignSensor();
  _delay(500);
  
  if(monitor_port) monitor_port->println("MONITOR: FOC init finished - motor ready.");

  return exit_flag;
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void BLDCMotor::loopFOC() {
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
void BLDCMotor::move(float new_target) {
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
      voltage_q = velocityPI(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity:
      // velocity set point
      // include velocity loop
      shaft_velocity_sp = target;
      voltage_q = velocityPI(shaft_velocity_sp - shaft_velocity);
      break;
  }
}


// Method using FOC to set Uq to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
// 
// Function using sine approximation
// regular sin + cos ~300us    (no memory usaage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void BLDCMotor::setPhaseVoltage(float Uq, float angle_el) {
  switch (foc_modulation)
  {
    case FOCModulationType::SinePWM :
      // Sinusoidal PWM modulation 
      // Inverse Park + Clarke transformation

      // angle normalization in between 0 and 2pi
      // only necessary if using _sin and _cos - approximation functions
      angle_el = normalizeAngle(angle_el + zero_electric_angle);
      // Inverse park transform
      Ualpha =  -_sin(angle_el) * Uq;  // -sin(angle) * Uq;
      Ubeta =  _cos(angle_el) * Uq;    //  cos(angle) * Uq;

      // Clarke transform
      Ua = Ualpha + voltage_power_supply/2;
      Ub = -0.5 * Ualpha  + _SQRT3_2 * Ubeta + voltage_power_supply/2;
      Uc = -0.5 * Ualpha - _SQRT3_2 * Ubeta + voltage_power_supply/2;
      break;

    case FOCModulationType::SpaceVectorPWM :
      // Nice video explaining the SpaceVectorModulation (SVPWM) algorithm 
      // https://www.youtube.com/watch?v=QMSWUMEAejg

      // if negative voltages change inverse the phase 
      // angle + 180degrees
      if(Uq < 0) angle_el += _PI;
      Uq = abs(Uq);

      // angle normalisation in between 0 and 2pi
      // only necessary if using _sin and _cos - approximation functions
      angle_el = normalizeAngle(angle_el + zero_electric_angle + _PI_2);

      // find the sector we are in currently
      int sector = floor(angle_el / _PI_3) + 1;
      // calculate the duty cycles
      float T1 = _SQRT3*_sin(sector*_PI_3 - angle_el);
      float T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3);
      // two versions possible 
      // centered around voltage_power_supply/2
      float T0 = 1 - T1 - T2;
      // pulled to 0 - better for low power supply voltage
      //T0 = 0;

      // calculate the duty cycles(times)
      float Ta,Tb,Tc; 
      switch(sector){
        case 1:
          Ta = T1 + T2 + T0/2;
          Tb = T2 + T0/2;
          Tc = T0/2;
          break;
        case 2:
          Ta = T1 +  T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T0/2;
          break;
        case 3:
          Ta = T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T2 + T0/2;
          break;
        case 4:
          Ta = T0/2;
          Tb = T1+ T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 5:
          Ta = T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 6:
          Ta = T1 + T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T0/2;
          break;
        default:
         // possible error state
          Ta = 0;
          Tb = 0;
          Tc = 0;
      }

      // calculate the phase voltages
      Ua = Ta*Uq;
      Ub = Tb*Uq;
      Uc = Tc*Uq;
      break;
  }
  
  // set the voltages in hardware
  setPwm(pwmA, Ua);
  setPwm(pwmB, Ub);
  setPwm(pwmC, Uc);
}




// Set voltage to the pwm pin
// - function a bit optimized to get better performance
void BLDCMotor::setPwm(int pinPwm, float U) {
  // max value
  float U_max = voltage_power_supply;
      
  // sets the voltage [0,12V(U_max)] to pwm [0,255]
  // - U_max you can set in header file - default 12V
  int U_pwm = 255.0 * U / U_max;

  // limit the values between 0 and 255
  U_pwm = (U_pwm < 0) ? 0 : (U_pwm >= 255) ? 255 : U_pwm;

  analogWrite(pinPwm, U_pwm);
}

/**
	Utility functions
*/
// normalizing radian angle to [0,2PI]
float BLDCMotor::normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}
// determining if the enable pin has been provided
int BLDCMotor::hasEnable(){
  return enable_pin != NOT_SET;
}


/**
	Motor control functions
*/
// PI controller function
float BLDCMotor::controllerPI(float tracking_error, PI_s& cont){
  float Ts = (_micros() - cont.timestamp) * 1e-6;

  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

  // u(s) = (P + I/s)e(s)
  // Tustin transform of the PI controller ( a bit optimized )
  // uk = uk_1  + (I*Ts/2 + P)*ek + (I*Ts/2 - P)*ek_1
  float tmp = cont.I*Ts*0.5;
  float voltage = cont.voltage_prev + (tmp + cont.P) * tracking_error + (tmp - cont.P) * cont.tracking_error_prev;

  // antiwindup - limit the output voltage_q
  if (abs(voltage) > cont.voltage_limit) voltage = voltage > 0 ? cont.voltage_limit : -cont.voltage_limit;
  // limit the acceleration by ramping the the voltage
  float d_voltage = voltage - cont.voltage_prev;
  if (abs(d_voltage)/Ts > cont.voltage_ramp) voltage = d_voltage > 0 ? cont.voltage_prev + cont.voltage_ramp*Ts : cont.voltage_prev - cont.voltage_ramp*Ts;


  cont.voltage_prev = voltage;
  cont.tracking_error_prev = tracking_error;
  cont.timestamp = _micros();
  return voltage;
}
// velocity control loop PI controller
float BLDCMotor::velocityPI(float tracking_error) {
  return controllerPI(tracking_error, PI_velocity);
}

// P controller for position control loop
float BLDCMotor::positionP(float ek) {
  // calculate the target velocity from the position error
  float velocity_target = P_angle.P * ek;
  // constrain velocity target value
  if (abs(velocity_target) > P_angle.velocity_limit) velocity_target = velocity_target > 0 ? P_angle.velocity_limit : -P_angle.velocity_limit;
  return velocity_target;
}

/**
 *  Monitoring functions
 */
// function implementing the monitor_port setter
void BLDCMotor::useMonitoring(Print &print){
  monitor_port = &print; //operate on the address of print
  if(monitor_port ) monitor_port->println("MONITOR: Serial monitor enabled!");
}
// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void BLDCMotor::monitor() {
  if(!monitor_port) return;
  switch (controller) {
    case ControlType::velocity:
      monitor_port->print(voltage_q);
      monitor_port->print("\t");
      monitor_port->print(shaft_velocity_sp);
      monitor_port->print("\t");
      monitor_port->println(shaft_velocity);
      break;
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

int BLDCMotor::command(String user_command) {
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

  // apply the the command
  switch(cmd){
    case 'P':      // velocity P gain change
      if(monitor_port) monitor_port->print("PI velocity P: ");
      if(!GET) PI_velocity.P = value;
      if(monitor_port) monitor_port->println(PI_velocity.P);
      break;
    case 'I':      // velocity I gain change
      if(monitor_port) monitor_port->print("PI velocity I: ");
      if(!GET) PI_velocity.I = value;
      if(monitor_port) monitor_port->println(PI_velocity.I);
      break;
    case 'L':      // velocity voltage limit change
      if(monitor_port) monitor_port->print("PI velocity voltage limit: ");
      if(!GET)PI_velocity.voltage_limit = value;
      if(monitor_port) monitor_port->println(PI_velocity.voltage_limit);
      break;
    case 'R':      // velocity voltage ramp change
      if(monitor_port) monitor_port->print("PI velocity voltage ramp: ");
      if(!GET) PI_velocity.voltage_ramp = value;
      if(monitor_port) monitor_port->println(PI_velocity.voltage_ramp);
      break;
    case 'F':      // velocity Tf low pass filter change
      if(monitor_port) monitor_port->print("LPF velocity time constant: ");
      if(!GET) LPF_velocity.Tf = value;
      if(monitor_port) monitor_port->println(LPF_velocity.Tf);
      break;
    case 'K':      // angle loop gain P change
      if(monitor_port) monitor_port->print("P angle P value: ");
      if(!GET) P_angle.P = value;
      if(monitor_port) monitor_port->println(P_angle.P);
      break;
    case 'N':      // angle loop gain velocity_limit change
      if(monitor_port) monitor_port->print("P angle velocity limit: ");
      if(!GET) P_angle.velocity_limit = value;
      if(monitor_port) monitor_port->println(P_angle.velocity_limit);
      break;
    case 'T':      // angle loop gain velocity_limit change
      if(monitor_port) monitor_port->print("P angle velocity limit: ");
      if(!GET) P_angle.velocity_limit = value;
      if(monitor_port) monitor_port->println(P_angle.velocity_limit);
      break;
    case 'C':
      // change control type
      if(monitor_port) monitor_port->print("Contoller type: ");
      
      if(GET){ // if get commang
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


