#include "BLDCMotor.h"

// // BLDCMotor( int phA, int phB, int phC, int pp, int cpr, int en)
// // - phA, phB, phC - motor A,B,C phase pwm pins
// // - pp            - pole pair number
// // - cpr           - counts per rotation number (cpm=ppm*4)
// // - enable pin    - (optional input)
// BLDCMotor::BLDCMotor( int phA, int phB, int phC, int pp, int cpr, int en){
//   noop;
// }

// BLDCMotor( int pp)
// - phA, phB, phC - motor A,B,C phase pwm pins
// - pp            - pole pair number
// - cpr           - counts per rotation number (cpm=ppm*4)
// - enable pin    - (optional input)
BLDCMotor::BLDCMotor(int pp)
: FOCMotor()
{
  // save pole pairs number
  pole_pairs = pp;
}


/**
	Link the driver which controls the motor
*/
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// init hardware pins
void BLDCMotor::init() {
  if(monitor_port) monitor_port->println("MOT: Initialise variables.");
  // sanity check for the voltage limit configuration
  if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
  // constrain voltage for sensor alignment
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
  // update the controller limits
  PID_velocity.limit = voltage_limit;
  P_angle.limit = velocity_limit;

  _delay(500);
  // enable motor
  if(monitor_port) monitor_port->println("MOT: Enable driver.");
  enable();
  _delay(500);
}


// disable motor driver
void BLDCMotor::disable()
{
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable the driver
  driver->disable();
}
// enable motor driver
void BLDCMotor::enable()
{
  // enable the driver
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);

}

/**
  FOC functions
*/
// FOC initialization function
int  BLDCMotor::initFOC( float zero_electric_offset, Direction sensor_direction ) {
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
// Encoder alignment to electrical 0 angle
int BLDCMotor::alignSensor() {
  if(monitor_port) monitor_port->println("MOT: Align sensor.");
  // align the electrical phases of the motor and sensor
  // set angle -90 degrees

  float start_angle = shaftAngle();
  for (int i = 0; i <=5; i++ ) {
    float angle = _3PI_2 + _2PI * i / 6.0;
    setPhaseVoltage(voltage_sensor_align, 0,  angle);
    _delay(200);
  }
  float mid_angle = shaftAngle();
  for (int i = 5; i >=0; i-- ) {
    float angle = _3PI_2 + _2PI * i / 6.0;
    setPhaseVoltage(voltage_sensor_align, 0,  angle);
    _delay(200);
  }
  if (mid_angle < start_angle) {
    if(monitor_port) monitor_port->println("MOT: natural_direction==CCW");
    sensor->natural_direction = Direction::CCW;
  } else if (mid_angle == start_angle) {
    if(monitor_port) monitor_port->println("MOT: Sensor failed to notice movement");
  } else{
    if(monitor_port) monitor_port->println("MOT: natural_direction==CW");
  }

  // let the motor stabilize for 2 sec
  _delay(2000);
  // set sensor to zero
  sensor->initRelativeZero();
  _delay(500);
  setPhaseVoltage(0, 0, 0);
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
int BLDCMotor::absoluteZeroAlign() {

  if(monitor_port) monitor_port->println("MOT: Absolute zero align.");
    // if no absolute zero return
  if(!sensor->hasAbsoluteZero()) return 0;


  if(monitor_port && sensor->needsAbsoluteZeroSearch()) monitor_port->println("MOT: Searching...");
  // search the absolute zero with small velocity
  while(sensor->needsAbsoluteZeroSearch() && shaft_angle < _2PI){
    loopFOC();
    voltage_q = PID_velocity(velocity_index_search - shaftVelocity());
  }
  voltage_q = 0;
  // disable motor
  setPhaseVoltage(0, 0, 0);

  // align absolute zero if it has been found
  if(!sensor->needsAbsoluteZeroSearch()){
    // align the sensor with the absolute zero
    float zero_offset = sensor->initAbsoluteZero();
    // remember zero electric angle
    zero_electric_angle = _normalizeAngle(_electricalAngle(zero_offset, pole_pairs));
  }
  // return bool if zero found
  return !sensor->needsAbsoluteZeroSearch() ? 1 : -1;
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void BLDCMotor::loopFOC() {
  // shaft angle
  shaft_angle = shaftAngle();
  // set the phase voltage - FOC heart function :)
  setPhaseVoltage(voltage_q, voltage_d, _electricalAngle(shaft_angle,pole_pairs));
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
      shaft_velocity_sp = P_angle( shaft_angle_sp - shaft_angle );
      voltage_q = PID_velocity(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity:
      // velocity set point
      // include velocity loop
      shaft_velocity_sp = target;
      voltage_q = PID_velocity(shaft_velocity_sp - shaft_velocity);
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


// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usaage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  const bool centered = true;
  int sector;
  float _ca,_sa;

  switch (foc_modulation)
  {
    case FOCModulationType::Trapezoid_120 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
      static int trap_120_map[6][3] = {
        {0,1,-1},{-1,1,0},{-1,0,1},{0,-1,1},{1,-1,0},{1,0,-1} // each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
      };
      // static int trap_120_state = 0;
      sector = 6 * (_normalizeAngle(angle_el + PI/6.0 + zero_electric_angle) / _2PI); // adding PI/6 to align with other modes

      Ua = Uq + trap_120_map[sector][0] * Uq;
      Ub = Uq + trap_120_map[sector][1] * Uq;
      Uc = Uq + trap_120_map[sector][2] * Uq;

      if (centered) {
        Ua += (driver->voltage_limit)/2 -Uq;
        Ub += (driver->voltage_limit)/2 -Uq;
        Uc += (driver->voltage_limit)/2 -Uq;
      }
    break;

    case FOCModulationType::Trapezoid_150 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
      static int trap_150_map[12][3] = {
        {0,1,-1},{-1,1,-1},{-1,1,0},{-1,1,1},{-1,0,1},{-1,-1,1},{0,-1,1},{1,-1,1},{1,-1,0},{1,-1,-1},{1,0,-1},{1,1,-1} // each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
      };
      // static int trap_150_state = 0;
      sector = 12 * (_normalizeAngle(angle_el + PI/6.0 + zero_electric_angle) / _2PI); // adding PI/6 to align with other modes

      Ua = Uq + trap_150_map[sector][0] * Uq;
      Ub = Uq + trap_150_map[sector][1] * Uq;
      Uc = Uq + trap_150_map[sector][2] * Uq;

      //center
      if (centered) {
        Ua += (driver->voltage_limit)/2 -Uq;
        Ub += (driver->voltage_limit)/2 -Uq;
        Uc += (driver->voltage_limit)/2 -Uq;
      }

    break;

    case FOCModulationType::SinePWM :
      // Sinusoidal PWM modulation
      // Inverse Park + Clarke transformation

      // angle normalization in between 0 and 2pi
      // only necessary if using _sin and _cos - approximation functions
      angle_el = _normalizeAngle(angle_el + zero_electric_angle);
      _ca = _cos(angle_el);
      _sa = _sin(angle_el);
      // Inverse park transform
      Ualpha =  _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
      Ubeta =  _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

      // Clarke transform
      Ua = Ualpha + driver->voltage_limit/2;
      Ub = -0.5 * Ualpha  + _SQRT3_2 * Ubeta + driver->voltage_limit/2;
      Uc = -0.5 * Ualpha - _SQRT3_2 * Ubeta + driver->voltage_limit/2;

      if (!centered) {
        float Umin = min(Ua, min(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      }

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
      angle_el = _normalizeAngle(angle_el + zero_electric_angle + _PI_2);

      // find the sector we are in currently
      sector = floor(angle_el / _PI_3) + 1;
      // calculate the duty cycles
      float T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uq/driver->voltage_limit;
      float T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uq/driver->voltage_limit;
      // two versions possible
      float T0 = 0; // pulled to 0 - better for low power supply voltage
      if (centered) {
        T0 = 1 - T1 - T2; //centered around driver->voltage_limit/2
      }

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

      // calculate the phase voltages and center
      Ua = Ta*driver->voltage_limit;
      Ub = Tb*driver->voltage_limit;
      Uc = Tc*driver->voltage_limit;
      break;

  }

  // set the voltages in driver
  driver->setPwm(Ua, Ub, Uc);
}



// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
void BLDCMotor::velocityOpenloop(float target_velocity){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6;

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);

  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(voltage_limit,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
void BLDCMotor::angleOpenloop(float target_angle){
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
  setPhaseVoltage(voltage_limit,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;
}
