#include "StepperMotor.h"
#include "./communication/SimpleFOCDebug.h"


// StepperMotor(int pp)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rmp/v)
// - Lq            - motor q-axis inductance [H]
// - Ld            - motor d-axis inductance [H]
StepperMotor::StepperMotor(int pp, float _R, float _KV, float _Lq, float _Ld)
: FOCMotor()
{
  // number od pole pairs
  pole_pairs = pp;
  // save phase resistance number
  phase_resistance = _R;
  // save back emf constant KV = 1/K_bemf
  // usually used rms
  KV_rating = _KV;
  // save phase inductance
  axis_inductance = {_Ld, _Lq};
  phase_inductance = _Lq;  // FOR BACKWARDS COMPATIBILITY

  // torque control type is voltage by default
  // current and foc_current not supported yet
  torque_controller = TorqueControlType::voltage;
}

/**
	Link the driver which controls the motor
*/
void StepperMotor::linkDriver(StepperDriver* _driver) {
  driver = _driver;
}

// init hardware pins
int StepperMotor::init() {
  if (!driver || !driver->initialized) {
    motor_status = FOCMotorStatus::motor_init_failed;
    SIMPLEFOC_MOTOR_ERROR("Init not possible, driver not init");
    return 0;
  }
  motor_status = FOCMotorStatus::motor_initializing;
  SIMPLEFOC_MOTOR_DEBUG("Init");

  // sanity check for the voltage limit configuration
  if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
  // constrain voltage for sensor alignment
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  // update limits in the motor controllers
  updateCurrentLimit(current_limit);
  updateVoltageLimit(voltage_limit);
  updateVelocityLimit(velocity_limit);

  if(_isset(phase_inductance) && !(_isset(axis_inductance.q))) {
    // if only single inductance value is set, use it for both d and q axis
    axis_inductance = {phase_inductance, phase_inductance};
  } 

  
  // if using open loop control, set a CW as the default direction if not already set
  // only if no sensor is used
  if(!sensor){
    if ((controller==MotionControlType::angle_openloop
      ||controller==MotionControlType::velocity_openloop)
      && (sensor_direction == Direction::UNKNOWN)) {
        sensor_direction = Direction::CW;
    }
  }

  _delay(500);
  // enable motor
  SIMPLEFOC_MOTOR_DEBUG("Enable driver.");
  enable();
  _delay(500);
  motor_status = FOCMotorStatus::motor_uncalibrated;
  return 1;
}


// disable motor driver
void StepperMotor::disable()
{
  // disable the current sense
  if(current_sense) current_sense->disable();
  // set zero to PWM
  driver->setPwm(0, 0);
  // disable driver
  driver->disable();
  // motor status update
  enabled = 0;
}
// enable motor driver
void StepperMotor::enable()
{
  // disable enable
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0);
  // enable the current sense
  if(current_sense) current_sense->enable();
  // reset the pids
  PID_velocity.reset();
  P_angle.reset();
  PID_current_q.reset();
  PID_current_d.reset();
  // motor status update
  enabled = 1;
}


float StepperMotor::estimateBEMF(float vel){
  // bemf constant is approximately 1/KV rating
  // V_bemf = K_bemf * velocity
  return vel/(KV_rating*_SQRT2)/_RPM_TO_RADS;
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Sine PWM algorithms
void StepperMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {
  // Sinusoidal PWM modulation
  // Inverse Park transformation
  float _sa, _ca;
  _sincos(angle_el, &_sa, &_ca);

  // Inverse park transform
  Ualpha =  _ca * Ud - _sa * Uq;  
  Ubeta =  _sa * Ud + _ca * Uq;   

  // set the voltages in hardware
  driver->setPwm(Ualpha, Ubeta);
}
