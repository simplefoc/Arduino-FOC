#include "HybridStepperMotor.h"
#include "./communication/SimpleFOCDebug.h"

// HybridStepperMotor(int pp)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rmp/v)
// - Lq            - motor q-axis inductance [H]
// - Ld            - motor d-axis inductance [H]
HybridStepperMotor::HybridStepperMotor(int pp, float _R, float _KV, float _Lq, float _Ld)
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
  phase_inductance_dq = {_Ld, _Lq};
  phase_inductance = _Lq;  // FOR BACKWARDS COMPATIBILITY


  // torque control type is voltage by default
  // current and foc_current not supported yet
  torque_controller = TorqueControlType::voltage;
}

/**
  Link the driver which controls the motor
*/
void HybridStepperMotor::linkDriver(BLDCDriver *_driver)
{
  driver = _driver;
  SIMPLEFOC_MOTOR_DEBUG("BLDCDriver linked, using pin C as the mid-phase");
}

// override of the FOCMotor's current sense linking 
// setting the driver type directly
void HybridStepperMotor::linkCurrentSense(CurrentSense* _cs){
  FOCMotor::linkCurrentSense(_cs);
  current_sense->driver_type = DriverType::Hybrid;
}

// init hardware pins
int HybridStepperMotor::init()
{
  if (!driver || !driver->initialized)
  {
    motor_status = FOCMotorStatus::motor_init_failed;
    SIMPLEFOC_MOTOR_ERROR("Init not possible, driver not init");
    return 0;
  }
  motor_status = FOCMotorStatus::motor_initializing;
  SIMPLEFOC_MOTOR_DEBUG("Init");
  // sanity check for the voltage limit configuration
  if (voltage_limit > driver->voltage_limit)
    voltage_limit = driver->voltage_limit;
  // constrain voltage for sensor alignment
  if (voltage_sensor_align > voltage_limit)
    voltage_sensor_align = voltage_limit;

  // update limits in the motor controllers
  updateCurrentLimit(current_limit);
  updateVoltageLimit(voltage_limit);
  updateVelocityLimit(velocity_limit);

  if(_isset(phase_inductance) && !(_isset(phase_inductance_dq.q))) {
    // if only single inductance value is set, use it for both d and q axis
    phase_inductance_dq = {phase_inductance, phase_inductance};
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
void HybridStepperMotor::disable()
{
  // disable the current sense
  if(current_sense) current_sense->disable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable driver
  driver->disable();
  // motor status update
  enabled = 0;
}

// enable motor driver
void HybridStepperMotor::enable()
{
  // disable enable
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
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


float HybridStepperMotor::estimateBEMF(float vel){
  // bemf constant is approximately 1/KV rating
  // V_bemf = K_bemf * velocity
  return vel/(KV_rating*_SQRT2)/_RPM_TO_RADS;
}


// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Sine PWM and SVPWM algorithms
void HybridStepperMotor::setPhaseVoltage(float Uq, float Ud, float angle_el)
{
  float center;
  float _sa, _ca;

  _sincos(angle_el, &_sa, &_ca);

  switch (foc_modulation) {
  case FOCModulationType::Trapezoid_120:
  case FOCModulationType::Trapezoid_150:
    // not handled
    Ua = 0;
    Ub = 0;
    Uc = 0;
    break;
  case FOCModulationType::SinePWM:
    // C phase is fixed at half-rail to provide bias point for A, B legs
    Ua = (_ca * Ud) - (_sa * Uq);
    Ub = (_sa * Ud) + (_ca * Uq);

    center = driver->voltage_limit / 2;

    Ua += center;
    Ub += center;
    Uc = center;
    break;

  case FOCModulationType::SpaceVectorPWM:
    // C phase moves in order to increase max bias on coils
    Ua = (_ca * Ud) - (_sa * Uq);
    Ub = (_sa * Ud) + (_ca * Uq);

    float Umin = fmin(fmin(Ua, Ub), 0);
    float Umax = fmax(fmax(Ua, Ub), 0);
    float Vo = -(Umin + Umax)/2 + driver->voltage_limit/2;
    
    Ua = Ua + Vo;
    Ub = Ub + Vo;
    Uc = Vo;
  }
  driver->setPwm(Ua, Ub, Uc);
  
}
