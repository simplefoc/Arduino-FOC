#include "BLDCMotor.h"
#include "./communication/SimpleFOCDebug.h"


// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
// each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,_HIGH_IMPEDANCE,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,_HIGH_IMPEDANCE,-1} 
};

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
// each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_150_map[12][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,1,1},
    {-1,_HIGH_IMPEDANCE,1},
    {-1,-1,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,-1,-1},
    {1,_HIGH_IMPEDANCE,-1},
    {1,1,-1} 
};

// BLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rmp/v)
// - L             - motor phase inductance
BLDCMotor::BLDCMotor(int pp, float _R, float _KV, float _Lq, float _Ld)
: FOCMotor()
{
  // save pole pairs number
  pole_pairs = pp;
  // save phase resistance number
  phase_resistance = _R;
  // save back emf constant KV = 1/KV
  // 1/sqrt(3) - rms value
  KV_rating = _KV;
  // save phase inductance
  phase_inductance_dq = {_Ld, _Lq};
  phase_inductance = _Lq;  // FOR BACKWARDS COMPATIBILITY

  // torque control type is voltage by default
  torque_controller = TorqueControlType::voltage;
}


/**
	Link the driver which controls the motor
*/
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// init hardware pins
int BLDCMotor::init() {
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
void BLDCMotor::disable()
{
  // disable the current sense
  if(current_sense) current_sense->disable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable the driver
  driver->disable();
  // motor status update
  enabled = 0;
}
// enable motor driver
void BLDCMotor::enable()
{
  // enable the driver
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

/**
  FOC functions
*/

float BLDCMotor::estimateBEMF(float vel){
  // bemf constant is approximately 1/KV rating
  // V_bemf = K_bemf * velocity
  return vel/(KV_rating*_SQRT3)/_RPM_TO_RADS;
}


// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM, Sine PWM and Trapezoidal commutation algorithms
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  float center;
  int sector;
  float _ca,_sa;

  switch (foc_modulation)
  {
    case FOCModulationType::Trapezoid_120 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
      // determine the sector
      sector = 6 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);// disable phase if possible
      }else{
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF);// disable phase if possible
      }

    break;

    case FOCModulationType::Trapezoid_150 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
      // determine the sector
      sector = 12 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_150_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][2]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); // disable phase if possible
      }else{
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // enable all phases
      }

    break;

    case FOCModulationType::SinePWM :
    case FOCModulationType::SpaceVectorPWM :

        // Sinusoidal PWM modulation
        // Inverse Park + Clarke transformation
        _sincos(angle_el, &_sa, &_ca);

        // Inverse park transform
        Ualpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
        Ubeta = _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

        // Clarke transform
        Ua = Ualpha;
        Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
        Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

        // centering the voltages around either
        // - centered modulation: around driver.voltage_limit/2
        // - non-centered modulation: pulls the lowest voltage to 0 
        //     - Can be useful for low-side current sensing 
        //       in cases where the ADC had long sample time
        //     - The part of the duty cycle in which all phases are 
        //       off is longer than in centered modulation   
        //     - Both SinePWM and SpaceVectorPWM have the same form for non-centered modulation
      if (modulation_centered) {
        center = driver->voltage_limit/2;
        if (foc_modulation == FOCModulationType::SpaceVectorPWM){
          // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
          // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
          // Midpoint Clamp
          float Umin = min(Ua, min(Ub, Uc));
          float Umax = max(Ua, max(Ub, Uc));
          center -= (Umax+Umin) / 2;
        } 
        Ua += center;
        Ub += center;
        Uc += center;
      }else{
        float Umin = min(Ua, min(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      }
      break;
  }

  // set the voltages in driver
  driver->setPwm(Ua, Ub, Uc);
}