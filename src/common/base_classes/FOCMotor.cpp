#include "FOCMotor.h"
#include "../../communication/SimpleFOCDebug.h"

/**
 * Default constructor - setting all variabels to default values
 */
FOCMotor::FOCMotor()
{
  // maximum angular velocity to be used for positioning 
  velocity_limit = DEF_VEL_LIM;
  // maximum voltage to be set to the motor
  voltage_limit = DEF_POWER_SUPPLY;
  // not set on the begining
  current_limit = DEF_CURRENT_LIM;

  // index search velocity
  velocity_index_search = DEF_INDEX_SEARCH_TARGET_VELOCITY;
  // sensor and motor align voltage
  voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;

  // default modulation is SinePWM
  foc_modulation = FOCModulationType::SinePWM;

  // default target value
  target = 0;
  voltage.d = 0;
  voltage.q = 0;
  // current target values
  current_sp = 0;
  current.q = 0;
  current.d = 0;

  // voltage bemf 
  voltage_bemf = 0;

  // Initialize phase voltages U alpha and U beta used for inverse Park and Clarke transform
  Ualpha = 0;
  Ubeta = 0;
  
  //monitor_port 
  monitor_port = nullptr;
  //sensor 
  sensor_offset = 0.0f;
  sensor = nullptr;
  //current sensor 
  current_sense = nullptr;
}


/**
	Sensor linking method
*/
void FOCMotor::linkSensor(Sensor* _sensor) {
  sensor = _sensor;
}

/**
	CurrentSense linking method
*/
void FOCMotor::linkCurrentSense(CurrentSense* _current_sense) {
  current_sense = _current_sense;
}

// shaft angle calculation
float FOCMotor::shaftAngle() {
  // if no sensor linked return previous value ( for open loop )
  if(!sensor) return shaft_angle;
  return sensor_direction*LPF_angle(sensor->getAngle()) - sensor_offset;
}
// shaft velocity calculation
float FOCMotor::shaftVelocity() {
  // if no sensor linked return previous value ( for open loop )
  if(!sensor) return shaft_velocity;
  return sensor_direction*LPF_velocity(sensor->getVelocity());
}

float FOCMotor::electricalAngle(){
  // if no sensor linked return previous value ( for open loop )
  if(!sensor) return electrical_angle;
  return  _normalizeAngle( (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle()  - zero_electric_angle );
}

/**
 *  Monitoring functions
 */
// function implementing the monitor_port setter
void FOCMotor::useMonitoring(Print &print){
  monitor_port = &print; //operate on the address of print
  #ifndef SIMPLEFOC_DISABLE_DEBUG
  SimpleFOCDebug::enable(&print);
  SIMPLEFOC_MOTOR_DEBUG("Monitor enabled!");
  #endif
}

// Measure resistance and inductance of a motor
int FOCMotor::characteriseMotor(float voltage, float correction_factor=1.0f){
    if (!this->current_sense || !this->current_sense->initialized)
    {
      SIMPLEFOC_MOTOR_ERROR("Fail. CS not init.");
      return 1;
    }

    if (voltage <= 0.0f){
      SIMPLEFOC_MOTOR_ERROR("Fail. Volt. <= 0");
      return 2;
    }
    voltage = _constrain(voltage, 0.0f, voltage_limit);
    
    SIMPLEFOC_MOTOR_DEBUG("Meas R..");

    float current_electric_angle = electricalAngle();

    // Apply zero volts and measure a zero reference
    setPhaseVoltage(0, 0, current_electric_angle);
    _delay(500);
    
    PhaseCurrent_s zerocurrent_raw = current_sense->readAverageCurrents();
    DQCurrent_s zerocurrent = current_sense->getDQCurrents(current_sense->getABCurrents(zerocurrent_raw), current_electric_angle);


    // Ramp and hold the voltage to measure resistance
    // 300 ms of ramping
    current_electric_angle = electricalAngle();
    for(int i=0; i < 100; i++){
        setPhaseVoltage(0, voltage/100.0*((float)i), current_electric_angle);
        _delay(3);
    }
    _delay(10);
    PhaseCurrent_s r_currents_raw = current_sense->readAverageCurrents();
    DQCurrent_s r_currents = current_sense->getDQCurrents(current_sense->getABCurrents(r_currents_raw), current_electric_angle);

    // Zero again
    setPhaseVoltage(0, 0, current_electric_angle);
    

    if (fabsf(r_currents.d - zerocurrent.d) < 0.2f)
    {
      SIMPLEFOC_MOTOR_ERROR("Fail. current too low");
      return 3;
    }
    
    float resistance = voltage / (correction_factor * (r_currents.d - zerocurrent.d));
    if (resistance <= 0.0f)
    {
      SIMPLEFOC_MOTOR_ERROR("Fail. Est. R<= 0");
      return 4;
    }
    
    SIMPLEFOC_MOTOR_DEBUG("Est. R: ", 2.0f * resistance);
    _delay(100);


    // Start inductance measurement
    SIMPLEFOC_MOTOR_DEBUG("Meas L...");

    unsigned long t0 = 0;
    unsigned long t1 = 0;
    float Ltemp = 0;
    float Ld = 0;
    float Lq = 0;
    float d_electrical_angle = 0;

    unsigned int iterations = 40;    // how often the algorithm gets repeated.
    unsigned int cycles = 3;         // averaged measurements for each iteration
    unsigned int risetime_us = 200;  // initially short for worst case scenario with low inductance
    unsigned int settle_us = 100000; // initially long for worst case scenario with high inductance

    // Pre-rotate the angle to the q-axis (only useful with sensor, else no harm in doing it)
    current_electric_angle += 0.5f * _PI;
    current_electric_angle = _normalizeAngle(current_electric_angle);

    for (size_t axis = 0; axis < 2; axis++)
    {
      for (size_t i = 0; i < iterations; i++)
      {
        // current_electric_angle = i * _2PI / iterations; // <-- Do a sweep of the inductance. Use eg. for graphing
        float inductanced = 0.0f;
        
        float qcurrent = 0.0f;
        float dcurrent = 0.0f;
        for (size_t j = 0; j < cycles; j++)
        {
          // read zero current
          zerocurrent_raw = current_sense->readAverageCurrents(20);
          zerocurrent = current_sense->getDQCurrents(current_sense->getABCurrents(zerocurrent_raw), current_electric_angle);
          
          // step the voltage
          setPhaseVoltage(0, voltage, current_electric_angle);
          t0 = micros();
          delayMicroseconds(risetime_us); // wait a little bit

          PhaseCurrent_s l_currents_raw = current_sense->getPhaseCurrents();
          t1 = micros();
          setPhaseVoltage(0, 0, current_electric_angle);

          DQCurrent_s l_currents = current_sense->getDQCurrents(current_sense->getABCurrents(l_currents_raw), current_electric_angle);
          delayMicroseconds(settle_us); // wait a bit for the currents to go to 0 again

          if (t0 > t1) continue; // safety first

          // calculate the inductance
          float dt = (t1 - t0)/1000000.0f;
          if (l_currents.d - zerocurrent.d <= 0 || (voltage - resistance * (l_currents.d - zerocurrent.d)) <= 0)
          {
            continue;
          }
          
          inductanced += fabsf(- (resistance * dt) / log((voltage - resistance * (l_currents.d - zerocurrent.d)) / voltage))/correction_factor;
          
          qcurrent+= l_currents.q - zerocurrent.q; // average the measured currents
          dcurrent+= l_currents.d - zerocurrent.d;
        }
        qcurrent /= cycles;
        dcurrent /= cycles;

        float delta = qcurrent / (fabsf(dcurrent) + fabsf(qcurrent)); 


        inductanced /= cycles;
        Ltemp = i < 2 ? inductanced : Ltemp * 0.6 + inductanced * 0.4;
        
        float timeconstant = fabsf(Ltemp / resistance); // Timeconstant of an RL circuit (L/R) 
        // SIMPLEFOC_MOTOR_DEBUG("Estimated time constant in us: ", 1000000.0f * timeconstant);

        // Wait as long as possible (due to limited timing accuracy & sample rate), but as short as needed (while the current still changes)
        risetime_us = _constrain(risetime_us * 0.6f + 0.4f * 1000000 * 0.6f * timeconstant, 100, 10000);
        settle_us = 10 * risetime_us;
        

        // Serial.printf(">inductance:%f:%f|xy\n", current_electric_angle, Ltemp * 1000.0f); // <-- Plot an angle sweep


        /**
         * How this code works:
         * If we apply a current spike in the d´-axis, there will be cross coupling to the q´-axis current, if we didn´t use the actual d-axis (ie. d´ != d).
         * This has to do with saliency (Ld != Lq). 
         * The amount of cross coupled current is somewhat proportional to the angle error, which means that if we iteratively change the angle to min/maximise this current, we get the correct d-axis (and q-axis).
        */
        if (axis)
        {
          qcurrent *= -1.0f; // to d or q axis
        }
        
        if (qcurrent < 0)
        {
          current_electric_angle+=fabsf(delta);
        } else
        {
          current_electric_angle-=fabsf(delta);
        }
        current_electric_angle = _normalizeAngle(current_electric_angle);


        // Average the d-axis angle further for calculating the electrical zero later
        if (axis)
        {
          d_electrical_angle = i < 2 ? current_electric_angle : d_electrical_angle * 0.9 + current_electric_angle * 0.1;
        }
        
      }

      // We know the minimum is 0.5*PI radians away, so less iterations are needed.
      current_electric_angle += 0.5f * _PI;
      current_electric_angle = _normalizeAngle(current_electric_angle);
      iterations /= 2;

      if (axis == 0)
      {
        Lq = Ltemp;
      }else
      {
        Ld = Ltemp;
      }
      
    }

    if (sensor)
    {
      /**
       * The d_electrical_angle should now be aligned to the d axis or the -d axis. We can therefore calculate two possible electrical zero angles. 
       * We then report the one closest to the actual value. This could be useful if the zero search method is not reliable enough (eg. high pole count).
      */

      float estimated_zero_electric_angle_A = _normalizeAngle(  (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle() - d_electrical_angle);
      float estimated_zero_electric_angle_B = _normalizeAngle(  (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle() - d_electrical_angle + _PI);
      float estimated_zero_electric_angle = 0.0f;
      if (fabsf(estimated_zero_electric_angle_A - zero_electric_angle) < fabsf(estimated_zero_electric_angle_B - zero_electric_angle))
      {
        estimated_zero_electric_angle = estimated_zero_electric_angle_A;
      } else
      {
        estimated_zero_electric_angle = estimated_zero_electric_angle_B;
      }

      SIMPLEFOC_MOTOR_DEBUG("New el. zero: ", estimated_zero_electric_angle);
      SIMPLEFOC_MOTOR_DEBUG("Curr. el. zero: ", zero_electric_angle);
    }
    

    SIMPLEFOC_MOTOR_DEBUG("Ld [mH]: ", Ld * 1000.0f);
    SIMPLEFOC_MOTOR_DEBUG("Lq [mH]: ", Lq * 1000.0f);
    if (Ld > Lq)
    {
      SIMPLEFOC_MOTOR_WARN("Ld>Lq. Likely error.");
    }
    if (Ld * 2.0f < Lq)
    {
      SIMPLEFOC_MOTOR_WARN("Lq > 2*Ld. Likely error.");
    }    

    // store the measured values
    phase_resistance = 2.0f * resistance;
    phase_inductance_dq = {Ld, Lq};
    phase_inductance = (Ld + Lq) / 2.0f; // FOR BACKWARDS COMPATIBILITY
    return 0;
    
}

// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void FOCMotor::monitor() {
  if( !monitor_downsample || monitor_cnt++ < (monitor_downsample-1) ) return;
  monitor_cnt = 0;
  if(!monitor_port) return;
  bool printed = 0;

  if(monitor_variables & _MON_TARGET){
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    monitor_port->print(target,monitor_decimals);    
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_Q) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(voltage.q,monitor_decimals);
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_D) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(voltage.d,monitor_decimals);
    printed= true;
  }
  // read currents if possible - even in voltage mode (if current_sense available)
  if(monitor_variables & _MON_CURR_Q || monitor_variables & _MON_CURR_D) {
    DQCurrent_s c = current;
    if( current_sense && torque_controller != TorqueControlType::foc_current ){
      c = current_sense->getFOCCurrents(electrical_angle);
      c.q = LPF_current_q(c.q);
      c.d = LPF_current_d(c.d);
    }
    if(monitor_variables & _MON_CURR_Q) {
      if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
      else if(printed) monitor_port->print(monitor_separator);
      monitor_port->print(c.q*1000, monitor_decimals); // mAmps
      printed= true;
    }
    if(monitor_variables & _MON_CURR_D) {
      if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
      else if(printed) monitor_port->print(monitor_separator);
      monitor_port->print(c.d*1000, monitor_decimals); // mAmps
      printed= true;
    }
  }
 
  if(monitor_variables & _MON_VEL) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(shaft_velocity,monitor_decimals);
    printed= true;
  }
  if(monitor_variables & _MON_ANGLE) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(shaft_angle,monitor_decimals);
    printed= true;
  }
  if(printed){
    if(monitor_end_char) monitor_port->println(monitor_end_char);
    else monitor_port->println("");
  }
}   



// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float FOCMotor::velocityOpenloop(float target_velocity){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  // for display purposes
  shaft_velocity = target_velocity;

  // save timestamp for next call
  open_loop_timestamp = now_us;

  if (torque_controller == TorqueControlType::voltage)
    return voltage_limit;
  else
    return current_limit;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float FOCMotor::angleOpenloop(float target_angle){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
  //                        where small position changes are no longer captured by the precision of floats
  //                        when the total position is large.
  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
    shaft_velocity = velocity_limit;
  }else{
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  // save timestamp for next call
  open_loop_timestamp = now_us;

  // use voltage limit or current limit
  if (torque_controller == TorqueControlType::voltage)
    return voltage_limit;
  else
    return current_limit;
}

// Update limit values in controllers when changed
void FOCMotor::updateVelocityLimit(float new_velocity_limit) {
  velocity_limit = new_velocity_limit;
  if(controller != MotionControlType::angle_nocascade) 
    P_angle.limit = abs(velocity_limit); // if angle control but no velocity cascade, limit the angle controller by the velocity limit
}

// Update limit values in controllers when changed
void FOCMotor::updateCurrentLimit(float new_current_limit) {
  current_limit = new_current_limit;
  if(torque_controller != TorqueControlType::voltage) {
    // if current control
    PID_velocity.limit = new_current_limit;
    if(controller == MotionControlType::angle_nocascade) 
      // if angle control but no velocity cascade, limit the angle controller by the current limit
      P_angle.limit = new_current_limit;
  }
}

// Update limit values in controllers when changed
// PID values and limits
void FOCMotor::updateVoltageLimit(float new_voltage_limit) {
  voltage_limit = new_voltage_limit;
  PID_current_q.limit = new_voltage_limit;
  PID_current_d.limit = new_voltage_limit;
  if(torque_controller == TorqueControlType::voltage) {
    // if voltage control
    PID_velocity.limit = new_voltage_limit;
    if(controller == MotionControlType::angle_nocascade) 
      // if angle control but no velocity cascade, limit the angle controller by the voltage limit
      P_angle.limit = new_voltage_limit;
  }
}

// Update torque control type and related controller limit values
void FOCMotor::updateTorqueControlType(TorqueControlType new_torque_controller) {
  torque_controller = new_torque_controller;
  // update the 
  if (torque_controller == TorqueControlType::voltage)
    // voltage control
    updateVoltageLimit(voltage_limit);
  else 
    // current control
    updateCurrentLimit(current_limit);
}

// Update motion control type and related target values
// - if changing to angle control set target to current angle
// - if changing to velocity control set target to zero
// - if changing to torque control set target to zero
void FOCMotor::updateMotionControlType(MotionControlType new_motion_controller) {
 
  if (controller == new_motion_controller) return; // no change
  
  switch(new_motion_controller)
  {
  case(MotionControlType::angle_nocascade):
    if(controller != MotionControlType::angle && controller != MotionControlType::angle_openloop) break;
  case MotionControlType::angle:
    if(controller != MotionControlType::angle_openloop && controller != MotionControlType::angle_nocascade) break; 
  case MotionControlType::angle_openloop:
    if(controller != MotionControlType::angle && controller != MotionControlType::angle_nocascade) break;
    // if the previous controller was not angle control
    // set target to current angle
    target = shaft_angle;
    break;
  case MotionControlType::velocity:
    if(controller != MotionControlType::velocity_openloop) break; // nothing to do if we are already in velocity control
  case MotionControlType::velocity_openloop:
    if(controller != MotionControlType::velocity) break;
    // if the previous controller was not velocity control
    // stop the motor
    target = 0;
    break;
  case MotionControlType::torque:
    // if torque control set target to zero
    target = 0;
    break;
  default:
    break;
  }

  // finally set the new controller
  controller = new_motion_controller;  
  // update limits in case they need to be changed for the new controller 
  updateVelocityLimit(velocity_limit); 
  updateCurrentLimit(current_limit);
  updateVoltageLimit(voltage_limit);
}


int FOCMotor::tuneCurrentController(float bandwidth) {
  if (bandwidth <= 0.0f) {
    // check bandwidth is positive
    SIMPLEFOC_MOTOR_ERROR("Fail. BW <= 0");
    return 1;
  }
  if (loopfoc_time_us && bandwidth > 0.5f * (1e6f / loopfoc_time_us)) {
    // check bandwidth is not too high for the control loop frequency
    SIMPLEFOC_MOTOR_ERROR("Fail. BW too high, current loop freq:" , (1e6f / loopfoc_time_us));
    return 2;
  }
  if (!_isset(phase_resistance) || (!_isset(phase_inductance) && !_isset(phase_inductance_dq.q))) {
    // need motor parameters to tune the controller
    SIMPLEFOC_MOTOR_WARN("Motor params missing!");
    if(characteriseMotor( voltage_sensor_align )) { 
      return 3;
    }
  }else if (_isset(phase_inductance) && !(_isset(phase_inductance_dq.q))) {
    // if only single inductance value is set, use it for both d and q axis
    phase_inductance_dq = {phase_inductance, phase_inductance};
  }

  PID_current_q.P = phase_inductance_dq.q * (_2PI * bandwidth);
  PID_current_q.I = phase_resistance * (_2PI * bandwidth);
  PID_current_d.P = phase_inductance_dq.d * (_2PI * bandwidth);
  PID_current_d.I = phase_resistance * (_2PI * bandwidth);
  LPF_current_d.Tf = 1.0f / (_2PI * bandwidth * 5.0f); // filter cutoff at 5x bandwidth
  LPF_current_q.Tf = 1.0f / (_2PI * bandwidth * 5.0f); // filter cutoff at 5x bandwidth

  SIMPLEFOC_MOTOR_DEBUG("Tunned PI params for BW [Hz]: ", bandwidth);
  SIMPLEFOC_MOTOR_DEBUG("Pq: ", PID_current_q.P);
  SIMPLEFOC_MOTOR_DEBUG("Iq: ", PID_current_q.I);
  SIMPLEFOC_MOTOR_DEBUG("Pd: ", PID_current_d.P);
  SIMPLEFOC_MOTOR_DEBUG("Id: ", PID_current_d.I);

  return 0;
}



// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void FOCMotor::loopFOC() {
  // update loop time measurement
  updateLoopFOCTime();
  
  // update sensor - do this even in open-loop mode, as user may be switching between modes and we could lose track
  //                 of full rotations otherwise.
  if (sensor) sensor->update();

  // if disabled do nothing
  if(!enabled) return;

  // if open-loop do nothing
  if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) 
    // calculate the open loop electirical angle
    electrical_angle = _electricalAngle((shaft_angle), pole_pairs);
  else
    // Needs the update() to be called first
    // This function will not have numerical issues because it uses Sensor::getMechanicalAngle() 
    // which is in range 0-2PI
    electrical_angle = electricalAngle();

  switch (torque_controller) {
    case TorqueControlType::voltage:
      voltage.q = _constrain(current_sp, -voltage_limit, voltage_limit) + feed_forward_voltage.q;
      voltage.d = feed_forward_voltage.d;
      break;
    case TorqueControlType::estimated_current:
      if(! _isset(phase_resistance)) return; 
      // constrain current setpoint
      current_sp = _constrain(current_sp, -current_limit, current_limit)  + feed_forward_current.q; // desired current is the setpoint
      // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
      if (_isset(KV_rating)) voltage_bemf = estimateBEMF(shaft_velocity);
      // filter the value values
      current.q = LPF_current_q(current_sp);
      // calculate the phase voltage
      voltage.q = current.q * phase_resistance + voltage_bemf;
      // constrain voltage within limits
      voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit) + feed_forward_voltage.q;
      // d voltage  - lag compensation
      if(_isset(phase_inductance_dq.d)) voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance_dq.d, -voltage_limit, voltage_limit) + feed_forward_voltage.d;
      else voltage.d = feed_forward_voltage.d;
      break;
    case TorqueControlType::dc_current:
      if(!current_sense) return;
      // constrain current setpoint
      current_sp = _constrain(current_sp, -current_limit, current_limit) + feed_forward_current.q;
      // read overall current magnitude
      current.q = current_sense->getDCCurrent(electrical_angle);
      // filter the value values
      current.q = LPF_current_q(current.q);
      // calculate the phase voltage
      voltage.q = PID_current_q(current_sp - current.q) + feed_forward_voltage.q;
      // d voltage  - lag compensation
      if(_isset(phase_inductance_dq.d)) voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance_dq.d, -voltage_limit, voltage_limit) + feed_forward_voltage.d;
      else voltage.d = feed_forward_voltage.d;
      break;
    case TorqueControlType::foc_current:
      if(!current_sense) return;
      // constrain current setpoint
      current_sp = _constrain(current_sp, -current_limit, current_limit) + feed_forward_current.q;
      // read dq currents
      current = current_sense->getFOCCurrents(electrical_angle);
      // filter values
      current.q = LPF_current_q(current.q);
      current.d = LPF_current_d(current.d); 
      // calculate the phase voltages
      voltage.q = PID_current_q(current_sp - current.q) + feed_forward_voltage.q;
      voltage.d = PID_current_d(feed_forward_current.d - current.d) + feed_forward_voltage.d;
      // d voltage - lag compensation - TODO verify
      // if(_isset(phase_inductance_dq.d)) voltage.d = _constrain( voltage.d - current_sp*shaft_velocity*pole_pairs*phase_inductance_dq.d, -voltage_limit, voltage_limit);
      break;
    default:
      // no torque control selected
      SIMPLEFOC_MOTOR_ERROR("no torque control selected!");
      break;
  }
  // set the phase voltage - FOC heart function :)
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or voltage loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void FOCMotor::move(float new_target) {

  // set internal target variable
  if(_isset(new_target) ) target = new_target;
  
  // downsampling (optional)
  if(motion_cnt++ < motion_downsample) return;
  motion_cnt = 0;

  // calculate the elapsed time between the calls
  // TODO replace downsample by runnind the code at 
  // a specific frequency (or almost)
  updateMotionControlTime();

  // read value even if motor is disabled to keep the monitoring updated
  // except for the open loop modes where the values are updated within angle/velocityOpenLoop functions
  
  // shaft angle/velocity need the update() to be called first
  // get shaft angle
  // TODO sensor precision: the shaft_angle actually stores the complete position, including full rotations, as a float
  //                        For this reason it is NOT precise when the angles become large.
  //                        Additionally, the way LPF works on angle is a precision issue, and the angle-LPF is a problem
  //                        when switching to a 2-component representation.
  if( controller!=MotionControlType::angle_openloop && controller!=MotionControlType::velocity_openloop ){
    // read the values only if the motor is not in open loop
    // because in open loop the shaft angle/velocity is updated within angle/velocityOpenLoop functions
    shaft_angle = shaftAngle(); 
    shaft_velocity = shaftVelocity(); 
  }

  // if disabled do nothing
  if(!enabled) return;
  

  // upgrade the current based voltage limit
  switch (controller) {
    case MotionControlType::torque:
        current_sp =  target;
        break;
    case MotionControlType::angle_nocascade:
      // TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
      //                        the angles are large. This results in not being able to command small changes at high position values.
      //                        to solve this, the delta-angle has to be calculated in a numerically precise way.
      // angle set point
      shaft_angle_sp = target;
      // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
      current_sp = P_angle(shaft_angle_sp - shaft_angle); // if current/foc_current torque control
      break;
    case MotionControlType::angle:
      // TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
      //                        the angles are large. This results in not being able to command small changes at high position values.
      //                        to solve this, the delta-angle has to be calculated in a numerically precise way.
      // angle set point
      shaft_angle_sp = target;
      // calculate velocity set point
      shaft_velocity_sp = feed_forward_velocity + P_angle( shaft_angle_sp - shaft_angle );
      shaft_velocity_sp = _constrain(shaft_velocity_sp, -velocity_limit, velocity_limit);
      // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); 
      break;
    case MotionControlType::velocity:
      // velocity set point - sensor precision: this calculation is numerically precise.
      shaft_velocity_sp = target;
      // calculate the torque command
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); 
      break;
    case MotionControlType::velocity_openloop:
      // velocity control in open loop - sensor precision: this calculation is numerically precise.
      shaft_velocity_sp = target;
      // this function updates the shaft_angle and shaft_velocity
      // returns the voltage or current that is to be set to the motor (depending on torque control mode)
      // returned values correspond to the voltage_limit and current_limit
      current_sp = velocityOpenloop(shaft_velocity_sp); 
      break;
    case MotionControlType::angle_openloop:
      // angle control in open loop - 
      // TODO sensor precision: this calculation NOT numerically precise, and subject
      //                        to the same problems in small set-point changes at high angles 
      //                        as the closed loop version.
      shaft_angle_sp = target;
      // this function updates the shaft_angle and shaft_velocity
      // returns the voltage or current that is to be set to the motor (depending on torque control mode)
      // returned values correspond to the voltage_limit and current_limit
      current_sp = angleOpenloop(shaft_angle_sp); 
      break;
  }
}


// FOC initialization function
int  FOCMotor::initFOC() {
  int exit_flag = 1;

  motor_status = FOCMotorStatus::motor_calibrating;

  // align motor if necessary
  // alignment necessary for encoders!
  // sensor and motor alignment - can be skipped
  // by setting motor.sensor_direction and motor.zero_electric_angle
  if(sensor){
    exit_flag *= alignSensor();
    // added the shaft_angle update
    sensor->update();
    shaft_angle = shaftAngle();

    // aligning the current sensor - can be skipped
    // checks if driver phases are the same as current sense phases
    // and checks the direction of measuremnt.
    if(exit_flag){
      if(current_sense){ 
        if (!current_sense->initialized) {
          motor_status = FOCMotorStatus::motor_calib_failed;
          SIMPLEFOC_MOTOR_ERROR("Init FOC error, current sense not init");
          exit_flag = 0;
        }else{
          exit_flag *= alignCurrentSense();
        }
      }
      else { SIMPLEFOC_MOTOR_ERROR("No current sense"); }
    }

  } else {
    SIMPLEFOC_MOTOR_DEBUG("No sensor.");
    if ((controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop)){
      exit_flag = 1;    
      SIMPLEFOC_MOTOR_ERROR("Openloop only!");
    }else{
      exit_flag = 0; // no FOC without sensor
    }
  }

  if(exit_flag){
    SIMPLEFOC_MOTOR_DEBUG("Ready.");
    motor_status = FOCMotorStatus::motor_ready;
  }else{
    SIMPLEFOC_MOTOR_ERROR("Init FOC fail");
    motor_status = FOCMotorStatus::motor_calib_failed;
    disable();
  }

  return exit_flag;
}

// Calibarthe the motor and current sense phases
int FOCMotor::alignCurrentSense() {
  int exit_flag = 1; // success

  SIMPLEFOC_MOTOR_DEBUG("Align current sense.");

  // align current sense and the driver
  exit_flag = current_sense->driverAlign(voltage_sensor_align, modulation_centered);
  if(!exit_flag){
    // error in current sense - phase either not measured or bad connection
    SIMPLEFOC_MOTOR_ERROR("Align error!");
    exit_flag = 0;
  }else{
    // output the alignment status flag
    SIMPLEFOC_MOTOR_DEBUG("Success: ", exit_flag);
  }

  return exit_flag > 0;
}

// Encoder alignment to electrical 0 angle
int FOCMotor::alignSensor() {
  int exit_flag = 1; // success
  SIMPLEFOC_MOTOR_DEBUG("Align sensor.");

  // check if sensor needs zero search
  if(sensor->needsSearch()) exit_flag = absoluteZeroSearch();
  // stop init if not found index
  if(!exit_flag) return exit_flag;

  // v2.3.3 fix for R_AVR_7_PCREL against symbol" bug for AVR boards
  // TODO figure out why this works
  float voltage_align = voltage_sensor_align;

  // if unknown natural direction
  if(sensor_direction == Direction::UNKNOWN){

    // find natural direction
    // move one electrical revolution forward
    for (int i = 0; i <=500; i++ ) {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_align, 0,  angle);
	    sensor->update();
      _delay(2);
    }
    // take and angle in the middle
    sensor->update();
    float mid_angle = sensor->getAngle();
    // move one electrical revolution backwards
    for (int i = 500; i >=0; i-- ) {
      float angle = _3PI_2 + _2PI * i / 500.0f ;
      setPhaseVoltage(voltage_align, 0,  angle);
	    sensor->update();
      _delay(2);
    }
    sensor->update();
    float end_angle = sensor->getAngle();
    // setPhaseVoltage(0, 0, 0);
    _delay(200);
    // determine the direction the sensor moved
    float moved =  fabs(mid_angle - end_angle);
    if (moved<MIN_ANGLE_DETECT_MOVEMENT) { // minimum angle to detect movement
      SIMPLEFOC_MOTOR_ERROR("Failed to notice movement");
      return 0; // failed calibration
    } else if (mid_angle < end_angle) {
      SIMPLEFOC_MOTOR_DEBUG("sensor dir: CCW");
      sensor_direction = Direction::CCW;
    } else{
      SIMPLEFOC_MOTOR_DEBUG("sensor dir: CW");
      sensor_direction = Direction::CW;
    }
    // check pole pair number
    pp_check_result = !(fabs(moved*pole_pairs - _2PI) > 0.5f);  // 0.5f is arbitrary number it can be lower or higher!
    if( pp_check_result==false ) {
      SIMPLEFOC_MOTOR_WARN("PP check: fail - est. pp: ", _2PI/moved);
    } else {
      SIMPLEFOC_MOTOR_DEBUG("PP check: OK!");
    }

  } else SIMPLEFOC_MOTOR_DEBUG("Skip dir calib."); 

  // zero electric angle not known
  if(!_isset(zero_electric_angle)){
    // align the electrical phases of the motor and sensor
    // set angle -90(270 = 3PI/2) degrees
    setPhaseVoltage(voltage_align, 0,  _3PI_2);
    _delay(700);
    // read the sensor
    sensor->update();
    // get the current zero electric angle
    zero_electric_angle = 0;
    zero_electric_angle = electricalAngle();
    _delay(20);
    SIMPLEFOC_MOTOR_DEBUG("Zero elec. angle: ", zero_electric_angle);
    // stop everything
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  } else { SIMPLEFOC_MOTOR_DEBUG("Skip offset calib."); }
  return exit_flag;
}


// Encoder alignment the absolute zero angle
// - to the index
int FOCMotor::absoluteZeroSearch() {
  // sensor precision: this is all ok, as the search happens near the 0-angle, where the precision
  //                    of float is sufficient.
  SIMPLEFOC_MOTOR_DEBUG("Index search...");
  // search the absolute zero with small velocity
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while(sensor->needsSearch() && shaft_angle < _2PI){
    angleOpenloop(1.5f*_2PI);
    // call important for some sensors not to loose count
    // not needed for the search
    sensor->update();
  }
  // disable motor
  setPhaseVoltage(0, 0, 0);
  // reinit the limits
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // check if the zero found
  if(monitor_port){
    if(sensor->needsSearch()) { SIMPLEFOC_MOTOR_ERROR("Not found!"); }
    else { SIMPLEFOC_MOTOR_DEBUG("Success!"); }
  }
  return !sensor->needsSearch();
}