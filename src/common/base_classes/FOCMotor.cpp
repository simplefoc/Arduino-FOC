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
  SIMPLEFOC_DEBUG("MOT: Monitor enabled!");
  #endif
}

// Measure resistance and inductance of a motor
int FOCMotor::characteriseMotor(float voltage, float correction_factor=1.0f){
    if (!this->current_sense || !this->current_sense->initialized)
    {
      SIMPLEFOC_DEBUG("ERR: MOT: Cannot characterise motor: CS unconfigured or not initialized");
      return 1;
    }

    if (voltage <= 0.0f){
      SIMPLEFOC_DEBUG("ERR: MOT: Cannot characterise motor: Voltage is negative or less than zero");
      return 2;
    }
    voltage = _constrain(voltage, 0.0f, voltage_limit);
    
    SIMPLEFOC_DEBUG("MOT: Measuring phase to phase resistance, keep motor still...");

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
      SIMPLEFOC_DEBUG("ERR: MOT: Motor characterisation failed: measured current too low");
      return 3;
    }
    
    float resistance = voltage / (correction_factor * (r_currents.d - zerocurrent.d));
    if (resistance <= 0.0f)
    {
      SIMPLEFOC_DEBUG("ERR: MOT: Motor characterisation failed: Calculated resistance <= 0");
      return 4;
    }
    
    SIMPLEFOC_DEBUG("MOT: Estimated phase to phase resistance: ", 2.0f * resistance);
    _delay(100);


    // Start inductance measurement
    SIMPLEFOC_DEBUG("MOT: Measuring inductance, keep motor still...");

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
        // SIMPLEFOC_DEBUG("MOT: Estimated time constant in us: ", 1000000.0f * timeconstant);

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

      SIMPLEFOC_DEBUG("MOT: Newly estimated electrical zero: ", estimated_zero_electric_angle);
      SIMPLEFOC_DEBUG("MOT: Current electrical zero: ", zero_electric_angle);
    }
    

    SIMPLEFOC_DEBUG("MOT: Inductance measurement complete!");
    SIMPLEFOC_DEBUG("MOT: Measured D-inductance in mH: ", Ld * 1000.0f);
    SIMPLEFOC_DEBUG("MOT: Measured Q-inductance in mH: ", Lq * 1000.0f);
    if (Ld > Lq)
    {
      SIMPLEFOC_DEBUG("WARN: MOT: Measured inductance is larger in D than in Q axis. This is normally a sign of a measurement error.");
    }
    if (Ld * 2.0f < Lq)
    {
      SIMPLEFOC_DEBUG("WARN: MOT: Measured Q inductance is more than twice the D inductance. This is probably wrong. From experience, the lower value is probably close to reality.");
    }    

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

