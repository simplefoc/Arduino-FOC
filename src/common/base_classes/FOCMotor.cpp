#include "FOCMotor.h"

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
  
  //monitor_port 
  monitor_port = nullptr;
  //sensor 
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
  if(monitor_port ) monitor_port->println(F("MOT: Monitor enabled!"));
}

// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void FOCMotor::monitor() {
  if( !monitor_downsample || monitor_cnt++ < monitor_downsample ) return;
  monitor_cnt = 0;
  if(!monitor_port) return;
  bool printed = 0;

  if(monitor_variables & _MON_TARGET){
    monitor_port->print(target,4);
    monitor_port->print("\t");
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_Q) {
    monitor_port->print(voltage.q,4);
    monitor_port->print("\t");
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_D) {
    monitor_port->print(voltage.d,4);
    monitor_port->print("\t");
    printed= true;
  }
  // read currents if possible - even in voltage mode (if current_sense available)
  if(monitor_variables & _MON_CURR_Q || monitor_variables & _MON_CURR_D) {
    DQCurrent_s c{0,0};
    if(current_sense){
      if(torque_controller == TorqueControlType::foc_current) c = current;
      else{
    	  c = current_sense->getFOCCurrents(electrical_angle);
        c.q = LPF_current_q(c.q);
        c.d = LPF_current_d(c.d);
      }
    }
    if(monitor_variables & _MON_CURR_Q) {
      monitor_port->print(c.q*1000, 2); // mAmps
      monitor_port->print("\t");
      printed= true;
    }
    if(monitor_variables & _MON_CURR_D) {
      monitor_port->print(c.d*1000, 2); // mAmps
      monitor_port->print("\t");
      printed= true;
    }
  }
 
  if(monitor_variables & _MON_VEL) {
    monitor_port->print(shaft_velocity,4);
    monitor_port->print("\t");
    printed= true;
  }
  if(monitor_variables & _MON_ANGLE) {
    monitor_port->print(shaft_angle,4);
    printed= true;
  }
  if(printed) monitor_port->println();
  
}

