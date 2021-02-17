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
  return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
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
  if(!monitor_port) return;
  switch (controller) {
    case MotionControlType::velocity_openloop:
    case MotionControlType::velocity:
      monitor_port->print(voltage.q);
      monitor_port->print("\t");
      monitor_port->print(shaft_velocity_sp);
      monitor_port->print("\t");
      monitor_port->println(shaft_velocity);
      break;
    case MotionControlType::angle_openloop:
    case MotionControlType::angle:
      monitor_port->print(voltage.q);
      monitor_port->print("\t");
      monitor_port->print(shaft_angle_sp);
      monitor_port->print("\t");
      monitor_port->println(shaft_angle);
      break;
    case MotionControlType::torque:
      monitor_port->print(voltage.q);
      monitor_port->print("\t");
      monitor_port->print(shaft_angle);
      monitor_port->print("\t");
      monitor_port->println(shaft_velocity);
      break;
  }
}

String FOCMotor::communicate(String user_command) {
  // error flag
  String ret = "";
  // if empty string
  if(user_command.length() < 1) return ret;

  // parse command letter
  char cmd = user_command.charAt(0);
  char sub_cmd = user_command.charAt(1);
  int value_index = (sub_cmd >= 'A'  && sub_cmd <= 'Z') ?  2 :  1;
  // check if get command
  char GET = user_command.charAt(value_index) == '\n';
  // parse command values
  float  value  = user_command.substring(value_index).toFloat();

  // a bit of optimisation of variable memory for Arduino UNO (atmega328)
  switch(cmd){
    case 'Q':      // 
      ret = ret + F("PID curr q| ");
      if(sub_cmd == 'F') ret = ret + LPF_current_q.communicate(user_command.substring(1));
      else ret = ret + PID_current_q.communicate(user_command.substring(1));
      break;
    case 'D':      // 
      ret = ret + F("PID curr d| ");
      if(sub_cmd == 'F') ret = ret + LPF_current_d.communicate(user_command.substring(1));
      else ret = ret + PID_current_d.communicate(user_command.substring(1));
      break;
    case 'V':      // 
      ret = ret + F("PID vel| ");
      if(sub_cmd == 'F') ret = ret + LPF_velocity.communicate(user_command.substring(1));
      else ret = ret + PID_velocity.communicate(user_command.substring(1));
      break;
    case 'A':      // 
      ret = ret + F("PID angle| ");
      if(sub_cmd == 'F') ret = ret + LPF_angle.communicate(user_command.substring(1));
      else ret = ret + P_angle.communicate(user_command.substring(1));
      break;
    case 'L':      // 
      ret = ret + F("Limits| ");
      switch (sub_cmd){
        case 'U':      // voltage limit change
          ret = ret + F("voltage: ");
          if(!GET) {
            voltage_limit = value;
            PID_current_d.limit = value;
            PID_current_q.limit = value;
            // change velocity pid limit if in voltage mode and no phase resistance set
            if( !_isset(phase_resistance) && torque_controller==TorqueControlType::voltage) PID_velocity.limit = value;
          }
          ret = ret + voltage_limit;
          break;
        case 'C':      // current limit
          ret = ret + F("current: ");
          if(!GET){
            current_limit = value;
            // if phase resistance is set, change the voltage limit as well.
            if(_isset(phase_resistance)) voltage_limit = value*phase_resistance;
            // if phase resistance specified or the current control is on set the current limit to the velocity PID
            if(_isset(phase_resistance) ||  torque_controller != TorqueControlType::voltage ) PID_velocity.limit = value;
          }
          ret = ret + current_limit;
          break;
        case 'V':      // velocity limit
          ret = ret + F("velocity: ");
          if(!GET){
            velocity_limit = value;
            P_angle.limit = value;
          }
          ret = ret + velocity_limit;
          break;
        default:
          ret = ret + F("error");
          break;
      }
      break;
    case 'C':
      ret = ret + F("Control: ");
      // change control type
      if(!GET && value >= 0 && (int)value < 5)// if set command
        controller = (MotionControlType)value;
      switch(controller){
        case MotionControlType::torque:
          ret = ret + F("torque");
          break;
        case MotionControlType::velocity:
          ret = ret + F("velocity");
          break;
        case MotionControlType::angle:
          ret = ret + F("angle");
          break;
        case MotionControlType::velocity_openloop:
          ret = ret + F("velocity openloop");
          break;
        case MotionControlType::angle_openloop:
          ret = ret + F("angle openloop");
          break;
      }
      break;
    case 'T':
      // change control type
      ret = ret + F("Torque: ");
      if(!GET && value >= 0 && (int)value < 3)// if set command
        torque_controller = (TorqueControlType)value;
      switch(torque_controller){
        case TorqueControlType::voltage:
          ret = ret + F("voltage");
          break;
        case TorqueControlType::current:
          ret = ret + F("current");
          break;
        case TorqueControlType::foc_current:
          ret = ret + F("foc current");
          break;
      }
      break;
    case 'E':
      // enable/disable
      ret = ret + F("Status: ");
      if(!GET) (bool)value ? enable() : disable();
      ret = ret + enabled;
      break;
    case 'S':
      // Sensor zero offset
      ret = ret + F("Sensor | ");
       switch (sub_cmd){
        case 'M':      // zero offset
          ret = ret + F("offset: ");
          if(!GET) sensor_offset = value;
          ret = ret + sensor_offset;
          break;
        case 'E':      // electrical zero offset - not suggested to touch
          ret = ret + F("el. offset: ");
          if(!GET) zero_electric_angle = value;
          ret = ret + zero_electric_angle;
          break;
        default:
          ret = ret + F("error");
          break;
       }
      break;
    case 'M':     // get current values of the state variables
        switch((int)value){
          case 0: // get voltage
            ret = ret + F("Uq: ");
            ret = ret + voltage.q;
            break;
          case 1: // get velocity
            ret = ret + F("Velocity: ");
            ret = ret + shaft_velocity;
            break;
          case 2: // get angle
            ret = ret + F("Angle: ");
            ret = ret + shaft_angle;
            break;
          case 3: // get angle
            ret = ret + F("Target: ");
            ret = ret + target;
            break;
        }
      break;
    default:  // target change
      ret = ret + F("Target: ");
      target = user_command.toFloat();
      ret = ret + target;
  }

  return ret;
}
