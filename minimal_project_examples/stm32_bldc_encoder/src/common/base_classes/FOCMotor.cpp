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

  // index search velocity
  velocity_index_search = DEF_INDEX_SEARCH_TARGET_VELOCITY;
  // sensor and motor align voltage
  voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;

  // electric angle of comthe zero angle
  zero_electric_angle = 0;

  // default modulation is SinePWM
  foc_modulation = FOCModulationType::SinePWM;

  // default target value
  target = 0;
  voltage_d = 0;
  voltage_q = 0;
  
  //monitor_port 
  monitor_port = nullptr;
  //sensor 
  sensor = nullptr;
}


/**
	Sensor communication methods
*/
void FOCMotor::linkSensor(Sensor* _sensor) {
  sensor = _sensor;
}
// shaft angle calculation
float FOCMotor::shaftAngle() {
  // if no sensor linked return 0
  if(!sensor) return shaft_angle;
  return sensor->getAngle();
}
// shaft velocity calculation
float FOCMotor::shaftVelocity() {
  // if no sensor linked return 0
  if(!sensor) return 0;
  return LPF_velocity(sensor->getVelocity());
}

/**
 *  Monitoring functions
 */
// function implementing the monitor_port setter
void FOCMotor::useMonitoring(Print &print){
  monitor_port = &print; //operate on the address of print
  if(monitor_port ) monitor_port->println("MOT: Monitor enabled!");
}
// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void FOCMotor::monitor() {
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

int FOCMotor::command(String user_command) {
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
      if(!GET) {
        voltage_limit = value;
        PID_velocity.limit = value;
      }
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
      if(!GET){
        velocity_limit = value;
        P_angle.limit = value;
      }
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