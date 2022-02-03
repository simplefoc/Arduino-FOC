#include "Commander.h"

Commander::Commander(Stream& serial, char eol, bool echo){
  com_port = &serial;
  this->eol = eol;
  this->echo = echo;
}
Commander::Commander(char eol, bool echo){
  this->eol = eol;
  this->echo = echo;
}


void Commander::add(char id, CommandCallback onCommand, char* label ){
  call_list[call_count] = onCommand;
  call_ids[call_count] = id;
  call_label[call_count] = label;
  call_count++;
}


void Commander::run(){
  if(!com_port) return;
  run(*com_port, eol);
}

void Commander::run(Stream& serial, char eol){
  Stream* tmp = com_port; // save the serial instance
  char eol_tmp = this->eol;
  this->eol = eol;
  com_port = &serial;

  // a string to hold incoming data
  while (serial.available()) {
    // get the new byte:
    int ch = serial.read();
    received_chars[rec_cnt++] = (char)ch;
    // end of user input
    if(echo)
      print((char)ch);
    if (isSentinel(ch)) {
      // execute the user command
      run(received_chars);

      // reset the command buffer
      received_chars[0] = 0;
      rec_cnt=0;
    }
    if (rec_cnt>=MAX_COMMAND_LENGTH) { // prevent buffer overrun if message is too long
        received_chars[0] = 0;
        rec_cnt=0;
    }
  }

  com_port = tmp; // reset the instance to the internal value
  this->eol = eol_tmp;
}

void Commander::run(char* user_input){
  // execute the user command
  char id = user_input[0];
  switch(id){
    case CMD_SCAN:
      for(int i=0; i < call_count; i++){
          print(call_ids[i]);
          print(":");
          if(call_label[i]) println(call_label[i]);
          else println("");
      }
      break;
    case CMD_VERBOSE:
      if(!isSentinel(user_input[1])) verbose = (VerboseMode)atoi(&user_input[1]);
      printVerbose(F("Verb:"));
      switch (verbose){
      case VerboseMode::nothing:
        println(F("off!"));
        break;
      case VerboseMode::on_request:
      case VerboseMode::user_friendly:
        println(F("on!"));
        break;
      }
      break;
    case CMD_DECIMAL:
      if(!isSentinel(user_input[1])) decimal_places = atoi(&user_input[1]);
      printVerbose(F("Decimal:"));
      println(decimal_places);
      break;
    default:
      for(int i=0; i < call_count; i++){
        if(id == call_ids[i]){
          call_list[i](&user_input[1]);
          break;
        }
      }
      break;
  }
}

void Commander::motor(FOCMotor* motor, char* user_command) {

  // if target setting
  if(isDigit(user_command[0]) || user_command[0] == '-' || user_command[0] == '+'){
    target(motor, user_command);
    return;
  }

  // parse command letter
  char cmd = user_command[0];
  char sub_cmd = user_command[1];
  // check if there is a subcommand or not
  int value_index = (sub_cmd >= 'A'  && sub_cmd <= 'Z') ?  2 :  1;
  // check if get command
  bool GET = isSentinel(user_command[value_index]);
  // parse command values
  float value = atof(&user_command[value_index]);


  // a bit of optimisation of variable memory for Arduino UNO (atmega328)
  switch(cmd){
    case CMD_C_Q_PID:      //
      printVerbose(F("PID curr q| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_q, &user_command[1]);
      else pid(&motor->PID_current_q,&user_command[1]);
      break;
    case CMD_C_D_PID:      //
      printVerbose(F("PID curr d| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_d, &user_command[1]);
      else pid(&motor->PID_current_d, &user_command[1]);
      break;
    case CMD_V_PID:      //
      printVerbose(F("PID vel| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_velocity, &user_command[1]);
      else pid(&motor->PID_velocity, &user_command[1]);
      break;
    case CMD_A_PID:      //
      printVerbose(F("PID angle| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_angle, &user_command[1]);
      else pid(&motor->P_angle, &user_command[1]);
      break;
    case CMD_LIMITS:      //
     printVerbose(F("Limits| "));
      switch (sub_cmd){
        case SCMD_LIM_VOLT:      // voltage limit change
          printVerbose(F("volt: "));
          if(!GET) {
            motor->voltage_limit = value;
            motor->PID_current_d.limit = value;
            motor->PID_current_q.limit = value;
            // change velocity pid limit if in voltage mode and no phase resistance set
            if( !_isset(motor->phase_resistance) && motor->torque_controller==TorqueControlType::voltage) motor->PID_velocity.limit = value;
          }
          println(motor->voltage_limit);
          break;
        case SCMD_LIM_CURR:      // current limit
          printVerbose(F("curr: "));
          if(!GET){
            motor->current_limit = value;
            // if phase resistance specified or the current control is on set the current limit to the velocity PID
            if(_isset(motor->phase_resistance) || motor->torque_controller != TorqueControlType::voltage ) motor->PID_velocity.limit = value;
          }
          println(motor->current_limit);
          break;
        case SCMD_LIM_VEL:      // velocity limit
          printVerbose(F("vel: "));
          if(!GET){
            motor->velocity_limit = value;
            motor->P_angle.limit = value;
          }
          println(motor->velocity_limit);
          break;
        default:
          printError();
          break;
      }
      break;
    case CMD_MOTION_TYPE:
    case CMD_TORQUE_TYPE:
    case CMD_STATUS:
      motion(motor, &user_command[0]);
      break;
    case CMD_PWMMOD:
       // PWM modulation change
       printVerbose(F("PWM Mod | "));
       switch (sub_cmd){
        case SCMD_PWMMOD_TYPE:      // zero offset
          printVerbose(F("type: "));
          if(!GET) motor->foc_modulation = (FOCModulationType)value;
          switch(motor->foc_modulation){
            case FOCModulationType::SinePWM:
              println(F("SinePWM"));
              break;
            case FOCModulationType::SpaceVectorPWM:
              println(F("SVPWM"));
              break;
            case FOCModulationType::Trapezoid_120:
              println(F("Trap 120"));
              break;
            case FOCModulationType::Trapezoid_150:
              println(F("Trap 150"));
              break;
          }
          break;
        case SCMD_PWMMOD_CENTER:      // centered modulation
          printVerbose(F("center: "));
          if(!GET) motor->modulation_centered = value;
          println(motor->modulation_centered);
          break;
        default:
          printError();
          break;
       }
      break;
    case CMD_RESIST:
      // enable/disable
      printVerbose(F("R phase: "));
      if(!GET){
        motor->phase_resistance = value;
        if(motor->torque_controller==TorqueControlType::voltage){
          motor->voltage_limit = motor->current_limit*value;
          motor->PID_velocity.limit= motor->current_limit;
        }
      }
      if(_isset(motor->phase_resistance)) println(motor->phase_resistance);
      else println(0);
      break;
    case CMD_SENSOR:
      // Sensor zero offset
       printVerbose(F("Sensor | "));
       switch (sub_cmd){
        case SCMD_SENS_MECH_OFFSET:      // zero offset
          printVerbose(F("offset: "));
          if(!GET) motor->sensor_offset = value;
          println(motor->sensor_offset);
          break;
        case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
          printVerbose(F("el. offset: "));
          if(!GET) motor->zero_electric_angle = value;
          println(motor->zero_electric_angle);
          break;
        default:
          printError();
          break;
       }
      break;
    case CMD_MONITOR:     // get current values of the state variables
      printVerbose(F("Monitor | "));
      switch (sub_cmd){
        case SCMD_GET:      // get command
          switch((uint8_t)value){
            case 0: // get target
              printVerbose(F("target: "));
              println(motor->target);
              break;
            case 1: // get voltage q
              printVerbose(F("Vq: "));
              println(motor->voltage.q);
              break;
            case 2: // get voltage d
              printVerbose(F("Vd: "));
              println(motor->voltage.d);
              break;
            case 3: // get current q
              printVerbose(F("Cq: "));
              println(motor->current.q);
              break;
            case 4: // get current d
              printVerbose(F("Cd: "));
              println(motor->current.d);
              break;
            case 5: // get velocity
              printVerbose(F("vel: "));
              println(motor->shaft_velocity);
              break;
            case 6: // get angle
              printVerbose(F("angle: "));
              println(motor->shaft_angle);
              break;
            case 7: // get all states
              printVerbose(F("all: "));
              print(motor->target);
              print(";");
              print(motor->voltage.q);
              print(";");
              print(motor->voltage.d);
              print(";");
              print(motor->current.q);
              print(";");
              print(motor->current.d);
              print(";");
              print(motor->shaft_velocity);
              print(";");
              println(motor->shaft_angle);
              break;
            default:
              printError();
              break;
          }
          break;
        case SCMD_DOWNSAMPLE:
          printVerbose(F("downsample: "));
          if(!GET) motor->monitor_downsample = value;
          println((int)motor->monitor_downsample);
          break;
        case SCMD_CLEAR:
          motor->monitor_variables = (uint8_t) 0;
          println(F("clear"));
          break;
        case SCMD_SET:
          if(!GET) motor->monitor_variables = (uint8_t) 0;
          for(int i = 0; i < 7; i++){
            if(isSentinel(user_command[value_index+i])) break;
            if(!GET) motor->monitor_variables |=  (user_command[value_index+i] - '0') << (6-i);
            print( (user_command[value_index+i] - '0') );
          }
          println("");
          break;
        default:
          printError();
          break;
       }
      break;
    default:  // unknown cmd
      printVerbose(F("unknown cmd "));
      printError();
  }
}

void Commander::motion(FOCMotor* motor, char* user_cmd, char* separator){
  char cmd = user_cmd[0];
  char sub_cmd = user_cmd[1];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[(sub_cmd >= 'A'  && sub_cmd <= 'Z') ?  2 :  1]);

  switch(cmd){
    case CMD_MOTION_TYPE:
      printVerbose(F("Motion:"));
      switch(sub_cmd){
        case SCMD_DOWNSAMPLE:
            printVerbose(F(" downsample: "));
            if(!GET) motor->motion_downsample = value;
            println((int)motor->motion_downsample);
          break;
        default:
          // change control type
          if(!GET && value >= 0 && (int)value < 5) // if set command
            motor->controller = (MotionControlType)value;
          switch(motor->controller){
            case MotionControlType::torque:
              println(F("torque"));
              break;
            case MotionControlType::velocity:
              println(F("vel"));
              break;
            case MotionControlType::angle:
              println(F("angle"));
              break;
            case MotionControlType::velocity_openloop:
              println(F("vel open"));
              break;
            case MotionControlType::angle_openloop:
              println(F("angle open"));
              break;
          }
            break;
        }
      break;
    case CMD_TORQUE_TYPE:
      // change control type
      printVerbose(F("Torque: "));
      if(!GET && (int8_t)value >= 0 && (int8_t)value < 3)// if set command
        motor->torque_controller = (TorqueControlType)value;
      switch(motor->torque_controller){
        case TorqueControlType::voltage:
          println(F("volt"));
          // change the velocity control limits if necessary
          if( !_isset(motor->phase_resistance) ) motor->PID_velocity.limit = motor->voltage_limit;
          break;
        case TorqueControlType::dc_current:
          println(F("dc curr"));
          // change the velocity control limits if necessary
          motor->PID_velocity.limit = motor->current_limit;
          break;
        case TorqueControlType::foc_current:
          println(F("foc curr"));
          // change the velocity control limits if necessary
          motor->PID_velocity.limit = motor->current_limit;
          break;
      }
      break;
    case CMD_STATUS:
      // enable/disable
      printVerbose(F("Status: "));
      if(!GET) (bool)value ? motor->enable() : motor->disable();
       println(motor->enabled);
      break;
    default:
      target(motor,  user_cmd, separator);
      break;
  }
}

void Commander::pid(PIDController* pid, char* user_cmd){
  char cmd = user_cmd[0];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_PID_P:      // P gain change
      printVerbose("P: ");
      if(!GET) pid->P = value;
      println(pid->P);
      break;
    case SCMD_PID_I:      // I gain change
      printVerbose("I: ");
      if(!GET) pid->I = value;
      println(pid->I);
      break;
    case SCMD_PID_D:      // D gain change
      printVerbose("D: ");
      if(!GET) pid->D = value;
      println(pid->D);
      break;
    case SCMD_PID_RAMP:      //  ramp change
      printVerbose("ramp: ");
      if(!GET) pid->output_ramp = value;
      println(pid->output_ramp);
      break;
    case SCMD_PID_LIM:      //  limit change
      printVerbose("limit: ");
      if(!GET) pid->limit = value;
      println(pid->limit);
      break;
    default:
      printError();
      break;
  }
}

void Commander::lpf(LowPassFilter* lpf, char* user_cmd){
  char cmd = user_cmd[0];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_LPF_TF:      // Tf value change
      printVerbose(F("Tf: "));
      if(!GET) lpf->Tf = value;
      println(lpf->Tf);
      break;
    default:
      printError();
      break;
  }
}

void Commander::scalar(float* value,  char* user_cmd){
  bool GET  = isSentinel(user_cmd[0]);
  if(!GET) *value = atof(user_cmd);
  println(*value);
}


void Commander::target(FOCMotor* motor,  char* user_cmd, char* separator){
  // if no values sent
  if(isSentinel(user_cmd[0])) return;
  
  float pos, vel, torque;
  char* next_value;
  switch(motor->controller){
    case MotionControlType::torque: // setting torque target
      torque = atof(strtok (user_cmd, separator));
      motor->target = torque;
      break;
    case MotionControlType::velocity: // setting velocity target + torque limit
      // set the target
      vel= atof(strtok (user_cmd, separator));
      motor->target = vel;

      // allow for setting only the target velocity without chaning the torque limit
      next_value = strtok (NULL, separator);
      if (next_value){
        torque = atof(next_value);
        motor->PID_velocity.limit = torque;
        // torque command can be voltage or current
        if(!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage) motor->voltage_limit = torque;
        else  motor->current_limit = torque;
      }
      break;
    case MotionControlType::angle: // setting angle target + torque, velocity limit
      // setting the target position
      pos= atof(strtok (user_cmd, separator));
      motor->target = pos;

      // allow for setting only the target position without chaning the velocity/torque limits 
      next_value = strtok (NULL, separator);
      if( next_value ){
        vel = atof(next_value);
        motor->velocity_limit = vel;
        motor->P_angle.limit = vel;
  
        // allow for setting only the target position and velocity limit without the torque limit 
        next_value = strtok (NULL, separator);
        if( next_value ){
          torque= atof(next_value);
          motor->PID_velocity.limit = torque;
          // torque command can be voltage or current
          if(!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage) motor->voltage_limit = torque;
          else  motor->current_limit = torque;
        }
      }
      break;
    case MotionControlType::velocity_openloop: // setting velocity target + torque limit
      // set the target
      vel= atof(strtok (user_cmd, separator));
      motor->target = vel;
      // allow for setting only the target velocity without chaning the torque limit
      next_value = strtok (NULL, separator);
      if (next_value ){
        torque = atof(next_value);
        // torque command can be voltage or current
        if(!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
        else  motor->current_limit = torque;
      }
      break;
    case MotionControlType::angle_openloop: // setting angle target + torque, velocity limit
      // set the target
      pos= atof(strtok (user_cmd, separator));
      motor->target = pos; 
      
      // allow for setting only the target position without chaning the velocity/torque limits 
      next_value = strtok (NULL, separator);
      if( next_value ){
        vel = atof(next_value);
        motor->velocity_limit = vel;
        // allow for setting only the target velocity without chaning the torque limit
        next_value = strtok (NULL, separator);
        if (next_value ){
          torque = atof(next_value);
          // torque command can be voltage or current
          if(!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
          else  motor->current_limit = torque;
        }
      }
      break;
  }
  printVerbose(F("Target: "));
  println(motor->target);
}


bool Commander::isSentinel(char ch)
{
  if(ch == eol)
    return true;
  else if (ch == '\r')
  {
      printVerbose(F("Warn: \\r detected! \n"));
  }
  return false;
}

void Commander::print(const int number){
  if( !com_port || verbose == VerboseMode::nothing ) return;
  com_port->print(number);
}
void Commander::print(const float number){
  if(!com_port || verbose == VerboseMode::nothing ) return;
  com_port->print((float)number,(int)decimal_places);
}
void Commander::print(const char* message){
  if(!com_port || verbose == VerboseMode::nothing ) return;
  com_port->print(message);
}
void Commander::print(const __FlashStringHelper *message){
  if(!com_port || verbose == VerboseMode::nothing ) return;
  com_port->print(message);
}
void Commander::print(const char message){
  if(!com_port || verbose == VerboseMode::nothing ) return;
  com_port->print(message);
}

void Commander::println(const int number){
  if(!com_port || verbose == VerboseMode::nothing ) return;
  com_port->println(number);
}
void Commander::println(const float number){
  if(!com_port || verbose == VerboseMode::nothing ) return;
  com_port->println((float)number, (int)decimal_places);
}
void Commander::println(const char* message){
  if(!com_port || verbose == VerboseMode::nothing ) return;
  com_port->println(message);
}
void Commander::println(const __FlashStringHelper *message){
  if(!com_port || verbose == VerboseMode::nothing ) return;
  com_port->println(message);
}
void Commander::println(const char message){
  if(!com_port || verbose == VerboseMode::nothing ) return;
  com_port->println(message);
}


void Commander::printVerbose(const char* message){
  if(verbose == VerboseMode::user_friendly) print(message);
}
void Commander::printVerbose(const __FlashStringHelper *message){
  if(verbose == VerboseMode::user_friendly) print(message);
}
void Commander::printError(){
 println(F("err"));
}
