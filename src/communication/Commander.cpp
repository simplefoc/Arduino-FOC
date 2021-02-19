#include "Commander.h"


Commander::Commander(HardwareSerial& serial){
  com_port = &serial;
}
Commander::Commander(){
  // do nothing
}


void Commander::add(char id, CommandCallback onCommand){
  call_list[call_count] = onCommand;
  call_ids[call_count] = id;
  call_count++;
}


void Commander::run(){
  if(!com_port) return;
  // a string to hold incoming data
  while (com_port->available()) {
    // get the new byte:
    received_chars[rec_cnt] = (char)com_port->read();
    // end of user input
    if (received_chars[rec_cnt++] == '\n') {
      // execute the user command
      run(received_chars);

      // reset the command buffer 
      received_chars[0] = 0;
      rec_cnt=0;
    }
  }
}

void Commander::run(HardwareSerial &serial){
  // a string to hold incoming data
  while (serial.available()) {
    // get the new byte:
    received_chars[rec_cnt] = (char)serial.read();
    // end of user input
    if (received_chars[rec_cnt++] == '\n') {
      // execute the user command
      run(received_chars);

      // reset the command buffer 
      received_chars[0] = 0;
      rec_cnt=0;
    }
  }
}

void Commander::run(char* user_input){
  // execute the user command
  char id = user_input[0];
  switch(id){
    case CMD_SCAN:
      for(int i=0; i < call_count; i++){
          dump(call_ids[i], 0);
          dump(":", 0);
          call_list[i](cmd_scan_msg);
      }
      break;
    case CMD_VERBOSE:
      if(user_input[1] != '\n') verbose = atoi(&user_input[1]);
      if(verbose) dump(F("Verb. on!"),1);
      else dump(F("Verb. off"),1);
      break;
    case CMD_DECIMAL:
      if(user_input[1] != '\n') decimal_places = atoi(&user_input[1]);
      verbosePrint(F("Decimal:"));
      dump(decimal_places,1);
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
  // if empty string
  if( user_command[0] == CMD_SCAN ){
     dump(F("mot"),1);
     return;
  }

  // parse command letter
  char cmd = user_command[0];
  char sub_cmd = user_command[1];
  int value_index = (sub_cmd >= 'A'  && sub_cmd <= 'Z') ?  2 :  1;
  // check if get command
  bool GET = user_command[value_index] == '\n';
  // parse command values
  float  value  = atof(&user_command[value_index]);

  // a bit of optimisation of variable memory for Arduino UNO (atmega328)
  switch(cmd){
    case CMD_C_Q_PID:      // 
      verbosePrint(F("PID curr q| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_q, &user_command[1]);
      else pid(&motor->PID_current_q,&user_command[1]);
      break;
    case CMD_C_D_PID:      // 
      verbosePrint(F("PID curr d| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_d, &user_command[1]);
      else pid(&motor->PID_current_d, &user_command[1]);
      break;
    case CMD_V_PID:      // 
      verbosePrint(F("PID vel| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_velocity, &user_command[1]);
      else pid(&motor->PID_velocity, &user_command[1]);
      break;
    case CMD_A_PID:      // 
      verbosePrint(F("PID angle| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_angle, &user_command[1]);
      else pid(&motor->P_angle, &user_command[1]);
      break;
    case CMD_LIMITS:      // 
     verbosePrint(F("Limits| "));
      switch (sub_cmd){
        case SCMD_LIM_VOLT:      // voltage limit change
          verbosePrint(F("volt: "));
          if(!GET) {
            motor->voltage_limit = value;
            motor->PID_current_d.limit = value;
            motor->PID_current_q.limit = value;
            // change velocity pid limit if in voltage mode and no phase resistance set
            if( !_isset(motor->phase_resistance) && motor->torque_controller==TorqueControlType::voltage) motor->PID_velocity.limit = value;
          }
          dump(motor->voltage_limit,1);
          break;
        case SCMD_LIM_CURR:      // current limit
          verbosePrint(F("curr: "));
          if(!GET){
            motor->current_limit = value;
            // if phase resistance is set, change the voltage limit as well.
            if(_isset(motor->phase_resistance)) motor->voltage_limit = value*motor->phase_resistance;
            // if phase resistance specified or the current control is on set the current limit to the velocity PID
            if(_isset(motor->phase_resistance) ||  motor->torque_controller != TorqueControlType::voltage ) motor->PID_velocity.limit = value;
          }
          dump(motor->current_limit,1);
          break;
        case SCMD_LIM_VEL:      // velocity limit
          verbosePrint(F("vel: "));
          if(!GET){
            motor->velocity_limit = value;
            motor->P_angle.limit = value;
          }
          dump(motor->velocity_limit,1);
          break;
        default:
          printError();
          break;
      }
      break;
    case CMD_MOTION_TYPE:
      verbosePrint(F("Motion: "));
      switch(sub_cmd){
        case SCMD_DOWNSAMPLE:
            verbosePrint(F("downsample: "));
            if(!GET) motor->motion_downsample = value;
            dump((int)motor->motion_downsample, 1);
          break;
        default:
          // change control type
          if(!GET && value >= 0 && (int)value < 5)// if set command
            motor->controller = (MotionControlType)value;
          switch(motor->controller){
            case MotionControlType::torque:
              dump(F("torque"),1);
              break;
            case MotionControlType::velocity:
              dump(F("vel"),1);
              break;
            case MotionControlType::angle:
              dump(F("angle"),1);
              break;
            case MotionControlType::velocity_openloop:
              dump(F("vel open"),1);
              break;
            case MotionControlType::angle_openloop:
              dump(F("angle open"),1);
              break;
          }
            break;
        }
      break;
    case CMD_TORQUE_TYPE:
      // change control type
      verbosePrint(F("Torque: "));
      if(!GET && (int8_t)value >= 0 && (int8_t)value < 3)// if set command
        motor->torque_controller = (TorqueControlType)value;
      switch(motor->torque_controller){
        case TorqueControlType::voltage:
          dump(F("volt"),1);
          break;
        case TorqueControlType::current:
          dump(F("curr"),1);
          break;
        case TorqueControlType::foc_current:
          dump(F("foc"),1);
          break;
      }
      break;
    case CMD_STATUS:
      // enable/disable
      verbosePrint(F("Status: "));
      if(!GET) (bool)value ? motor->enable() : motor->disable();
       dump(motor->enabled,1);
      break;
    case CMD_RESIST:
      // enable/disable
      verbosePrint(F("R phase: "));
      if(!GET){
        motor->phase_resistance = value;
        if(motor->torque_controller==TorqueControlType::voltage){
          motor->voltage_limit = motor->current_limit*value;
          motor->PID_velocity.limit= motor->current_limit;
        }
      }
      if(_isset(motor->phase_resistance)) dump(motor->phase_resistance,1);
      else dump(0,0);
      break;
    case CMD_SENSOR:
      // Sensor zero offset
       verbosePrint(F("Sensor | "));
       switch (sub_cmd){
        case SCMD_SENS_MECH_OFFSET:      // zero offset
          verbosePrint(F("offset: "));
          if(!GET) motor->sensor_offset = value;
          dump(motor->sensor_offset,1);
          break;
        case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
          verbosePrint(F("el. offset: "));
          if(!GET) motor->zero_electric_angle = value;
          dump(motor->zero_electric_angle,1);
          break;
        default:
          printError();
          break;
       }
      break;
    case CMD_MONITOR:     // get current values of the state variables
      verbosePrint(F("Monitor | "));
      switch (sub_cmd){
        case SCMD_GET:      // get command
          switch((uint8_t)value){
            case 0: // get target
              verbosePrint(F("target: "));
              dump(motor->target,1);
              break;
            case 1: // get voltage q
              verbosePrint(F("Vq: "));
              dump(motor->voltage.q,1);
              break;
            case 2: // get voltage d
              verbosePrint(F("Vd: "));
              dump(motor->voltage.q,1);
              break;
            case 3: // get current q
              verbosePrint(F("Cq: "));
              dump(motor->voltage.q,1);
              break;
            case 4: // get current d
              verbosePrint(F("Cd: "));
              dump(motor->voltage.q,1);
              break;
            case 5: // get velocity
              verbosePrint(F("vel: "));
              dump(motor->shaft_velocity,1);
              break;
            case 6: // get angle
              verbosePrint(F("Angle: "));
              dump(motor->shaft_angle,1);
              break;
            default:
              printError();
              break;
          }
          break;
        case SCMD_DOWNSAMPLE:   
          verbosePrint(F("downsample: "));
          if(!GET) motor->monitor_downsample = value;
          dump((int)motor->monitor_downsample,1);
          break;
        case SCMD_CLEAR:    
          for(int i=0; i<7; i++) motor->monitor_variables[i] = 0; 
          dump(F("clear"),1);
          break;
        case SCMD_SET:   
          for(int i=0; i<7; i++){
            motor->monitor_variables[i] = user_command[value_index+i] - '0';  
            dump(motor->monitor_variables[i],0);
          }
          dump("",1);
          break;
        default:
          printError();
          break;
       }
      break;
    default:  // target change
      verbosePrint(F("Target: "));
      motor->target = atof(user_command);
      dump(motor->target,1);
  }
}

void Commander::pid(PIDController* pid, char* user_cmd){
  if( user_cmd[0] == CMD_SCAN ){
     dump(F("pid"),1);
     return;
  }
  char cmd = user_cmd[0];
  bool GET  = user_cmd[1] == '\n';
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_PID_P:      // P gain change
      verbosePrint("P: ");
      if(!GET) pid->P = value;
      dump(pid->P,1);
      break;
    case SCMD_PID_I:      // I gain change
      verbosePrint("I: ");
      if(!GET) pid->I = value;
      dump(pid->I,1);
      break;
    case SCMD_PID_D:      // D gain change
      verbosePrint("D: ");
      if(!GET) pid->D = value;
      dump(pid->D,1);
      break;
    case SCMD_PID_RAMP:      //  ramp change
      verbosePrint("ramp: ");
      if(!GET) pid->output_ramp = value;
      dump(pid->output_ramp,1);
      break;
    case SCMD_PID_LIM:      //  limit change
      verbosePrint("limit: ");
      if(!GET) pid->limit = value;
      dump(pid->limit,1);
      break;
    default:
      printError();
      break;
  }
}

void Commander::lpf(LowPassFilter* lpf, char* user_cmd){
  if( user_cmd[0] == CMD_SCAN ){
     dump(F("lpf"),1);
     return;
  }
  char cmd = user_cmd[0];
  bool GET  = user_cmd[1] == '\n';
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_LPF_TF:      // Tf value change
      verbosePrint(F("Tf: "));
      if(!GET) lpf->Tf = value;
      dump(lpf->Tf,1);
      break;  
    default:
      printError();
      break;
  }
}

void Commander::variable(float* value,  char* user_cmd){
  if( user_cmd[0] == CMD_SCAN ){
     dump(F("var"),1);
     return;
  }
  bool GET  = user_cmd[0] == '\n';
  if(!GET) *value = atof(user_cmd);
  dump(*value,1);
}


void Commander::dump(const int number, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println(number);
  else com_port->print(number);
}
void Commander::dump(const float number, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println((float)number,(int)decimal_places);
  else com_port->print((float)number,(int)decimal_places);
}
void Commander::dump(const char* message, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println(message);
  else  com_port->print(message);
}
void Commander::dump(const __FlashStringHelper *message, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println(message);
  else  com_port->print(message);
}
void Commander::dump(const char message, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println(message);
  else  com_port->print(message);
}


void Commander::verbosePrint(const char* message){
  if(verbose) dump(message,0);
}
void Commander::verbosePrint(const __FlashStringHelper *message){
  if(verbose) dump(message,0);
}
void Commander::printError(){
 dump(F("err"), 1);
}