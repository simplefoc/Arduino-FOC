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
  if(id == CMD_SCAN)
    for(int i=0; i < call_count; i++){
        print(call_ids[i], 0);
        print(":", 0);
        call_list[i](cmd_scan_msg);
    }
  else
    for(int i=0; i < call_count; i++){
      if(id == call_ids[i]){
        call_list[i](&user_input[1]);
        break;
      }
  }
}

void Commander::motor(FOCMotor* motor, char* user_command) {
  // if empty string
  if( user_command[0] == CMD_SCAN ){
     print(F("mot"),1);
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
          print(motor->voltage_limit,1);
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
          print(motor->current_limit,1);
          break;
        case SCMD_LIM_VEL:      // velocity limit
          verbosePrint(F("vel: "));
          if(!GET){
            motor->velocity_limit = value;
            motor->P_angle.limit = value;
          }
          print(motor->velocity_limit,1);
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
            print((int)motor->motion_downsample, 1);
          break;
        default:
          // change control type
          if(!GET && value >= 0 && (int)value < 5)// if set command
            motor->controller = (MotionControlType)value;
          switch(motor->controller){
            case MotionControlType::torque:
              print(F("torque"),1);
              break;
            case MotionControlType::velocity:
              print(F("vel"),1);
              break;
            case MotionControlType::angle:
              print(F("angle"),1);
              break;
            case MotionControlType::velocity_openloop:
              print(F("vel open"),1);
              break;
            case MotionControlType::angle_openloop:
              print(F("angle open"),1);
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
          print(F("volt"),1);
          break;
        case TorqueControlType::current:
          print(F("curr"),1);
          break;
        case TorqueControlType::foc_current:
          print(F("foc"),1);
          break;
      }
      break;
    case CMD_STATUS:
      // enable/disable
      verbosePrint(F("Status: "));
      if(!GET) (bool)value ? motor->enable() : motor->disable();
       print(motor->enabled,1);
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
      if(_isset(motor->phase_resistance)) print(motor->phase_resistance,1);
      else print(0,0);
      break;
    case CMD_SENSOR:
      // Sensor zero offset
       verbosePrint(F("Sensor | "));
       switch (sub_cmd){
        case SCMD_SENS_MECH_OFFSET:      // zero offset
          verbosePrint(F("offset: "));
          if(!GET) motor->sensor_offset = value;
          print(motor->sensor_offset,1);
          break;
        case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
          verbosePrint(F("el. offset: "));
          if(!GET) motor->zero_electric_angle = value;
          print(motor->zero_electric_angle,1);
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
              print(motor->target,1);
              break;
            case 1: // get voltage q
              verbosePrint(F("Vq: "));
              print(motor->voltage.q,1);
              break;
            case 2: // get voltage d
              verbosePrint(F("Vd: "));
              print(motor->voltage.q,1);
              break;
            case 3: // get current q
              verbosePrint(F("Cq: "));
              print(motor->voltage.q,1);
              break;
            case 4: // get current d
              verbosePrint(F("Cd: "));
              print(motor->voltage.q,1);
              break;
            case 5: // get velocity
              verbosePrint(F("vel: "));
              print(motor->shaft_velocity,1);
              break;
            case 6: // get angle
              verbosePrint(F("Angle: "));
              print(motor->shaft_angle,1);
              break;
            default:
              printError();
              break;
          }
          break;
        case SCMD_DOWNSAMPLE:   
          verbosePrint(F("downsample: "));
          if(!GET) motor->monitor_downsample = value;
          print((int)motor->monitor_downsample,1);
          break;
        case SCMD_CLEAR:    
          for(int i=0; i<7; i++) motor->monitor_variables[i] = 0; 
          print(F("clear"),1);
          break;
        case SCMD_SET:   
          for(int i=0; i<7; i++){
            motor->monitor_variables[i] = user_command[value_index+i] - '0';  
            print(motor->monitor_variables[i],0);
          }
          print("",1);
          break;
        default:
          printError();
          break;
       }
      break;
    default:  // target change
      verbosePrint(F("Target: "));
      motor->target = atof(user_command);
      print(motor->target,1);
  }
}

void Commander::pid(PIDController* pid, char* user_cmd){
  if( user_cmd[0] == CMD_SCAN ){
     print(F("pid"),1);
     return;
  }
  char cmd = user_cmd[0];
  bool GET  = user_cmd[1] == '\n';
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_PID_P:      // P gain change
      verbosePrint("P: ");
      if(!GET) pid->P = value;
      print(pid->P,1);
      break;
    case SCMD_PID_I:      // I gain change
      verbosePrint("I: ");
      if(!GET) pid->I = value;
      print(pid->I,1);
      break;
    case SCMD_PID_D:      // D gain change
      verbosePrint("D: ");
      if(!GET) pid->D = value;
      print(pid->D,1);
      break;
    case SCMD_PID_RAMP:      //  ramp change
      verbosePrint("ramp: ");
      if(!GET) pid->output_ramp = value;
      print(pid->output_ramp,1);
      break;
    case SCMD_PID_LIM:      //  limit change
      verbosePrint("limit: ");
      if(!GET) pid->limit = value;
      print(pid->limit,1);
      break;
    default:
      printError();
      break;
  }
}

void Commander::lpf(LowPassFilter* lpf, char* user_cmd){
  if( user_cmd[0] == CMD_SCAN ){
     print(F("lpf"),1);
     return;
  }
  char cmd = user_cmd[0];
  bool GET  = user_cmd[1] == '\n';
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_LPF_TF:      // Tf value change
      verbosePrint(F("Tf: "));
      if(!GET) lpf->Tf = value;
      print(lpf->Tf,1);
      break;  
    default:
      printError();
      break;
  }
}

void Commander::variable(float* value,  char* user_cmd){
  if( user_cmd[0] == CMD_SCAN ){
     print(F("var"),1);
     return;
  }
  bool GET  = user_cmd[0] == '\n';
  if(!GET) *value = atof(user_cmd);
  print(*value,1);
}


void Commander::print(const int number, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println(number);
  else com_port->print(number);
}
void Commander::print(const float number, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println(number,decimal_places);
  else com_port->print(number,decimal_places);
}
void Commander::print(const char* message, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println(message);
  else  com_port->print(message);
}
void Commander::print(const __FlashStringHelper *message, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println(message);
  else  com_port->print(message);
}
void Commander::print(const char message, const bool newline){
  if(!com_port) return;
  if(newline) com_port->println(message);
  else  com_port->print(message);
}


void Commander::verbosePrint(const char* message){
  if(verbose) print(message,0);
}
void Commander::verbosePrint(const __FlashStringHelper *message){
  if(verbose) print(message,0);
}
void Commander::printError(){
 print(F("err"), 1);
}