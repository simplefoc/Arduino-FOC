#include "Commander.h"


Commander::Commander(HardwareSerial& serial){
  com_port = &serial;
}

void Commander::add(char id, CommandCallback onCommand){
  call_list[call_count] = onCommand;
  call_ids[call_count] = id;
  call_count++;
}

void Commander::run(){
  // a string to hold incoming data
  while (com_port->available()) {
    // get the new byte:
    received_chars[rec_cnt] = (char)com_port->read();
    // end of user input
    if (received_chars[rec_cnt++] == '\n') {
      // execute the user command
      char id = received_chars[0];
      if(id == CMD_SCAN)
        for(int i=0; i < call_count; i++){
            com_port->print(call_ids[i]);
            com_port->print(":");
            call_list[i](cmd_scan_msg);
        }
      else
        for(int i=0; i < call_count; i++){
          if(id == call_ids[i]){
            call_list[i](&received_chars[1]);
            break;
          }
        }

      // reset the command buffer 
      received_chars[0] = 0;
      rec_cnt=0;
    }
  }
}

void Commander::motor(FOCMotor* motor, char* user_command) {
  // if empty string
  if( user_command[0] == CMD_SCAN ){
     com_port->println(F("mot"));
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
      com_port->print(F("PID curr q| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_q, &user_command[1]);
      else pid(&motor->PID_current_q,&user_command[1]);
      break;
    case CMD_C_D_PID:      // 
      com_port->print(F("PID curr d| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_d, &user_command[1]);
      else pid(&motor->PID_current_d, &user_command[1]);
      break;
    case CMD_V_PID:      // 
      com_port->print(F("PID vel| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_velocity, &user_command[1]);
      else pid(&motor->PID_velocity, &user_command[1]);
      break;
    case CMD_A_PID:      // 
      com_port->print(F("PID angle| "));
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_angle, &user_command[1]);
      else pid(&motor->P_angle, &user_command[1]);
      break;
    case CMD_LIMITS:      // 
     com_port->print(F("Limits| "));
      switch (sub_cmd){
        case SCMD_LIM_VOLT:      // voltage limit change
          com_port->print(F("volt: "));
          if(!GET) {
            motor->voltage_limit = value;
            motor->PID_current_d.limit = value;
            motor->PID_current_q.limit = value;
            // change velocity pid limit if in voltage mode and no phase resistance set
            if( !_isset(motor->phase_resistance) && motor->torque_controller==TorqueControlType::voltage) motor->PID_velocity.limit = value;
          }
          com_port->println(motor->voltage_limit);
          break;
        case SCMD_LIM_CURR:      // current limit
          com_port->print(F("curr: "));
          if(!GET){
            motor->current_limit = value;
            // if phase resistance is set, change the voltage limit as well.
            if(_isset(motor->phase_resistance)) motor->voltage_limit = value*motor->phase_resistance;
            // if phase resistance specified or the current control is on set the current limit to the velocity PID
            if(_isset(motor->phase_resistance) ||  motor->torque_controller != TorqueControlType::voltage ) motor->PID_velocity.limit = value;
          }
          com_port->println(motor->current_limit);
          break;
        case SCMD_LIM_VEL:      // velocity limit
          com_port->print(F("vel: "));
          if(!GET){
            motor->velocity_limit = value;
            motor->P_angle.limit = value;
          }
          com_port->println(motor->velocity_limit);
          break;
        default:
          com_port->println(F("err"));
          break;
      }
      break;
    case CMD_MOTION_TYPE:
      com_port->print(F("Motion: "));
      switch(sub_cmd){
        case SCMD_DOWNSAMPLE:
            com_port->print(F("downsample: "));
            if(!GET) motor->motion_downsample = value;
            com_port->println(motor->motion_downsample);
          break;
        default:
          // change control type
          if(!GET && value >= 0 && (int)value < 5)// if set command
            motor->controller = (MotionControlType)value;
          switch(motor->controller){
            case MotionControlType::torque:
              com_port->println(F("torque"));
              break;
            case MotionControlType::velocity:
              com_port->println(F("vel"));
              break;
            case MotionControlType::angle:
              com_port->println(F("angle"));
              break;
            case MotionControlType::velocity_openloop:
              com_port->println(F("vel open"));
              break;
            case MotionControlType::angle_openloop:
              com_port->println(F("angle open"));
              break;
          }
            break;
        }
      break;
    case CMD_TORQUE_TYPE:
      // change control type
      com_port->print(F("Torque: "));
      if(!GET && (int8_t)value >= 0 && (int8_t)value < 3)// if set command
        motor->torque_controller = (TorqueControlType)value;
      switch(motor->torque_controller){
        case TorqueControlType::voltage:
          com_port->println(F("volt"));
          break;
        case TorqueControlType::current:
          com_port->println(F("curr"));
          break;
        case TorqueControlType::foc_current:
          com_port->println(F("foc"));
          break;
      }
      break;
    case CMD_STATUS:
      // enable/disable
      com_port->print(F("Status: "));
      if(!GET) (bool)value ? motor->enable() : motor->disable();
       com_port->println(motor->enabled);
      break;
    case CMD_RESIST:
      // enable/disable
      com_port->print(F("R phase: "));
      if(!GET){
        motor->phase_resistance = value;
        if(motor->torque_controller==TorqueControlType::voltage){
          motor->voltage_limit = motor->current_limit*value;
          motor->PID_velocity.limit= motor->current_limit;
        }
      }
      com_port->println(_isset(motor->phase_resistance) ? motor->phase_resistance : 0 );
      break;
    case CMD_SENSOR:
      // Sensor zero offset
       com_port->print(F("Sensor | "));
       switch (sub_cmd){
        case SCMD_SENS_MECH_OFFSET:      // zero offset
          com_port->print(F("offset: "));
          if(!GET) motor->sensor_offset = value;
          com_port->println(motor->sensor_offset);
          break;
        case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
          com_port->print(F("el. offset: "));
          if(!GET) motor->zero_electric_angle = value;
          com_port->println(motor->zero_electric_angle);
          break;
        default:
          com_port->println(F("err"));
          break;
       }
      break;
    case CMD_MONITOR:     // get current values of the state variables
      com_port->print(F("Monitor | "));
      switch (sub_cmd){
        case SCMD_GET:      // get command
          switch((uint8_t)value){
            case 0: // get target
              com_port->print(F("target: "));
              com_port->println(motor->target);
              break;
            case 1: // get voltage q
              com_port->print(F("Vq: "));
              com_port->println(motor->voltage.q);
              break;
            case 2: // get voltage d
              com_port->print(F("Vd: "));
              com_port->println(motor->voltage.q);
              break;
            case 3: // get current q
              com_port->print(F("Cq: "));
              com_port->println(motor->voltage.q);
              break;
            case 4: // get current d
              com_port->print(F("Cd: "));
              com_port->println(motor->voltage.q);
              break;
            case 5: // get velocity
              com_port->print(F("vel: "));
              com_port->println(motor->shaft_velocity);
              break;
            case 6: // get angle
              com_port->print(F("Angle: "));
              com_port->println(motor->shaft_angle);
              break;
            default:
              com_port->println(F("err"));
              break;
          }
          break;
        case SCMD_DOWNSAMPLE:   
          com_port->print(F("downsample: "));
          if(!GET) motor->monitor_downsample = value;
          com_port->println(motor->monitor_downsample);
          break;
        case SCMD_CLEAR:    
          for(int i=0; i<7; i++) motor->monitor_variables[i] = 0; 
          com_port->println(F("clear"));
          break;
        case SCMD_SET:   
          for(int i=0; i<7; i++){
            motor->monitor_variables[i] = user_command[value_index+i] - '0';  
            com_port->print(motor->monitor_variables[i]);
          }
          com_port->println();
          break;
        default:
          com_port->println(F("err"));
          break;
       }
        
      break;
    default:  // target change
      com_port->print(F("Target: "));
      motor->target = atof(user_command);
      com_port->println(motor->target);
  }
}

void Commander::pid(PIDController* pid, char* user_cmd){
  if( user_cmd[0] == CMD_SCAN ){
     com_port->println(F("pid"));
     return;
  }
  char cmd = user_cmd[0];
  bool GET  = user_cmd[1] == '\n';
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_PID_P:      // P gain change
      com_port->print("P: ");
      if(!GET) pid->P = value;
      com_port->println(pid->P);
      break;
    case SCMD_PID_I:      // I gain change
      com_port->print("I: ");
      if(!GET) pid->I = value;
      com_port->println(pid->I);
      break;
    case SCMD_PID_D:      // D gain change
      com_port->print("D: ");
      if(!GET) pid->D = value;
      com_port->println(pid->D);
      break;
    case SCMD_PID_RAMP:      //  ramp change
      com_port->print("ramp: ");
      if(!GET) pid->output_ramp = value;
      com_port->println(pid->output_ramp);
      break;
    case SCMD_PID_LIM:      //  limit change
      com_port->print("limit: ");
      if(!GET) pid->limit = value;
      com_port->println(pid->limit);
      break;
    default:
      com_port->println(F("err"));
      break;
  }
}

void Commander::lpf(LowPassFilter* lpf, char* user_cmd){
  if( user_cmd[0] == CMD_SCAN ){
     com_port->println(F("lpf"));
     return;
  }
  char cmd = user_cmd[0];
  bool GET  = user_cmd[1] == '\n';
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_LPF_TF:      // Tf value change
      com_port->print("Tf: ");
      if(!GET) lpf->Tf = value;
      com_port->println(lpf->Tf);
      break;  
    default:
      com_port->println(F("err"));
      break;
  }
}

void Commander::variable(float* value,  char* user_cmd){
  if( user_cmd[0] == CMD_SCAN ){
     com_port->println(F("var"));
     return;
  }
  bool GET  = user_cmd[0] == '\n';
  if(!GET) *value = atof(user_cmd);
  com_port->println(*value);
}