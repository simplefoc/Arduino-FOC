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


void Commander::add(char id, CommandCallback onCommand, const char* label ){
  call_list[call_count] = onCommand;
  call_ids[call_count] = id;
  call_label[call_count] = (char*)label;
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
      memset(received_chars, 0, MAX_COMMAND_LENGTH);
      rec_cnt=0;
    }
    if (rec_cnt>=MAX_COMMAND_LENGTH) { // prevent buffer overrun if message is too long
        memset(received_chars, 0, MAX_COMMAND_LENGTH);
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
          printMachineReadable(CMD_SCAN);
          print(call_ids[i]);
          print(":");
          if(call_label[i]) println(call_label[i]);
          else println("");
      }
      break;
    case CMD_VERBOSE:
      if(!isSentinel(user_input[1])) verbose = (VerboseMode)atoi(&user_input[1]);
      printVerbose(F("Verb:"));
      printMachineReadable(CMD_VERBOSE);
      switch (verbose){
      case VerboseMode::nothing:
        println(F("off!"));
        break;
      case VerboseMode::on_request:
      case VerboseMode::user_friendly:
        println(F("on!"));
        break;
      case VerboseMode::machine_readable:
        printlnMachineReadable(F("machine"));
        break;
      }
      break;
    case CMD_DECIMAL:
      if(!isSentinel(user_input[1])) decimal_places = atoi(&user_input[1]);
      printVerbose(F("Decimal:"));
      printMachineReadable(CMD_DECIMAL);
      println(decimal_places);
      break;
    default:
      for(int i=0; i < call_count; i++){
        if(id == call_ids[i]){
          printMachineReadable(user_input[0]);
          call_list[i](&user_input[1]);
          break;
        }
      }
      break;
  }
}

parsed_user_command Commander::parse_user_command(char* user_command)
{
  const int len = strlen(user_command);
  int data_start = 0;
  int data_end = len;
  for (int i = 0; i < len; i++)
  {
    switch (user_command[i])
    {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    case '+':
    case '-':
    case '.':
    case ',':
    case ' ':
      break;
    case '\n':
    case '\r':
      data_end = min(data_end, i);
      break;

    default:
      data_start = i+1;
      data_end = len;
      break;
    }
  }
  parsed_user_command retval = {
    .value = atof(&user_command[data_start]),
    .command_chars = user_command,
    .num_command_chars = data_start,
    .data_chars = &user_command[data_start],
    .has_data = data_start < data_end,
  };
  return retval;
}

void Commander::motor(FOCMotor* motor, char* user_command) {
  return this->motor(motor, parse_user_command(user_command));
  // if target setting
  if(isDigit(user_command[0]) || user_command[0] == '-' || user_command[0] == '+' || isSentinel(user_command[0])){
    target(motor, user_command);
    return;
  }

  // parse command letter
  char cmd = user_command[0];
  char sub_cmd = user_command[1];
  // check if there is a subcommand or not
  int value_index = (sub_cmd >= 'A'  && sub_cmd <= 'Z') ||  (sub_cmd == '#') ?  2 :  1;
  // check if get command
  bool GET = isSentinel(user_command[value_index]);
  // parse command values
  float value = atof(&user_command[value_index]);
  printMachineReadable(cmd);
  if (sub_cmd >= 'A'  && sub_cmd <= 'Z') {
    printMachineReadable(sub_cmd);
  }
}

void Commander::motor(FOCMotor* motor, parsed_user_command command) {

  if (command.num_command_chars == 0)
  {
    return target(motor, command);
  }

  for (size_t i = 0; i < command.num_command_chars; i++)
  {
    printMachineReadable(command.command_chars[i]);
  }

  // a bit of optimisation of variable memory for Arduino UNO (atmega328)
  switch(command.command_chars[0]){
    case CMD_C_Q_PID:      //
      printVerbose(F("PID curr q| "));
      if(command.command_chars[1] == SCMD_LPF_TF) lpf(&motor->LPF_current_q, command.consume_command_chars(1));
      else pid(&motor->PID_current_q, command.consume_command_chars(1));
      break;
    case CMD_C_D_PID:      //
      printVerbose(F("PID curr d| "));
      if(command.command_chars[1] == SCMD_LPF_TF) lpf(&motor->LPF_current_d, command.consume_command_chars(1));
      else pid(&motor->PID_current_d, command.consume_command_chars(1));
      break;
    case CMD_V_PID:      //
      printVerbose(F("PID vel| "));
      if(command.command_chars[1] == SCMD_LPF_TF) lpf(&motor->LPF_velocity, command.consume_command_chars(1));
      else pid(&motor->PID_velocity, command.consume_command_chars(1));
      break;
    case CMD_A_PID:      //
      printVerbose(F("PID angle| "));
      if(command.command_chars[1] == SCMD_LPF_TF) lpf(&motor->LPF_angle, command.consume_command_chars(1));
      else pid(&motor->P_angle, command.consume_command_chars(1));
      break;
    case CMD_LIMITS:      //
     printVerbose(F("Limits| "));
      switch (command.command_chars[1]){
        case SCMD_LIM_VOLT:      // voltage limit change
          printVerbose(F("volt: "));
          if(command.has_data) {
            motor->updateVoltageLimit(command.value);
          }
          println(motor->voltage_limit);
          break;
        case SCMD_LIM_CURR:      // current limit
          printVerbose(F("curr: "));
          if(command.has_data){
            motor->updateCurrentLimit(command.value);
          }
          println(motor->current_limit);
          break;
        case SCMD_LIM_VEL:      // velocity limit
          printVerbose(F("vel: "));
          if(command.has_data){
            motor->updateVelocityLimit(command.value);
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
      motion(motor, command);
      break;
    case CMD_PWMMOD:
       // PWM modulation change
       printVerbose(F("PWM Mod | "));
       switch (command.command_chars[1]){
        case SCMD_PWMMOD_TYPE:      // zero offset
          printVerbose(F("type: "));
          if(command.has_data) motor->foc_modulation = (FOCModulationType)command.value;
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
          if(command.has_data) motor->modulation_centered = command.value;
          println(motor->modulation_centered);
          break;
        default:
          printError();
          break;
       }
      break;
    case CMD_RESIST:
      printVerbose(F("R phase: "));
      if(command.has_data){
        motor->phase_resistance = command.value;
      }
      if(_isset(motor->phase_resistance)) println(motor->phase_resistance);
      else println(0);
      break;
    case CMD_INDUCTANCE:
      printVerbose(F("L phase: "));
      switch (command.command_chars[1]){
        case SCMD_INDUCT_D:
          printVerbose(F("d: "));
          if(command.has_data){
            motor->axis_inductance.d = command.value;
            motor->phase_inductance = command.value;
          }
          if(_isset(motor->axis_inductance.d)) println(motor->axis_inductance.d);
          else println(0);
          break;
        case SCMD_INDUCT_Q:
          printVerbose(F("q: "));
          if(command.has_data){
            motor->axis_inductance.q = command.value;
          }
          if(_isset(motor->axis_inductance.q)) println(motor->axis_inductance.q);
          else println(0);
          break;
        default:
          if(command.has_data){
            motor->phase_inductance = command.value;
            motor->axis_inductance.d = command.value;
            motor->axis_inductance.q = command.value;
          }
          if(_isset(motor->phase_inductance)) println(motor->phase_inductance);
          else println(0);
          break;
      }
      break;
    case CMD_KV_RATING:
      printVerbose(F("Motor KV: "));
      if(command.has_data){
        motor->KV_rating = command.value;
      }
      if(_isset(motor->KV_rating)) println(motor->KV_rating);
      else println(0);
      break;
    case CMD_SENSOR:
      // Sensor zero offset
      printVerbose(F("Sensor | "));
      switch (command.command_chars[1]){
        case SCMD_SENS_MECH_OFFSET:      // zero offset
          printVerbose(F("offset: "));
          if(command.has_data) motor->sensor_offset = command.value;
          println(motor->sensor_offset);
          break;
        case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
          printVerbose(F("el. offset: "));
          if(command.has_data) motor->zero_electric_angle = command.value;
          println(motor->zero_electric_angle);
          break;
        case SCMD_SENS_VEL_DT:
          printVerbose(F("vel dt: "));
          if(command.has_data) motor->sensor->min_elapsed_time = command.value;
          println(motor->sensor->min_elapsed_time);
          break;
        default:
          printError();
          break;
      }
      break;
    case CMD_FOC_PARAMS:
      printVerbose(F("MOT | "));
      switch (command.command_chars[1]){
        case SCMD_REINIT_FOC:
          printVerbose(F("Reinit!"));
          motor->sensor_direction = Direction::UNKNOWN; // reset sensor direction estimation
          motor->zero_electric_angle = NOT_SET; // reset zero electric angle
          motor->target = 0; // reset target
          motor->initFOC();
          println(F("done"));
          break;
        case SCMD_MEAS_PARAMS:
          printVerbose(F("Meas motor params!"));
          {
            int res = motor->characteriseMotor(command.value, 1.5f);
            if(res == 0){
              println(F("done"));
            } else {
              printVerbose(F("failed, err code: "));
              println(res);
            }
          }
          break;
        case SCMD_TUNE_CURR:
          printVerbose(F("PI tune curr.| "));
          {
            int res = motor->tuneCurrentController(command.value);
            if(res == 0){
              println(F("done"));
            } else {
              printVerbose(F("failed, err code: "));
              println(res);
            }
          }
          break;
        default:
          printError();
          break;
      }
      break;
    case CMD_MONITOR:     // get current values of the state variables
      printVerbose(F("Monitor | "));
      switch (command.command_chars[1]){
        case SCMD_GET:      // get command
          switch((uint8_t)command.value){
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
          if(command.has_data) motor->monitor_downsample = command.value;
          println((int)motor->monitor_downsample);
          break;
        case SCMD_CLEAR:
          motor->monitor_variables = (uint8_t) 0;
          println(F("clear"));
          break;
        case CMD_DECIMAL:
          printVerbose(F("decimal: "));
          motor->monitor_decimals = command.value;
          println((int)motor->monitor_decimals);
          break;
        case SCMD_SET:
          if(command.has_data){
            // set the variables
            motor->monitor_variables = (uint8_t) 0;
            for(int i = 0; i < 7; i++){
              if(isSentinel(command.data_chars[i])) break;
              motor->monitor_variables |=  (command.data_chars[i] - '0') << (6-i);
            }
          }
          // print the variables
          for(int i = 0; i < 7; i++){
            print( (motor->monitor_variables & (1 << (6-i))) >> (6-i));
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
  return motion(motor, parse_user_command(user_cmd), separator);
  char cmd = user_cmd[0];
  char sub_cmd = user_cmd[1];
  int value_index = (sub_cmd == SCMD_DOWNSAMPLE) ?  2 :  1;
  bool GET  = isSentinel(user_cmd[value_index]);
  float value = atof(&user_cmd[(sub_cmd >= 'A'  && sub_cmd <= 'Z') ?  2 :  1]);
}

void Commander::motion(FOCMotor* motor, parsed_user_command command, char* separator){
  switch(command.command_chars[0]){
    case CMD_MOTION_TYPE:
      printVerbose(F("Motion:"));
      switch(command.command_chars[1]){
        case SCMD_TIME:
          printVerbose(F(" time: "));
          println((int)motor->move_time_us);
          break;
        case SCMD_DOWNSAMPLE:
            printVerbose(F(" downsample: "));
            if(command.has_data) motor->motion_downsample = command.value;
            println((int)motor->motion_downsample);
          break;
        default:
          // change control type
          if(command.has_data && command.value >= 0 && (int)command.value < 7) // if set command
            motor->updateMotionControlType((MotionControlType)command.value); // update motion control type
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
            case MotionControlType::angle_nocascade:
              println(F("angle nocascade"));
              break;
            case MotionControlType::custom:
              println(F("custom"));
              break;
          }
          break;
        }
      break;
    case CMD_TORQUE_TYPE:
      // change control type
      printVerbose(F("Torque: "));
      switch(command.command_chars[1]){
        case SCMD_TIME:
          printVerbose(F(" time: "));
          println((int)motor->loopfoc_time_us);
          break;
        default:
          if(command.has_data && (int8_t)command.value >= 0 && (int8_t)command.value < 4) // if set command
            motor->updateTorqueControlType((TorqueControlType)command.value); // update torque control type
          switch(motor->torque_controller){
            case TorqueControlType::voltage:
              println(F("volt"));
              break;
            case TorqueControlType::dc_current:
              println(F("dc curr"));
              break;
            case TorqueControlType::foc_current:
              println(F("foc curr"));
              break;
            case TorqueControlType::estimated_current:
              println(F("est. curr"));
              break;
            default:
              printError();
          }
          break;
        }
      break;
    case CMD_STATUS:
      // enable/disable
      printVerbose(F("Status: "));
      if(command.has_data) (bool)command.value ? motor->enable() : motor->disable();
       println(motor->enabled);
      break;
    default:
      target(motor,  command, separator);
      break;
  }
}

void Commander::pid(PIDController* pid, char* user_cmd){
  return this->pid(pid, parse_user_command(user_cmd));
  char cmd = user_cmd[0];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);
}

void Commander::pid(PIDController* pid, parsed_user_command command){
  if (command.num_command_chars == 0)
  {
    printError();
    return;
  }
  
  switch (command.command_chars[0]){
    case SCMD_PID_P:      // P gain change
      printVerbose("P: ");
      if(command.has_data) pid->P = command.value;
      println(pid->P);
      break;
    case SCMD_PID_I:      // I gain change
      printVerbose("I: ");
      if(command.has_data) pid->I = command.value;
      println(pid->I);
      break;
    case SCMD_PID_D:      // D gain change
      printVerbose("D: ");
      if(command.has_data) pid->D = command.value;
      println(pid->D);
      break;
    case SCMD_PID_RAMP:      //  ramp change
      printVerbose("ramp: ");
      if(command.has_data) pid->output_ramp = command.value;
      println(pid->output_ramp);
      break;
    case SCMD_PID_LIM:      //  limit change
      printVerbose("limit: ");
      if(command.has_data) pid->limit = command.value;
      println(pid->limit);
      break;
    default:
      printError();
      break;
  }
}

void Commander::lpf(LowPassFilter* lpf, char* user_cmd){
  return this->lpf(lpf, parse_user_command(user_cmd));
  char cmd = user_cmd[0];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);
}
void Commander::lpf(LowPassFilter* lpf, parsed_user_command command){
  if (command.num_command_chars == 0)
  {
    printError();
    return;
  }

  switch (command.command_chars[0]){
    case SCMD_LPF_TF:      // Tf value change
      printVerbose(F("Tf: "));
      if(command.has_data) lpf->Tf = command.value;
      println(lpf->Tf);
      break;
    default:
      printError();
      break;
  }
}

void Commander::scalar(float* value,  char* user_cmd){
  return scalar(value, parse_user_command(user_cmd));
}

void Commander::scalar(float* value,  parsed_user_command command){
  if(command.has_data) *value = command.value;
  println(*value);
}


void Commander::target(FOCMotor* motor,  char* user_cmd, char* separator){
  return target(motor, parse_user_command(user_cmd), separator);
}

void Commander::target(FOCMotor* motor, parsed_user_command command, char* separator){
  // if no values sent
  if(command.num_command_chars == 0 && !command.has_data) {
    printlnMachineReadable(motor->target);
    return;
  };

  float pos, vel, torque;
  char* next_value;
  switch(motor->controller){
    case MotionControlType::custom:
    case MotionControlType::angle_nocascade:
    case MotionControlType::torque: // setting torque target
      torque = atof(strtok (command.data_chars, separator));
      motor->target = torque;
      break;
    case MotionControlType::velocity: // setting velocity target + torque limit
      // set the target
      vel= atof(strtok (command.data_chars, separator));
      motor->target = vel;

      // allow for setting only the target velocity without chaning the torque limit
      next_value = strtok (NULL, separator);
      if (next_value){
        torque = atof(next_value);
        // torque command can be voltage or current
        if(motor->torque_controller == TorqueControlType::voltage) motor->updateVoltageLimit(torque);
        else  motor->updateCurrentLimit(torque);
      }
      break;
    case MotionControlType::angle: // setting angle target + torque, velocity limit
      // setting the target position
      pos= atof(strtok (command.data_chars, separator));
      motor->target = pos;

      // allow for setting only the target position without chaning the velocity/torque limits 
      next_value = strtok (NULL, separator);
      if( next_value ){
        vel = atof(next_value);
        motor->updateVelocityLimit(vel);
  
        // allow for setting only the target position and velocity limit without the torque limit 
        next_value = strtok (NULL, separator);
        if( next_value ){
          torque= atof(next_value);
          // torque command can be voltage or current
          if(motor->torque_controller == TorqueControlType::voltage) motor->updateVoltageLimit(torque);
          else  motor->updateCurrentLimit(torque);
        }
      }
      break;
    case MotionControlType::velocity_openloop: // setting velocity target + torque limit
      // set the target
      vel= atof(strtok (command.data_chars, separator));
      motor->target = vel;
      // allow for setting only the target velocity without chaning the torque limit
      next_value = strtok (NULL, separator);
      if (next_value ){
        torque = atof(next_value);
        // torque command can be voltage or current
        if(motor->torque_controller == TorqueControlType::voltage) motor->updateVoltageLimit(torque);
        else  motor->updateCurrentLimit(torque);
      }
      break;
    case MotionControlType::angle_openloop: // setting angle target + torque, velocity limit
      // set the target
      pos= atof(strtok (command.data_chars, separator));
      motor->target = pos; 
      
      // allow for setting only the target position without chaning the velocity/torque limits 
      next_value = strtok (NULL, separator);
      if( next_value ){
        vel = atof(next_value);
        motor->updateVelocityLimit(vel);
        // allow for setting only the target velocity without chaning the torque limit
        next_value = strtok (NULL, separator);
        if (next_value ){
          torque = atof(next_value);
          // torque command can be voltage or current
          if(motor->torque_controller == TorqueControlType::voltage) motor->updateVoltageLimit(torque);
          else  motor->updateCurrentLimit(torque);
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
      return true; // lets still consider it to end the line...
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

void Commander::printMachineReadable(const int number){
  if(verbose == VerboseMode::machine_readable) print(number);
}
void Commander::printMachineReadable(const float number){
  if(verbose == VerboseMode::machine_readable) print(number);
}
void Commander::printMachineReadable(const char* message){
  if(verbose == VerboseMode::machine_readable) print(message);
}
void Commander::printMachineReadable(const __FlashStringHelper *message){
  if(verbose == VerboseMode::machine_readable) print(message);
}
void Commander::printMachineReadable(const char message){
  if(verbose == VerboseMode::machine_readable) print(message);
}

void Commander::printlnMachineReadable(const int number){
  if(verbose == VerboseMode::machine_readable) println(number);
}
void Commander::printlnMachineReadable(const float number){
  if(verbose == VerboseMode::machine_readable) println(number);
}
void Commander::printlnMachineReadable(const char* message){
  if(verbose == VerboseMode::machine_readable) println(message);
}
void Commander::printlnMachineReadable(const __FlashStringHelper *message){
  if(verbose == VerboseMode::machine_readable) println(message);
}
void Commander::printlnMachineReadable(const char message){
  if(verbose == VerboseMode::machine_readable) println(message);
}

void Commander::printError(){
 println(F("err"));
}
