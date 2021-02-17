#ifndef COMMANDS_h
#define  COMMANDS_h

// list o commands
 #define CMD_C_D_PID  'D'
 #define CMD_C_Q_PID  'Q'
 #define CMD_V_PID  'V'
 #define CMD_A_PID   'A'
 #define CMD_STATUS  'E'
 #define CMD_LIMITS   'L'
 #define CMD_MOTION_TYPE   'C'
 #define CMD_TORQUE_TYPE    'T'
 #define CMD_SENSOR    'S'
 #define CMD_MONITOR  'M'
 #define CMD_RESIST  'R'

// subcomands
 #define SCMD_PID_P   'P'
 #define SCMD_PID_I    'I'
 #define SCMD_PID_D     'D'
 #define SCMD_PID_RAMP  'R'
 #define SCMD_PID_LIM   'L'
 #define SCMD_LPF_TF    'F'
 #define SCMD_LIM_CURR   'C'
 #define SCMD_LIM_VOLT   'U'
 #define SCMD_LIM_VEL   'V'
 #define SCMD_SENS_MECH_OFFSET   'M'
 #define SCMD_SENS_ELEC_OFFSET    'E'
 
#endif