// default configuration values
// change this file to optimal values for your application

#define DEF_POWER_SUPPLY 12.0 //!< default power supply voltage
// velocity PI controller params
#define DEF_PID_VEL_P 0.5 //!< default PID controller P value
#define DEF_PID_VEL_I 10.0 //!<  default PID controller I value
#define DEF_PID_VEL_D 0.0 //!<  default PID controller D value
#define DEF_PID_VEL_U_RAMP 1000.0 //!< default PID controller voltage ramp value
// angle P params
#define DEF_P_ANGLE_P 20.0 //!< default P controller P value
#define DEF_VEL_LIM 20.0 //!< angle velocity limit default
// index search 
#define DEF_INDEX_SEARCH_TARGET_VELOCITY 1.0 //!< default index search velocity
// align voltage
#define DEF_VOLTAGE_SENSOR_ALIGN 6.0 //!< default voltage for sensor and motor zero alignemt
// low pass filter velocity
#define DEF_VEL_FILTER_Tf 0.005 //!< default velocity filter time constant