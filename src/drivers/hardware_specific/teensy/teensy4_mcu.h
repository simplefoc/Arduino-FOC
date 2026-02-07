#ifndef TEENSY4_MCU_DRIVER_H
#define TEENSY4_MCU_DRIVER_H

#include "teensy_mcu.h"

// if defined 
// - Teensy 4.0 
// - Teensy 4.1 
#if defined(__arm__) && defined(CORE_TEENSY) && ( defined(__IMXRT1062__) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY_MICROMOD) )

// teensy 4 driver configuration parameters  
typedef struct Teensy4DriverParams {
  IMXRT_FLEXPWM_t* flextimers[3] = {NULL};
  int submodules[3];
  int channels[6];
  float dead_zone;
} Teensy4DriverParams;


// pin definition from https://github.com/PaulStoffregen/cores/blob/master/teensy4/pwm.c
struct pwm_pin_info_struct {
  uint8_t type;    // 0=no pwm, 1=flexpwm, 2=quad
  uint8_t module;  // 0-3, 0-3
  uint8_t channel; // 0=X, 1=A, 2=B
  uint8_t muxval;  //
};
#define M(a, b) ((((a) - 1) << 4) | (b))
#if defined(__IMXRT1062__)
const struct pwm_pin_info_struct pwm_pin_info[] = {
  {1, M(1, 1), 0, 4},  // FlexPWM1_1_X   0  // AD_B0_03
  {1, M(1, 0), 0, 4},  // FlexPWM1_0_X   1  // AD_B0_02
  {1, M(4, 2), 1, 1},  // FlexPWM4_2_A   2  // EMC_04
  {1, M(4, 2), 2, 1},  // FlexPWM4_2_B   3  // EMC_05
  {1, M(2, 0), 1, 1},  // FlexPWM2_0_A   4  // EMC_06
  {1, M(2, 1), 1, 1},  // FlexPWM2_1_A   5  // EMC_08
  {1, M(2, 2), 1, 2},  // FlexPWM2_2_A   6  // B0_10
  {1, M(1, 3), 2, 6},  // FlexPWM1_3_B   7  // B1_01
  {1, M(1, 3), 1, 6},  // FlexPWM1_3_A   8  // B1_00
  {1, M(2, 2), 2, 2},  // FlexPWM2_2_B   9  // B0_11
  {2, M(1, 0), 0, 1},  // QuadTimer1_0  10  // B0_00
  {2, M(1, 2), 0, 1},  // QuadTimer1_2  11  // B0_02
  {2, M(1, 1), 0, 1},  // QuadTimer1_1  12  // B0_01
  {2, M(2, 0), 0, 1},  // QuadTimer2_0  13  // B0_03
  {2, M(3, 2), 0, 1},  // QuadTimer3_2  14  // AD_B1_02
  {2, M(3, 3), 0, 1},  // QuadTimer3_3  15  // AD_B1_03
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {2, M(3, 1), 0, 1},  // QuadTimer3_1  18  // AD_B1_01
  {2, M(3, 0), 0, 1},  // QuadTimer3_0  19  // AD_B1_00
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(4, 0), 1, 1},  // FlexPWM4_0_A  22  // AD_B1_08
  {1, M(4, 1), 1, 1},  // FlexPWM4_1_A  23  // AD_B1_09
  {1, M(1, 2), 0, 4},  // FlexPWM1_2_X  24  // AD_B0_12
  {1, M(1, 3), 0, 4},  // FlexPWM1_3_X  25  // AD_B0_13
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(3, 1), 2, 1},  // FlexPWM3_1_B  28  // EMC_32
  {1, M(3, 1), 1, 1},  // FlexPWM3_1_A  29  // EMC_31
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(2, 0), 2, 1},  // FlexPWM2_0_B  33  // EMC_07
#ifdef ARDUINO_TEENSY40
  {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  34  // SD_B0_03
  {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  35  // SD_B0_02
  {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  36  // SD_B0_01
  {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  37  // SD_B0_00
  {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  38  // SD_B0_05
  {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  39  // SD_B0_04
#endif
#ifdef ARDUINO_TEENSY41
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(2, 3), 1, 6},  // FlexPWM2_3_A  36  // B1_00
  {1, M(2, 3), 2, 6},  // FlexPWM2_3_B  37  // B1_01
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  42  // SD_B0_03
  {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  43  // SD_B0_02
  {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  44  // SD_B0_01
  {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  45  // SD_B0_00
  {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  46  // SD_B0_05
  {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  47  // SD_B0_04
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_0_B
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_A
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_B
  {1, M(3, 3), 2, 1},  // FlexPWM3_3_B  51  // EMC_22
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_B
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_A
  {1, M(3, 0), 1, 1},  // FlexPWM3_0_A  54  // EMC_29
#endif
#ifdef ARDUINO_TEENSY_MICROMOD
  {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  34  // SD_B0_03
  {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  35  // SD_B0_02
  {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  36  // SD_B0_01
  {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  37  // SD_B0_00
  {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  38  // SD_B0_04
  {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  39  // SD_B0_05
  {2, M(2, 1), 0, 1},  // QuadTimer2_1  40  // B0_04
  {2, M(2, 2), 0, 1},  // QuadTimer2_2  41  // B0_05
  {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_0
  {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_1
  {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_2
  {2, M(4, 0), 0, 1},  // QuadTimer4_0  45  // B0_09
#endif
};

// function finding the flexpwm instance given the submodule
int flexpwm_to_index(IMXRT_FLEXPWM_t* flexpwm);
// find the trigger TRG0 for the given timer and submodule
int flexpwm_submodule_to_trig(IMXRT_FLEXPWM_t* flexpwm, int submodule);
// find the external trigger for the given timer and submodule
int flexpwm_submodule_to_ext_sync(IMXRT_FLEXPWM_t* flexpwm, int submodule);
// function to connecting the triggers
void xbar_connect(unsigned int input, unsigned int output);
// function to initialize the xbar
void xbar_init();

#endif

#endif
#endif