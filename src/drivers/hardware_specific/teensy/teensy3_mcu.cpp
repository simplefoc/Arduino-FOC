#include "teensy_mcu.h"

// if defined 
// - Teensy 3.0 MK20DX128
// - Teensy 3.1/3.2 MK20DX256
// - Teensy 3.5 MK20DX128
// - Teensy LC MKL26Z64
// - Teensy 3.5 MK64FX512
// - Teensy 3.6 MK66FX1M0
#if defined(__arm__) && defined(CORE_TEENSY) &&  (defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__))


// pin definition from https://github.com/PaulStoffregen/cores/blob/286511f3ec849a6c9e0ec8b73ad6a2fada52e44c/teensy3/pins_teensy.c
#if defined(__MK20DX128__)
#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM0_CH6_PIN 21
#define FTM0_CH7_PIN  5
#define FTM1_CH0_PIN  3
#define FTM1_CH1_PIN  4
#elif defined(__MK20DX256__)
#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM0_CH6_PIN 21
#define FTM0_CH7_PIN  5
#define FTM1_CH0_PIN  3
#define FTM1_CH1_PIN  4
#define FTM2_CH0_PIN 32
#define FTM2_CH1_PIN 25
#elif defined(__MKL26Z64__)
#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM1_CH0_PIN 16
#define FTM1_CH1_PIN 17
#define FTM2_CH0_PIN  3
#define FTM2_CH1_PIN  4
#elif defined(__MK64FX512__)
#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM0_CH6_PIN 21
#define FTM0_CH7_PIN  5
#define FTM1_CH0_PIN  3
#define FTM1_CH1_PIN  4
#define FTM2_CH0_PIN 29
#define FTM2_CH1_PIN 30
#define FTM3_CH0_PIN  2
#define FTM3_CH1_PIN 14
#define FTM3_CH2_PIN  7
#define FTM3_CH3_PIN  8
#define FTM3_CH4_PIN 35
#define FTM3_CH5_PIN 36
#define FTM3_CH6_PIN 37
#define FTM3_CH7_PIN 38
#elif defined(__MK66FX1M0__)
#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM0_CH6_PIN 21
#define FTM0_CH7_PIN  5
#define FTM1_CH0_PIN  3
#define FTM1_CH1_PIN  4
#define FTM2_CH0_PIN 29
#define FTM2_CH1_PIN 30
#define FTM3_CH0_PIN  2
#define FTM3_CH1_PIN 14
#define FTM3_CH2_PIN  7
#define FTM3_CH3_PIN  8
#define FTM3_CH4_PIN 35
#define FTM3_CH5_PIN 36
#define FTM3_CH6_PIN 37
#define FTM3_CH7_PIN 38
#define TPM1_CH0_PIN 16
#define TPM1_CH1_PIN 17
#endif

int _findTimer( const int Ah, const int Al,  const int Bh, const int Bl, const int Ch, const int Cl){

  if((Ah == FTM0_CH0_PIN && Al == FTM0_CH1_PIN) || 
    (Ah == FTM0_CH2_PIN && Al == FTM0_CH3_PIN) ||
    (Ah == FTM0_CH4_PIN && Al == FTM0_CH5_PIN) ){
      if((Bh == FTM0_CH0_PIN && Bl == FTM0_CH1_PIN) || 
        (Bh == FTM0_CH2_PIN && Bl == FTM0_CH3_PIN) ||
        (Bh == FTM0_CH4_PIN && Bl == FTM0_CH5_PIN) ){
         if((Ch == FTM0_CH0_PIN && Cl == FTM0_CH1_PIN) || 
          (Ch == FTM0_CH2_PIN && Cl == FTM0_CH3_PIN) ||
          (Ch == FTM0_CH4_PIN && Cl == FTM0_CH5_PIN) ){
#ifdef SIMPLEFOC_TEENSY_DEBUG
                SIMPLEFOC_DEBUG("TEENSY-DRV: Using timer FTM0.");
#endif
            // timer FTM0 
            return 0;
        }
      }
  }

  #ifdef FTM3_SC // if the board has FTM3 timer
    if((Ah == FTM3_CH0_PIN && Al == FTM3_CH1_PIN) || 
      (Ah == FTM3_CH2_PIN && Al == FTM3_CH3_PIN) ||
      (Ah == FTM3_CH4_PIN && Al == FTM3_CH5_PIN) ){
        if((Bh == FTM3_CH0_PIN && Bl == FTM3_CH1_PIN) || 
          (Bh == FTM3_CH2_PIN && Bl == FTM3_CH3_PIN) ||
          (Bh == FTM3_CH4_PIN && Bl == FTM3_CH5_PIN) ){
          if((Ch == FTM3_CH0_PIN && Cl == FTM3_CH1_PIN) || 
            (Ch == FTM3_CH2_PIN && Cl == FTM3_CH3_PIN) ||
            (Ch == FTM3_CH4_PIN && Cl == FTM3_CH5_PIN) ){
              // timer FTM3 
#ifdef SIMPLEFOC_TEENSY_DEBUG
                SIMPLEFOC_DEBUG("TEENSY-DRV: Using timer FTM3.");
#endif
              return 3;
          }
        }
    }
  #endif
  
#ifdef SIMPLEFOC_TEENSY_DEBUG
  SIMPLEFOC_DEBUG("TEENSY-DRV: ERR: Pins not on timers FTM0 or FTM3!");
#endif
  return -1;

}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 6PWM setting
// - hardware specific
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  unsigned long pwm_freq = 2*pwm_frequency; // center-aligned pwm has 4 times lower freq
  _setHighFrequency(pwm_freq, pinA_h);
  _setHighFrequency(pwm_freq, pinA_l);
  _setHighFrequency(pwm_freq, pinB_h);
  _setHighFrequency(pwm_freq, pinB_l);
  _setHighFrequency(pwm_freq, pinC_h);
  _setHighFrequency(pwm_freq, pinC_l);

  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA_h,pinA_l, pinB_h,pinB_l, pinC_h, pinC_l },
    .pwm_frequency = pwm_frequency
  };


  int timer = _findTimer(pinA_h,pinA_l,pinB_h,pinB_l,pinC_h,pinC_l);
  if(timer<0) return SIMPLEFOC_DRIVER_INIT_FAILED;

  // find the best combination of prescalers and counter value
  double dead_time = dead_zone/pwm_freq;
  int prescaler = 1; // initial prescaler (1,4 or 16)
  double count = 1; // inital count (1 - 63)
  for (; prescaler<=16; prescaler*=4){
    count = dead_time*((double)F_CPU)/((double)prescaler);
    if(count < 64) break; // found the solution
  }
  count = _constrain(count, 1, 63);

  // configure the timer 
  if(timer==0){
    // Configure FTM0
    // // inverting and deadtime insertion for FTM1
    FTM0_COMBINE = 0x00121212; // 0x2 - complemetary mode, 0x1 - dead timer insertion enabled

    // Deadtime config
    FTM0_DEADTIME = (int)count; // set counter - 1-63
    FTM0_DEADTIME |= ((prescaler>1) << 7) | ((prescaler>4) << 6); // set prescaler (0b01 - 1, 0b10 - 4, 0b11 - 16)

    // configure center aligned PWM
    FTM0_SC = 0x00000028; // 0x2 - center-alignment, 0x8 - fixed clock freq
  }else if(timer==3){
    // Configure FTM3
    // inverting and deadtime insertion for FTM1
    FTM3_COMBINE = 0x00121212; // 0x2 - complemetary mode, 0x1 - dead timer insertion enabled

    // Deadtime config
    FTM3_DEADTIME = (int)count; // set counter - 1-63
    FTM3_DEADTIME |= ((prescaler>1) << 7) | ((prescaler>4) << 6); // set prescaler (0b01 - 1, 0b10 - 4, 0b11 - 16)

    // configure center aligned PWM
    FTM3_SC = 0x00000028; // 0x2 - center-alignment, 0x8 - fixed clock freq
  }
  
  return params;
}



// function setting the pwm duty cycle to the hardware
// - Stepper motor - 6PWM setting
// - hardware specific
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, void* params){
  // transform duty cycle from [0,1] to [0,255]
  // phase A
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_a);

  // phase B
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_b);
  analogWrite(((GenericDriverParams*)params)->pins[3], 255.0f*dc_b);

  // phase C
  analogWrite(((GenericDriverParams*)params)->pins[4], 255.0f*dc_c);
  analogWrite(((GenericDriverParams*)params)->pins[5], 255.0f*dc_c);
}
#endif