#include "teensy_mcu.h"
#include "teensy4_mcu.h"
#include "../../../communication/SimpleFOCDebug.h"

// if defined 
// - Teensy 4.0 
// - Teensy 4.1 
#if defined(__arm__) && defined(CORE_TEENSY) && ( defined(__IMXRT1062__) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY_MICROMOD) )

// half_cycle of the PWM variable
int half_cycle = 0;


// function which finds the flexpwm instance for a pair of pins
// if they do not belong to the same timer it returns a nullpointer
IMXRT_FLEXPWM_t* get_flexpwm(uint8_t pin, uint8_t pin1){
 
  const struct pwm_pin_info_struct *info;
  if (pin >= CORE_NUM_DIGITAL || pin1 >= CORE_NUM_DIGITAL) {
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: ERR: Pins: %d, %d not Flextimer pins!", pin, pin1);
    SIMPLEFOC_DEBUG(s);
#endif
    return nullptr;
  }
  info = pwm_pin_info + pin;
  // FlexPWM pin
  IMXRT_FLEXPWM_t *flexpwm1,*flexpwm2;
  switch ((info->module >> 4) & 3) {
    case 0: flexpwm1 = &IMXRT_FLEXPWM1; break;
    case 1: flexpwm1 = &IMXRT_FLEXPWM2; break;
    case 2: flexpwm1 = &IMXRT_FLEXPWM3; break;
    default: flexpwm1 = &IMXRT_FLEXPWM4;
  }
  
  info = pwm_pin_info + pin1;
  switch ((info->module >> 4) & 3) {
    case 0: flexpwm2 = &IMXRT_FLEXPWM1; break;
    case 1: flexpwm2 = &IMXRT_FLEXPWM2; break;
    case 2: flexpwm2 = &IMXRT_FLEXPWM3; break;
    default: flexpwm2 = &IMXRT_FLEXPWM4;
  }
  if(flexpwm1 == flexpwm2){
    char s[60];
    sprintf (s, "TEENSY-DRV: Pins: %d, %d on Flextimer %d.", pin, pin1, ((info->module >> 4) & 3) + 1);
    SIMPLEFOC_DEBUG(s);
    return flexpwm1;
  } else {
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: ERR: Pins: %d, %d not in same Flextimer!", pin, pin1);
    SIMPLEFOC_DEBUG(s);
#endif
    return nullptr; 
  }
}


// function which finds the timer submodule for a pair of pins
// if they do not belong to the same submodule it returns a -1
int get_submodule(uint8_t pin, uint8_t pin1){
 
  const struct pwm_pin_info_struct *info;
  if (pin >= CORE_NUM_DIGITAL || pin1 >= CORE_NUM_DIGITAL){
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: ERR: Pins: %d, %d not Flextimer pins!", pin, pin1);
    SIMPLEFOC_DEBUG(s);
#endif
    return -1;
  } 
  
  info = pwm_pin_info + pin;
  int sm1 = info->module&0x3;
  info = pwm_pin_info + pin1;
  int sm2 = info->module&0x3;

  if (sm1 == sm2) {
    char s[60];
    sprintf (s, "TEENSY-DRV: Pins: %d, %d on submodule %d.", pin, pin1, sm1);
    SIMPLEFOC_DEBUG(s);
    return sm1;
  }  else  {
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[50];
    sprintf (s, "TEENSY-DRV: ERR: Pins: %d, %d not in same submodule!", pin, pin1);
    SIMPLEFOC_DEBUG(s);
#endif
    return -1; 
  }
}


// function which finds the timer submodule for a pair of pins
// if they do not belong to the same submodule it returns a -1
int get_inverted_channel(uint8_t pin, uint8_t pin1){
 
  const struct pwm_pin_info_struct *info;
  if (pin >= CORE_NUM_DIGITAL || pin1 >= CORE_NUM_DIGITAL){
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: ERR: Pins: %d, %d not Flextimer pins!", pin, pin1);
    SIMPLEFOC_DEBUG(s);
#endif
    return -1;
  } 
  
  info = pwm_pin_info + pin;
  int ch1 = info->channel;
  info = pwm_pin_info + pin1;
  int ch2 = info->channel;

  if (ch1 != 1) {
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: ERR: Pin: %d on channel %s - only A supported", pin1, ch1==2 ? "B" : "X");
    SIMPLEFOC_DEBUG(s);
#endif
    return -1;
  }  else  if (ch2 != 2) {
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: ERR: Inverted pin: %d on channel %s - only B supported", pin1, ch2==1 ? "A" : "X");
    SIMPLEFOC_DEBUG(s);
#endif
    return -1;
  } else {
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: Pin: %d on channel B inverted.", pin1);
    SIMPLEFOC_DEBUG(s);
#endif 
return ch2;
  } 
}

// Helper to set up A/B pair on a FlexPWM submodule.
// can configure sync, prescale and B inversion.
// sets the desired frequency of the PWM 
// sets the center-aligned pwm
void setup_pwm_pair (IMXRT_FLEXPWM_t * flexpwm, int submodule, const long frequency, float dead_zone )
{
  int submodule_mask = 1 << submodule ;
  flexpwm->MCTRL &= ~ FLEXPWM_MCTRL_RUN (submodule_mask) ;  // stop it if its already running
  flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK (submodule_mask) ;  // clear load OK
  

  // calculate the counter and prescaler for the desired pwm frequency
  uint32_t newdiv = (uint32_t)((float)F_BUS_ACTUAL / frequency + 0.5f);
	uint32_t prescale = 0;
	//printf(" div=%lu\n", newdiv);
	while (newdiv > 65535 && prescale < 7) {
		newdiv = newdiv >> 1;
		prescale = prescale + 1;
	}
	if (newdiv > 65535) {
		newdiv = 65535;
	} else if (newdiv < 2) {
		newdiv = 2;
	}

  // the halfcycle of the PWM
  half_cycle = int(newdiv/2.0f);
  int dead_time = int(dead_zone*half_cycle); //default dead-time - 2% 
  int mid_pwm = int((half_cycle)/2.0f);

  // setup the timer
  // https://github.com/PaulStoffregen/cores/blob/master/teensy4/imxrt.h
  flexpwm->SM[submodule].CTRL2 =  FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN |
                                 FLEXPWM_SMCTRL2_FRCEN | FLEXPWM_SMCTRL2_INIT_SEL(0) | FLEXPWM_SMCTRL2_FORCE_SEL(6);
  flexpwm->SM[submodule].CTRL  = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(prescale) ;
  // https://github.com/PaulStoffregen/cores/blob/70ba01accd728abe75ebfc8dcd8b3d3a8f3e3f25/teensy4/imxrt.h#L4948
  flexpwm->SM[submodule].OCTRL = 0;//channel_to_invert==2 ? 0 : FLEXPWM_SMOCTRL_POLA | FLEXPWM_SMOCTRL_POLB ;
  flexpwm->SM[submodule].DTCNT0 = dead_time ;  // should try this out (deadtime control)
  flexpwm->SM[submodule].DTCNT1 = dead_time ;  // should try this out (deadtime control)
  // flexpwm->SM[submodule].TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN (0b000010)  ; // sync trig out on VAL1 match.
  flexpwm->SM[submodule].INIT = -half_cycle;      // count from -HALFCYCLE to +HALFCYCLE
  flexpwm->SM[submodule].VAL0 = 0 ;
  flexpwm->SM[submodule].VAL1 = half_cycle ;
  flexpwm->SM[submodule].VAL2 = -mid_pwm  ;
  flexpwm->SM[submodule].VAL3 = +mid_pwm  ;
  // flexpwm->SM[submodule].VAL4 = -mid_pwm ;
  // flexpwm->SM[submodule].VAL5 = +mid_pwm  ;
  
  flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK (submodule_mask) ;  // loading reenabled
  flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN (submodule_mask) ;   // start it running
}


// staring the PWM on A and B channels of the submodule
void startup_pwm_pair (IMXRT_FLEXPWM_t * flexpwm, int submodule)
{
  int submodule_mask = 1 << submodule ;

  flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMA_EN (submodule_mask); // enable A output
  flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMB_EN (submodule_mask); // enable B output
}



// PWM setting on the high and low pair of the PWM channels
void write_pwm_pair(IMXRT_FLEXPWM_t * flexpwm, int submodule, float duty){
  
  int mid_pwm = int((half_cycle)/2.0f);
  int count_pwm = int(mid_pwm*(duty*2-1)) + mid_pwm;

  flexpwm->SM[submodule].VAL2 =  count_pwm; // A on
  flexpwm->SM[submodule].VAL3 =  -count_pwm  ; // A off
  // flexpwm->SM[submodule].VAL4 = - count_pwm  ; // B off  (assuming B inverted)
  // flexpwm->SM[submodule].VAL5 = +  count_pwm ; // B on
  flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK (1<<submodule) ;  // signal new values
}




// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 6PWM setting
// - hardware specific
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  
  IMXRT_FLEXPWM_t *flexpwmA,*flexpwmB,*flexpwmC;
  int submoduleA,submoduleB,submoduleC;
  int inverted_channelA,inverted_channelB,inverted_channelC;
  flexpwmA = get_flexpwm(pinA_h,pinA_l);
  submoduleA = get_submodule(pinA_h,pinA_l);
  inverted_channelA = get_inverted_channel(pinA_h,pinA_l);
  flexpwmB = get_flexpwm(pinB_h,pinB_l);
  submoduleB = get_submodule(pinB_h,pinB_l);
  inverted_channelB = get_inverted_channel(pinB_h,pinB_l);
  flexpwmC = get_flexpwm(pinC_h,pinC_l);
  submoduleC = get_submodule(pinC_h,pinC_l);
  inverted_channelC = get_inverted_channel(pinC_h,pinC_l);

  if((flexpwmA == nullptr) || (flexpwmB == nullptr) || (flexpwmC == nullptr) ){
#ifdef SIMPLEFOC_TEENSY_DEBUG
      SIMPLEFOC_DEBUG("TEENSY-DRV: ERR: Flextimer problem - failed driver config!");
#endif
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if((submoduleA < 0) || (submoduleB < 0) || (submoduleC < 0) ){
#ifdef SIMPLEFOC_TEENSY_DEBUG
      SIMPLEFOC_DEBUG("TEENSY-DRV: ERR: Flextimer submodule problem - failed driver config!");
#endif
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if((inverted_channelA < 0) || (inverted_channelB < 0) || (inverted_channelC < 0) ){
#ifdef SIMPLEFOC_TEENSY_DEBUG
      SIMPLEFOC_DEBUG("TEENSY-DRV: ERR: Flextimer channel problem - failed driver config!");
#endif
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }


  Teensy4DriverParams* params = new Teensy4DriverParams {
    .flextimers = { flexpwmA,flexpwmB, flexpwmC},
    .submodules = { submoduleA, submoduleB, submoduleC},
    .pwm_frequency = pwm_frequency,
    .dead_zone = dead_zone
  };

  // Configure FlexPWM units, each driving A/B pair, B inverted.
  // full speed about 80kHz, prescale 2 (div by 4) gives 20kHz
  setup_pwm_pair (flexpwmA, submoduleA, pwm_frequency, dead_zone) ;  // this is the master, internally synced
  setup_pwm_pair (flexpwmB, submoduleB, pwm_frequency, dead_zone) ;   // others externally synced
  setup_pwm_pair (flexpwmC, submoduleC, pwm_frequency, dead_zone) ;
  delayMicroseconds (100) ;

  startup_pwm_pair (flexpwmA, submoduleA) ;
  startup_pwm_pair (flexpwmB, submoduleB) ;
  startup_pwm_pair (flexpwmC, submoduleC) ;

  delayMicroseconds(50) ;
  // config the pins 2/3/6/9/8/7 as their FLEXPWM alternates.
  *portConfigRegister(pinA_h) = pwm_pin_info[pinA_h].muxval ;
  *portConfigRegister(pinA_l) = pwm_pin_info[pinA_l].muxval ;
  *portConfigRegister(pinB_h) = pwm_pin_info[pinB_h].muxval ;
  *portConfigRegister(pinB_l) = pwm_pin_info[pinB_l].muxval ;
  *portConfigRegister(pinC_h) = pwm_pin_info[pinC_h].muxval ;
  *portConfigRegister(pinC_l) = pwm_pin_info[pinC_l].muxval ;
  
  return params;
}



// function setting the pwm duty cycle to the hardware
// - Stepper motor - 6PWM setting
// - hardware specific
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, void* params){
  write_pwm_pair (((Teensy4DriverParams*)params)->flextimers[0], ((Teensy4DriverParams*)params)->submodules[0], dc_a);
  write_pwm_pair (((Teensy4DriverParams*)params)->flextimers[1], ((Teensy4DriverParams*)params)->submodules[1], dc_b);
  write_pwm_pair (((Teensy4DriverParams*)params)->flextimers[2], ((Teensy4DriverParams*)params)->submodules[2], dc_c);
}

#endif