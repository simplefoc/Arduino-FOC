#include "teensy4_mcu.h"
#include "../../../communication/SimpleFOCDebug.h"

// if defined 
// - Teensy 4.0 
// - Teensy 4.1 
#if defined(__arm__) && defined(CORE_TEENSY) && ( defined(__IMXRT1062__) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY_MICROMOD) )

#pragma message("")
#pragma message("SimpleFOC: compiling for Teensy 4.x")
#pragma message("")

// #define SIMPLEFOC_TEENSY4_FORCE_CENTER_ALIGNED_3PWM


// function finding the TRIG event given the flexpwm timer and the submodule
// returning -1 if the submodule is not valid or no trigger is available
// allowing flexpwm1-4 and submodule 0-3
//
// the flags are defined in the imxrt.h file
// https://github.com/PaulStoffregen/cores/blob/dd6aa8419ee173a0a6593eab669fbff54ed85f48/teensy4/imxrt.h#L9662-L9693
int flexpwm_submodule_to_trig(IMXRT_FLEXPWM_t* flexpwm, int submodule){
  if(submodule <0 && submodule > 3) return -1;
  if(flexpwm == &IMXRT_FLEXPWM1){
      return XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0 + submodule; 
  }else if(flexpwm == &IMXRT_FLEXPWM2){
      return XBARA1_IN_FLEXPWM2_PWM1_OUT_TRIG0 + submodule;
  }else if(flexpwm == &IMXRT_FLEXPWM3){
      return XBARA1_IN_FLEXPWM3_PWM1_OUT_TRIG0 + submodule;
  }else if(flexpwm == &IMXRT_FLEXPWM4){
      return XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0 + submodule;
  }
  return -1;
}

// function finding the EXT_SYNC event given the flexpwm timer and the submodule
// returning -1 if the submodule is not valid or no trigger is available
// allowing flexpwm1-4 and submodule 0-3
//
// the flags are defined in the imxrt.h file
// https://github.com/PaulStoffregen/cores/blob/dd6aa8419ee173a0a6593eab669fbff54ed85f48/teensy4/imxrt.h#L9757
int flexpwm_submodule_to_ext_sync(IMXRT_FLEXPWM_t* flexpwm, int submodule){
  if(submodule < 0 && submodule > 3) return -1;
  if(flexpwm == &IMXRT_FLEXPWM1){
      return XBARA1_OUT_FLEXPWM1_PWM0_EXT_SYNC + submodule; 
  }else if(flexpwm == &IMXRT_FLEXPWM2){
      return XBARA1_OUT_FLEXPWM2_PWM0_EXT_SYNC + submodule;
  }else if(flexpwm == &IMXRT_FLEXPWM3){
      return XBARA1_OUT_FLEXPWM3_EXT_SYNC0 + submodule; // TODO verify why they are not called PWM0_EXT_SYNC but EXT_SYNC0
  }else if(flexpwm == &IMXRT_FLEXPWM4){
      return XBARA1_OUT_FLEXPWM4_EXT_SYNC0 + submodule; // TODO verify why they are not called PWM0_EXT_SYNC but EXT_SYNC0
  }
  return -1;
}

// function finding the flexpwm instance given the submodule
int flexpwm_to_index(IMXRT_FLEXPWM_t* flexpwm){
  if(flexpwm == &IMXRT_FLEXPWM1) return 1;
  if(flexpwm == &IMXRT_FLEXPWM2) return 2;
  if(flexpwm == &IMXRT_FLEXPWM3) return 3;
  if(flexpwm == &IMXRT_FLEXPWM4) return 4;
  return -1;
}

// The i.MXRT1062 uses one config register per two XBAR outputs, so a helper
// function to make code more readable. 
void xbar_connect(unsigned int input, unsigned int output)
{
  if (input >= 88) return;
  if (output >= 132) return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1)) {
    val = (val & 0xFF00) | input;
  } else {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}

void xbar_init() {
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);   //turn clock on for xbara1
}

// function which finds the flexpwm instance for a pin
// if it does not belong to the flexpwm timer it returns a null-pointer
IMXRT_FLEXPWM_t* get_flexpwm(uint8_t pin){
 
  const struct pwm_pin_info_struct *info;
  info = pwm_pin_info + pin;
  if (pin >= CORE_NUM_DIGITAL || info->type == 2) {
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: ERR: Pin: %d not Flextimer pin!", pin);
    SIMPLEFOC_DEBUG(s);
#endif
    return nullptr;
  }
  // FlexPWM pin
  IMXRT_FLEXPWM_t *flexpwm;
  switch ((info->module >> 4) & 3) {
    case 0: flexpwm = &IMXRT_FLEXPWM1; break;
    case 1: flexpwm = &IMXRT_FLEXPWM2; break;
    case 2: flexpwm = &IMXRT_FLEXPWM3; break;
    default: flexpwm = &IMXRT_FLEXPWM4;
  }
  if(flexpwm != nullptr){
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: Pin: %d on Flextimer %d.", pin, ((info->module >> 4) & 3) + 1);
    SIMPLEFOC_DEBUG(s);
#endif
    return flexpwm;
  } 
  return nullptr;
}


// function which finds the timer submodule for a pin
// if it does not belong to the submodule it returns a -1
int get_submodule(uint8_t pin){
 
  const struct pwm_pin_info_struct *info;
  if (pin >= CORE_NUM_DIGITAL){
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: ERR: Pin: %d not Flextimer pin!", pin);
    SIMPLEFOC_DEBUG(s);
#endif
    return -1;
  } 
  
  info = pwm_pin_info + pin;
  int sm1 = info->module&0x3;

  if (sm1 >= 0 && sm1 < 4) {
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: Pin %d on submodule %d.", pin, sm1);
    SIMPLEFOC_DEBUG(s);
#endif
    return sm1;
  }  else  {
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[50];
    sprintf (s, "TEENSY-DRV: ERR: Pin: %d not in submodule!", pin);
    SIMPLEFOC_DEBUG(s);
#endif
    return -1; 
  }
}

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


// function which finds the channel for flexpwm timer pin
// 0 - X
// 1 - A
// 2 - B
int get_channel(uint8_t pin){
  const struct pwm_pin_info_struct *info;
  info = pwm_pin_info + pin;
  if (pin >= CORE_NUM_DIGITAL || info->type == 2){
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[90];
    sprintf (s, "TEENSY-DRV: ERR: Pin: %d not Flextimer pin!", pin);
    SIMPLEFOC_DEBUG(s);
#endif
    return -1;
  }
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: Pin: %d on channel %s.", pin, info->channel==0 ? "X" : info->channel==1 ? "A" : "B");
    SIMPLEFOC_DEBUG(s);
#endif
  return info->channel;
}

// function which finds the timer submodule for a pair of pins
// if they do not belong to the same submodule it returns a -1
int get_inverted_channel(uint8_t pin, uint8_t pin1){
 
  if (pin >= CORE_NUM_DIGITAL || pin1 >= CORE_NUM_DIGITAL){
#ifdef SIMPLEFOC_TEENSY_DEBUG
    char s[60];
    sprintf (s, "TEENSY-DRV: ERR: Pins: %d, %d not Flextimer pins!", pin, pin1);
    SIMPLEFOC_DEBUG(s);
#endif
    return -1;
  } 
  
  int ch1 = get_channel(pin);
  int ch2 = get_channel(pin1);

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
void setup_pwm_pair (IMXRT_FLEXPWM_t * flexpwm, int submodule, bool ext_sync,  const long frequency, float dead_zone )
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
  int half_cycle = int(newdiv/2.0f);
  int dead_time = int(dead_zone*half_cycle); //default dead-time - 2% 
  int mid_pwm = int((half_cycle)/2.0f);

  // if the timer should be externally synced with the master timer
  int sel = ext_sync ? 3 : 0;

  // setup the timer
  // https://github.com/PaulStoffregen/cores/blob/master/teensy4/imxrt.h
  flexpwm->SM[submodule].CTRL2 =  FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN |
                                 FLEXPWM_SMCTRL2_FRCEN | FLEXPWM_SMCTRL2_INIT_SEL(sel) | FLEXPWM_SMCTRL2_FORCE_SEL(6);
  flexpwm->SM[submodule].CTRL  = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(prescale) ;
  // https://github.com/PaulStoffregen/cores/blob/70ba01accd728abe75ebfc8dcd8b3d3a8f3e3f25/teensy4/imxrt.h#L4948
  flexpwm->SM[submodule].OCTRL = 0; //FLEXPWM_SMOCTRL_POLA | FLEXPWM_SMOCTRL_POLB;//channel_to_invert==2 ? 0 : FLEXPWM_SMOCTRL_POLA | FLEXPWM_SMOCTRL_POLB ;
  if (!ext_sync) flexpwm->SM[submodule].TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN (0b000010)  ; // sync trig out on VAL1 match if master timer
  flexpwm->SM[submodule].DTCNT0 = dead_time ;  // should try this out (deadtime control)
  flexpwm->SM[submodule].DTCNT1 = dead_time ;  // should try this out (deadtime control)
  flexpwm->SM[submodule].INIT = -half_cycle;      // count from -HALFCYCLE to +HALFCYCLE
  flexpwm->SM[submodule].VAL0 = 0;
  flexpwm->SM[submodule].VAL1 = half_cycle ;
  flexpwm->SM[submodule].VAL2 = -mid_pwm  ;
  flexpwm->SM[submodule].VAL3 = +mid_pwm  ;

  flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK (submodule_mask) ;  // loading reenabled
  flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN (submodule_mask) ;   // start it running
}


// Helper to set up a FlexPWM submodule.
// can configure sync, prescale 
// sets the desired frequency of the PWM 
// sets the center-aligned pwm
void setup_pwm_timer_submodule (IMXRT_FLEXPWM_t * flexpwm, int submodule, bool ext_sync,  const long frequency)
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
  int half_cycle = int(newdiv/2.0f);

  // if the timer should be externally synced with the master timer
  int sel = ext_sync ? 3 : 0;

  // setup the timer
  // https://github.com/PaulStoffregen/cores/blob/master/teensy4/imxrt.h
  flexpwm->SM[submodule].CTRL2 =  FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN | 
                                 FLEXPWM_SMCTRL2_FRCEN | FLEXPWM_SMCTRL2_INIT_SEL(sel) | FLEXPWM_SMCTRL2_FORCE_SEL(6);
  flexpwm->SM[submodule].CTRL  = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(prescale) ;
  // https://github.com/PaulStoffregen/cores/blob/70ba01accd728abe75ebfc8dcd8b3d3a8f3e3f25/teensy4/imxrt.h#L4948
  flexpwm->SM[submodule].OCTRL = 0; //FLEXPWM_SMOCTRL_POLA | FLEXPWM_SMOCTRL_POLB;//channel_to_invert==2 ? 0 : FLEXPWM_SMOCTRL_POLA | FLEXPWM_SMOCTRL_POLB ;
  if (!ext_sync) flexpwm->SM[submodule].TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN (0b000010)  ; // sync trig out on VAL1 match if master timer
  flexpwm->SM[submodule].DTCNT0 = 0 ;  
  flexpwm->SM[submodule].DTCNT1 = 0 ;  
  flexpwm->SM[submodule].INIT = -half_cycle;      // count from -HALFCYCLE to +HALFCYCLE
  flexpwm->SM[submodule].VAL0 = 0;
  flexpwm->SM[submodule].VAL1 = half_cycle;
  flexpwm->SM[submodule].VAL2 = 0  ;
  flexpwm->SM[submodule].VAL3 = 0  ;
  flexpwm->SM[submodule].VAL2 = 0  ;
  flexpwm->SM[submodule].VAL3 = 0  ;

  flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK (submodule_mask) ;  // loading reenabled
  flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN (submodule_mask) ;   // start it running
}


// staring the PWM on A and B channels of the submodule
void startup_pwm_pair (IMXRT_FLEXPWM_t * flexpwm, int submodule, int channel)
{
  int submodule_mask = 1 << submodule ;

  if(channel==1)  flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMA_EN (submodule_mask); // enable A output
  else if(channel==2)  flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMB_EN (submodule_mask); // enable B output
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
  int half_cycle = int(flexpwm->SM[submodule].VAL1);
  int mid_pwm = int((half_cycle)/2.0f);
  int count_pwm = int(mid_pwm*(duty*2-1)) + mid_pwm;

  flexpwm->SM[submodule].VAL2 =  -count_pwm; // A on
  flexpwm->SM[submodule].VAL3 =  count_pwm  ; // A off
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
  int channelA,channelB,channelC;
  flexpwmA = get_flexpwm(pinA_h,pinA_l);
  submoduleA = get_submodule(pinA_h,pinA_l);
  inverted_channelA = get_inverted_channel(pinA_h,pinA_l);
  flexpwmB = get_flexpwm(pinB_h,pinB_l);
  submoduleB = get_submodule(pinB_h,pinB_l);
  inverted_channelB = get_inverted_channel(pinB_h,pinB_l);
  flexpwmC = get_flexpwm(pinC_h,pinC_l);
  submoduleC = get_submodule(pinC_h,pinC_l);
  inverted_channelC = get_inverted_channel(pinC_h,pinC_l);
  channelA = get_channel(pinA_h);
  channelB = get_channel(pinB_h);
  channelC = get_channel(pinC_h);

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

  #ifdef SIMPLEFOC_TEENSY_DEBUG
    char buff[100];
    sprintf(buff, "TEENSY-DRV: Syncing to Master FlexPWM: %d, Submodule: %d", flexpwm_to_index(flexpwmC), submoduleC);
    SIMPLEFOC_DEBUG(buff);
    sprintf(buff, "TEENSY-DRV: Slave timers FlexPWM: %d, Submodule: %d and FlexPWM: %d, Submodule: %d", flexpwm_to_index(flexpwmA), submoduleA, flexpwm_to_index(flexpwmB), submoduleB);
    SIMPLEFOC_DEBUG(buff);
  #endif

  // Configure FlexPWM units, each driving A/B pair, B inverted.
  setup_pwm_pair (flexpwmA, submoduleA, true, pwm_frequency, dead_zone) ;   // others externally synced
  setup_pwm_pair (flexpwmB, submoduleB, true, pwm_frequency, dead_zone) ;   // others externally synced
  setup_pwm_pair (flexpwmC, submoduleC, false, pwm_frequency, dead_zone) ; // this is the master, internally synced
  delayMicroseconds (100) ;

  // turn on XBAR1 clock for all but stop mode
  xbar_init() ;

  // // Connect trigger to synchronize all timers 
  xbar_connect (flexpwm_submodule_to_trig(flexpwmC, submoduleC), flexpwm_submodule_to_ext_sync(flexpwmA, submoduleA)) ;
  xbar_connect (flexpwm_submodule_to_trig(flexpwmC, submoduleC), flexpwm_submodule_to_ext_sync(flexpwmB, submoduleB)) ;

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
  
  
  TeensyDriverParams* params = new TeensyDriverParams {
    .pins = { pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l },
    .pwm_frequency = pwm_frequency,
    .additional_params =  new Teensy4DriverParams {
      .flextimers = { flexpwmA, flexpwmB, flexpwmC},
      .submodules = { submoduleA, submoduleB, submoduleC},
      .channels = {1,2, 1, 2, 1, 2},
      .dead_zone = dead_zone
    }
  };
  return params;
}



// function setting the pwm duty cycle to the hardware
// - Stepper motor - 6PWM setting
// - hardware specific
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
  Teensy4DriverParams* p = (Teensy4DriverParams*)((TeensyDriverParams*)params)->additional_params;
  _UNUSED(phase_state);
  write_pwm_pair (p->flextimers[0], p->submodules[0], dc_a);
  write_pwm_pair (p->flextimers[1], p->submodules[1], dc_b);
  write_pwm_pair (p->flextimers[2], p->submodules[2], dc_c);
}

void write_pwm_on_pin(IMXRT_FLEXPWM_t *p, unsigned int submodule, uint8_t channel, float duty)
{
	uint16_t mask = 1 << submodule;
	uint32_t half_cycle = p->SM[submodule].VAL1;
  int mid_pwm = int((half_cycle)/2.0f);
  int cval = int(mid_pwm*(duty*2-1)) + mid_pwm;

	//printf("flexpwmWrite, p=%08lX, sm=%d, ch=%c, cval=%ld\n",
	//(uint32_t)p, submodule, channel == 0 ? 'X' : (channel == 1 ? 'A' : 'B'), cval);
	p->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
	switch (channel) {
	  case 0: // X
		p->SM[submodule].VAL0 = half_cycle - cval;
		p->OUTEN |= FLEXPWM_OUTEN_PWMX_EN(mask);
		//printf(" write channel X\n");
		break;
	  case 1: // A
		p->SM[submodule].VAL2 = -cval;
		p->SM[submodule].VAL3 = cval;
		p->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask);
		//printf(" write channel A\n");
		break;
	  case 2: // B
		p->SM[submodule].VAL4 = -cval;
		p->SM[submodule].VAL5 = cval;
		p->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(mask);
		//printf(" write channel B\n");
	}
	p->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
}

#ifdef SIMPLEFOC_TEENSY4_FORCE_CENTER_ALIGNED_3PWM

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// in generic case dont do anything
 void* _configureCenterAligned3PMW(long pwm_frequency,const int pinA, const int pinB, const int pinC) {

  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  
  IMXRT_FLEXPWM_t *flexpwmA,*flexpwmB,*flexpwmC;
  int submoduleA,submoduleB,submoduleC;
  flexpwmA = get_flexpwm(pinA);
  submoduleA = get_submodule(pinA);
  flexpwmB = get_flexpwm(pinB);
  submoduleB = get_submodule(pinB);
  flexpwmC = get_flexpwm(pinC);
  submoduleC = get_submodule(pinC);  
  int channelA = get_channel(pinA);
  int channelB = get_channel(pinB);
  int channelC = get_channel(pinC);


  // if pins belong to the flextimers and they only use submodules A and B
  // we can configure the center-aligned pwm
  if((flexpwmA != nullptr) && (flexpwmB != nullptr) && (flexpwmC != nullptr) && (channelA > 0) && (channelB > 0) && (channelC > 0) ){
    #ifdef SIMPLEFOC_TEENSY_DEBUG
      SIMPLEFOC_DEBUG("TEENSY-DRV: All pins on Flexpwm A or B channels - Configuring center-aligned pwm!");
    #endif

    // Configure FlexPWM units
    setup_pwm_timer_submodule (flexpwmA, submoduleA, true, pwm_frequency) ;  // others externally synced
    setup_pwm_timer_submodule (flexpwmB, submoduleB, true, pwm_frequency) ;  // others externally synced
    setup_pwm_timer_submodule (flexpwmC, submoduleC, false, pwm_frequency) ; // this is the master, internally synced
    delayMicroseconds (100) ;

    
  #ifdef SIMPLEFOC_TEENSY_DEBUG
    char buff[100];
    sprintf(buff, "TEENSY-CS: Syncing to Master FlexPWM: %d, Submodule: %d", flexpwm_to_index(flexpwmC), submoduleC);
    SIMPLEFOC_DEBUG(buff);
    sprintf(buff, "TEENSY-CS: Slave timers FlexPWM: %d, Submodule: %d and FlexPWM: %d, Submodule: %d", flexpwm_to_index(flexpwmA), submoduleA, flexpwm_to_index(flexpwmB), submoduleB);
    SIMPLEFOC_DEBUG(buff);
  #endif

    // // turn on XBAR1 clock for all but stop mode
    xbar_init() ;

    // // Connect trigger to synchronize all timers 
    xbar_connect (flexpwm_submodule_to_trig(flexpwmC, submoduleC), flexpwm_submodule_to_ext_sync(flexpwmA, submoduleA)) ;
    xbar_connect (flexpwm_submodule_to_trig(flexpwmC, submoduleC), flexpwm_submodule_to_ext_sync(flexpwmB, submoduleB)) ;
  
    TeensyDriverParams* params = new TeensyDriverParams {
      .pins = { pinA, pinB, pinC },
      .pwm_frequency = pwm_frequency,
      .additional_params = new Teensy4DriverParams {
        .flextimers = { flexpwmA, flexpwmB, flexpwmC},
        .submodules = { submoduleA, submoduleB, submoduleC},
        .channels = {channelA, channelB, channelC},
      }
    };

    startup_pwm_pair (flexpwmA, submoduleA, channelA) ;
    startup_pwm_pair (flexpwmB, submoduleB, channelB) ;
    startup_pwm_pair (flexpwmC, submoduleC, channelC) ;

    // // config the pins 2/3/6/9/8/7 as their FLEXPWM alternates.
    *portConfigRegister(pinA) = pwm_pin_info[pinA].muxval ;
    *portConfigRegister(pinB) = pwm_pin_info[pinB].muxval ;
    *portConfigRegister(pinC) = pwm_pin_info[pinC].muxval ;

    return params;
  }else{
    #ifdef SIMPLEFOC_TEENSY_DEBUG
      SIMPLEFOC_DEBUG("TEENSY-DRV: Not all pins on Flexpwm A and B channels - cannot configure center-aligned pwm!");
    #endif
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }  
  
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 6PWM setting
// - hardware specific
void _writeCenterAligned3PMW(float dc_a,  float dc_b, float dc_c, void* params){
  Teensy4DriverParams* p = (Teensy4DriverParams*)((TeensyDriverParams*)params)->additional_params;
  write_pwm_on_pin (p->flextimers[0], p->submodules[0], p->channels[0], dc_a);
  write_pwm_on_pin (p->flextimers[1], p->submodules[1], p->channels[1], dc_b);
  write_pwm_on_pin (p->flextimers[2], p->submodules[2], p->channels[2], dc_c);
}

#endif

#endif
