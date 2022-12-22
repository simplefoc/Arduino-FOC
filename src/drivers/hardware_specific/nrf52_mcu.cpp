
#include "../hardware_api.h"

#if defined(NRF52_SERIES)


#define PWM_CLK (16000000)
#define PWM_FREQ (40000)
#define PWM_RESOLUTION (PWM_CLK/PWM_FREQ)
#define PWM_MAX_FREQ (62500)
#define DEAD_ZONE (250) // in ns
#define DEAD_TIME (DEAD_ZONE  / (PWM_RESOLUTION * 0.25 * 62.5)) // 62.5ns resolution of PWM

#ifdef NRF_PWM3
#define PWM_COUNT 4
#else
#define PWM_COUNT 3
#endif

// empty motor slot 
#define _EMPTY_SLOT (-0xAA)
#define _TAKEN_SLOT (-0x55)

int pwm_range;


static NRF_PWM_Type* pwms[PWM_COUNT] = {
  NRF_PWM0,
  NRF_PWM1,
  NRF_PWM2,
  #ifdef  NRF_PWM3
  NRF_PWM3
  #endif
};

typedef struct {
  int pinA;
  NRF_PWM_Type* mcpwm;
  uint16_t mcpwm_channel_sequence[4];
} bldc_3pwm_motor_slots_t;

typedef struct {
  int pin1A;
  NRF_PWM_Type* mcpwm;
  uint16_t mcpwm_channel_sequence[4];
} stepper_motor_slots_t;

typedef struct {
  int pinAH;
  NRF_PWM_Type* mcpwm1;
  NRF_PWM_Type* mcpwm2;
  uint16_t mcpwm_channel_sequence[8];
} bldc_6pwm_motor_slots_t;

// define bldc motor slots array
bldc_3pwm_motor_slots_t nrf52_bldc_3pwm_motor_slots[4] =  { 
  {_EMPTY_SLOT, pwms[0], {0,0,0,0}},// 1st motor will be PWM0 
  {_EMPTY_SLOT, pwms[1], {0,0,0,0}},// 2nd motor will be PWM1 
  {_EMPTY_SLOT, pwms[2], {0,0,0,0}},// 3rd motor will be PWM2 
  {_EMPTY_SLOT, pwms[3], {0,0,0,0}} // 4th motor will be PWM3 
  };

// define stepper motor slots array
stepper_motor_slots_t nrf52_stepper_motor_slots[4] =  { 
  {_EMPTY_SLOT, pwms[0], {0,0,0,0}},// 1st motor will be on PWM0
  {_EMPTY_SLOT, pwms[1], {0,0,0,0}},// 1st motor will be on PWM1
  {_EMPTY_SLOT, pwms[2], {0,0,0,0}},// 1st motor will be on PWM2
  {_EMPTY_SLOT, pwms[3], {0,0,0,0}} // 1st motor will be on PWM3
  };  

// define BLDC motor slots array
bldc_6pwm_motor_slots_t nrf52_bldc_6pwm_motor_slots[2] =  { 
  {_EMPTY_SLOT, pwms[0], pwms[1], {0,0,0,0,0,0,0,0}},// 1st motor will be on PWM0 & PWM1
  {_EMPTY_SLOT, pwms[2], pwms[3], {0,0,0,0,0,0,0,0}} // 2nd motor will be on PWM1 & PWM2
  };



typedef struct NRF52DriverParams {
  union {
    bldc_3pwm_motor_slots_t* slot3pwm;
    bldc_6pwm_motor_slots_t* slot6pwm;
    stepper_motor_slots_t* slotstep;
  } slot;
  long pwm_frequency;
  float dead_time;
} NRF52DriverParams;




// configuring high frequency pwm timer
void _configureHwPwm(NRF_PWM_Type* mcpwm1,  NRF_PWM_Type* mcpwm2){

  mcpwm1->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
  mcpwm1->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);
  mcpwm1->MODE = (PWM_MODE_UPDOWN_UpAndDown << PWM_MODE_UPDOWN_Pos); 
  mcpwm1->COUNTERTOP = pwm_range; //pwm freq.
  mcpwm1->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  mcpwm1->DECODER = ((uint32_t)PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | ((uint32_t)PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
  mcpwm1->SEQ[0].REFRESH  = 0;
  mcpwm1->SEQ[0].ENDDELAY = 0;

  if(mcpwm1 != mcpwm2){
    mcpwm2->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    mcpwm2->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);
    mcpwm2->MODE = (PWM_MODE_UPDOWN_UpAndDown << PWM_MODE_UPDOWN_Pos); 
    mcpwm2->COUNTERTOP = pwm_range; //pwm freq.
    mcpwm2->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
    mcpwm2->DECODER = ((uint32_t)PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | ((uint32_t)PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    mcpwm2->SEQ[0].REFRESH  = 0;
    mcpwm2->SEQ[0].ENDDELAY = 0;
  }else{
    mcpwm1->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
  }
}



// can we support it using the generic driver on this MCU? Commented out to fall back to generic driver for 2-pwm
// void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
//   return SIMPLEFOC_DRIVER_INIT_FAILED; // not supported
// }




// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {

  if( !pwm_frequency || pwm_frequency == NOT_SET) pwm_frequency = PWM_FREQ; // default frequency 20khz for a resolution of 800
  else pwm_frequency = _constrain(pwm_frequency, 0, PWM_MAX_FREQ); // constrain to 62.5kHz max for a resolution of 256  

  pwm_range = (PWM_CLK / pwm_frequency);
  
  int pA = digitalPinToPinName(pinA); //g_ADigitalPinMap[pinA];
  int pB = digitalPinToPinName(pinB); //g_ADigitalPinMap[pinB];
  int pC = digitalPinToPinName(pinC); //g_ADigitalPinMap[pinC];

  // determine which motor are we connecting
  // and set the appropriate configuration parameters 
  int slot_num;
  for(slot_num = 0; slot_num < 4; slot_num++){
    if(nrf52_bldc_3pwm_motor_slots[slot_num].pinA == _EMPTY_SLOT){ // put the new motor in the first empty slot
      nrf52_bldc_3pwm_motor_slots[slot_num].pinA = pinA;
      break;
    }
  }
  // if no slots available
  if(slot_num >= 4) return SIMPLEFOC_DRIVER_INIT_FAILED;

  // disable all the slots with the same MCPWM 
  if(slot_num < 2){
    // slot 0 of the stepper
    nrf52_stepper_motor_slots[slot_num].pin1A = _TAKEN_SLOT;
    // slot 0 of the 6pwm bldc
    nrf52_bldc_6pwm_motor_slots[0].pinAH = _TAKEN_SLOT;
    //NRF_PPI->CHEN &= ~1UL;
  }else{
    // slot 1 of the stepper
    nrf52_stepper_motor_slots[slot_num].pin1A = _TAKEN_SLOT;
    // slot 0 of the 6pwm bldc
    nrf52_bldc_6pwm_motor_slots[1].pinAH = _TAKEN_SLOT;
    //NRF_PPI->CHEN &= ~2UL;
  }

  // configure pwm outputs
  
  nrf52_bldc_3pwm_motor_slots[slot_num].mcpwm->PSEL.OUT[0] = pA;
  nrf52_bldc_3pwm_motor_slots[slot_num].mcpwm->PSEL.OUT[1] = pB;
  nrf52_bldc_3pwm_motor_slots[slot_num].mcpwm->PSEL.OUT[2] = pC;
   
  nrf52_bldc_3pwm_motor_slots[slot_num].mcpwm->SEQ[0].PTR = (uint32_t)&nrf52_bldc_3pwm_motor_slots[slot_num].mcpwm_channel_sequence[0];
  nrf52_bldc_3pwm_motor_slots[slot_num].mcpwm->SEQ[0].CNT = 4;

  // configure the pwm
  _configureHwPwm(nrf52_bldc_3pwm_motor_slots[slot_num].mcpwm, nrf52_bldc_3pwm_motor_slots[slot_num].mcpwm);

  NRF52DriverParams* params = new NRF52DriverParams();
  params->slot.slot3pwm = &(nrf52_bldc_3pwm_motor_slots[slot_num]);
  params->pwm_frequency = pwm_frequency;
  return params;
}




// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
void* _configure4PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC, const int pinD) {

  if( !pwm_frequency || pwm_frequency == NOT_SET) pwm_frequency = PWM_FREQ; // default frequency 20khz for a resolution of 800
  else pwm_frequency = _constrain(pwm_frequency, 0, PWM_MAX_FREQ); // constrain to 62.5kHz max for a resolution of 256

  pwm_range = (PWM_CLK / pwm_frequency);
  
  int pA = digitalPinToPinName(pinA); //g_ADigitalPinMap[pinA];
  int pB = digitalPinToPinName(pinB); //g_ADigitalPinMap[pinB];
  int pC = digitalPinToPinName(pinC); //g_ADigitalPinMap[pinC];
  int pD = digitalPinToPinName(pinD); //g_ADigitalPinMap[pinD];
  
  // determine which motor are we connecting
  // and set the appropriate configuration parameters 
  int slot_num;
  for(slot_num = 0; slot_num < 4; slot_num++){
    if(nrf52_stepper_motor_slots[slot_num].pin1A == _EMPTY_SLOT){ // put the new motor in the first empty slot
      nrf52_stepper_motor_slots[slot_num].pin1A = pinA;
      break;
    }
  }
  // if no slots available
  if (slot_num >= 4) return SIMPLEFOC_DRIVER_INIT_FAILED;

  // disable all the slots with the same MCPWM 
  if (slot_num < 2){
    // slots 0 and 1 of the 3pwm bldc
    nrf52_bldc_3pwm_motor_slots[slot_num].pinA = _TAKEN_SLOT;
    // slot 0 of the 6pwm bldc
    nrf52_bldc_6pwm_motor_slots[0].pinAH = _TAKEN_SLOT;
    //NRF_PPI->CHEN &= ~1UL;
  }else{
    // slots 2 and 3 of the 3pwm bldc
    nrf52_bldc_3pwm_motor_slots[slot_num].pinA = _TAKEN_SLOT;
    // slot 1 of the 6pwm bldc
    nrf52_bldc_6pwm_motor_slots[1].pinAH = _TAKEN_SLOT;
    //NRF_PPI->CHEN &= ~2UL;
  } 

  // configure pwm outputs
  
  nrf52_stepper_motor_slots[slot_num].mcpwm->PSEL.OUT[0] = pA;
  nrf52_stepper_motor_slots[slot_num].mcpwm->PSEL.OUT[1] = pB;
  nrf52_stepper_motor_slots[slot_num].mcpwm->PSEL.OUT[2] = pC;
  nrf52_stepper_motor_slots[slot_num].mcpwm->PSEL.OUT[3] = pD;

  nrf52_stepper_motor_slots[slot_num].mcpwm->SEQ[0].PTR = (uint32_t)&nrf52_stepper_motor_slots[slot_num].mcpwm_channel_sequence[0];
  nrf52_stepper_motor_slots[slot_num].mcpwm->SEQ[0].CNT = 4;

  // configure the pwm
  _configureHwPwm(nrf52_stepper_motor_slots[slot_num].mcpwm, nrf52_stepper_motor_slots[slot_num].mcpwm);

  NRF52DriverParams* params = new NRF52DriverParams();
  params->slot.slotstep = &(nrf52_stepper_motor_slots[slot_num]);
  params->pwm_frequency = pwm_frequency;
  return params;  
}




// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
// - hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  // transform duty cycle from [0,1] to [0,range]
  bldc_3pwm_motor_slots_t* p = ((NRF52DriverParams*)params)->slot.slot3pwm;
  p->mcpwm_channel_sequence[0] = (int)(dc_a * pwm_range) | 0x8000;
  p->mcpwm_channel_sequence[1] = (int)(dc_b * pwm_range) | 0x8000;
  p->mcpwm_channel_sequence[2] = (int)(dc_c * pwm_range) | 0x8000;

  p->mcpwm->TASKS_SEQSTART[0] = 1;
}




// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
// - hardware speciffic
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){

  stepper_motor_slots_t* p = ((NRF52DriverParams*)params)->slot.slotstep;
  p->mcpwm_channel_sequence[0] = (int)(dc_1a * pwm_range) | 0x8000;
  p->mcpwm_channel_sequence[1] = (int)(dc_1b * pwm_range) | 0x8000;
  p->mcpwm_channel_sequence[2] = (int)(dc_2a * pwm_range) | 0x8000;
  p->mcpwm_channel_sequence[3] = (int)(dc_2b * pwm_range) | 0x8000;

  p->mcpwm->TASKS_SEQSTART[0] = 1;
}

/* Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
*/
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  
  if( !pwm_frequency || pwm_frequency == NOT_SET) pwm_frequency = PWM_FREQ; // default frequency 20khz - centered pwm has twice lower frequency for a resolution of 400
  else pwm_frequency = _constrain(pwm_frequency*2, 0, PWM_MAX_FREQ); // constrain to 62.5kHz max => 31.25kHz for a resolution of 256  

  pwm_range = (PWM_CLK / pwm_frequency);
  pwm_range /= 2; // scale the frequency (centered PWM)

  float dead_time;
  if (dead_zone != NOT_SET){
    dead_time = dead_zone/2;
  }else{
    dead_time = DEAD_TIME/2; 
  }
  
  int pA_l = digitalPinToPinName(pinA_l); //g_ADigitalPinMap[pinA_l];
  int pA_h = digitalPinToPinName(pinA_h); //g_ADigitalPinMap[pinA_h];
  int pB_l = digitalPinToPinName(pinB_l); //g_ADigitalPinMap[pinB_l];
  int pB_h = digitalPinToPinName(pinB_h); //g_ADigitalPinMap[pinB_h];
  int pC_l = digitalPinToPinName(pinC_l); //g_ADigitalPinMap[pinC_l];
  int pC_h = digitalPinToPinName(pinC_h); //g_ADigitalPinMap[pinC_h];


  // determine which motor are we connecting
  // and set the appropriate configuration parameters 
  int slot_num;
  for(slot_num = 0; slot_num < 2; slot_num++){
    if(nrf52_bldc_6pwm_motor_slots[slot_num].pinAH == _EMPTY_SLOT){ // put the new motor in the first empty slot
      nrf52_bldc_6pwm_motor_slots[slot_num].pinAH = pinA_h;
      break;
    }
  }
  // if no slots available
  if(slot_num >= 2) return SIMPLEFOC_DRIVER_INIT_FAILED;

  // disable all the slots with the same MCPWM 
  if( slot_num == 0 ){
    // slots 0 and 1 of the 3pwm bldc
    nrf52_bldc_3pwm_motor_slots[0].pinA = _TAKEN_SLOT;
    nrf52_bldc_3pwm_motor_slots[1].pinA = _TAKEN_SLOT;
    // slot 0 and 1 of the stepper
    nrf52_stepper_motor_slots[0].pin1A = _TAKEN_SLOT;
    nrf52_stepper_motor_slots[1].pin1A = _TAKEN_SLOT;
  }else{
    // slots 2 and 3 of the 3pwm bldc
    nrf52_bldc_3pwm_motor_slots[2].pinA = _TAKEN_SLOT;
    nrf52_bldc_3pwm_motor_slots[3].pinA = _TAKEN_SLOT;
    // slot 1 of the stepper
    nrf52_stepper_motor_slots[2].pin1A = _TAKEN_SLOT;
    nrf52_stepper_motor_slots[3].pin1A = _TAKEN_SLOT;
  } 

  // Configure pwm outputs
  
  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm1->PSEL.OUT[0] = pA_h;
  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm1->PSEL.OUT[1] = pA_l;
  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm1->PSEL.OUT[2] = pB_h;
  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm1->PSEL.OUT[3] = pB_l;
  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm1->SEQ[0].PTR = (uint32_t)&nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm_channel_sequence[0];
  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm1->SEQ[0].CNT = 4;

  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm2->PSEL.OUT[0] = pC_h;
  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm2->PSEL.OUT[1] = pC_l;
  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm2->SEQ[0].PTR = (uint32_t)&nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm_channel_sequence[4];
  nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm2->SEQ[0].CNT = 4;

  // Initializing the PPI peripheral for sync the pwm slots

  NRF_PPI->CH[slot_num].EEP = (uint32_t)&NRF_EGU0->EVENTS_TRIGGERED[0];
  NRF_PPI->CH[slot_num].TEP = (uint32_t)&nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm1->TASKS_SEQSTART[0];
  NRF_PPI->FORK[slot_num].TEP = (uint32_t)&nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm2->TASKS_SEQSTART[0];
  NRF_PPI->CHEN =  1UL << slot_num;
  
  // configure the pwm type
  _configureHwPwm(nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm1, nrf52_bldc_6pwm_motor_slots[slot_num].mcpwm2);
  
  NRF52DriverParams* params = new NRF52DriverParams();
  params->slot.slot6pwm = &(nrf52_bldc_6pwm_motor_slots[slot_num]);
  params->pwm_frequency = pwm_frequency;
  params->dead_time = dead_time;
  return params; 
}




/* Function setting the duty cycle to the pwm pin
//  - BLDC driver - 6PWM setting
//  - hardware specific
*/
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
      bldc_6pwm_motor_slots_t* p = ((NRF52DriverParams*)params)->slot.slot6pwm;
      float dead_time = ((NRF52DriverParams*)params)->dead_time;
      p->mcpwm_channel_sequence[0] = (int)(_constrain(dc_a-dead_time,0,1)*pwm_range) | 0x8000;
      p->mcpwm_channel_sequence[1] = (int)(_constrain(dc_a+dead_time,0,1)*pwm_range);
      p->mcpwm_channel_sequence[2] = (int)(_constrain(dc_b-dead_time,0,1)*pwm_range) | 0x8000;
      p->mcpwm_channel_sequence[3] = (int)(_constrain(dc_b+dead_time,0,1)*pwm_range);
      p->mcpwm_channel_sequence[4] = (int)(_constrain(dc_c-dead_time,0,1)*pwm_range) | 0x8000;
      p->mcpwm_channel_sequence[5] = (int)(_constrain(dc_c+dead_time,0,1)*pwm_range);     
      NRF_EGU0->TASKS_TRIGGER[0] = 1;

      _UNUSED(phase_state);
}


#endif
