#include "../hardware_api.h" 

#if defined(__arm__) && defined(__SAM3X8E__)

#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

#define _PWM_RES_MIN 255 // 50khz

// pwm frequency and max duty cycle
static unsigned long _pwm_frequency;
static int _max_pwm_value = 1023;

// array mapping the timer values to the interrupt handlers
static IRQn_Type irq_type[] = {TC0_IRQn, TC0_IRQn, TC1_IRQn, TC1_IRQn, TC2_IRQn, TC2_IRQn, TC3_IRQn, TC3_IRQn, TC4_IRQn, TC4_IRQn, TC5_IRQn, TC5_IRQn, TC6_IRQn, TC6_IRQn, TC7_IRQn, TC7_IRQn, TC8_IRQn, TC8_IRQn};
// current counter values 
static volatile uint32_t pwm_counter_vals[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


// variables copied from wiring_analog.cpp for arduino due
static uint8_t PWMEnabled = 0;
static uint8_t TCChanEnabled[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint32_t channelToChNo[] = { 0, 0, 1, 1, 2, 2, 0, 0, 1, 1, 2, 2, 0, 0, 1, 1, 2, 2 };
static const uint32_t channelToAB[]   = { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 };
static Tc *channelToTC[] = {
  TC0, TC0, TC0, TC0, TC0, TC0,
  TC1, TC1, TC1, TC1, TC1, TC1,
  TC2, TC2, TC2, TC2, TC2, TC2 };
static const uint32_t channelToId[] = { 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8 };


// function setting the CMR register
static void TC_SetCMR_ChannelA(Tc *tc, uint32_t chan, uint32_t v){  tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xFFF0FFFF) | v;}
static void TC_SetCMR_ChannelB(Tc *tc, uint32_t chan, uint32_t v){ tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xF0FFFFFF) | v; }


// function which starts and syncs the timers 
// if the pin is the true PWM pin this function does not do anything
void syncTimers(uint32_t ulPin1,uint32_t ulPin2, uint32_t ulPin3 = -1, uint32_t ulPin4 = -1){
  uint32_t chNo1,chNo2,chNo3,chNo4;
  Tc *chTC1 = nullptr,*chTC2 = nullptr,*chTC3 = nullptr,*chTC4 = nullptr;

  // configure timer channel for the first pin if it is a timer pin
  uint32_t attr = g_APinDescription[ulPin1].ulPinAttribute;
  if ((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER) {
    ETCChannel channel1 = g_APinDescription[ulPin1].ulTCChannel;
    chNo1 = channelToChNo[channel1];
    chTC1 = channelToTC[channel1];
    TCChanEnabled[channelToId[channel1]] = 1;
  }
    
  // configure timer channel for the first pin if it is a timer pin
  attr = g_APinDescription[ulPin2].ulPinAttribute;
  if ((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER) {
    ETCChannel channel2 = g_APinDescription[ulPin2].ulTCChannel;
    chNo2 = channelToChNo[channel2];
    chTC2 = channelToTC[channel2];
    TCChanEnabled[channelToId[channel2]] = 1;
  }
  if(ulPin3 > 0 ){
    // configure timer channel for the first pin if it is a timer pin
    attr = g_APinDescription[ulPin3].ulPinAttribute;
    if ((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER) {
      ETCChannel channel3 = g_APinDescription[ulPin3].ulTCChannel;
      chNo3 = channelToChNo[channel3];
      chTC3 = channelToTC[channel3];
      TCChanEnabled[channelToId[channel3]] = 1;
    }
  }
  if(ulPin4  > 0 ){
    // configure timer channel for the first pin if it is a timer pin
    attr = g_APinDescription[ulPin4].ulPinAttribute;
    if ((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER) {
      ETCChannel channel4 = g_APinDescription[ulPin4].ulTCChannel;
      chNo4 = channelToChNo[channel4];
      chTC4 = channelToTC[channel4];
      TCChanEnabled[channelToId[channel4]] = 1;
    }
  }
  // start timers and make them synced
  if(chTC1){ 
    TC_Start(chTC1, chNo1);
    chTC1->TC_BCR = TC_BCR_SYNC;
  }
  if(chTC2){  
    TC_Start(chTC2, chNo2);
    chTC2->TC_BCR = TC_BCR_SYNC;
  }
  if(chTC3 && ulPin3){ 
    TC_Start(chTC3, chNo3);
    chTC3->TC_BCR = TC_BCR_SYNC;
  }
  if(chTC4 && ulPin4){ 
    TC_Start(chTC4, chNo4);
    chTC4->TC_BCR = TC_BCR_SYNC;
  }
}

// function configuring the pwm frequency for given pin
// possible to supply the pwm pin and the timer pin 
void initPWM(uint32_t ulPin, uint32_t pwm_freq){
  // check which pin type
  uint32_t attr = g_APinDescription[ulPin].ulPinAttribute;
  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM) { // if pwm pin

    if (!PWMEnabled) {
      // PWM Startup code
      pmc_enable_periph_clk(PWM_INTERFACE_ID);
			// this function does not work too well - I'll rewrite it
      // PWMC_ConfigureClocks(PWM_FREQUENCY * _max_pwm_value, 0, VARIANT_MCK);

      // finding the divisors an prescalers form FindClockConfiguration function
      uint32_t divisors[11] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
      uint8_t divisor = 0;
      uint32_t prescaler;

      /* Find prescaler and divisor values */
      prescaler = (VARIANT_MCK / divisors[divisor]) / (pwm_freq*_max_pwm_value);
      while ((prescaler > 255) && (divisor < 11)) {
          divisor++;
          prescaler = (VARIANT_MCK / divisors[divisor]) / (pwm_freq*_max_pwm_value);
      }
      // update the divisor*prescaler value 
      prescaler = prescaler | (divisor << 8);

      // now calculate the real resolution timer period necessary (pwm resolution)
      // pwm_res = bus_freq / (pwm_freq * (prescaler))
      _max_pwm_value = (double)VARIANT_MCK / (double)pwm_freq / (double)(prescaler);
      // set the prescaler value
      PWM->PWM_CLK = prescaler;

      PWMEnabled = 1;
    }

    uint32_t chan = g_APinDescription[ulPin].ulPWMChannel;
    if ((g_pinStatus[ulPin] & 0xF) != PIN_STATUS_PWM) {
      // Setup PWM for this pin
      PIO_Configure(g_APinDescription[ulPin].pPort,
          g_APinDescription[ulPin].ulPinType,
          g_APinDescription[ulPin].ulPin,
          g_APinDescription[ulPin].ulPinConfiguration);
      // PWM_CMR_CALG - center align
      // PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0);
      PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
      PWMC_SetPeriod(PWM_INTERFACE, chan, _max_pwm_value);
      PWMC_SetDutyCycle(PWM_INTERFACE, chan, 0);
      PWMC_EnableChannel(PWM_INTERFACE, chan);
      g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_PWM;
    }
    return;
  }

  if ((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER) { // if timer pin
    // We use MCLK/2 as clock.
    const uint32_t TC = VARIANT_MCK / 2 / pwm_freq ;
    // Setup Timer for this pin
    ETCChannel channel = g_APinDescription[ulPin].ulTCChannel;
    uint32_t chNo = channelToChNo[channel];
    uint32_t chA  = channelToAB[channel];
    Tc *chTC = channelToTC[channel];
    uint32_t interfaceID = channelToId[channel];

      if (!TCChanEnabled[interfaceID]) {
        pmc_enable_periph_clk(TC_INTERFACE_ID + interfaceID);
        TC_Configure(chTC, chNo,
          TC_CMR_TCCLKS_TIMER_CLOCK1 |
          TC_CMR_WAVE |         // Waveform mode
          TC_CMR_WAVSEL_UP_RC | // Counter running up and reset when equals to RC
          TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
          TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
          TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
        TC_SetRC(chTC, chNo, TC);
      } 

    // disable the counter on start
    if (chA){
      TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
    }else{
      TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET);
    }
    // configure input-ouput structure
    if ((g_pinStatus[ulPin] & 0xF) != PIN_STATUS_PWM) {
      PIO_Configure(g_APinDescription[ulPin].pPort,
          g_APinDescription[ulPin].ulPinType,
          g_APinDescription[ulPin].ulPin,
          g_APinDescription[ulPin].ulPinConfiguration);
      g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_PWM;
    }
    // enable interrupts
    chTC->TC_CHANNEL[chNo].TC_IER = TC_IER_CPAS       // interrupt on RA compare match
                          | TC_IER_CPBS         // interrupt on RB compare match
                          | TC_IER_CPCS;        // interrupt on RC compare match
    chTC->TC_CHANNEL[chNo].TC_IDR = ~TC_IER_CPAS       // interrupt on RA compare match
                            & ~TC_IER_CPBS         // interrupt on RB compare match
                            & ~ TC_IER_CPCS;        // interrupt on RC compare match
    // enable interrupts for this timer
    NVIC_EnableIRQ(irq_type[channel]);                  
    return;
  }
}

// pwm setting function
// it sets the duty cycle for pwm pin or timer pin
void setPwm(uint32_t ulPin, uint32_t ulValue) {
  // check pin type
  uint32_t attr = g_APinDescription[ulPin].ulPinAttribute;
  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM) { // if pwm
    uint32_t chan = g_APinDescription[ulPin].ulPWMChannel;
    PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue);
    return;
  }

  if ((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER) { // if timer pin
    // get the timer variables 
    ETCChannel channel = g_APinDescription[ulPin].ulTCChannel;
    Tc *chTC = channelToTC[channel];
    uint32_t chNo = channelToChNo[channel];
    if(!ulValue) {
      // if the value 0 disable counter
      if (channelToAB[channel])
        TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR);
      else
        TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
    }else{
      // if the value not zero
      // calculate clock
      const uint32_t TC = VARIANT_MCK / 2 / _pwm_frequency;
      // Map value to Timer ranges 0..max_duty_cycle => 0..TC
      // Setup Timer for this pin
      ulValue = ulValue * TC ;
      pwm_counter_vals[channel] = ulValue / _max_pwm_value;
      // enable counter
      if (channelToAB[channel])
        TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
      else
        TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET);
    }
    
    return;
  }
}

// interrupt handlers for seamless pwm duty-cycle setting
void TC0_Handler()
{
  // read/clear interrupt status
  TC_GetStatus(TC0, 0);
  // update the counters
  if(pwm_counter_vals[0]) TC_SetRA(TC0, 0, pwm_counter_vals[0]);
  if(pwm_counter_vals[1]) TC_SetRB(TC0, 0, pwm_counter_vals[1]);
}

void TC1_Handler()
{
  // read/clear interrupt status
  TC_GetStatus(TC0, 1);
  // update the counters
  if(pwm_counter_vals[2]) TC_SetRA(TC0, 1, pwm_counter_vals[2]);
  if(pwm_counter_vals[3]) TC_SetRB(TC0, 1, pwm_counter_vals[3]);
}

void TC2_Handler()
{
  // read/clear interrupt status
  TC_GetStatus(TC0, 2);
  // update the counters
  if(pwm_counter_vals[4]) TC_SetRA(TC0, 2, pwm_counter_vals[4]);
  if(pwm_counter_vals[5]) TC_SetRB(TC0, 2, pwm_counter_vals[5]);
}
void TC3_Handler()
{
  // read/clear interrupt status
  TC_GetStatus(TC1, 0);
  // update the counters
  if(pwm_counter_vals[6]) TC_SetRA(TC1, 0, pwm_counter_vals[6]);
  if(pwm_counter_vals[7]) TC_SetRB(TC1, 0, pwm_counter_vals[7]);
}

void TC4_Handler()
{
  // read/clear interrupt status
  TC_GetStatus(TC1, 1);
  // update the counters
  if(pwm_counter_vals[8]) TC_SetRA(TC1, 1, pwm_counter_vals[8]);
  if(pwm_counter_vals[9]) TC_SetRB(TC1, 1, pwm_counter_vals[9]);
}

void TC5_Handler()
{
  // read/clear interrupt status
  TC_GetStatus(TC1, 2);
  // update the counters
  if(pwm_counter_vals[10]) TC_SetRA(TC1, 2, pwm_counter_vals[10]);
  if(pwm_counter_vals[11]) TC_SetRB(TC1, 2, pwm_counter_vals[11]);
}
void TC6_Handler()
{
  // read/clear interrupt status
  TC_GetStatus(TC2, 0);
  // update the counters
  if(pwm_counter_vals[12]) TC_SetRA(TC2, 0, pwm_counter_vals[12]);
  if(pwm_counter_vals[13]) TC_SetRB(TC2, 0, pwm_counter_vals[13]);
}

void TC7_Handler()
{
  // read/clear interrupt status
  TC_GetStatus(TC2, 1);
  // update the counters
  if(pwm_counter_vals[14]) TC_SetRA(TC2, 1, pwm_counter_vals[14]);
  if(pwm_counter_vals[15]) TC_SetRB(TC2, 1, pwm_counter_vals[15]);
}

void TC8_Handler()
{
  // read/clear interrupt status
  TC_GetStatus(TC2, 2);
  // update the counters
  if(pwm_counter_vals[16]) TC_SetRA(TC2, 2, pwm_counter_vals[16]);
  if(pwm_counter_vals[17]) TC_SetRB(TC2, 2, pwm_counter_vals[17]);
}





// implementation of the hardware_api.cpp 
// ---------------------------------------------------------------------------------------------------------------------------------

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware specific
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 50khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // save the pwm frequency
  _pwm_frequency = pwm_frequency;
  // cinfigure pwm pins
  initPWM(pinA, _pwm_frequency);
  initPWM(pinB, _pwm_frequency);
  initPWM(pinC, _pwm_frequency);
  // sync the timers if possible
  syncTimers(pinA, pinB, pinC);

  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB, pinC },
    .pwm_frequency = pwm_frequency
  };
  return params;
}




// Configuring PWM frequency, resolution and alignment
//- Stepper driver - 2PWM setting
// - hardware specific
void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if(!pwm_frequency || !_isset(pwm_frequency)) pwm_frequency = _PWM_FREQUENCY; // default frequency 50khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // save the pwm frequency
  _pwm_frequency = pwm_frequency;
  // cinfigure pwm pins
  initPWM(pinA, _pwm_frequency);
  initPWM(pinB, _pwm_frequency);
  // sync the timers if possible
  syncTimers(pinA, pinB);

  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB },
    .pwm_frequency = pwm_frequency
  };
  return params;
}




// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
void* _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if(!pwm_frequency || !_isset(pwm_frequency)) pwm_frequency = _PWM_FREQUENCY; // default frequency 50khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // save the pwm frequency
  _pwm_frequency = pwm_frequency;
  // cinfigure pwm pins
  initPWM(pinA, _pwm_frequency);
  initPWM(pinB, _pwm_frequency);
  initPWM(pinC, _pwm_frequency);
  initPWM(pinD, _pwm_frequency);
  // sync the timers if possible
  syncTimers(pinA, pinB, pinC, pinD);

  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB, pinC, pinD },
    .pwm_frequency = pwm_frequency
  };
  return params;
}




// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
// - hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* param){
  // transform duty cycle from [0,1] to [0,_max_pwm_value]
  GenericDriverParams* p = (GenericDriverParams*)param;
  setPwm(p->pins[0], _max_pwm_value*dc_a);
  setPwm(p->pins[1], _max_pwm_value*dc_b);
  setPwm(p->pins[2], _max_pwm_value*dc_c);
}



// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
// - hardware speciffic
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* param){
  // transform duty cycle from [0,1] to [0,_max_pwm_value]
  GenericDriverParams* p = (GenericDriverParams*)param;
  setPwm(p->pins[0], _max_pwm_value*dc_1a);
  setPwm(p->pins[1], _max_pwm_value*dc_1b);
  setPwm(p->pins[2], _max_pwm_value*dc_2a);
  setPwm(p->pins[3], _max_pwm_value*dc_2b);
}



// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - Stepper driver - 2PWM setting
// - hardware specific
void  _writeDutyCycle2PWM(float dc_a,  float dc_b, void* param){
  // transform duty cycle from [0,1] to [0,_max_pwm_value]
  GenericDriverParams* p = (GenericDriverParams*)param;
  setPwm(p->pins[0], _max_pwm_value*dc_a);
  setPwm(p->pins[1], _max_pwm_value*dc_b);
}


#endif