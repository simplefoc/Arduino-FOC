
#include "./renesas.h"

#if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)

#include "communication/SimpleFOCDebug.h"
#include "FspTimer.h"

/*
  We use the GPT timers, there are 2 channels (32 bit) + 6 channels (16 bit)
  Each channel has 2 outputs (GTIOCAx and GTIOCBx) which can be complimentary

  So each timer channel can handle one half-bridge, using either a single or
  two complimentary PWM signals.

  For 1-PWM through 4-PWM, we need as many channels as PWM signals, and we can use 
  either output A or B of the timer (we can set the polarity).

  For 6-PWM we need 3 channels, and use both outputs A and B of each channel, then
  we can do hardware dead-time.
  Or we can use seperate channels for high and low side, with software dead-time.

  Selected channels can be started together, so we can keep the phases in sync for
  low-side current sensing and software 6-PWM.

  The event system should permit low side current sensing without needing interrupts,
  but this will be handled by the current sense driver.
 */


// track which channels are used
bool channel_used[GPT_HOWMANY] = { false };


struct RenesasTimerConfig {
  timer_cfg_t timer_cfg;
  gpt_instance_ctrl_t ctrl;
  gpt_extended_cfg_t ext_cfg;
  gpt_extended_pwm_cfg_t pwm_cfg;
  gpt_io_pin_t duty_pin;
};

struct ClockDivAndRange {
  timer_source_div_t clk_div;
  uint32_t range;
};

ClockDivAndRange getClockDivAndRange(uint32_t pwm_frequency, uint8_t timer_channel) {
  ClockDivAndRange result;
  uint32_t max_count = (timer_channel < GTP32_HOWMANY)? 4294967295 : 65535;
  uint32_t freq_hz = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD);
  float range = (float) freq_hz / ((float) pwm_frequency * 2.0f);
  if(range / 1.0 < max_count) {
      result.range = (uint32_t) (range / 1.0);
      result.clk_div = TIMER_SOURCE_DIV_1;
  }
  else if (range / 2.0 < max_count) {
      result.range = (uint32_t) (range / 2.0);
      result.clk_div = TIMER_SOURCE_DIV_2;
  }
  else if(range / 4.0 < max_count) {
      result.range = (uint32_t) (range / 4.0);
      result.clk_div = TIMER_SOURCE_DIV_4;
  }
  else if(range / 8.0 < max_count) {
      result.range = (uint32_t) (range / 8.0 );
      result.clk_div = TIMER_SOURCE_DIV_8;
  }
  else if(range / 16.0 < max_count) {
      result.range = (uint32_t) (range / 16.0 );
      result.clk_div = TIMER_SOURCE_DIV_16;
  }
  else if (range / 32.0 < max_count) {
      result.range = (uint32_t) (range / 32.0 );
      result.clk_div = TIMER_SOURCE_DIV_32;
  }
  else if(range / 64.0 < max_count) {
      result.range = (uint32_t) (range / 64.0 );
      result.clk_div = TIMER_SOURCE_DIV_64;
  }
  else if(range / 128.0 < max_count) {
      result.range = (uint32_t) (range / 128.0 );
      result.clk_div = TIMER_SOURCE_DIV_128;
  }
  else if(range / 256.0 < max_count) {
      result.range = (uint32_t) (range / 256.0 );
      result.clk_div = TIMER_SOURCE_DIV_256;
  }
  else if(range / 512.0 < max_count) {
      result.range = (uint32_t) (range / 512.0 );
      result.clk_div = TIMER_SOURCE_DIV_512;
  }
  else if(range / 1024.0 < max_count) {
      result.range = (uint32_t) (range / 1024.0 );
      result.clk_div = TIMER_SOURCE_DIV_1024;
  }
  else {
      SimpleFOCDebug::println("DRV: PWM frequency too low");
  }

  return result;
};


bool configureTimerPin(RenesasHardwareDriverParams* params, uint8_t index) {
  uint8_t pin = params->pins[index];
  std::array<uint16_t, 3> pinCfgs = getPinCfgs(pin, PIN_CFG_REQ_PWM);
  if(pinCfgs[0] == 0) {
    SIMPLEFOC_DEBUG("DRV: no PWM on pin");
    return false;
  }
  if (IS_PIN_AGT_PWM(pinCfgs[0])) {
    SIMPLEFOC_DEBUG("DRV: AGT timer not supported");
    return false;
  }
  // get the timer channel
  uint8_t timer_channel = GET_CHANNEL(pinCfgs[0]);
  // check its not used
  if (channel_used[timer_channel]) {
    SIMPLEFOC_DEBUG("DRV: channel in use");
    return false;
  }

  // configure GPIO pin
  // pinMode(pin, OUTPUT);
  fsp_err_t err = R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[pin].pin, (uint32_t) (IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));
  if (err != FSP_SUCCESS) {
    SIMPLEFOC_DEBUG("DRV: pin config failed");
    return false;
  }

  TimerPWMChannel_t pwm_output = IS_PWM_ON_A(pinCfgs[0]) ? CHANNEL_A : CHANNEL_B;  


  // configure timer channel - frequency / top value
  ClockDivAndRange timings = getClockDivAndRange(params->pwm_frequency, timer_channel);
  #if defined(SIMPLEFOC_RENESAS_DEBUG)
  SimpleFOCDebug::println("---PWM Config---");
  SimpleFOCDebug::println("DRV: pwm pin: ", pin);
  SimpleFOCDebug::println("DRV: pwm channel: ", timer_channel);
  SimpleFOCDebug::print("DRV: pwm A/B: "); SimpleFOCDebug::println((pwm_output==CHANNEL_A)?"A":"B");
  SimpleFOCDebug::println("DRV: pwm freq: ", (int)params->pwm_frequency);
  SimpleFOCDebug::println("DRV: pwm range: ", (int)timings.range);
  SimpleFOCDebug::println("DRV: pwm clkdiv: ", timings.clk_div);
  #endif

  RenesasTimerConfig* t = new RenesasTimerConfig();
  // configure timer channel - count mode
  t->timer_cfg.channel = timer_channel;
  t->timer_cfg.mode = TIMER_MODE_TRIANGLE_WAVE_SYMMETRIC_PWM;
  t->timer_cfg.source_div = timings.clk_div;
  t->timer_cfg.period_counts = timings.range;
  t->timer_cfg.duty_cycle_counts = 0;
  t->timer_cfg.p_callback = nullptr;
  t->timer_cfg.p_context = nullptr;
  t->timer_cfg.p_extend = &(t->ext_cfg);
  t->timer_cfg.cycle_end_ipl = BSP_IRQ_DISABLED;

  t->ext_cfg.p_pwm_cfg = &(t->pwm_cfg);
  t->pwm_cfg.trough_ipl = BSP_IRQ_DISABLED;
  t->pwm_cfg.trough_irq = FSP_INVALID_VECTOR;
  t->pwm_cfg.poeg_link = GPT_POEG_LINK_POEG0;
  t->pwm_cfg.output_disable = GPT_OUTPUT_DISABLE_NONE;
  t->pwm_cfg.adc_trigger = GPT_ADC_TRIGGER_NONE;
  t->pwm_cfg.dead_time_count_up = 0;
  t->pwm_cfg.dead_time_count_down = 0;
  t->pwm_cfg.adc_a_compare_match = 0;
  t->pwm_cfg.adc_b_compare_match = 0;
  t->pwm_cfg.interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE;
  t->pwm_cfg.interrupt_skip_count = GPT_INTERRUPT_SKIP_COUNT_0;
  t->pwm_cfg.interrupt_skip_adc = GPT_INTERRUPT_SKIP_ADC_NONE;
  t->pwm_cfg.gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED;
  t->pwm_cfg.gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED;



  // configure timer channel - polarity
  t->ext_cfg.gtior_setting.gtior = 0L;
  if(pwm_output == CHANNEL_A) {
      t->duty_pin = GPT_IO_PIN_GTIOCA;
      t->ext_cfg.gtioca.output_enabled = true;
      t->ext_cfg.gtiocb.output_enabled = false;
      t->ext_cfg.gtior_setting.gtior_b.gtioa = 0x03 | (SIMPLEFOC_PWM_ACTIVE_HIGH ? 0x00 : 0x08);
      t->ext_cfg.gtior_setting.gtior_b.oadflt = SIMPLEFOC_PWM_ACTIVE_HIGH ? 0x00 : 0x01;
      t->ext_cfg.gtior_setting.gtior_b.oahld = 0x0;
      t->ext_cfg.gtior_setting.gtior_b.oadf = 0x0;
      t->ext_cfg.gtior_setting.gtior_b.nfaen = 0x0;
  } 
  else {
      t->duty_pin = GPT_IO_PIN_GTIOCB;
      t->ext_cfg.gtiocb.output_enabled = true;
      t->ext_cfg.gtioca.output_enabled = false;
      t->ext_cfg.gtior_setting.gtior_b.gtiob = 0x03 | (SIMPLEFOC_PWM_ACTIVE_HIGH ? 0x00 : 0x08);
      t->ext_cfg.gtior_setting.gtior_b.obdflt = SIMPLEFOC_PWM_ACTIVE_HIGH ? 0x00 : 0x01;
      t->ext_cfg.gtior_setting.gtior_b.obhld = 0x0;
      t->ext_cfg.gtior_setting.gtior_b.obdf = 0x0;
      t->ext_cfg.gtior_setting.gtior_b.nfben = 0x0;
  }
  // t->duty_pin = GPT_IO_PIN_GTIOCA_AND_GTIOCB;
  // TODO configure timer channel - dead-time if both outputs are used
  memset(&(t->ctrl), 0, sizeof(gpt_instance_ctrl_t));

  err = R_GPT_Open(&(t->ctrl),&(t->timer_cfg));
  if ((err != FSP_ERR_ALREADY_OPEN) && (err != FSP_SUCCESS)) {
    SIMPLEFOC_DEBUG("DRV: open failed");
    return false;
  }
  #if defined(SIMPLEFOC_RESENSAS_DEBUG)
  if (err == FSP_ERR_ALREADY_OPEN) {
    SimpleFOCDebug::println("DRV: timer already open");
  }
  #endif
  // err = R_GPT_Enable(&(t->ctrl));
  // if (err != FSP_SUCCESS) {
  //   SIMPLEFOC_DEBUG("DRV: enable failed");
  //   return false;
  // }
  err = R_GPT_PeriodSet(&(t->ctrl), t->timer_cfg.period_counts);
  if (err != FSP_SUCCESS) {
    SIMPLEFOC_DEBUG("DRV: period set failed");
    return false;
  }  
  err = R_GPT_OutputEnable(&(t->ctrl), t->duty_pin);
  if (err != FSP_SUCCESS) {
    SIMPLEFOC_DEBUG("DRV: pin enable failed");
    return false;
  }
  params->timer_config[index] = t;
  params->channels[index] = timer_channel;
  channel_used[timer_channel] = true;

  return true;
}


bool startTimerChannels(RenesasHardwareDriverParams* params, int num_channels) {

  // start the channels
  uint32_t mask = 0;
  for (int i = 0; i < num_channels; i++) {
    RenesasTimerConfig* t = params->timer_config[i];
    // if (R_GPT_Start(&(t->ctrl)) != FSP_SUCCESS) {
    //   SIMPLEFOC_DEBUG("DRV: timer start failed");
    //   return false;
    // }
    mask |= (1 << params->channels[i]);
#if defined(SIMPLEFOC_RENESAS_DEBUG)
    SimpleFOCDebug::println("DRV: starting timer: ", params->channels[i]);
#endif
  }
  params->timer_config[0]->ctrl.p_reg->GTSTR |= mask;
  #if defined(SIMPLEFOC_RENESAS_DEBUG)
    SimpleFOCDebug::println("DRV: timers started");
  #endif
  return true;
}





void* _configure1PWM(long pwm_frequency, const int pinA) {
  RenesasHardwareDriverParams* params = new RenesasHardwareDriverParams;
  params->pins[0] = pinA;
  params->pwm_frequency = (pwm_frequency==NOT_SET)?RENESAS_DEFAULT_PWM_FREQUENCY:pwm_frequency;
  bool success = true;
  success = configureTimerPin(params, 0);
  if (success)
    success = startTimerChannels(params, 1);
  if (!success)
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  return params;
}


void* _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
  RenesasHardwareDriverParams* params = new RenesasHardwareDriverParams;
  params->pins[0] = pinA; params->pins[1] = pinB;
  params->pwm_frequency = (pwm_frequency==NOT_SET)?RENESAS_DEFAULT_PWM_FREQUENCY:pwm_frequency;
  bool success = true;
  success = configureTimerPin(params, 0);
  if (success)
    success = configureTimerPin(params, 1);
  if (success)
    success = startTimerChannels(params, 2);
  if (!success)
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  return params;
}


void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  RenesasHardwareDriverParams* params = new RenesasHardwareDriverParams;
  params->pins[0] = pinA; params->pins[1] = pinB; params->pins[2] = pinC;
  params->pwm_frequency = (pwm_frequency==NOT_SET)?RENESAS_DEFAULT_PWM_FREQUENCY:pwm_frequency;
  bool success = true;
  success = configureTimerPin(params, 0);
  if (success)
    success = configureTimerPin(params, 1);
  if (success)
    success = configureTimerPin(params, 2);
  if (success)
    success = startTimerChannels(params, 3);
  if (!success)
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  return params;
}


void* _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
  RenesasHardwareDriverParams* params = new RenesasHardwareDriverParams;
  params->pins[0] = pin1A; params->pins[1] = pin1B; params->pins[2] = pin2A; params->pins[3] = pin2B;
  params->pwm_frequency = (pwm_frequency==NOT_SET)?RENESAS_DEFAULT_PWM_FREQUENCY:pwm_frequency;
  bool success = true;
  success = configureTimerPin(params, 0);
  if (success)
    success = configureTimerPin(params, 1);
  if (success)
    success = configureTimerPin(params, 2);
  if (success)
    success = configureTimerPin(params, 3);
  if (success)
    success = startTimerChannels(params, 4);
  if (!success)
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  return params;
}


void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l },
    .pwm_frequency = pwm_frequency
  };
  return params;
}




void _writeDutyCycle1PWM(float dc_a, void* params){
  RenesasTimerConfig* t = ((RenesasHardwareDriverParams*)params)->timer_config[0];
  uint32_t duty_cycle_counts = (uint32_t)(dc_a * (float)(t->timer_cfg.period_counts));
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
}


void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  RenesasTimerConfig* t = ((RenesasHardwareDriverParams*)params)->timer_config[0];
  uint32_t duty_cycle_counts = (uint32_t)(dc_a * (float)(t->timer_cfg.period_counts));
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
  t = ((RenesasHardwareDriverParams*)params)->timer_config[1];
  duty_cycle_counts = (uint32_t)(dc_b * (float)(t->timer_cfg.period_counts));
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
}


void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  RenesasTimerConfig* t = ((RenesasHardwareDriverParams*)params)->timer_config[0];
  uint32_t duty_cycle_counts = (uint32_t)(dc_a * (float)(t->timer_cfg.period_counts));
  //SimpleFOCDebug::println("Duty A: ", (int)duty_cycle_counts);
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
      Serial.println("pwm set error A");
  }
  t = ((RenesasHardwareDriverParams*)params)->timer_config[1];
  duty_cycle_counts = (uint32_t)(dc_b * (float)(t->timer_cfg.period_counts));
  //SimpleFOCDebug::println("Duty B: ", (int)duty_cycle_counts);
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
      Serial.println("pwm set error B");
  }
  t = ((RenesasHardwareDriverParams*)params)->timer_config[2];
  duty_cycle_counts = (uint32_t)(dc_c * (float)(t->timer_cfg.period_counts));
  //SimpleFOCDebug::println("Duty C: ", (int)duty_cycle_counts);
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
      Serial.println("pwm set error C");
  }
}


void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  RenesasTimerConfig* t = ((RenesasHardwareDriverParams*)params)->timer_config[0];
  uint32_t duty_cycle_counts = (uint32_t)(dc_1a * (float)(t->timer_cfg.period_counts));
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
  t = ((RenesasHardwareDriverParams*)params)->timer_config[1];
  duty_cycle_counts = (uint32_t)(dc_1b * (float)(t->timer_cfg.period_counts));
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
  t = ((RenesasHardwareDriverParams*)params)->timer_config[2];
  duty_cycle_counts = (uint32_t)(dc_2a * (float)(t->timer_cfg.period_counts));
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
  t = ((RenesasHardwareDriverParams*)params)->timer_config[3];
  duty_cycle_counts = (uint32_t)(dc_2b * (float)(t->timer_cfg.period_counts));
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
}


void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){

}

#endif