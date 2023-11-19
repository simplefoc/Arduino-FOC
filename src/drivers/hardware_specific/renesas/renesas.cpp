
#include "./renesas.h"

#if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)


#pragma message("")
#pragma message("SimpleFOC: compiling for Arduino/Renesas (UNO R4)")
#pragma message("")



#include "communication/SimpleFOCDebug.h"
#include "FspTimer.h"

#define GPT_OPEN                                         (0x00475054ULL)

/*
  We use the GPT timers, there are 2 channels (32 bit) + 6 channels (16 bit)
  Each channel has 2 outputs (GTIOCAx and GTIOCBx) which can be complimentary.

  So each timer channel can handle one half-bridge, using either a single (3-PWM) or
  two complimentary PWM signals (6-PWM).

  For 1-PWM through 4-PWM, we need as many channels as PWM signals, and we can use 
  either output A or B of the timer (we can set the polarity) - but not both.

  For 6-PWM we need 3 channels, and use both outputs A and B of each channel, then
  we can do hardware dead-time.
  Or we can use seperate channels for high and low side, with software dead-time.
  Each phase can be either hardware (1 channel) or software (2 channels) dead-time
  individually, they don't all have to be one or the other.

  Selected channels can be started together, so we can keep the phases in sync for
  low-side current sensing and software 6-PWM.

  The event system should permit low side current sensing without needing interrupts,
  but this will be handled by the current sense driver.

  Supported:
    - arbitrary PWM frequencies between 1Hz (minimum we can set with our integer based API)
      and around 48kHz (more would be possible but the range will be low)
    - PWM range at 24kHz (default) is 1000
    - PWM range at 48kHz is 500
    - polarity setting is supported, in all modes
    - phase state setting is supported, in 3-PWM, 6-PWM hardware dead-time and 6-PWM software dead-time

  TODOs:
    - change setDutyCycle to use register access for speed
    - add event system support for low-side current sensing
    - perhaps add support to reserve timers used also in 
      Arduino Pwm.h code, for compatibility with analogWrite()
    - check if there is a better way for phase-state setting
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


bool configureTimerPin(RenesasHardwareDriverParams* params, uint8_t index, bool active_high, bool complementary = false, bool complementary_active_high = true) {
  uint8_t pin = params->pins[index];
  uint8_t pin_C;
  std::array<uint16_t, 3> pinCfgs = getPinCfgs(pin, PIN_CFG_REQ_PWM);
  std::array<uint16_t, 3> pinCfgs_C;
  if(pinCfgs[0] == 0) {
    SIMPLEFOC_DEBUG("DRV: no PWM on pin ", pin);
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
    SIMPLEFOC_DEBUG("DRV: channel in use: ", timer_channel);
    return false;
  }

  if (complementary) {
    pin_C = params->pins[index+1];
    pinCfgs_C = getPinCfgs(pin_C, PIN_CFG_REQ_PWM);
    if(pinCfgs_C[0] == 0) {
      SIMPLEFOC_DEBUG("DRV: no PWM on pin ", pin_C);
      return false;
    }
    if (IS_PIN_AGT_PWM(pinCfgs_C[0]) || GET_CHANNEL(pinCfgs_C[0])!=timer_channel) {
      SIMPLEFOC_DEBUG("DRV: comp. channel different");
      return false;
    }
  }
  TimerPWMChannel_t pwm_output = IS_PWM_ON_A(pinCfgs[0]) ? CHANNEL_A : CHANNEL_B;
  if (complementary) {
    TimerPWMChannel_t pwm_output_C = IS_PWM_ON_A(pinCfgs_C[0]) ? CHANNEL_A : CHANNEL_B;
    if (pwm_output_C != CHANNEL_A || pwm_output != CHANNEL_B) {
      SIMPLEFOC_DEBUG("DRV: output A/B mismatch");
      return false;
    }
  }

  // configure GPIO pin
  fsp_err_t err = R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[pin].pin, (uint32_t) (IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));
  if ((err == FSP_SUCCESS) && complementary)
    err = R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[pin_C].pin, (uint32_t) (IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));
  if (err != FSP_SUCCESS) {
    SIMPLEFOC_DEBUG("DRV: pin config failed");
    return false;
  }


  // configure timer channel - frequency / top value
  ClockDivAndRange timings = getClockDivAndRange(params->pwm_frequency, timer_channel);
  #if defined(SIMPLEFOC_RENESAS_DEBUG)
  SimpleFOCDebug::println("---PWM Config---");
  SimpleFOCDebug::println("DRV: pwm pin: ", pin);
  if (complementary)
    SimpleFOCDebug::println("DRV: compl. pin: ", pin_C);
  SimpleFOCDebug::println("DRV: pwm channel: ", timer_channel);
  SimpleFOCDebug::print("DRV: pwm A/B: "); SimpleFOCDebug::println(complementary?"A+B":((pwm_output==CHANNEL_A)?"A":"B"));
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
  t->timer_cfg.cycle_end_irq = FSP_INVALID_VECTOR;

  t->ext_cfg.p_pwm_cfg = &(t->pwm_cfg);
  t->ext_cfg.capture_a_ipl = BSP_IRQ_DISABLED;
  t->ext_cfg.capture_a_irq = FSP_INVALID_VECTOR;
  t->ext_cfg.capture_b_ipl = BSP_IRQ_DISABLED;
  t->ext_cfg.capture_b_irq = FSP_INVALID_VECTOR;
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
  // configure dead-time if both outputs are used
  if (complementary) {
    uint32_t dt = params->dead_zone * timings.range;
    t->pwm_cfg.dead_time_count_up = dt;
    t->pwm_cfg.dead_time_count_down = dt;
  }

  // configure timer channel - outputs and polarity
  t->ext_cfg.gtior_setting.gtior = 0L;
  if (!complementary) {
    if(pwm_output == CHANNEL_A) {
        t->duty_pin = GPT_IO_PIN_GTIOCA;
        t->ext_cfg.gtioca.output_enabled = true;
        t->ext_cfg.gtiocb.output_enabled = false;
        t->ext_cfg.gtior_setting.gtior_b.gtioa = 0x03 | (active_high ? 0x00 : 0x10);
        t->ext_cfg.gtior_setting.gtior_b.oadflt = active_high ? 0x00 : 0x01;
        // t->ext_cfg.gtior_setting.gtior_b.oahld = 0x0;
        // t->ext_cfg.gtior_setting.gtior_b.oadf = 0x0;
        // t->ext_cfg.gtior_setting.gtior_b.nfaen = 0x0;
    } 
    else {
        t->duty_pin = GPT_IO_PIN_GTIOCB;
        t->ext_cfg.gtiocb.output_enabled = true;
        t->ext_cfg.gtioca.output_enabled = false;
        t->ext_cfg.gtior_setting.gtior_b.gtiob = 0x03 | (active_high ? 0x00 : 0x10);
        t->ext_cfg.gtior_setting.gtior_b.obdflt = active_high ? 0x00 : 0x01;
    }
  }
  else {
    t->duty_pin = GPT_IO_PIN_GTIOCA_AND_GTIOCB;
    t->ext_cfg.gtioca.output_enabled = true;
    t->ext_cfg.gtiocb.output_enabled = true;
    t->ext_cfg.gtior_setting.gtior_b.gtioa = 0x03 | (!complementary_active_high ? 0x00 : 0x10);
    t->ext_cfg.gtior_setting.gtior_b.oadflt = !complementary_active_high ? 0x00 : 0x01;
    t->ext_cfg.gtior_setting.gtior_b.gtiob = 0x03 | (active_high ? 0x00 : 0x10);
    t->ext_cfg.gtior_setting.gtior_b.obdflt = active_high ? 0x00 : 0x01;
  }
  memset(&(t->ctrl), 0, sizeof(gpt_instance_ctrl_t));
  err = R_GPT_Open(&(t->ctrl),&(t->timer_cfg));
  if ((err != FSP_ERR_ALREADY_OPEN) && (err != FSP_SUCCESS)) {
    SIMPLEFOC_DEBUG("DRV: open failed");
    return false;
  }
  if (err == FSP_ERR_ALREADY_OPEN) {
    SimpleFOCDebug::println("DRV: timer already open");
    return false;
  }

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

  channel_used[timer_channel] = true;
  params->timer_config[index] = t;
  params->channels[index] = timer_channel;
  if (complementary) {
    params->timer_config[index+1] = t;
    params->channels[index+1] = timer_channel;
  }

  return true;
}


// start the timer channels for the motor, synchronously
bool startTimerChannels(RenesasHardwareDriverParams* params, int num_channels) {
  uint32_t mask = 0;
  for (int i = 0; i < num_channels; i++) {
    // RenesasTimerConfig* t = params->timer_config[i];
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


// check if the given pins are on the same timer channel
bool isHardware6Pwm(const int pin1, const int pin2) {
  std::array<uint16_t, 3> pinCfgs1 = getPinCfgs(pin1, PIN_CFG_REQ_PWM);
  std::array<uint16_t, 3> pinCfgs2 = getPinCfgs(pin2, PIN_CFG_REQ_PWM);
  if(pinCfgs1[0] == 0 || pinCfgs2[0] == 0)
    return false;
  if (IS_PIN_AGT_PWM(pinCfgs1[0]) || IS_PIN_AGT_PWM(pinCfgs2[0]))
    return false;
  uint8_t timer_channel1 = GET_CHANNEL(pinCfgs1[0]);
  uint8_t timer_channel2 = GET_CHANNEL(pinCfgs2[0]);
  return timer_channel1==timer_channel2;
}



void* _configure1PWM(long pwm_frequency, const int pinA) {
  RenesasHardwareDriverParams* params = new RenesasHardwareDriverParams;
  params->pins[0] = pinA;
  params->pwm_frequency = (pwm_frequency==NOT_SET)?RENESAS_DEFAULT_PWM_FREQUENCY:pwm_frequency;
  bool success = true;
  success = configureTimerPin(params, 0, SIMPLEFOC_PWM_ACTIVE_HIGH);
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
  success = configureTimerPin(params, 0, SIMPLEFOC_PWM_ACTIVE_HIGH);
  success &= configureTimerPin(params, 1, SIMPLEFOC_PWM_ACTIVE_HIGH);
  if (!success)
    success &= startTimerChannels(params, 2);
  if (!success)
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  return params;
}


void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  RenesasHardwareDriverParams* params = new RenesasHardwareDriverParams;
  params->pins[0] = pinA; params->pins[1] = pinB; params->pins[2] = pinC;
  params->pwm_frequency = (pwm_frequency==NOT_SET)?RENESAS_DEFAULT_PWM_FREQUENCY:pwm_frequency;
  bool success = true;
  success = configureTimerPin(params, 0, SIMPLEFOC_PWM_ACTIVE_HIGH);
  success &= configureTimerPin(params, 1, SIMPLEFOC_PWM_ACTIVE_HIGH);
  success &= configureTimerPin(params, 2, SIMPLEFOC_PWM_ACTIVE_HIGH);
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
  success = configureTimerPin(params, 0, SIMPLEFOC_PWM_ACTIVE_HIGH);
  success &= configureTimerPin(params, 1, SIMPLEFOC_PWM_ACTIVE_HIGH);
  success &= configureTimerPin(params, 2, SIMPLEFOC_PWM_ACTIVE_HIGH);
  success &= configureTimerPin(params, 3, SIMPLEFOC_PWM_ACTIVE_HIGH);
  if (success)
    success = startTimerChannels(params, 4);
  if (!success)
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  return params;
}


void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  RenesasHardwareDriverParams* params = new RenesasHardwareDriverParams;
  params->pins[0] = pinA_h; params->pins[1] = pinA_l; params->pins[2] = pinB_h; params->pins[3] = pinB_l; params->pins[4] = pinC_h; params->pins[5] = pinC_l;
  params->pwm_frequency = (pwm_frequency==NOT_SET)?RENESAS_DEFAULT_PWM_FREQUENCY:pwm_frequency;
  params->dead_zone = (dead_zone==NOT_SET)?RENESAS_DEFAULT_DEAD_ZONE:dead_zone;

  bool success = true;
  if (isHardware6Pwm(pinA_h, pinA_l))
    success &= configureTimerPin(params, 0, SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH, true, SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH);
  else {
    success &= configureTimerPin(params, 0, SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH);
    success &= configureTimerPin(params, 1, !SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH); // reverse polarity on low side gives desired active high/low behaviour
  }

  if (isHardware6Pwm(pinB_h, pinB_l))
    success &= configureTimerPin(params, 2, SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH, true, SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH);
  else {
    success &= configureTimerPin(params, 2, SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH);
    success &= configureTimerPin(params, 3, !SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH);
  }

  if (isHardware6Pwm(pinC_h, pinC_l))
    success &= configureTimerPin(params, 4, SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH, true, SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH);
  else {
    success &= configureTimerPin(params, 4, SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH);
    success &= configureTimerPin(params, 5, !SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH);
  }

  if (success)
    success = startTimerChannels(params, 6);
  if (!success)
    return SIMPLEFOC_DRIVER_INIT_FAILED;
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
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
  t = ((RenesasHardwareDriverParams*)params)->timer_config[1];
  duty_cycle_counts = (uint32_t)(dc_b * (float)(t->timer_cfg.period_counts));
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
  t = ((RenesasHardwareDriverParams*)params)->timer_config[2];
  duty_cycle_counts = (uint32_t)(dc_c * (float)(t->timer_cfg.period_counts));
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
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



void _setSinglePhaseState(RenesasTimerConfig* hi, RenesasTimerConfig* lo, PhaseState state) {
  gpt_gtior_setting_t gtior;
  gtior.gtior = hi->ctrl.p_reg->GTIOR;
  bool on = (state==PHASE_ON) || (state==PHASE_HI);

  if (hi->duty_pin == GPT_IO_PIN_GTIOCA_AND_GTIOCB) {
    bool ch = false;
    if (gtior.gtior_b.obe != on) {
      gtior.gtior_b.obe = on;
      ch = true;
    } // B is high side
    on = (state==PHASE_ON) || (state==PHASE_LO);
    if (gtior.gtior_b.oae != on) {
      gtior.gtior_b.oae = on;
      ch = true;
    }
    if (ch)
      hi->ctrl.p_reg->GTIOR = gtior.gtior;
    return;
  }

  if (hi->duty_pin == GPT_IO_PIN_GTIOCA) {
    if (gtior.gtior_b.oae != on) {
      gtior.gtior_b.oae = on;
      hi->ctrl.p_reg->GTIOR = gtior.gtior;
    }
  }
  else if (hi->duty_pin == GPT_IO_PIN_GTIOCB) {
    if (gtior.gtior_b.obe != on) {
      gtior.gtior_b.obe = on;
      hi->ctrl.p_reg->GTIOR = gtior.gtior;
    }
  }

  gtior.gtior = lo->ctrl.p_reg->GTIOR;
  on = (state==PHASE_ON) || (state==PHASE_LO);
  if (lo->duty_pin == GPT_IO_PIN_GTIOCA) {
    if (gtior.gtior_b.oae != on) {
      gtior.gtior_b.oae = on;
      lo->ctrl.p_reg->GTIOR = gtior.gtior;
    }
  }
  else if (lo->duty_pin == GPT_IO_PIN_GTIOCB) {
    if (gtior.gtior_b.obe != on) {
      gtior.gtior_b.obe = on;
      lo->ctrl.p_reg->GTIOR = gtior.gtior;
    }
  }

}


void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
  RenesasTimerConfig* t = ((RenesasHardwareDriverParams*)params)->timer_config[0];
  RenesasTimerConfig* t1 = ((RenesasHardwareDriverParams*)params)->timer_config[1];
  uint32_t dt = (uint32_t)(((RenesasHardwareDriverParams*)params)->dead_zone * (float)(t->timer_cfg.period_counts));
  uint32_t duty_cycle_counts = (uint32_t)(dc_a * (float)(t->timer_cfg.period_counts));
  bool hw_deadtime = ((RenesasHardwareDriverParams*)params)->channels[0] == ((RenesasHardwareDriverParams*)params)->channels[1];
  uint32_t dt_act = (duty_cycle_counts>0 && !hw_deadtime)?dt:0;
  _setSinglePhaseState(t, t1, phase_state[0]);
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts - dt_act, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
  if (!hw_deadtime) {
    if (R_GPT_DutyCycleSet(&(t1->ctrl), duty_cycle_counts + dt_act, t1->duty_pin) != FSP_SUCCESS) {
        // error
    }    
  }

  t = ((RenesasHardwareDriverParams*)params)->timer_config[2];
  t1 = ((RenesasHardwareDriverParams*)params)->timer_config[3];
  duty_cycle_counts = (uint32_t)(dc_b * (float)(t->timer_cfg.period_counts));
  hw_deadtime = ((RenesasHardwareDriverParams*)params)->channels[2] == ((RenesasHardwareDriverParams*)params)->channels[3];
  dt_act = (duty_cycle_counts>0 && !hw_deadtime)?dt:0;
  _setSinglePhaseState(t, t1, phase_state[1]);
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts - dt_act, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
  if (!hw_deadtime) {
    if (R_GPT_DutyCycleSet(&(t1->ctrl), duty_cycle_counts + dt_act, t1->duty_pin) != FSP_SUCCESS) {
        // error
    }    
  }

  t = ((RenesasHardwareDriverParams*)params)->timer_config[4];
  t1 = ((RenesasHardwareDriverParams*)params)->timer_config[5];
  duty_cycle_counts = (uint32_t)(dc_c * (float)(t->timer_cfg.period_counts));
  hw_deadtime = ((RenesasHardwareDriverParams*)params)->channels[4] == ((RenesasHardwareDriverParams*)params)->channels[5];
  dt_act = (duty_cycle_counts>0 && !hw_deadtime)?dt:0;
  _setSinglePhaseState(t, t1, phase_state[2]);
  if (R_GPT_DutyCycleSet(&(t->ctrl), duty_cycle_counts, t->duty_pin) != FSP_SUCCESS) {
      // error
  }
  if (!hw_deadtime) {
    if (R_GPT_DutyCycleSet(&(t1->ctrl), duty_cycle_counts + dt_act, t1->duty_pin) != FSP_SUCCESS) {
        // error
    }    
  }

}

#endif