
/*
* MCPWM in espressif v5.x has
* - 2x groups (units)
*   each one has
*   - 3 timers
*   - 3 operators (that can be associated with any timer)
*     which control a 2xPWM signals
*     - 1x comparator + 1x generator per PWM signal


* Independent mode:
* ------------------
*  6 PWM independent signals per unit
*  unit(0/1) > timer(0-2) > operator(0-2) > comparator(0-1) > generator(0-1) > pwm(A/B)
*
* -------------------------------------- Table View ----------------------------- 
*
* group    |  timer  |  operator   | comparator            |  generator    |  pwm
* --------------------------------------------------------------------------------
*   0-1    |  0-2    |  0          |  0                    |  0            |  A
*   0-1    |  0-2    |  0          |  1                    |  1            |  B
*   0-1    |  0-2    |  1          |  0                    |  0            |  A
*   0-1    |  0-2    |  1          |  1                    |  1            |  B
*   0-1    |  0-2    |  2          |  0                    |  0            |  A
*   0-1    |  0-2    |  2          |  1                    |  1            |  B
*
* ------------------------------------- Example 3PWM ------------------------------
*                                      ┌─ comparator 0 - generator 0 -> pwm A
*                       ┌─ operator 0 -|
*                       |              └─ comparator 1 - generator 1 -> pmw B  
*    unit  - timer 0-2 -|
*    0-1                └─ operator 1 - comparator 0 - generator 0 - pwm C
*    
* ------------------------------------- Example 2PWM ------------------------------
*                                   ┌─ comparator 0 - generator 0 -> pwm A
*    unit - timer 0-2 - operator 0 -|
*    0-1                            └─ comparator 1 - generator 1 -> pmw B  
*
* -------------------------------------- Example 4PWM ----------------------------- 
*                                     ┌─ comparator 0 - generator 0 -> pwm A
*                      ┌─ operator 0 -|
*                      |              └─ comparator 1 - generator 1 -> pmw B  
*    unit - timer 0-2 -| 
*    0-1               |              ┌─ comparator 0 - generator 0 -> pwm C
*                      └─ operator 1 -|
*                                     └─ comparator 0 - generator 0 -> pwm D   


* Complementary mode
* ------------------
*  - : 3 pairs of complementary PWM signals per unit
*  unit(0/1) > timer(0) > operator(0-2) > comparator(0-1) > generator(0-1) > pwm(high/low pair)
* 
* -------------------------------------- Table View ----------------------------- 
*
* group    |  timer  |  operator   | comparator  |  generator    |  pwm
* ------------------------------------------------------------------------
*   0-1    |  0      |  0          |  0          |  0            |  A
*   0-1    |  0      |  0          |  1          |  1            |  B
*   0-1    |  0      |  1          |  0          |  0            |  A
*   0-1    |  0      |  1          |  1          |  1            |  B
*   0-1    |  0      |  2          |  0          |  0            |  A
*   0-1    |  0      |  2          |  1          |  1            |  B
*
* -------------------------------------- Example 6PWM ----------------------------- 
*     
*                                        ┌─ comparator 0 - generator 0 -> pwm A_h
*                         ┌─ operator 0 -| 
*                         |              └─ comparator 1 - generator 1 -> pmw A_l
*                         | 
*     unit                |              ┌─ comparator 0 - generator 0 -> pwm B_h
*    (group)  -  timer 0 -|- operator 1 -|
*      0-1                |              └─ comparator 1 - generator 1 -> pmw B_l
*                         |
*                         |              ┌─ comparator 0 - generator 0 -> pwm C_h
*                         └─ operator 2 -|
*                                        └─ comparator 1 - generator 1 -> pmw C_l
*   


* More info
* ----------
* - timers can be associated with any operator, and multiple operators can be associated with the same timer
* - comparators can be associated with any operator 
*   - two comparators per operator for independent mode
*   - one comparator per operator for complementary mode
* - generators can be associated with any comparator
*   - one generator per PWM signal for independent mode
*   - two generators per pair of PWM signals for complementary mode (not used in simplefoc)
* - dead-time can be set for each generator pair in complementary mode
*
* Docs
* -------
* More info here: https:*www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#mcpwm
* and here: // https://docs.espressif.com/projects/esp-idf/en/v5.1.4/esp32/migration-guides/release-5.x/5.0/peripherals.html
*/

#include "../../hardware_api.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

#include "esp32_driver_mcpwm.h"

// MCPWM driver hardware timer pointers
mcpwm_timer_handle_t timers[2][3] = {NULL};
// MCPWM timer periods configured (directly related to the pwm frequency) 
uint32_t pwm_periods[2][3];
// how many pins from the groups 6 pins is used
uint8_t group_pins_used[2] = {0};
// last operator in the group
mcpwm_oper_handle_t last_operator[2];



// checking if group has pins available
bool _hasAvailablePins(int group, int no_pins){
  if(group_pins_used[group] + no_pins > 6){
    return false;
  }
  return true;
}

// returns the index of the last timer in the group
// -1 if no timer instantiated yet
uint8_t _findLastTimer(int group){
  int i = 0;
  for(; i<3; i++){
    if(timers[group][i] == NULL){
       return i-1;
    }
  }
  // return the last index
  return i;
}
// returns the index of the next timer to instantiate 
// -1 if no timers available
uint8_t _findNextTimer(int group){
  int i = 0;
  for(; i<3; i++){
    if(timers[group][i] == NULL){
      return i;
    }
  }
  return -1;
}

/*
 * find the best group for the pins
 * if 6pwm 
 *   - Only option is an an empty group
 * if 3pwm
 *   - Best is an empty group (we can set a pwm frequency)
 *   - Second best is a group with 4pwms (2 operators) available (we can set the pwm frequency -> new timer+new operator)
 *   - Third best option is any group which has 3pwms available (but uses previously defined pwm_frequency)
 * if 1pwm
 *   - Best option is an empty group (we can set the pwm frequency)
 *   - Second best is a group with 2pwms (one operator) available (we can set the pwm frequency -> new timer+new operator)
 *   - Third best is a group with 1pwm available (but uses previously defined pwm_frequency )
 * if 2pwm
 *   - Best option is an empty group  (we can set the pwm frequency)
 *   - Second best is a group with 2pwms available (we can set the pwm frequency -> new timer+new operator)
 *   - Third best is one pin per group (but uses previously defined pwm_frequency )
 * if 4pwm
 *   - best option is an empty group  (we can set the pwm frequency)
 *   - second best is a group with 4pwms available  (we can set the pwm frequency -> new timer + new operators)
 *   - third best is 2pwms per group (we can set the pwm frequency -> new timers + new operators)
 *
 * PROBLEM: Skipping/loosing channels happens in some cases when the group has already used some odd number of pwm channels (for example 3pwm or 1pwm)
 * For example if the group has already used 3pwms, there is one generator that has one pwm channel left.
 * If we use this channel we have to use the same timer it has been used with before, so we cannot change the pwm frequency.
 * Current implementation does use the remaining channel only if there isn't other options that would allow changing the pwm frequency.
 * In this example where we have 3pwms already configured, if we try to configure 2pws after, we will skip the remaining channel 
 * and use a new timer and operator to allow changing the pwm frequency. In such cases we loose (cannot be used) the remaining channel.
 * TODO: use the pwm_frequency to avoid skipping pwm channels !
 *
 * returns 
 *  - 1 if solution found in one group
 *  - 2 if solution requires using both groups
 *  - 0 if no solution possible
*/
int _findBestGroup(int no_pins, long pwm_freq, int* group, int* timer){
  // an empty group is always the best option
  for(int i=0; i<2; i++){
    if(!group_pins_used[i]){
      *group = i;
      *timer=0; // use the first timer in an empty group
      return 1;
    }
  }

  // if 3 or 1pwm 
  // check if there is available space in one of the groups
  // otherwise fail
  if(no_pins == 3 || no_pins==1){
    // second best option is if there is a group with 
    // pair number of pwms available as we can then 
    // set the pwm frequency 
    for(int i=0; i<2; i++){
      if(_hasAvailablePins(i, no_pins+1)) {
        *group=i;
        *timer = _findNextTimer(i);
        return 1;
      }
    }
    // third best option is any group that has enough pins
    for(int i=0; i<2; i++){
      if(_hasAvailablePins(i, no_pins)) {
        *group=i;
        *timer = _findLastTimer(i);
        return 1;
      }
    }
  }

  // if 2 or 4 pwm
  // check if there is available space in one of the groups
  // if not check if they can be separated in two groups
  if(no_pins == 2 || no_pins==4){
    // second best option is any group that has enough pins
    for(int i=0; i<2; i++){
      if(_hasAvailablePins(i, no_pins)) {
        *group=i;
        *timer = _findNextTimer(i);
        return 1;
      }
    }
    // third best option is half pwms per group
    int half_no_pins = (int)no_pins/2;
    if(_hasAvailablePins(0,half_no_pins) && _hasAvailablePins(1 ,half_no_pins)){
      return 2;
    }
  }

  // otherwise fail
  return 0;
}


// configuring center aligned pwm
// More info here: https://docs.espressif.com/projects/esp-idf/en/v5.1.4/esp32/api-reference/peripherals/mcpwm.html#symmetric-dual-edge-active-low
int _configureCenterAlign(mcpwm_gen_handle_t gena, mcpwm_cmpr_handle_t cmpa, bool inverted = false){
    if(inverted)
      return mcpwm_generator_set_actions_on_compare_event(gena,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpa, MCPWM_GEN_ACTION_HIGH),
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, cmpa, MCPWM_GEN_ACTION_LOW),
                      MCPWM_GEN_COMPARE_EVENT_ACTION_END());
   else
      return mcpwm_generator_set_actions_on_compare_event(gena,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpa, MCPWM_GEN_ACTION_LOW),
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, cmpa, MCPWM_GEN_ACTION_HIGH),
                      MCPWM_GEN_COMPARE_EVENT_ACTION_END());
}



// Helper function calculating the pwm period from the pwm frequency
// - pwm_frequency - pwm frequency in hertz
// returns pwm period in ticks (uint32_t)
uint32_t _calcPWMPeriod(long pwm_frequency) {
    return  (uint32_t)(1 * _PWM_TIMEBASE_RESOLUTION_HZ / pwm_frequency);
}
/*
    Helper function calculating the pwm frequency from the pwm period
    - pwm_period - pwm period in ticks
    returns pwm frequency in hertz (long)
*/
long _calcPWMFreq(long pwm_period) {
    return  (uint32_t)(1 * _PWM_TIMEBASE_RESOLUTION_HZ / pwm_period / 2);
}

void* _configure6PWMPinsMCPWM(long pwm_frequency, int mcpwm_group, int timer_no, float dead_zone, int* pins){
  ESP32MCPWMDriverParams*  params = new ESP32MCPWMDriverParams{
    .pwm_frequency = pwm_frequency,
    .group_id = mcpwm_group
  };
  
  mcpwm_timer_config_t pwm_config;
  pwm_config.group_id = mcpwm_group;
  pwm_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  pwm_config.resolution_hz = _PWM_TIMEBASE_RESOLUTION_HZ;
  pwm_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;    
  pwm_config.intr_priority = 0;              
  pwm_config.period_ticks = _calcPWMPeriod(pwm_frequency);

  CHECK_ERR(mcpwm_new_timer(&pwm_config, &timers[mcpwm_group][timer_no]), "Could not initialize the timer in group: " + String(mcpwm_group));
  pwm_periods[mcpwm_group][timer_no] = pwm_config.period_ticks / 2;
  params->timers[0] = timers[mcpwm_group][timer_no];
  params->mcpwm_period = pwm_periods[mcpwm_group][timer_no];

  uint8_t no_operators = 3; // use 3 comparators one per pair of pwms
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring " + String(no_operators) + " operators."); 
  mcpwm_operator_config_t operator_config = { .group_id = mcpwm_group };
  operator_config.intr_priority = 0;
  operator_config.flags.update_gen_action_on_tep = true;
  operator_config.flags.update_gen_action_on_tez = true;
  for (int i = 0; i < no_operators; i++) {
    CHECK_ERR(mcpwm_new_operator(&operator_config, &params->oper[i]),"Could not create operator "+String(i));
    CHECK_ERR(mcpwm_operator_connect_timer(params->oper[i], params->timers[0]),"Could not connect timer to operator: " + String(i));
  }

#if SIMPLEFOC_ESP32_HW_DEADTIME == true // hardware dead-time (hardware 6pwm)
  
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 6PWM with hardware dead-time");
  
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring " + String(no_operators) + " comparators.");
  // Create and configure comparators
  mcpwm_comparator_config_t comparator_config = {0};
  comparator_config.flags.update_cmp_on_tez = true;
  for (int i = 0; i < no_operators; i++) {
    CHECK_ERR(mcpwm_new_comparator(params->oper[i], &comparator_config, &params->comparator[i]),"Could not create comparator: " + String(i));
    CHECK_ERR(mcpwm_comparator_set_compare_value(params->comparator[i], (0)), "Could not set duty on comparator: " + String(i));
  }
  
#else // software dead-time (software 6pwm)
// software dead-time (software 6pwm)
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring 6PWM with software dead-time");

  int no_pins = 6;
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring " + String(no_pins) + " comparators.");
  // Create and configure comparators
  mcpwm_comparator_config_t comparator_config = {0};
  comparator_config.flags.update_cmp_on_tez = true;
  for (int i = 0; i < no_pins; i++) {
    int oper_index = (int)floor(i / 2);
    CHECK_ERR(mcpwm_new_comparator(params->oper[oper_index], &comparator_config, &params->comparator[i]),"Could not create comparator: " + String(i));
    CHECK_ERR(mcpwm_comparator_set_compare_value(params->comparator[i], (0)), "Could not set duty on comparator: " + String(i));
  }
#endif 

  int no_generators = 6; // one per pwm
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring " + String(no_generators) + " generators.");
  // Create and configure generators
  mcpwm_generator_config_t generator_config = {};
  for (int i = 0; i < no_generators; i++) {
    generator_config.gen_gpio_num = pins[i];
    int oper_index = (int)floor(i / 2);
    CHECK_ERR(mcpwm_new_generator(params->oper[oper_index], &generator_config, &params->generator[i]),"Could not create generator " + String(i) +String(" on pin: ")+String(pins[i]));
  }

  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring Center-Aligned 6 pwm.");

#if SIMPLEFOC_ESP32_HW_DEADTIME == true // hardware dead-time (hardware 6pwm)
  for (int i = 0; i < no_operators; i++) {
    CHECK_ERR(_configureCenterAlign(params->generator[2*i],params->comparator[i]), "Failed to configure high-side center align pwm: " + String(2*i));  
    CHECK_ERR(_configureCenterAlign(params->generator[2*i+1],params->comparator[i]), "Failed to configure low-side center align pwm: " + String(2*i+1));  
  
  }
  // only available for 6pwm
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring dead-time.");
  uint32_t dead_time = (int)pwm_periods[mcpwm_group][timer_no] * dead_zone;
  mcpwm_dead_time_config_t dt_config_high;
  dt_config_high.posedge_delay_ticks = dead_time;
  dt_config_high.negedge_delay_ticks = 0;
  dt_config_high.flags.invert_output = !SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH;
  mcpwm_dead_time_config_t dt_config_low;
  dt_config_low.posedge_delay_ticks = 0;
  dt_config_low.negedge_delay_ticks = dead_time;
  dt_config_low.flags.invert_output = SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH; 
  for (int i = 0; i < no_operators; i++) {
      CHECK_ERR(mcpwm_generator_set_dead_time(params->generator[2*i], params->generator[2*i], &dt_config_high),"Could not set dead time for generator: " + String(i));
      CHECK_ERR(mcpwm_generator_set_dead_time(params->generator[2*i+1], params->generator[2*i+1], &dt_config_low),"Could not set dead time for generator: " + String(i+1));
  }
#else // software dead-time (software 6pwm)
  for (int i = 0; i < 3; i++) {
    CHECK_ERR(_configureCenterAlign(params->generator[2*i],params->comparator[2*i], !SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH), "Failed to configure high-side center align pwm: " + String(2*i));  
    CHECK_ERR(_configureCenterAlign(params->generator[2*i+1],params->comparator[2*i+1], SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH) , "Failed to configure low-side center align pwm: " + String(2*i+1)); 
  }
#endif

  SIMPLEFOC_ESP32_DRV_DEBUG("Enabling timer: "+String(timer_no));
  // Enable and start timer
  CHECK_ERR(mcpwm_timer_enable(params->timers[0]), "Failed to enable timer!");
  CHECK_ERR(mcpwm_timer_start_stop(params->timers[0], MCPWM_TIMER_START_NO_STOP), "Failed to start the timer!");

  _delay(1);
  SIMPLEFOC_ESP32_DRV_DEBUG("MCPWM configured!");
  params->dead_zone = dead_zone;
  // save the configuration variables for later
  group_pins_used[mcpwm_group] = 6;
  return params;
}


/*
  function configuring the pins for the mcpwm
  - pwm_frequency - pwm frequency
  - mcpwm_group - mcpwm group
  - timer_no - timer number
  - no_pins - number of pins
  - pins - array of pins
  - dead_zone - dead zone

  returns the driver parameters
*/
void* _configurePinsMCPWM(long pwm_frequency, int mcpwm_group, int timer_no, int no_pins, int* pins){

  ESP32MCPWMDriverParams*  params = new ESP32MCPWMDriverParams{
    .pwm_frequency = pwm_frequency,
    .group_id = mcpwm_group
  };
  
  bool shared_timer = false;
  // check if timer is configured
  if (timers[mcpwm_group][timer_no] == NULL){
    mcpwm_timer_config_t pwm_config;
    pwm_config.group_id = mcpwm_group;
    pwm_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    pwm_config.resolution_hz = _PWM_TIMEBASE_RESOLUTION_HZ;
    pwm_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;    
    pwm_config.intr_priority = 0;              
    pwm_config.period_ticks = _calcPWMPeriod(pwm_frequency);
    // initialise the timer
    CHECK_ERR(mcpwm_new_timer(&pwm_config, &timers[mcpwm_group][timer_no]), "Could not initialize the timer in group: " + String(mcpwm_group));
    // save variables for later
    pwm_periods[mcpwm_group][timer_no] = pwm_config.period_ticks / 2;
    params->timers[0] = timers[mcpwm_group][timer_no];    
    // if the numer of used channels it not pair skip one channel
    // the skipped channel cannot be used with the new timer
    // TODO avoid loosing channels like this 
    if(group_pins_used[mcpwm_group] %2) group_pins_used[mcpwm_group]++;
  }else{
    // we will use an already instantiated timer
    params->timers[0] = timers[mcpwm_group][timer_no];
    SIMPLEFOC_ESP32_DRV_DEBUG("Using previously configured timer: " + String(timer_no));
    // but we cannot change its configuration without affecting the other drivers
    // so let's first verify that the configuration is the same
    if(_calcPWMPeriod(pwm_frequency)/2 != pwm_periods[mcpwm_group][timer_no]){
      SIMPLEFOC_ESP32_DRV_DEBUG("ERR: Timer: "+String(timer_no)+" is confgured for freq: "+String(_calcPWMFreq(pwm_periods[mcpwm_group][timer_no]))+", not for freq:" +String(pwm_frequency));
      return SIMPLEFOC_DRIVER_INIT_FAILED;
    }
    CHECK_ERR(mcpwm_timer_start_stop( params->timers[0], MCPWM_TIMER_STOP_EMPTY), "Failed to stop the timer!");
    
    shared_timer = true;
  }

  uint8_t no_operators = ceil(no_pins / 2.0);
  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring " + String(no_operators) + " operators.");
  mcpwm_operator_config_t operator_config = { .group_id = mcpwm_group };
  operator_config.intr_priority = 0;
  operator_config.flags.update_gen_action_on_tep = true;
  operator_config.flags.update_gen_action_on_tez = true;
  for (int i = 0; i < no_operators; i++) {
    if (shared_timer && i == 0) { // first operator already configured
        params->oper[0] = last_operator[mcpwm_group];
        continue;
    }
    CHECK_ERR(mcpwm_new_operator(&operator_config, &params->oper[i]),"Could not create operator "+String(i));
    CHECK_ERR(mcpwm_operator_connect_timer(params->oper[i], params->timers[0]),"Could not connect timer to operator: " + String(i));
  } 
  // save the last operator in this group
  last_operator[mcpwm_group] = params->oper[no_operators - 1];

  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring " + String(no_pins) + " comparators.");
  // Create and configure comparators
  mcpwm_comparator_config_t comparator_config = {0};
  comparator_config.flags.update_cmp_on_tez = true;
  for (int i = 0; i < no_pins; i++) {
    int oper_index = shared_timer ? (int)floor((i + 1) / 2) : (int)floor(i / 2);
    CHECK_ERR(mcpwm_new_comparator(params->oper[oper_index], &comparator_config, &params->comparator[i]),"Could not create comparator: " + String(i));
    CHECK_ERR(mcpwm_comparator_set_compare_value(params->comparator[i], (0)), "Could not set duty on comparator: " + String(i));
  }

  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring " + String(no_pins) + " generators.");
  // Create and configure generators;
  mcpwm_generator_config_t generator_config = {};
  for (int i = 0; i < no_pins; i++) {
    generator_config.gen_gpio_num = pins[i];
    int oper_index = shared_timer ? (int)floor((i + 1) / 2) : (int)floor(i / 2);
    CHECK_ERR(mcpwm_new_generator(params->oper[oper_index], &generator_config, &params->generator[i]), "Could not create generator " + String(i) +String(" on pin: ")+String(pins[i]));
  }
  

  SIMPLEFOC_ESP32_DRV_DEBUG("Configuring center-aligned pwm.");
  for (int i = 0; i < no_pins; i++) {
    CHECK_ERR(_configureCenterAlign(params->generator[i],params->comparator[i], !SIMPLEFOC_PWM_ACTIVE_HIGH), "Failed to configure center align pwm: " + String(i));
  }

  SIMPLEFOC_ESP32_DRV_DEBUG("Enabling timer: "+String(timer_no));
  // Enable and start timer if not shared
  if (!shared_timer) CHECK_ERR(mcpwm_timer_enable(params->timers[0]), "Failed to enable timer!");
  // start the timer
  CHECK_ERR(mcpwm_timer_start_stop(params->timers[0], MCPWM_TIMER_START_NO_STOP), "Failed to start the timer!");

  _delay(1);
  SIMPLEFOC_ESP32_DRV_DEBUG("MCPWM configured!");
  // save the configuration variables for later
  params->mcpwm_period = pwm_periods[mcpwm_group][timer_no];
  group_pins_used[mcpwm_group] += no_pins;
  return params;
}

// function setting the duty cycle to the MCPWM pin
void _setDutyCycle(mcpwm_cmpr_handle_t cmpr, uint32_t mcpwm_period, float duty_cycle){
  float duty = _constrain(duty_cycle, 0.0, 1.0);
  mcpwm_comparator_set_compare_value(cmpr, (uint32_t)(mcpwm_period*duty));
}

// function setting the duty cycle to the MCPWM pin
void _forcePhaseState(mcpwm_gen_handle_t generator_high, mcpwm_gen_handle_t generator_low, PhaseState phase_state){
  // phase state is forced in hardware pwm mode
  // esp-idf docs:  https://docs.espressif.com/projects/esp-idf/en/v5.1.4/esp32/api-reference/peripherals/mcpwm.html#generator-force-actions 
  // github issue: https://github.com/espressif/esp-idf/issues/12237
  mcpwm_generator_set_force_level(generator_high, (phase_state == PHASE_ON || phase_state == PHASE_HI) ? -1 : 0, true);
  mcpwm_generator_set_force_level(generator_low, (phase_state == PHASE_ON || phase_state == PHASE_LO) ? -1 : 1, true);
}

#endif