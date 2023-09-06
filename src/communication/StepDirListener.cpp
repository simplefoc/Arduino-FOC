#include "StepDirListener.h"
#include "common/time_utils.h"

StepDirListener::StepDirListener(int _pinStep, int _pinDir, float _counter_to_value){
    pin_step = _pinStep;
    pin_dir = _pinDir;
    counter_to_value = _counter_to_value;
}

void StepDirListener::init(){
    pinMode(pin_dir, INPUT);
    pinMode(pin_step, INPUT_PULLUP);
    count = 0;
    prev_pulse_time = 0;
    current_pulse_time = 0;
    elapsed_time = 0;
}

void StepDirListener::enableInterrupt(void (*doA)()){
    attachInterrupt(digitalPinToInterrupt(pin_step), doA, polarity);
}

void StepDirListener::attach(float* pos_var){
    attached_position = pos_var;
}
void StepDirListener::attach(float* pos_var, float* vel_var){
    attached_position = pos_var;
    attached_velocity = vel_var;
}


void StepDirListener::handle(){ 
  // read step status
  //bool step = digitalRead(pin_step);
  // update counter only on rising edge 
  //if(step && step != step_active){
    dir_state = digitalRead(pin_dir);
    if(dir_state) 
        count++;
    else 
        count--;
   //}
   current_pulse_time = _micros();
   //step_active = step;
   // if attached variable update it
   if(attached_position) {
        *attached_position = getValue();
        if(attached_velocity) {
            // can use STM32 input capture peripheral to do this too
            if(dir_state) {
                *attached_velocity = getVelocityValue();
            } else {
                *attached_velocity = -getVelocityValue();
            }
        }
    }
   prev_pulse_time = current_pulse_time;

}
// calculate the position from counter
float StepDirListener::getValue(){
    return (float) count * counter_to_value;
}
float StepDirListener::getVelocityValue(){
    elapsed_time = current_pulse_time - prev_pulse_time;
    if (elapsed_time == 0) {
        // caps feedfoarward velocity based on time precision
        // if zero microseconds have elapsed, assume 1us has elapsed
        elapsed_time = 1;
    }
    // no need to subtract current position from previous position 
    // that's always 1 unless we missed an IRQ
    return (float) counter_to_value * 1e6 / elapsed_time; 
}

void StepDirListener::cleanLowVelocity(){
    int current_time = _micros();
    if ((current_time - prev_pulse_time) > 5 * elapsed_time) {
        *attached_velocity = 0;
    }
}
