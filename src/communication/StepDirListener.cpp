#include "StepDirListener.h"

StepDirListener::StepDirListener(int _pinStep, int _pinDir, float _counter_to_value){
    pin_step = _pinStep;
    pin_dir = _pinDir;
    counter_to_value = _counter_to_value;
}

void StepDirListener::init(){
    pinMode(pin_dir, INPUT);
    pinMode(pin_step, INPUT_PULLUP);
    count = 0;
}

void StepDirListener::enableInterrupt(void (*doA)()){
    attachInterrupt(digitalPinToInterrupt(pin_step), doA, polarity);
}

void StepDirListener::attach(float* variable){
    attached_variable = variable;
}

void StepDirListener::handle(){ 
  // read step status
  //bool step = digitalRead(pin_step);
  // update counter only on rising edge 
  //if(step && step != step_active){
    if(digitalRead(pin_dir)) 
        count++;
    else 
        count--;
   //}
   //step_active = step;
   // if attached variable update it
   if(attached_variable) *attached_variable = getValue();
}
// calculate the position from counter
float StepDirListener::getValue(){
    return (float) count * counter_to_value;
}