#include "StepDirListener.h"

StepDirListener::StepDirListener(int _pinStep, int _pinDir, float _step_per_rotation){

    pin_step = _pinStep;
    pin_dir = _pinDir;

    step_per_rotation = _step_per_rotation;

}

void StepDirListener::init(){
    pinMode(pin_dir, INPUT);
    pinMode(pin_step, INPUT_PULLUP);
    count = 0;
}

void StepDirListener::enableInterrupt(void (*doA)()){
    attachInterrupt(digitalPinToInterrupt(pin_step), doA, CHANGE);
}

void StepDirListener::attach(float* variable){
    attached_variable = variable;
}

void StepDirListener::handle(){
  bool step = digitalRead(pin_step);
  if(step && step != step_active){
     if(digitalRead(pin_dir)) 
        count++;
     else 
        count--;
   }
   step_active = step;
   if(attached_variable) *attached_variable = getValue();
}

float StepDirListener::getValue(){
    return (float) count / step_per_rotation * _2PI;
}