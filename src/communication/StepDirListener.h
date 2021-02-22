#ifndef STEPDIR_H
#define STEPDIR_H

#include "Arduino.h"
#include "../common/foc_utils.h"

class StepDirListener
{
  public:
    StepDirListener(int pinStep, int pinDir, float step_per_rotation);
    void enableInterrupt(void (*doA)());
    void init();
    void handle();

    float getValue();

    void attach(float* variable);

    int pin_step;
    int pin_dir;
    long count;

  private:
    float* attached_variable = nullptr;
    long step_per_rotation;
    bool step_active = 0;
};

#endif