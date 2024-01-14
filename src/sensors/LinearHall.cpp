#include "LinearHall.h"

// This function can be overridden with custom ADC code on platforms with poor analogRead performance.
__attribute__((weak)) void ReadLinearHalls(int hallA, int hallB, int *a, int *b)
{
  *a = analogRead(hallA);
  *b = analogRead(hallB);
}

LinearHall::LinearHall(int _hallA, int _hallB, int _pp){
  centerA = 512;
  centerB = 512;
  pinA = _hallA;
  pinB = _hallB;
  pp = _pp;
}

float LinearHall::getSensorAngle() {
  ReadLinearHalls(pinA, pinB, &lastA, &lastB);
  //offset readings using center values, then compute angle
  float reading = _atan2(lastA - centerA, lastB - centerB);

  //handle rollover logic between each electrical revolution of the motor
  if (reading > prev_reading) {
    if (reading - prev_reading >= PI) {
      if (electrical_rev - 1 < 0) {
        electrical_rev = pp - 1;
      } else {
        electrical_rev = electrical_rev - 1;
      }
    }
  } else if (reading < prev_reading) {
    if (prev_reading - reading >= PI) {
      if (electrical_rev + 1 >= pp) {
        electrical_rev = 0;
      } else {
        electrical_rev = electrical_rev + 1;
      }
    }
  }

  //convert result from electrical angle and electrical revolution count to shaft angle in radians
  float result = (reading + PI) / _2PI;
  result = _2PI * (result + electrical_rev) / pp;

  //update previous reading for rollover handling
  prev_reading = reading;
  return result;
}

void LinearHall::init(int _centerA, int _centerB) {
  // Skip configuring the pins here because they normally default to input anyway, and
  // this makes it possible to use ADC channel numbers instead of pin numbers when using
  // custom implementation of ReadLinearHalls, to avoid having to remap them every update.
  // If pins do need to be configured, it can be done by user code before calling init.
  //pinMode(pinA, INPUT);
  //pinMode(pinB, INPUT);

  centerA = _centerA;
  centerB = _centerB;

  //establish initial reading for rollover handling
  electrical_rev = 0;
  ReadLinearHalls(pinA, pinB, &lastA, &lastB);
  prev_reading = _atan2(lastA - centerA, lastB - centerB);
}

void LinearHall::init(FOCMotor *motor) {
  if (!motor->enabled) {
    SIMPLEFOC_DEBUG("LinearHall::init failed. Call after motor.init, but before motor.initFOC.");
    return;
  }

  // See comment in other version of init for why these are commented out
  //pinMode(pinA, INPUT);
  //pinMode(pinB, INPUT);

  int minA, maxA, minB, maxB;

  ReadLinearHalls(pinA, pinB, &lastA, &lastB);
  minA = maxA = centerA = lastA;
  minB = maxB = centerB = lastB;

  // move one mechanical revolution forward
  for (int i = 0; i <= 2000; i++)
  {
    float angle = _3PI_2 + _2PI * i * pp / 2000.0f;
    motor->setPhaseVoltage(motor->voltage_sensor_align, 0, angle);

    ReadLinearHalls(pinA, pinB, &lastA, &lastB);

    if (lastA < minA)
      minA = lastA;
    if (lastA > maxA)
      maxA = lastA;
    centerA = (minA + maxA) / 2;

    if (lastB < minB)
      minB = lastB;
    if (lastB > maxB)
      maxB = lastB;
    centerB = (minB + maxB) / 2;

    _delay(2);
  }

  //establish initial reading for rollover handling
  electrical_rev = 0;
  prev_reading = _atan2(lastA - centerA, lastB - centerB);
}
