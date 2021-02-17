#include "lowpass_filter.h"

LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
{
    timestamp_prev = _micros();
}


float LowPassFilter::operator() (float x)
{
    unsigned long timestamp = _micros();
    float dt = (timestamp - timestamp_prev)*1e-6f;

    if (dt < 0.0f || dt > 0.5f)
        dt = 1e-3f;

    float alpha = Tf/(Tf + dt);
    float y = alpha*y_prev + (1.0f - alpha)*x;

    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}

String LowPassFilter::communicate(String user_cmd){
  String ret = "";
  char cmd = user_cmd.charAt(0);
  char GET  = user_cmd.charAt(1) == '\n';
  float value = user_cmd.substring(1).toFloat();

  switch (cmd){
    case 'F':      // Tf value change
      ret = ret + "Tf: ";
      if(!GET) Tf = value;
      ret = ret + Tf;
      break;
    default:
      ret = ret + F("error");
      break;
  }
  return ret; 
}