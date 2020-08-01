#include "FOCutils.h"


void _setPwmFrequency(int pin) {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // if arduino uno and other atmega328p chips
//  High PWM frequency
//  https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if (pin == 5 || pin == 6) {
      // configure the pwm phase-corrected mode
      TCCR0A = ((TCCR0A & 0b11111100) | 0x01);
      // set prescaler to 1
      TCCR0B = ((TCCR0B & 0b11110000) | 0x01);
    } else {
      // set prescaler to 1
      TCCR1B = ((TCCR1B & 0b11111000) | 0x01);
    }
  }
  else if (pin == 3 || pin == 11) {
      // set prescaler to 1
    TCCR2B = ((TCCR2B & 0b11111000) | 0x01);
  }
#elif defined(_STM32_DEF_) // if stm chips
    analogWrite(pin,0);
    analogWriteFrequency(50000);  // la valeur par défaut est 20000 Hz
#endif
}

// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  // if arduino uno and other atmega328p chips
  // return the value based on the prescaler
  long t = _micros();
  while((_micros() - t)/1000 < ms){}; 
#else
  // regular micros
  return delay(ms);
#endif
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// if arduino uno and other atmega328p chips
    //return the value based on the prescaler
    if((TCCR0B & 0b00000111) == 0x01) return (micros()/32);
    else return (micros());
#else
  // regular micros
  return micros();
#endif
}


// lookup table for sine calculation in between 0 and 90 degrees
//float sine_array[200] = {0.0000,0.0079,0.0158,0.0237,0.0316,0.0395,0.0473,0.0552,0.0631,0.0710,0.0789,0.0867,0.0946,0.1024,0.1103,0.1181,0.1260,0.1338,0.1416,0.1494,0.1572,0.1650,0.1728,0.1806,0.1883,0.1961,0.2038,0.2115,0.2192,0.2269,0.2346,0.2423,0.2499,0.2575,0.2652,0.2728,0.2804,0.2879,0.2955,0.3030,0.3105,0.3180,0.3255,0.3329,0.3404,0.3478,0.3552,0.3625,0.3699,0.3772,0.3845,0.3918,0.3990,0.4063,0.4135,0.4206,0.4278,0.4349,0.4420,0.4491,0.4561,0.4631,0.4701,0.4770,0.4840,0.4909,0.4977,0.5046,0.5113,0.5181,0.5249,0.5316,0.5382,0.5449,0.5515,0.5580,0.5646,0.5711,0.5775,0.5839,0.5903,0.5967,0.6030,0.6093,0.6155,0.6217,0.6279,0.6340,0.6401,0.6461,0.6521,0.6581,0.6640,0.6699,0.6758,0.6815,0.6873,0.6930,0.6987,0.7043,0.7099,0.7154,0.7209,0.7264,0.7318,0.7371,0.7424,0.7477,0.7529,0.7581,0.7632,0.7683,0.7733,0.7783,0.7832,0.7881,0.7930,0.7977,0.8025,0.8072,0.8118,0.8164,0.8209,0.8254,0.8298,0.8342,0.8385,0.8428,0.8470,0.8512,0.8553,0.8594,0.8634,0.8673,0.8712,0.8751,0.8789,0.8826,0.8863,0.8899,0.8935,0.8970,0.9005,0.9039,0.9072,0.9105,0.9138,0.9169,0.9201,0.9231,0.9261,0.9291,0.9320,0.9348,0.9376,0.9403,0.9429,0.9455,0.9481,0.9506,0.9530,0.9554,0.9577,0.9599,0.9621,0.9642,0.9663,0.9683,0.9702,0.9721,0.9739,0.9757,0.9774,0.9790,0.9806,0.9821,0.9836,0.9850,0.9863,0.9876,0.9888,0.9899,0.9910,0.9920,0.9930,0.9939,0.9947,0.9955,0.9962,0.9969,0.9975,0.9980,0.9985,0.9989,0.9992,0.9995,0.9997,0.9999,1.0000,1.0000};

// int array instead of float array 
// 2x storage save (int 2Byte float 4 Byte )
// sin*10000
int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};

// function approximating the sine calculation by using fixed size array
// ~40us (float array)
// ~50us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _sin(float a){
  if(a < _PI_2){
    //return sine_array[(int)(199.0*( a / (_PI/2.0)))];
    //return sine_array[(int)(126.6873* a)];           // float array optimised
    return 0.0001*sine_array[_round(126.6873* a)];      // int array optimised
  }else if(a < _PI){
    // return sine_array[(int)(199.0*(1.0 - (a-_PI/2.0) / (_PI/2.0)))];
    //return sine_array[398 - (int)(126.6873*a)];          // float array optimised
    return 0.0001*sine_array[398 - _round(126.6873*a)];     // int array optimised
  }else if(a < _3PI_2){
    // return -sine_array[(int)(199.0*((a - _PI) / (_PI/2.0)))];
    //return -sine_array[-398 + (int)(126.6873*a)];           // float array optimised
    return -0.0001*sine_array[-398 + _round(126.6873*a)];      // int array optimised
  } else {
    // return -sine_array[(int)(199.0*(1.0 - (a - 3*_PI/2) / (_PI/2.0)))];
    //return -sine_array[796 - (int)(126.6873*a)];           // float array optimised
    return -0.0001*sine_array[796 - _round(126.6873*a)];      // int array optimised
  }
}

// function approfimating cosine calculaiton by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a){
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}
