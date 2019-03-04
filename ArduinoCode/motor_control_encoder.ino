// -- Pin change interrupt
#include "encoder.h"
#include "BLDCMotor.h"
#include <PciManager.h>
#include <PciListenerImp.h>


BLDCMotor motor = BLDCMotor(9,10,11,&counter[ENCODER_1],A0,A1,11,8196);
//BLDCMotorint(phA,int phB,int phC, long* counter, int encA, int encB , int pp, int cpr)

double angle_sp = PI/8;
int t = 0;

// encoder interrupt init
PciListenerImp listenerA(motor._encA, doEncoder1A);
PciListenerImp listenerB(motor._encB, doEncoder1B);

void setup() {
  Serial.begin(115200);
    
  // PWM pins
  pinMode(motor._phA, OUTPUT); 
  pinMode(motor._phB, OUTPUT); 
  pinMode(motor._phC, OUTPUT); 

  setPwmFrequency(motor._phA); // Increase PWM frequency to 32 kHz  (make unaudible)
  setPwmFrequency(motor._phB);
  setPwmFrequency(motor._phC);
  
  // Encoder PULLUP
  pinMode(motor._encA, INPUT_PULLUP); // new method of enabling pullups
  pinMode(motor._encB, INPUT_PULLUP); 
  
  // Initialize motor
  motor.init();
  
  PciManager.registerListener(&listenerA);
  PciManager.registerListener(&listenerB);

  Serial.println("Ready.");
  delay(1000);
  
}

void loop() { 
  t++;
  if (!(t%100)){
    t = 0;
    angle_sp = -angle_sp;
  }
  
  motor.setPosition(angle_sp);
    //Serial.println(*motor._encoderPosition);
}


/*
  High PWM frequency
*/
void setPwmFrequency(int pin) {
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | 0x01;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | 0x01;
    }
  }
  else if(pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
  }
}
