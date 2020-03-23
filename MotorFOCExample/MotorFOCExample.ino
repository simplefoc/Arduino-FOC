#include "BLDCMotor.h"


// Encoder variables
#define CPR 600  // counts per revolution
#define PPR (4*CPR) // pulses per revolution

// Only pins 2 and 3 are supported
#define encoderPinA 2             // Arduino UNO interrupt 0
#define encoderPinB 3             // Arduino UNO interrupt 1


/*
BLDCMotorEncoder( int phA, int phB, int phC, int pp, int encA, int encB , int cpr, int en)
- phA, phB, phC - motor A,B,C phase pwm pins
- pp            - pole pair number
- encA, encB    - encoder A and B pins
- cpr           - counts per rotation number (cpm=ppm*4)
- enable pin    - (optional input)
*/
BLDCMotorEncoder motor = BLDCMotorEncoder(9, 10, 11, 11, encoderPinA,encoderPinB, PPR, 8);

void setup() {
  // debugging port
  Serial.begin(115200);
    
  // PWM pins
  pinMode(motor.pwmA, OUTPUT); 
  pinMode(motor.pwmB, OUTPUT); 
  pinMode(motor.pwmC, OUTPUT); 

  // Increase PWM frequency to 32 kHz  
  // make silent
  setPwmFrequency(motor.pwmA); 
  setPwmFrequency(motor.pwmB);
  setPwmFrequency(motor.pwmC);
  
  // Encoder - check if pullup needed for your encoder
  pinMode(motor.encoder.pinA, INPUT_PULLUP); 
  pinMode(motor.encoder.pinB, INPUT_PULLUP); 
  // interrupt callback init
  attachInterrupt(digitalPinToInterrupt(motor.encoder.pinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor.encoder.pinB), doEncoderB, CHANGE);
  
  // Initialize motor
  motor.init();
  
  Serial.println("Ready.");
  delay(1000);
}

void loop() { 
  double velocity_sp = 1;
  // iterative state calculation velocity and angle
  motor.updateStates();
  // Set phase voltages using  FOC
  motor.setVelocity(velocity_sp);
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


//  Encoder interrupt callback functions
void doEncoderA() {
  motor.handleEncoderA();
}
// B channel
void doEncoderB() {
  motor.handleEncoderB();
}

