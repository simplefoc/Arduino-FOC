#include "BLDCMotor.h"

// Encoder counter variable
long counter;

/*
BLDCMotor(int phA,int phB,int phC, long* counter, int encA, int encB , int pp, int cpr)
- phA, phB, phC - motor A,B,C phase pwm pins
- *counter      - encoder counter variable
- encA, encB    - encoder A and B pins
- pp            - pole pair number
- cpr           - counts per rotation number (cpm=ppm*4)
- enable pin    - (optional input)
*/
BLDCMotor motor = BLDCMotor(9,10,11,&counter,2,3,11,8196,8);

double angle_sp = PI/8;
double voltage_sp = 2;
double velocity_sp = 1;
int t = 0;

void setup() {
  Serial.begin(115200);
    
  // PWM pins
  pinMode(motor._phA, OUTPUT); 
  pinMode(motor._phB, OUTPUT); 
  pinMode(motor._phC, OUTPUT); 

  // Increase PWM frequency to 32 kHz  (make unaudible)
  setPwmFrequency(motor._phA); 
  setPwmFrequency(motor._phB);
  setPwmFrequency(motor._phC);
  
  // Encoder PULLUP
  pinMode(motor._encA, INPUT_PULLUP); 
  pinMode(motor._encB, INPUT_PULLUP); 
  // pina
  attachInterrupt(digitalPinToInterrupt(motor._encA), doEncoderA, CHANGE);
  // pinb
  attachInterrupt(digitalPinToInterrupt(motor._encB), doEncoderB, CHANGE);
  
  // Initialize motor
  motor.init();
  
  Serial.println("Ready.");
  delay(1000);
}

void loop() { 
  t++;
  if (!(t%100)){
    t = 0;
    angle_sp = -angle_sp;
    voltage_sp = -voltage_sp;
    velocity_sp = -velocity_sp;
  }
  
  // Uncomment line to see different loop in action 
  // Set phase voltages using FOC
  //motor.setVoltage(voltage_sp);
  // Set velocity of the motor
  //motor.setVelocity(velocity_sp);
  // Set referent position P+PI
  motor.setPosition(angle_sp);
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


/**
  Encoder methods
*/

int A1_=0;
int B1_=0;
/*
  A channel
*/
void doEncoderA(){
  int A = digitalRead(motor._encA);
  if( A!= A1_ ){
    if(A1_ == B1_){
      counter += 1;
    }else{
      counter -= 1;
    }
   A1_ = A;
  }
}

/*
  B channel
*/
void doEncoderB(){
  int B = digitalRead(motor._encB);
  if( B!= B1_ ){
    if( A1_ != B1_ ){
      counter += 1;
    }else{
      counter -= 1;
    }
    B1_ = B;
  }
}
