#include "encoder.h"
#include "Arduino.h"

// countiner
long counter[2]={0};

/**
  Encoder methods
*/

int A1_=0;
int B1_=0;
int A2_=0;
int B2_=0;
/*
  A channel
*/
void doEncoder1A(){
  int A = digitalRead(A0);
  if( A!= A1_ ){
    if(A1_ == B1_){
      counter[ENCODER_1] += 1;
    }else{
      counter[ENCODER_1] -= 1;
    }
   A1_ = A;
  }
}

/*
  B channel
*/
void doEncoder1B(){
  int B = digitalRead(A1);
  if( B!= B1_ ){
    if( A1_ != B1_ ){
      counter[ENCODER_1] += 1;
    }else{
      counter[ENCODER_1] -= 1;
    }
    B1_ = B;
  }
}

/*
  A channel
*/
void doEncoder2A(){
  int A = digitalRead(A2);
  if( A!= A2_ ){
    if(A2_ == B2_){
      counter[ENCODER_2] += 1;
    }else{
      counter[ENCODER_2]  -= 1;
    }
    A2_ = A;
  }
}

/*
  B channel
*/
void doEncoder2B(){
  int B = digitalRead(A3);
  if( B!= B2_ ){
    if( A2_ != B2_ ){
      counter[ENCODER_2]  += 1;
    }else{
      counter[ENCODER_2]  -= 1;
    }
    B2_ = B;
  }
}


