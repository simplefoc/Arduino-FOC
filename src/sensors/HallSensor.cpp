#include "HallSensor.h"
#include "./communication/SimpleFOCDebug.h"

// seq 1 > 5 > 4 > 6 > 2 > 3 > 1        000 001 010 011 100 101 110 111
const int8_t ELECTRIC_SECTORS_120[8] = { -1,  0,  4,  5,  2,  1,  3 , -1 };

// seq 1 > 5 > 4 > 6 > 2 > 3 > 1      000 001 010 011 100 101 110 111
const int8_t ELECTRIC_SECTORS_60[8] = { 0,  5,  1,  2,  5,  4,  2 , 3 };

/*
  HallSensor(int hallA, int hallB , int cpr, int index)
  - hallA, hallB, hallC    - HallSensor A, B and C pins
  - pp           - pole pairs
*/
HallSensor::HallSensor(int _hallA, int _hallB, int _hallC, int _pp, HallType _hall_type){

  // hardware pins
  pinA = _hallA;
  pinB = _hallB;
  pinC = _hallC;
  hall_type = _hall_type;
  last_print_type = hall_type;
  for (size_t i = 0; i < sizeof(previous_states); i++)
  {
    previous_states[i] = -1;
  }
  

  // hall has 6 segments per electrical revolution
  cpr = _pp * 6;

  // extern pullup as default
  pullup = Pullup::USE_EXTERN;
}

//  HallSensor interrupt callback functions
// A channel
void HallSensor::handleA() {
  A_active= digitalRead(pinA);
  updateState();
}
// B channel
void HallSensor::handleB() {
  B_active = digitalRead(pinB);
  updateState();
}

// C channel
void HallSensor::handleC() {
  C_active = digitalRead(pinC);
  updateState();
}

/**
 * Updates the state and sector following an interrupt
 */
void HallSensor::updateState() {
  int8_t new_hall_state = C_active + (B_active << 1) + (A_active << 2);
  // glitch avoidance #1 - sometimes we get an interrupt but pins haven't changed
  if (new_hall_state == hall_state_raw) return;
  hall_state_raw = new_hall_state;

  static const int num_previous_states = sizeof(previous_states);

  //flip a line maybe
  if (hall_type != HallType::UNKNOWN)
  {
    new_hall_state ^= static_cast<int8_t>(hall_type);
  }

  long new_pulse_timestamp = _micros();
  if (hall_type == HallType::UNKNOWN) //Store previous steps for hall config detection
  {
    for (int i = num_previous_states - 2; i >= 0; i--)
    {
      previous_states[i+1] = previous_states[i];
    }
    previous_states[0] = new_hall_state;
    //7 and 0 are illegal in 120deg mode, so we're gonna try to see which line hel up during that time and flip it so it doesn't happen
    if ((previous_states[1] == 0b111 ||  previous_states[1] == 0b000) && previous_states[2] != -1)
    {
      if (previous_states[2] == previous_states[0])
      {
        //went back, can't do anything
      }
      else
      {
        hall_type = static_cast<HallType>((0b111 - previous_states[0] ^ previous_states[2])%8);
        previous_states[0] ^= static_cast<int8_t>(hall_type);
      }
    }
    if (abs(electric_rotations) > 2)
    {
      hall_type = HallType::HALL_120;
    }
  }
  
  
  

  int8_t new_electric_sector;
  new_electric_sector = ELECTRIC_SECTORS_120[new_hall_state];
  int8_t electric_sector_dif = new_electric_sector - electric_sector;
  if (electric_sector_dif > 3) {
    //underflow
    direction = Direction::CCW;
    electric_rotations += direction;
  } else if (electric_sector_dif < (-3)) {
    //overflow
    direction = Direction::CW;
    electric_rotations += direction;
  } else {
    direction = (new_electric_sector > electric_sector)? Direction::CW : Direction::CCW;
  }
  electric_sector = new_electric_sector;

  // glitch avoidance #2 changes in direction can cause velocity spikes.  Possible improvements needed in this area
  if (direction == old_direction) {
    // not oscilating or just changed direction
    pulse_diff = new_pulse_timestamp - pulse_timestamp;
  } else {
    pulse_diff = 0;
  }

  pulse_timestamp = new_pulse_timestamp;
  total_interrupts++;
  old_direction = direction;
  if (onSectorChange != nullptr) onSectorChange(electric_sector);
}

/**
 * Optionally set a function callback to be fired when sector changes
 * void onSectorChange(int sector) {
 *  ... // for debug or call driver directly?
 * }
 * sensor.attachSectorCallback(onSectorChange);
 */
void HallSensor::attachSectorCallback(void (*_onSectorChange)(int sector)) {
  onSectorChange = _onSectorChange;
}



// Sensor update function. Safely copy volatile interrupt variables into Sensor base class state variables.
void HallSensor::update() {
  // Copy volatile variables in minimal-duration blocking section to ensure no interrupts are missed
  if (use_interrupt){
    noInterrupts();
  }else{
    A_active = digitalRead(pinA);
    B_active = digitalRead(pinB);
    C_active = digitalRead(pinC);
    updateState();
  }

  angle_prev_ts = pulse_timestamp;
  long last_electric_rotations = electric_rotations;
  int8_t last_electric_sector = electric_sector;
  if (use_interrupt) interrupts();
  angle_prev = ((float)((last_electric_rotations * 6 + last_electric_sector) % cpr) / (float)cpr) * _2PI ;
  full_rotations = (int32_t)((last_electric_rotations * 6 + last_electric_sector) / cpr);
  if (last_print_type != hall_type)
  {
    last_print_type = hall_type;
    switch (hall_type)
    {
    case HallType::HALL_120 :
      SIMPLEFOC_DEBUG("HALL: Found type: HALL_120");
      break;
    case HallType::HALL_60A :
    SIMPLEFOC_DEBUG("HALL: Found type: HALL_60A");
      break;
    case HallType::HALL_60B :
      SIMPLEFOC_DEBUG("HALL: Found type: HALL_60B");
        break;
    case HallType::HALL_60C :
      SIMPLEFOC_DEBUG("HALL: Found type: HALL_60C");
      break;
    
    default:
      SIMPLEFOC_DEBUG("HALL: Type unknown! Wtf!");
      break;
    }
    
  }
  
}



/*
	Shaft angle calculation
  TODO: numerical precision issue here if the electrical rotation overflows the angle will be lost
*/
float HallSensor::getSensorAngle() {
  return ((float)(electric_rotations * 6 + electric_sector) / (float)cpr) * _2PI ;
}

/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float HallSensor::getVelocity(){
  noInterrupts();
  long last_pulse_timestamp = pulse_timestamp;
  long last_pulse_diff = pulse_diff;
  interrupts();
  if (last_pulse_diff == 0 || ((long)(_micros() - last_pulse_timestamp) > last_pulse_diff*2) ) { // last velocity isn't accurate if too old
    return 0;
  } else {
    return direction * (_2PI / (float)cpr) / (last_pulse_diff / 1000000.0f);
  }

}

int HallSensor::needsSearch()
{
  return hall_type == HallType::UNKNOWN;
}

// HallSensor initialisation of the hardware pins 
// and calculation variables
void HallSensor::init(){
  // initialise the electrical rotations to 0
  electric_rotations = 0;

  // HallSensor - check if pullup needed for your HallSensor
  if(pullup == Pullup::USE_INTERN){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinC, INPUT_PULLUP);
  }else{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pinMode(pinC, INPUT);
  }

    // init hall_state
  A_active = digitalRead(pinA);
  B_active = digitalRead(pinB);
  C_active = digitalRead(pinC);
  updateState();

  pulse_timestamp = _micros();

  // we don't call Sensor::init() here because init is handled in HallSensor class.
}

// function enabling hardware interrupts for the callback provided
// if callback is not provided then the interrupt is not enabled
void HallSensor::enableInterrupts(void (*doA)(), void(*doB)(), void(*doC)()){
  // attach interrupt if functions provided

  // A, B and C callback
  if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
  if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
  if(doC != nullptr) attachInterrupt(digitalPinToInterrupt(pinC), doC, CHANGE);

  use_interrupt = true;
}
