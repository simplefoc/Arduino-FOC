/**
 *  Hall sensor example code 
 * 
 * This is a code intended to test the hall sensors connections and to demonstrate the hall sensor setup.
 * 
 */

#include <SimpleFOC.h>

// Hall sensor instance
// HallSensor(int hallA, int hallB , int cpr, int index)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(2, 3, 4, 11);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}


void setup() {
  // monitoring port
  Serial.begin(115200);

  // check if you need internal pullups
  sensor.pullup = Pullup::USE_EXTERN;
  
  // initialise encoder hardware
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  sensor.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}
