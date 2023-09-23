/**
 *  Hall sensors example code using only software interrupts
 * 
 * This is a code intended to test the hall sensor connections and to 
 * demonstrate the hall sensor setup fully using software interrupts.
 * - We use PciManager library: https://github.com/prampec/arduino-pcimanager
 * 
 * This code will work on Arduino devices but not on STM32 devices
 */

#include <SimpleFOC.h>
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

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
// If no available hadware interrupt pins use the software interrupt
PciListenerImp listenA(sensor.pinA, doA);
PciListenerImp listenB(sensor.pinB, doB);
PciListenerImp listenC(sensor.pinC, doC);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // check if you need internal pullups
  sensor.pullup = Pullup::USE_EXTERN;
  
  // initialise encoder hardware
  sensor.init();
  
  // software interrupts
  PciManager.registerListener(&listenA);
  PciManager.registerListener(&listenB);
  PciManager.registerListener(&listenC);

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
