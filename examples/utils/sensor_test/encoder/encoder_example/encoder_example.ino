/**
 *  Encoder example code
 *
 * This is a code intended to test the encoder connections and to demonstrate the encoder setup.
 *
 */

#include <SimpleFOC.h>

// Encoder sensor instance
// Encoder(int encA, int encB, int cpr, int index)
//  - encA, encB        - encoder A and B pins
//  - cpr               - counts per rotation number (cpm=ppm*4)
Encoder encoder = Encoder(2, 3, 8192);
// interrupt routine initialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() {
  // monitoring port
  Serial.begin(115200);

  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::ON;

  // check if you need internal pullups
  encoder.pullup = Pullup::USE_EXTERN;

  // initialise encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);

  Serial.println("Encoder ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // not doing much for the encoder though
  encoder.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(encoder.getAngle());
  Serial.print("\t");
  Serial.println(encoder.getVelocity());
}
