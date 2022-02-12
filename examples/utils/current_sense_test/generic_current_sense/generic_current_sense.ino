/**
 * An example code for the generic current sensing implementation
*/
#include <SimpleFOC.h>


// user defined function for reading the phase currents
// returning the value per phase in amps
PhaseCurrent_s readCurrentSense(){
  PhaseCurrent_s c;
  // dummy example only reading analog pins
  c.a = analogRead(A0);
  c.b = analogRead(A1);
  c.c = analogRead(A2); // if no 3rd current sense set it to 0
  return(c);
}

// user defined function for intialising the current sense
// it is optional and if provided it will be called in current_sense.init()
void initCurrentSense(){
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
}


// GenericCurrentSense class constructor
// it receives the user defined callback for reading the current sense
// and optionally the user defined callback for current sense initialisation
GenericCurrentSense current_sense = GenericCurrentSense(readCurrentSense, initCurrentSense);


void setup() {
  // if callbacks are not provided in the constructor 
  // they can be assigned directly: 
  //current_sense.readCallback = readCurrentSense;
  //current_sense.initCallback = initCurrentSense;
  
  // initialise the current sensing
  current_sense.init();

  
  Serial.begin(115200);
  Serial.println("Current sense ready.");
}

void loop() {

    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    float current_magnitude = current_sense.getDCCurrent();

    Serial.print(currents.a); // milli Amps
    Serial.print("\t");
    Serial.print(currents.b); // milli Amps
    Serial.print("\t");
    Serial.print(currents.c); // milli Amps
    Serial.print("\t");
    Serial.println(current_magnitude); // milli Amps
}