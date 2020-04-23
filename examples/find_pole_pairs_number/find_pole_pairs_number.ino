#include <SimpleFOC.h>

// Only pins 2 and 3 are supported
#define arduinoInt1 2             // Arduino UNO interrupt 0
#define arduinoInt2 3             // Arduino UNO interrupt 1

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
// its not important how many pole pairs do you set, the progam will find it alone
BLDCMotor motor = BLDCMotor(9, 10, 11, 0, 8);
//  Encoder(int encA, int encB , int cpr, int index)
Encoder encoder = Encoder(arduinoInt1, arduinoInt2, 8192);
// interrupt ruotine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() {
  // debugging port
  Serial.begin(115200);

  // check if you need internal pullups
  //  Quadrature::ENABLE - CPR = 4xPPR  - default
  //  Quadrature::DISABLE - CPR = PPR
  encoder.quadrature = Quadrature::ENABLE;

  // check if you need internal pullups
  // Pullup::EXTERN - external pullup added - dafault
  // Pullup::INTERN - needs internal arduino pullup
  encoder.pullup = Pullup::EXTERN;
  
  // initialise encoder hardware
  encoder.init(doA, doB);

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;

  // set FOC loop to be used
  motor.controller = ControlType::voltage;

  // link the motor to the sensor
  motor.linkEncoder(&encoder);
  // intialise motor
  motor.init();



  // pole pairs calculation routine
  Serial.println("Motor pole pair number estimation example");
  Serial.println("---------------------------------------------\n");

  float pp_search_voltage = 4; // maximum power_supply_voltage/2
  float pp_search_angle = 6*M_PI; // search electrical angle to turn
  
  // move motor to the electrical angle 0
  motor.setPhaseVoltage(pp_search_voltage,0);
  _delay(1000);
  // read the encoder angle 
  float angle_begin = encoder.getAngle();
  _delay(50);
  
  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while(motor_angle <= pp_search_angle){
    motor_angle += 0.01;
    motor.setPhaseVoltage(pp_search_voltage, motor_angle);
  }
  _delay(1000);
  // read the encoder value for 180
  float angle_end = encoder.getAngle();
  _delay(50);
  // turn off the motor
  motor.setPhaseVoltage(0,0);
  _delay(1000);

  // caluclate the pole pair number
  int pp = round((pp_search_angle)/(angle_end-angle_begin));

  Serial.print("Estimated pole pairs number is: ");
  Serial.println(pp);
  Serial.println("Electrical angle / Encoder angle = Pole pairs ");
  Serial.print(pp_search_angle*180/M_PI);
  Serial.print("/");
  Serial.print((angle_end-angle_begin)*180/M_PI);
  Serial.print(" = ");
  Serial.println((pp_search_angle)/(angle_end-angle_begin));
  Serial.println();
   

  // a bit of debugging the result
  if(pp <= 0 ){
    Serial.println("Pole pair number cannot be negative");
    Serial.println(" - Try changing the search_voltage vlaue or motor/encoder configuration.");
    return;
  }else if(pp > 30){
    Serial.println("Pole pair number very high, possible error.");
  }else{
    Serial.println("If pp is estimated well your motor should turn now!");
    Serial.println(" - If it is not moving try to relaunch the program!");
    Serial.println(" - You can also try to adjust the target votage using serial terminal!");
  }

  
  // set the pole pair number to the motor
  motor.pole_pairs = pp;
  //align encoder and start FOC
  motor.initFOC();
  _delay(1000);

  Serial.println("\n Motor ready.");
  Serial.println("Set the target voltage using serial terminal:");
}

// uq voltage
float target_voltage = 2;

void loop() {

  // iterative foc voltage loop
  motor.loopFOC();

  // iterative function setting the outter loop target
  motor.move(target_voltage);

}


// Serial communication callback function
// gets the target value from the user
void serialEvent() {
  // a string to hold incoming data
  static String inputString; 
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline
    // end of input
    if (inChar == '\n') {
      target_voltage = inputString.toFloat();
      Serial.print("Tagret voltage: ");
      Serial.println(target_voltage);
      inputString = "";
    }
  }
}