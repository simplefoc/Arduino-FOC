#include <SimpleFOC.h>

// Only pins 2 and 3 are supported
#define arduinoInt1 2             // Arduino UNO interrupt 0
#define arduinoInt2 3             // Arduino UNO interrupt 1

//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
// its not important how many pole pairs do you set, the progam will find it alone
BLDCMotor motor = BLDCMotor(9, 5, 6, 0, 8);

// MagneticSensor(int cs, float _cpr, int _angle_register)
//  cs              - SPI chip select pin 
//  _cpr            - counts per revolution 
// _angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensor AS5x4x = MagneticSensor(10, 16384, 0x3FFF);

void setup() {
  // debugging port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  AS5x4x.init();

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;

  // set FOC loop to be used
  motor.controller = ControlType::voltage;

  // link the motor to the sensor
  motor.linkSensor(&AS5x4x);
  // initialize motor
  motor.init();


  // pole pairs calculation routine
  Serial.println("Motor pole pair number estimation example");
  Serial.println("---------------------------------------------\n");

  float pp_search_voltage = 4; // maximum power_supply_voltage/2
  float pp_search_angle = 6*M_PI; // search electrical angle to turn
  
  // move motor to the electrical angle 0
  motor.setPhaseVoltage(pp_search_voltage,0);
  _delay(1000);
  // read the sensor angle 
  float angle_begin = AS5x4x.getAngle();
  _delay(50);
  
  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while(motor_angle <= pp_search_angle){
    motor_angle += 0.01;
    motor.setPhaseVoltage(pp_search_voltage, motor_angle);
  }
  _delay(1000);
  // read the sensor value for 180
  float angle_end = AS5x4x.getAngle();
  _delay(50);
  // turn off the motor
  motor.setPhaseVoltage(0,0);
  _delay(1000);

  // calculate the pole pair number
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
    Serial.println(" - Try changing the search_voltage value or motor/sensor configuration.");
    return;
  }else if(pp > 30){
    Serial.println("Pole pair number very high, possible error.");
  }else{
    Serial.println("If pp is estimated well your motor should turn now!");
    Serial.println(" - If it is not moving try to relaunch the program!");
    Serial.println(" - You can also try to adjust the target voltage using serial terminal!");
  }

  
  // set the pole pair number to the motor
  motor.pole_pairs = pp;
  //align sensor and start FOC
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
      Serial.print("Target voltage: ");
      Serial.println(target_voltage);
      inputString = "";
    }
  }
}