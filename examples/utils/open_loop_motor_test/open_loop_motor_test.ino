// Open loop motor control example 
 #include <SimpleFOC.h>

// motor instance
BLDCMotor motor = BLDCMotor(3, 10, 6, 11, 7);

void setup() {
  
  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;
  
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // init motor hardware
  motor.init();

  Serial.begin(115200);
  Serial.println("Motor ready!");
  _delay(1000);
}

// target voltage to be set to the motor
float target_voltage = 2;
// target voltage to be set to the motor
float target_angle = 0;

void loop() {
  // integrating the angle in real time
  // the larger the constant added the faster the movement
  target_angle += 0.001;
  
  // set open loop voltage
  motor.setPhaseVoltage(target_voltage, target_angle);
}