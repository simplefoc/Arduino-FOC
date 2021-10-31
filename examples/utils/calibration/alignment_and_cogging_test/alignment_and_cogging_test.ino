#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);
MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0X0C, 4);


/**
 * This measures how closely sensor and electrical angle agree and how much your motor is affected by 'cogging'.
 * It can be used to investigate how much non linearity there is between what we set (electrical angle) and what we read (sensor angle)
 * This non linearity could be down to magnet placement, coil winding differences or simply that the magnetic field when travelling through a pole pair is not linear
 * An alignment error of ~10 degrees and cogging of ~4 degrees is normal for small gimbal.
 * The following article is an interesting read
 * https://hackaday.com/2016/02/23/anti-cogging-algorithm-brings-out-the-best-in-your-hobby-brushless-motors/
 */
void testAlignmentAndCogging(int direction) {

  motor.move(0);
  _delay(200);

  sensor.update();
  float initialAngle = sensor.getAngle();

  const int shaft_rotation = 720; // 720 deg test - useful to see repeating cog pattern
  int sample_count = int(shaft_rotation * motor.pole_pairs); // test every electrical degree

  float stDevSum = 0;

  float mean = 0.0f;
  float prev_mean = 0.0f;


  for (int i = 0; i < sample_count; i++) {

    float electricAngle = (float) direction * i * motor.pole_pairs * shaft_rotation / sample_count;
    // move and wait
    motor.move(electricAngle * PI / 180);
    _delay(5);

    // measure
    sensor.update();
    float sensorAngle = (sensor.getAngle() - initialAngle) * 180 / PI;
    float sensorElectricAngle = sensorAngle * motor.pole_pairs;
    float electricAngleError = electricAngle - sensorElectricAngle;

    // plot this - especially electricAngleError
    Serial.print(electricAngle);
    Serial.print("\t");
    Serial.print(sensorElectricAngle );
    Serial.print("\t");
    Serial.println(electricAngleError);

    // use knuth standard deviation algorithm so that we don't need an array too big for an Uno
    prev_mean = mean;
    mean = mean + (electricAngleError-mean)/(i+1);
    stDevSum = stDevSum + (electricAngleError-mean)*(electricAngleError-prev_mean);

  }

  Serial.println();
  Serial.println(F("ALIGNMENT AND COGGING REPORT"));
  Serial.println();
  Serial.print(F("Direction: "));
  Serial.println(direction);
  Serial.print(F("Mean error (alignment): "));
  Serial.print(mean);
  Serial.println(" deg (electrical)");
  Serial.print(F("Standard Deviation (cogging): "));
  Serial.print(sqrt(stDevSum/sample_count));
  Serial.println(F(" deg (electrical)"));
  Serial.println();
  Serial.println(F("Plotting 3rd column of data (electricAngleError) will likely show sinusoidal cogging pattern with a frequency of 4xpole_pairs per rotation"));
  Serial.println();

}

void setup() {

  Serial.begin(115200);
  while (!Serial) ;

  // driver config
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 3;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit=motor.voltage_sensor_align;

  sensor.init();
  motor.linkSensor(&sensor);

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();

  testAlignmentAndCogging(1);

  motor.move(0);
  Serial.println(F("Press any key to test in CCW direction"));
  while (!Serial.available()) { }

  testAlignmentAndCogging(-1);

  Serial.println(F("Complete"));

  motor.voltage_limit = 0;
  motor.move(0);
  while (true) ; //do nothing;

}




void loop() {

}
