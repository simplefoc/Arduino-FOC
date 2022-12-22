/*
    Odrive robotics' hardware is one of the best  BLDC motor foc supporting hardware out there.

    This is an example code that can be directly uploaded to the Odrive using the SWD programmer. 
    This code uses an magnetic spi sensor AS5047 and a BLDC motor with 11 pole pairs connected to the M0 interface of the Odrive. 

    This is a short template code and the idea is that you are able to adapt to your needs not to be a complete solution. :D 
*/
#include <SimpleFOC.h>

// Odrive M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15
// M0 currnets
#define M0_IB PC0
#define M0_IC PC1
// Odrive M0 encoder pinout
#define M0_ENC_A PB4
#define M0_ENC_B PB5
#define M0_ENC_Z PC9


// Odrive M1 motor pinout
#define M1_INH_A PC6
#define M1_INH_B PC7
#define M1_INH_C PC8
#define M1_INL_A PA7
#define M1_INL_B PB0
#define M1_INL_C PB1
// M0 currnets
#define M1_IB PC2
#define M1_IC PC3
// Odrive M1 encoder pinout
#define M1_ENC_A PB6
#define M1_ENC_B PB7
#define M1_ENC_Z PC15

// M1 & M2 common enable pin
#define EN_GATE PB12

// SPI pinout
#define SPI3_SCL  PC10
#define SPI3_MISO PC11
#define SPI3_MOSO PC12

// Motor instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A,M0_INL_A, M0_INH_B,M0_INL_B, M0_INH_C,M0_INL_C, EN_GATE);

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

// low side current sensing define
// 0.0005 Ohm resistor
// gain of 10x
// current sensing on B and C phases, phase A not connected
LowsideCurrentSense current_sense = LowsideCurrentSense(0.0005f, 10.0f, _NC, M0_IB, M0_IC);

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// config           - SPI config
//  cs              - SPI chip select pin 
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, M0_ENC_A);
SPIClass SPI_3(SPI3_MOSO, SPI3_MISO, SPI3_SCL);

void setup(){

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 20;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 20;
  // driver init
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // initialise magnetic sensor hardware
  sensor.init(&SPI_3);
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  
  // control loop type and torque mode 
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // max voltage  allowed for motion control 
  motor.voltage_limit = 8.0;
  // alignment voltage limit
  motor.voltage_sensor_align = 0.5;
  

  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D;
  motor.monitor_downsample = 1000;

  // add target command T
  command.add('M', doMotor, "motor M0");

  // initialise motor
  motor.init();

  // link the driver
  current_sense.linkDriver(&driver);
  // init the current sense
  current_sense.init();  
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);
  
  // init FOC  
  motor.initFOC();  
  delay(1000);
}

void loop(){

  // foc loop
  motor.loopFOC();
  // motion control
  motor.move();
  // monitoring 
  motor.monitor();
  // user communication
  command.run();
}