/**
 * 
 * HMBGC torque control example using voltage control loop.
 * 
 * - Motor is connected the MOT1 connector (MOT1 9,10,11; MOT2 3,5,6)
 * - Encoder is connected to A0 and A1
 * 
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers 
 * you a way to control motor torque by setting the voltage to the motor instead hte current. 
 * 
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way. position motion control example with encoder
 * 
 * NOTE:
 * > HMBGC doesn't have any interrupt pins so we need to run all the encoder channels with the software interrupt library 
 * > - For this example we use: PciManager library : https://github.com/prampec/arduino-pcimanager
 * 
 * See docs.simplefoc.com for more info.
 * 
 */
#include <SimpleFOC.h>
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11);

// encoder instance
Encoder encoder = Encoder(A0, A1, 8192);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// encoder interrupt init
PciListenerImp listenerA(encoder.pinA, doA);
PciListenerImp listenerB(encoder.pinB, doB);


// voltage set point variable
float target_voltage = 2;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() { 
  
  // initialize encoder sensor hardware
  encoder.init();
  // interrupt initialization
  PciManager.registerListener(&listenerA);
  PciManager.registerListener(&listenerB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial for motor init
  // comment out if not needed
  motor.useMonitoring(Serial);

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
  
  // add target command T
  command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);
  
  // user communication
  command.run();
}