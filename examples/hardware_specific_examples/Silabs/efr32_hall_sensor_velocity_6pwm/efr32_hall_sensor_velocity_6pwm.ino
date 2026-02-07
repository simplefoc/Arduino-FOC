/** 
 * Silabs MG24 6PWM closed loop velocity control example with HALL sensor based rotor position
 *
 * HARDWARE CONFIGURATION:
 *  CPU Board: Arduino Nano Matter
 *  Motor Driver Board: BOOSTXL-DRV8305EVM
 *  BLDC Motor: Newark DF45M024053-A2
 */

#include <SimpleFOC.h>

#define HALL_SENSOR_IRQ 1
#define ENABLE_MONITOR  0

static bool allow_run = false;

// BLDC motor instance
BLDCMotor *motor;

// BLDC driver instance
BLDCDriver6PWM *driver;

// Commander instance
Commander *command;

// Hall sensor instance
HallSensor *sensor;

// Interrupt routine initialization
// channel A and B callbacks
void doA() { sensor->handleA(); }
void doB() { sensor->handleB(); }
void doC() { sensor->handleC(); }

void doMotor(char* cmd) {
  if (!command) return;
  command->motor(motor, cmd);
}

void setup() {
  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // Commander
  command = new Commander(Serial);
  if (!command) return;

  // Driver
  driver = new BLDCDriver6PWM(6, 7, 8, 9, 10, 11, 12);
  if (!driver) return;

  // Driver config
  // power supply voltage [V]
  driver->voltage_power_supply = 24;
  // pwm frequency to be used [Hz]
  driver->pwm_frequency = 20000; // 20 kHz
  // Max DC voltage allowed - default voltage_power_supply
  driver->voltage_limit = 12;
  // dead zone percentage of the duty cycle - default 0.02 - 2%
  // Can set value to 0 because the DRV8305 will provide the 
  // required dead-time.
  driver->dead_zone = 0;

  // init driver
  if (!driver->init()) {
    Serial.println("Driver init failed!");
    return;
  }
  driver->enable();

  // HallSensor(int encA, int encB, int encC, int pp)
  //  - encA, encB, encC    - HallSensor A, B and C pins
  //  - pp                  - pole pairs
  sensor = new HallSensor(5, 4, 13, 8);
  if (!sensor) return;

  // initialize sensor sensor hardware
  sensor->init();

#if HALL_SENSOR_IRQ
  sensor->enableInterrupts(doA, doB, doC);
#else
  // Note: There is a bug when initializing HallSensor in heap, attribute
  // `use_interrupt` gets value not `false` even `enableInterrupts` has not been
  // called. So we initialize this attribute value `false` after creating a
  // `HallSensor` instance.
  sensor->use_interrupt = false;
#endif

  // Motor
  motor = new BLDCMotor(8);
  if (!motor) return;

  // link the motor and the driver
  motor->linkDriver(driver);

  // link the motor to the sensor
  motor->linkSensor(sensor);

  // Set below the motor's max 5600 RPM limit = 586 rad/s
  motor->velocity_limit = 530.0f;

  // set motion control loop to be used
  motor->controller = MotionControlType::velocity;

  // choose FOC modulation (optional) - SinePWM or SpaceVectorPWM
  motor->foc_modulation = FOCModulationType::SpaceVectorPWM;

  // controller configuration
  // velocity PI controller parameters
  motor->PID_velocity.P = 0.05f;
  motor->PID_velocity.I = 1;
  
  // velocity low pass filtering time constant
  motor->LPF_velocity.Tf = 0.01f;

#if ENABLE_MONITOR
  motor->useMonitoring(Serial);
  motor->monitor_variables = _MON_TARGET | _MON_VEL;
#endif

  // initialize motor
  if (!motor->init()) {
    Serial.println("Motor init failed!");
    return;
  }

  // align sensor and start FOC
  if (!motor->initFOC()) {
    Serial.println("FOC init failed!");
    return;
  }

  // add target command M
  command->add('M', doMotor, "motor");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");

  allow_run = true;
  _delay(1000);
}

void loop() {
  if (!allow_run) return;

  // main FOC algorithm function
  // the faster you run this function the better
  motor->loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor->move();

#if ENABLE_MONITOR
  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  motor->monitor();
#endif

  // user communication
  command->run();
}
