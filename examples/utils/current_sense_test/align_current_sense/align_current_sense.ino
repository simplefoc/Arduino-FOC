/**
 * This is an example code for visual aligning current sense and the driver phases as well 
 * it is used to test the current sense implementation.
 * 
 * In this example it uses the BLDCMotor and BLDCDriver3PWM classes to control a BLDC motor
 * and the InlineCurrentSense class to read the phase currents.
 * > In your application you can use any other motor, driver and current sense implementation.
 * > The rest of the code will stay the same
 * 
 * The example uses the teleplot (https://teleplot.fr) service to visualize the phase currents and voltages.
 * Its really awesome tool and you can use it to visualize any data you want.  
 */
#include <SimpleFOC.h>

// BLDC motor & driver instance
// NOTE: replace with your motor and driver configuration
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(6, 10, 5, 8);

// Current sense instance 
// NOTE: replace with your current sense configuration
// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
InlineCurrentSense current_sense = InlineCurrentSense(185.0f, A0, A2);
// or some other current sense
// LowsideCurrentSense current_sense = LowsideCurrentSense(185.0f, A0, A2); // ex. lowside current sense

// commander communication instance
Commander command = Commander(Serial);


bool start = false; // flag to start printing phase currents and voltages
float frequency = 10000; // frequency of printing phase currents and voltages

void doStart(char* cmd){
  // toggle the start flag
  start = !start;
  if(start){
    SIMPLEFOC_DEBUG("Start printing phase currents and voltages");
  } else {
    SIMPLEFOC_DEBUG("Stop printing phase currents and voltages");
  }
}

void doCurrentA(char* cmd){ 
    SIMPLEFOC_DEBUG("Inverted cs A gain"); 
    current_sense.gain_a = -current_sense.gain_a; 
    SIMPLEFOC_DEBUG("New gain A: ", current_sense.gain_a);
}
void doCurrentB(char* cmd){ 
    SIMPLEFOC_DEBUG("Inverted cs B gain"); 
    current_sense.gain_b = -current_sense.gain_b; 
    SIMPLEFOC_DEBUG("New gain B: ", current_sense.gain_b);
}
void doCurrentC(char* cmd){ 
    SIMPLEFOC_DEBUG("Inverted cs C gain"); 
    current_sense.gain_c = -current_sense.gain_c; 
    SIMPLEFOC_DEBUG("New gain C: ", current_sense.gain_c);
}

void doMotorLimit(char* cmd){
  // set the voltage limit for the motor
  command.scalar(&motor.voltage_limit, cmd);
}

void doTarget(char* cmd){
  // set the target value for the motor
  command.scalar(&motor.target, cmd);
}

void doFrequency(char* cmd){
  // set the frequency of printing phase currents and voltages
  command.scalar(&frequency, cmd);
}

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 20;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  current_sense.linkDriver(&driver);

  // set control loop type to be used
  motor.controller = MotionControlType::velocity_openloop;

  motor.voltage_limit = 1; // voltage limit for the motor

  // initialise motor
  motor.init();

  // current sense init and linking
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 0.5;


  // subscribe motor to the commander
  //command.add('T', doMotion, "motion control");
  command.add('A', doCurrentA, "Invert cs A gain");
  command.add('B', doCurrentB, "Invert cs B gain");
  command.add('C', doCurrentC, "Invert cs C gain");
  command.add('L', doMotorLimit, "Set motor voltage limit");
  command.add('T', doTarget, "Set motor target");
  command.add('S', doStart, "Start/Stop printing phase currents and voltages");
  command.add('F', doFrequency, "Set frequency of printing phase currents and voltages");
  
  SIMPLEFOC_DEBUG("To use this example:");
  SIMPLEFOC_DEBUG(" - use 'L' to control the motor voltage limit");
  SIMPLEFOC_DEBUG(" - use 'T' to set the motor target");
  SIMPLEFOC_DEBUG(" - use 'A', 'B', 'C' to invert current sense gains");
  SIMPLEFOC_DEBUG(" - use 'F' to set frequency of printing phase currents and voltages (100Hz by default)");
  SIMPLEFOC_DEBUG(" - use 'S' to start/stop printing phase currents and voltages");
  SIMPLEFOC_DEBUG("IMPORTANT: Use teleplot to visualize the phase currents and voltages: https://teleplot.fr/");

  _delay(1000);

}

float normalize_voltage(float v){
  return (v - driver.voltage_power_supply/2.0)/motor.voltage_limit;
}

float max_current = 0.0f; // max current for normalization
LowPassFilter lp_filter_maxc(0.3f); // low pass filter for current normalization
void normalize_currents(PhaseCurrent_s& c, float& max_current){
  static unsigned long timestamp = _micros();
  // normalize current to the max current
   
  float m_current = 0.0f;
  if(fabs(c.a) > m_current) m_current = fabs(c.a);
  if(fabs(c.b) > m_current) m_current = fabs(c.b);
  if(fabs(c.c) > m_current) m_current = fabs(c.c);
  // filter the max current 
  max_current = lp_filter_maxc(m_current);

  c.a = c.a / max_current;
  c.b = c.b / max_current;
  c.c = c.c / max_current;
}

unsigned long t = _micros();

void loop() {
  motor.loopFOC();
  motor.move();

  // print each 
  if( start & (_micros() - t > (1.0/frequency * 1e6))){
    // read phase currents
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    // normalize currents
    normalize_currents(currents, max_current);
    // print phase currents
    SIMPLEFOC_DEBUG(">c.a:",currents.a);
    SIMPLEFOC_DEBUG(">c.b:",currents.b);
    SIMPLEFOC_DEBUG(">c.c:",currents.c);
    // print phase voltages
    SIMPLEFOC_DEBUG(">v.a:",normalize_voltage(motor.Ua));
    SIMPLEFOC_DEBUG(">v.b:",normalize_voltage(motor.Ub));
    SIMPLEFOC_DEBUG(">v.c:",normalize_voltage(motor.Uc));
    t = _micros();
  }

  // user communication
  command.run();
}