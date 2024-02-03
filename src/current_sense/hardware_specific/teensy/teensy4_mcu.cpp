#include "teensy4_mcu.h"
#include "../../../drivers/hardware_specific/teensy/teensy4_mcu.h"

// if defined 
// - Teensy 4.0 
// - Teensy 4.1 
#if defined(__arm__) && defined(CORE_TEENSY) && ( defined(__IMXRT1062__) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY_MICROMOD) )

// function finding the TRIG event given the flexpwm timer and the submodule
// returning -1 if the submodule is not valid or no trigger is available
// allowing flexpwm1-4 and submodule 0-3
//
// the flags are defined in the imxrt.h file
// https://github.com/PaulStoffregen/cores/blob/dd6aa8419ee173a0a6593eab669fbff54ed85f48/teensy4/imxrt.h#L9662
int flextim__submodule_to_trig(IMXRT_FLEXPWM_t* flexpwm, int submodule){
  if(submodule <0 && submodule > 3) return -1;
  if(flexpwm == &IMXRT_FLEXPWM1){
      return XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0 + submodule; 
  }else if(flexpwm == &IMXRT_FLEXPWM2){
      return XBARA1_IN_FLEXPWM2_PWM1_OUT_TRIG0 + submodule;
  }else if(flexpwm == &IMXRT_FLEXPWM3){
      return XBARA1_IN_FLEXPWM3_PWM1_OUT_TRIG0 + submodule;
  }else if(flexpwm == &IMXRT_FLEXPWM4){
      return XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0 + submodule;
  }
  return -1;
}

volatile uint32_t val0, val1;

void read_currents(uint32_t *a, uint32_t*b){
  *a = val0;
  *b = val1;
}

// interrupt service routine for the ADC_ETC0
// reading the ADC values and clearing the interrupt
void adcetc0_isr() {
  digitalWrite(30,HIGH);
  ADC_ETC_DONE0_1_IRQ |= 1;   // clear
  val0 = ADC_ETC_TRIG0_RESULT_1_0 & 4095;
  val1 = (ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095;
  asm("dsb");
  digitalWrite(30,LOW);
}

// function initializing the ADC2
// and the ADC_ETC trigger for the low side current sensing
void adc1_init() {
    //Tried many configurations, but this seems to be best:
    ADC1_CFG =   ADC_CFG_OVWREN       //Allow overwriting of the next converted Data onto the existing
                | ADC_CFG_ADICLK(0)    // input clock select - IPG clock
                | ADC_CFG_MODE(2)      // 12-bit conversion 0 8-bit conversion 1 10-bit conversion 2  12-bit conversion
                | ADC_CFG_ADIV(2)      // Input clock / 4
                | ADC_CFG_ADSTS(0)     // Sample period (ADC clocks) = 3 if ADLSMP=0b
                | ADC_CFG_ADHSC        // High speed operation
                | ADC_CFG_ADTRG;       // Hardware trigger selected


    //Calibration of ADC1
    ADC1_GC |= ADC_GC_CAL;   // begin cal ADC1
    while (ADC1_GC & ADC_GC_CAL) ;

    ADC1_HC0 = 16;   // ADC_ETC channel
    // use the second interrupt if necessary (for more than 2 channels)
    // ADC1_HC1 = 16;
}

// function initializing the ADC2
// and the ADC_ETC trigger for the low side current sensing
void adc2_init(){

    // configuring ADC2
    //Tried many configurations, but this seems to be best:
    ADC1_CFG =   ADC_CFG_OVWREN       //Allow overwriting of the next converted Data onto the existing
                | ADC_CFG_ADICLK(0)    // input clock select - IPG clock
                | ADC_CFG_MODE(2)      // 12-bit conversion 0 8-bit conversion 1 10-bit conversion 2  12-bit conversion
                | ADC_CFG_ADIV(2)      // Input clock / 4
                | ADC_CFG_ADSTS(0)     // Sample period (ADC clocks) = 3 if ADLSMP=0b
                | ADC_CFG_ADHSC        // High speed operation
                | ADC_CFG_ADTRG;       // Hardware trigger selected

    //Calibration of ADC2
    ADC2_GC |= ADC_GC_CAL;   // begin cal ADC2
    while (ADC2_GC & ADC_GC_CAL) ;

    ADC2_HC0 = 16; // ADC_ETC channel
    // use the second interrupt if necessary (for more than 2 channels)
    //  ADC2_HC1 = 16;
}

void adc_etc_init(int pin1, int pin2) {
    ADC_ETC_CTRL &= ~(1 << 31); // SOFTRST
    ADC_ETC_CTRL = 0x40000001;  // start with trigger 0
    ADC_ETC_TRIG0_CTRL = 0x100;   // chainlength -1

    // ADC1 7 8, chain channel, HWTS, IE, B2B
    // pg 3516, section 66.5.1.8
    ADC_ETC_TRIG0_CHAIN_1_0 =
        ADC_ETC_TRIG_CHAIN_IE1(0) |   // no interrupt on first or set 2 if interrupt when Done1
        ADC_ETC_TRIG_CHAIN_B2B1 |     // Enable B2B, back to back ADC trigger
        ADC_ETC_TRIG_CHAIN_HWTS1(1) | 
        ADC_ETC_TRIG_CHAIN_CSEL1(pin_to_channel[pin1]) | // ADC channel 8
        ADC_ETC_TRIG_CHAIN_IE0(1) |   // interrupt when Done0
        ADC_ETC_TRIG_CHAIN_B2B1 |     // Enable B2B, back to back ADC trigger
        ADC_ETC_TRIG_CHAIN_HWTS0(1) | 
        ADC_ETC_TRIG_CHAIN_CSEL0(pin_to_channel[pin2]); // ADC channel 7

    attachInterruptVector(IRQ_ADC_ETC0, adcetc0_isr);
    NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
    // use the second interrupt if necessary (for more than 2 channels)
    //  attachInterruptVector(IRQ_ADC_ETC1, adcetc1_isr);
    //  NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);
}


void xbar_connect(unsigned int input, unsigned int output)
{
  if (input >= 88) return;
  if (output >= 132) return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1)) {
    val = (val & 0xFF00) | input;
  } else {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}
void xbar_init() {
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);   //turn clock on for xbara1
}


// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pinA, const void* cs_params){

    GenericCurrentSenseParams* params = (GenericCurrentSenseParams*) cs_params;
    float adc_voltage_conv = params->adc_voltage_conv;
    if (pinA == params->pins[0]) {
        return val0 * adc_voltage_conv;
    } else if (pinA == params->pins[1]) {
        return val1 * adc_voltage_conv;
    }
    return 0.0;
}

// Configure low side for generic mcu
// cannot do much but 
void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

  pinMode(30,OUTPUT);

  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  adc1_init();
  adc_etc_init(pinA, pinB); 
  xbar_init();
  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };
  return params;
}

// sync driver and the adc
void _driverSyncLowSide(void* driver_params, void* cs_params){
    Teensy4DriverParams* par = (Teensy4DriverParams*) driver_params;
    IMXRT_FLEXPWM_t* flexpwm = par->flextimers[0];
    int submodule = par->submodules[0];

    // do xbar connect here

    int xbar_trig_pwm = flextim__submodule_to_trig(flexpwm, submodule);
    if(xbar_trig_pwm<0) return;

    xbar_connect((uint32_t)xbar_trig_pwm, XBARA1_OUT_ADC_ETC_TRIG00); //FlexPWM to adc_etc

    // setup the ADC_ETC trigger
    flexpwm->SM[submodule].TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1<<4);
    // setup this val4 for interrupt on val5 match for ADC sync
    // reading two ARC takes about 5us. So put the interrupt 2.5us befor the center 
    flexpwm->SM[submodule].VAL4 = -int(2.5e-6*par->pwm_frequency*flexpwm->SM[submodule].VAL1)  ; // 2.5us before center 

}


#endif