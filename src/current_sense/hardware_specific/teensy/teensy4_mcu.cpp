#include "teensy4_mcu.h"
#include "../../../drivers/hardware_specific/teensy/teensy4_mcu.h"
// #include "../../../common/lowpass_filter.h"
#include "../../../common/foc_utils.h"
#include "../../../communication/SimpleFOCDebug.h"

// if defined 
// - Teensy 4.0 
// - Teensy 4.1 
#if defined(__arm__) && defined(CORE_TEENSY) && ( defined(__IMXRT1062__) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY_MICROMOD) )

// #define SIMPLEFOC_TEENSY4_ADC_INTERRUPT_DEBUG


volatile uint32_t val0, val1, val2;

// #define _BANDWIDTH_CS 10000.0f // [Hz] bandwidth for the current sense
// LowPassFilter lp1 = LowPassFilter(1.0/_BANDWIDTH_CS);
// LowPassFilter lp2 = LowPassFilter(1.0/_BANDWIDTH_CS);
// LowPassFilter lp3 = LowPassFilter(1.0/_BANDWIDTH_CS);

void read_currents(uint32_t *a, uint32_t*b, uint32_t *c=nullptr){
  *a = val0;
  *b = val1;
  *c = val2;
}

// interrupt service routine for the ADC_ETC0
// reading the ADC values and clearing the interrupt
void adcetc0_isr() {
#ifdef SIMPLEFOC_TEENSY4_ADC_INTERRUPT_DEBUG
  digitalWrite(30,HIGH);
#endif
 // page 3509 , section 66.5.1.3.3
  ADC_ETC_DONE0_1_IRQ |= 1;   // clear Done0 for trg0 at 1st bit
  // val0 = lp1(ADC_ETC_TRIG0_RESULT_1_0 & 4095);
  val0 = (ADC_ETC_TRIG0_RESULT_1_0 & 4095);
  // val1 = lp2((ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095);
  val1 = (ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095;
#ifdef SIMPLEFOC_TEENSY4_ADC_INTERRUPT_DEBUG
  digitalWrite(30,LOW);
#endif
}


void adcetc1_isr() {
#ifdef SIMPLEFOC_TEENSY4_ADC_INTERRUPT_DEBUG
  digitalWrite(30,HIGH);
#endif
 // page 3509 , section 66.5.1.3.3
 ADC_ETC_DONE0_1_IRQ |= 1 << 16;   // clear Done1 for trg0 at 16th bit
 val2 = ADC_ETC_TRIG0_RESULT_3_2  & 4095;
//  val2 = lp3( ADC_ETC_TRIG0_RESULT_3_2  & 4095);
#ifdef SIMPLEFOC_TEENSY4_ADC_INTERRUPT_DEBUG
  digitalWrite(30,LOW);
#endif
}

// function initializing the ADC2
// and the ADC_ETC trigger for the low side current sensing
void adc1_init(int pin1, int pin2, int pin3=NOT_SET) {
    //Tried many configurations, but this seems to be best:
    ADC1_CFG =   ADC_CFG_OVWREN       //Allow overwriting of the next converted Data onto the existing
                | ADC_CFG_ADICLK(0)    // input clock select - IPG clock
                | ADC_CFG_MODE(2)      // 12-bit conversion 0 8-bit conversion 1 10-bit conversion 2  12-bit conversion
                | ADC_CFG_ADIV(2)      // Input clock / 2 (0 for /1, 1 for /2 and 2  for / 4) (1 is faster and maybe with some filtering could provide better results but 2 for now)
                | ADC_CFG_ADSTS(0)     // Sample period (ADC clocks) = 3 if ADLSMP=0b
                | ADC_CFG_ADHSC        // High speed operation
                | ADC_CFG_ADTRG;       // Hardware trigger selected


    //Calibration of ADC1
    ADC1_GC |= ADC_GC_CAL;   // begin cal ADC1
    while (ADC1_GC & ADC_GC_CAL) ;

    ADC1_HC0 = 16;   // ADC_ETC channel
    // use the second interrupt if necessary (for more than 2 channels)
    if(_isset(pin3)) {      
      ADC1_HC1 = 16;
    }
}

// function initializing the ADC2
// and the ADC_ETC trigger for the low side current sensing
void adc2_init(){

    // configuring ADC2
    //Tried many configurations, but this seems to be best:
    ADC1_CFG =   ADC_CFG_OVWREN       //Allow overwriting of the next converted Data onto the existing
                | ADC_CFG_ADICLK(0)    // input clock select - IPG clock
                | ADC_CFG_MODE(2)      // 12-bit conversion 0 8-bit conversion 1 10-bit conversion 2  12-bit conversion
                | ADC_CFG_ADIV(2)      // Input clock / 2 (0 for /1, 1 for /2 and 2  for / 4)
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

// function initializing the ADC_ETC trigger for the low side current sensing
// it uses only the ADC1 
// if the pin3 is not set it uses only 2 channels
void adc_etc_init(int pin1, int pin2, int pin3=NOT_SET) {
    ADC_ETC_CTRL &= ~(1 << 31); // SOFTRST
    ADC_ETC_CTRL = 0x40000001;  // start with trigger 0
    ADC_ETC_TRIG0_CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN( _isset(pin3) ? 2 : 1) ; // 2 if 3 channels, 1 if 2 channels

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
    if(_isset(pin3)) {      
      ADC_ETC_TRIG0_CHAIN_3_2 =
        ADC_ETC_TRIG_CHAIN_IE0(2) |    // interrupt when Done1
        ADC_ETC_TRIG_CHAIN_B2B0 |      // Enable B2B, back to back ADC trigger
        ADC_ETC_TRIG_CHAIN_HWTS0(1) |
        ADC_ETC_TRIG_CHAIN_CSEL0(pin_to_channel[pin3]); 

      attachInterruptVector(IRQ_ADC_ETC1, adcetc1_isr);
      NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);
    }
}



// function reading an ADC value and returning the read voltage
float _readADCVoltageLowSide(const int pinA, const void* cs_params){

    if(!_isset(pinA)) return 0.0; // if the pin is not set return 0
    GenericCurrentSenseParams* params = (GenericCurrentSenseParams*) cs_params;
    float adc_voltage_conv = params->adc_voltage_conv;
    if (pinA == params->pins[0]) {
        return val0 * adc_voltage_conv;
    } else if (pinA == params->pins[1]) {
        return val1 * adc_voltage_conv;
    }else if (pinA == params->pins[2]) {
        return val2 * adc_voltage_conv;
    }
    return 0.0;
}

// Configure low side for generic mcu
// cannot do much but 
void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
  Teensy4DriverParams* par = (Teensy4DriverParams*) ((TeensyDriverParams*)driver_params)->additional_params;
  if(par == nullptr){
    SIMPLEFOC_DEBUG("TEENSY-CS: Low side current sense failed, driver not supported!");
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
  }

  SIMPLEFOC_DEBUG("TEENSY-CS: Configuring low side current sense!");

#ifdef SIMPLEFOC_TEENSY4_ADC_INTERRUPT_DEBUG
  pinMode(30,OUTPUT);
#endif

  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  // check if either of the pins are not set
  // and dont use it if it isn't
  int pin_count = 0;
  int pins[3] = {NOT_SET, NOT_SET, NOT_SET};
  if(_isset(pinA)) pins[pin_count++] = pinA;
  if(_isset(pinB)) pins[pin_count++] = pinB;
  if(_isset(pinC)) pins[pin_count++] = pinC;


  adc1_init(pins[0], pins[1], pins[2]);
  adc_etc_init(pins[0], pins[1], pins[2]);

  xbar_init();

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = {pins[0], pins[1], pins[2] },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };
  return params;
}

// sync driver and the adc
void* _driverSyncLowSide(void* driver_params, void* cs_params){
    Teensy4DriverParams* par = (Teensy4DriverParams*) ((TeensyDriverParams*)driver_params)->additional_params;
    IMXRT_FLEXPWM_t* flexpwm = par->flextimers[0];
    int submodule = par->submodules[0];

    SIMPLEFOC_DEBUG("TEENSY-CS: Syncing low side current sense!");
    char buff[50];
    sprintf(buff, "TEENSY-CS: Syncing to FlexPWM: %d, Submodule: %d", flexpwm_to_index(flexpwm), submodule);
    SIMPLEFOC_DEBUG(buff);

    // find the xbar trigger for the flexpwm
    int xbar_trig_pwm = flexpwm_submodule_to_trig(flexpwm, submodule);
    if(xbar_trig_pwm<0) return;

    // allow theFlexPWM  to trigger the ADC_ETC
    xbar_connect((uint32_t)xbar_trig_pwm, XBARA1_OUT_ADC_ETC_TRIG00); //FlexPWM to adc_etc

    // setup the ADC_ETC trigger to be triggered by the FlexPWM channel 1 (val1)
    //This val1 interrupt on match is in the center of the PWM
    flexpwm->SM[submodule].TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1<<1); 


    // if needed the interrupt can be moved to some other point in the PWM cycle by using an addional val register example: VAL4
    // setup the ADC_ETC trigger to be triggered by the FlexPWM channel 4 (val4)
    // flexpwm->SM[submodule].TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1<<4);
    // setup this val4 for interrupt on match for ADC sync
    // this code assumes that the val4 is not used for anything else!
    // reading two ADC takes about 2.5us. So put the interrupt 2.5us befor the center 
    // flexpwm->SM[submodule].VAL4 = int(flexpwm->SM[submodule].VAL1*(1.0f - 2.5e-6*par->pwm_frequency))  ; // 2.5us before center 


#ifdef SIMPLEFOC_TEENSY4_ADC_INTERRUPT_DEBUG
    // pin 4 observes out trigger line for 'scope
    xbar_connect (xbar_trig_pwm, XBARA1_OUT_IOMUX_XBAR_INOUT08) ;
    IOMUXC_GPR_GPR6 |= IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_8 ;  // select output mode for INOUT8
    // Select alt 3 for  EMC_06 (XBAR), rather than original 5 (GPIO)
    CORE_PIN4_CONFIG = 3 ; // shorthand for  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 =  3 ;
    // turn up drive & speed as very short pulse
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_06 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE ;
#endif

  
  // return the cs parameters 
  // successfully initialized
  // TODO verify if success in future
  return cs_params;
}


#endif
