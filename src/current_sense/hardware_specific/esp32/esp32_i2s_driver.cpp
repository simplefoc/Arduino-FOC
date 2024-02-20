#include "../../hardware_api.h"
#include "../../../drivers/hardware_api.h"
#include "../../../drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"

#include "esp32_i2s_driver.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#include <driver/i2s.h>
#include <soc/i2s_reg.h>

#include "soc/syscon_periph.h"
#include <soc/syscon_struct.h>
// #include <soc/syscon_reg.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include <soc/soc.h>
#include <soc/dport_reg.h>

#include <soc/adc_periph.h>
#include <driver/adc.h>
#include <driver/rtc_io.h>


#define BUF_LEN 1
#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f

#define I2S_USE_INTERRUPT false
#define DEBUG_ADC false

typedef struct ESP32MCPWMCurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
  mcpwm_unit_t mcpwm_unit;
  int buffer_index;
} ESP32MCPWMCurrentSenseParams;


/**
 *  I2S reading implementation by @mcells.
 */

static uint32_t IRAM_ATTR i2s_adc_buffer[ADC1_CHANNEL_MAX] = {0};
int globalActiveChannels = 0;
int channels[ADC1_CHANNEL_MAX] = {0};
bool running = false;
bool sampleOnCommand = true;
#if DEBUG_ADC == true
uint32_t IRAM_ATTR readscnt = 0;
uint32_t IRAM_ATTR intcnt = 0;
uint32_t IRAM_ATTR skipped = 0;
uint32_t IRAM_ATTR equal = 0;
uint32_t IRAM_ATTR currfifo = 0;
uint32_t IRAM_ATTR lastfifo = 0;
unsigned long IRAM_ATTR ts = 0;
unsigned long IRAM_ATTR fifotime = 0;
#endif

// This function reads data from the I2S FIFO and processes it to obtain average readings for each channel.
// The ADC counts get saved in uint32_t i2s_adc_buffer[].
void IRAM_ATTR readFiFo()
{
    // uint32_t readings[ADC1_CHANNEL_MAX][ADC1_CHANNEL_MAX*BUF_LEN];
    uint32_t avgreadings[ADC1_CHANNEL_MAX] = {0};
    uint32_t counts[ADC1_CHANNEL_MAX] = {0};
    uint32_t fifolen = GET_PERI_REG_BITS2(I2S_FIFO_CONF_REG(0), I2S_RX_DATA_NUM_M, I2S_RX_DATA_NUM_S); // I2S0.fifo_conf.rx_data_num;

#if DEBUG_ADC
    uint32_t lastrd = 0;
    uint32_t lasth = 0;
    uint32_t lastl = 0;
    uint32_t internalequal = 0;
#endif

    for (size_t i = 0; i < fifolen; i++)
    {
        // while(!GET_PERI_REG_MASK(I2S_INT_RAW_REG(0), I2S_RX_REMPTY_INT_RAW_M)){
        // I2S0.in_fifo_pop.pop = 1;
        // I2S0.in_fifo_pop.pop = 0;
        SET_PERI_REG_MASK(I2S_INFIFO_POP_REG(0), I2S_INFIFO_POP_M);
        CLEAR_PERI_REG_MASK(I2S_INFIFO_POP_REG(0), I2S_INFIFO_POP_M);

        uint32_t rd = *(uint32_t *)(REG_I2S_BASE(0) + 0x4); // GET_PERI_REG_BITS2(I2S_FIFO_CONF_REG, I2S_RX_DATA_NUM_M, I2S_RX_DATA_NUM_S);I2S0.fifo_rd;

        uint32_t highVal = rd >> 16;
        uint32_t lowVal = rd & 0xFFFF;

#if DEBUG_ADC == true
    if (rd == lastrd || highVal == lastl || lowVal == lasth)
    { //
        internalequal++;
        // currfifo = rd;
        // lastfifo = lastrd;
        // Serial.printf("\n|\nlast: ");
        // for (int i = 31; i >= 15; i--)
        // {
        //   Serial.printf("%d",(lastfifo >> i ) & 1);
        // }
        // Serial.printf("  ");
        // for (int i = 15; i >= 0; i--)
        // {
        //   Serial.printf("%d",(lastfifo >> i ) & 1);
        // }

        // Serial.printf("\ncurr: ");
        // for (int i = 31; i >= 15; i--)
        // {
        //   Serial.printf("%d",(currfifo >> i ) & 1);
        // }
        // Serial.printf("  ");
        // for (int i = 15; i >= 0; i--)
        // {
        //   Serial.printf("%d",(currfifo >> i ) & 1);
        // }
        // Serial.printf("\n");
    }

    lasth = highVal;
    lastl = lowVal;
    lastrd = rd;
    readscnt += 2;
#endif
    uint32_t chan = (lowVal >> 12) & 0x07;
    uint32_t adc_value = lowVal & 0xfff;
    // readings[chan][counts[chan]] = adc_value;
    avgreadings[chan] += adc_value;
    counts[chan]++;

    chan = (highVal >> 12) & 0x07;
    adc_value = highVal & 0xfff;
    // readings[chan][counts[chan]] = adc_value;
    avgreadings[chan] += adc_value;
    counts[chan]++;
  }
#if DEBUG_ADC == true
  equal = internalequal;
  intcnt += 1; // I2S0.fifo_conf.rx_data_num;
#endif
  for (int j = 0; j < ADC1_CHANNEL_MAX; j++)
  {
      if (counts[j] != 0)
      {
          i2s_adc_buffer[j] = avgreadings[j] / counts[j];

          // int32_t leastdiff = 4095;
          // uint32_t idx = 0;
          // for (size_t k = 0; k < counts[j]; k++)
          // {
          //   int32_t diff = abs(int(i2s_adc_buffer[j]) - int(readings[j][k]));
          //   if (leastdiff > diff && diff != 0)
          //   {
          //     leastdiff = diff;
          //     idx = k;
          //   }
          // }
          // i2s_adc_buffer[j] = readings[j][idx];

          // Serial.printf(">Channel%d:%d\n", j, i2s_adc_buffer[j]);
      }
  }
}

#if I2S_USE_INTERRUPT == true
static void IRAM_ATTR i2s_isr(void *arg)
{
#if DEBUG_ADC == true
    unsigned long fifostart = micros();
#endif

    if (I2S0.int_st.rx_take_data) // fifo is full
    {
        readFiFo();
    }

    SET_PERI_REG_MASK(I2S_INT_CLR_REG(0), GET_PERI_REG_MASK(I2S_INT_ST_REG(0), 0xffff));
    // I2S0.int_clr.val = I2S0.int_st.val;    // Clear all interrupt flags.

#if DEBUG_ADC == true
    fifotime = micros() - fifostart;
#endif
}
#endif

// Contrary to its name (so it can be called by the library), this function reads the already converted values from fifo
// and prints optional debug information.
// When using interrupt driven sampling, it only prints debug information.
void IRAM_ATTR _startADC3PinConversionLowSide()
{
#if I2S_USE_INTERRUPT != true
    if (sampleOnCommand)
    {
        readFiFo();
    }
#endif

#if DEBUG_ADC == true
    skipped++;
    if (skipped >= 1000)
    {
        skipped = 0;
        unsigned long now = micros();
        volatile uint32_t interr = intcnt;
        volatile uint32_t samplesss = readscnt;
        intcnt = 0;
        readscnt = 0;

        float readspersec = (1000000.0f * interr) / (now - ts);
        float samplespersec = (1000000.0f * samplesss) / (now - ts);

        ts = now;
        Serial.printf(">ips:%f\n", readspersec);
        Serial.printf(">sps:%f\n", samplespersec); // readspersec * GET_PERI_REG_BITS2(I2S_FIFO_CONF_REG(0), I2S_RX_DATA_NUM_M, I2S_RX_DATA_NUM_S)); //I2S0.fifo_conf.rx_data_num
        Serial.printf(">fifo:%ld\n", fifotime);
        Serial.printf(">doubles:%ld\n", equal);
        // if(equal > 0){
        //   volatile uint32_t aktuell = currfifo;
        //   volatile uint32_t zuletzt = lastfifo;

        // Serial.printf("\n|\nlast: ");
        // for (int i = 31; i >= 15; i--)
        // {
        //   Serial.printf("%d",(zuletzt >> i ) & 1);
        // }
        // Serial.printf("  ");
        // for (int i = 15; i >= 0; i--)
        // {
        //   Serial.printf("%d",(zuletzt >> i ) & 1);
        // }

        // Serial.printf("\ncurr: ");
        // for (int i = 31; i >= 15; i--)
        // {
        //   Serial.printf("%d",(aktuell >> i ) & 1);
        // }
        // Serial.printf("  ");
        // for (int i = 15; i >= 0; i--)
        // {
        //   Serial.printf("%d",(aktuell >> i ) & 1);
        // }
        // Serial.printf("\n");

        // }

        for (size_t i = 0; i < ADC1_CHANNEL_MAX; i++)
        {
            Serial.printf(">Channel%d:%d\n", i, i2s_adc_buffer[i]);
        }
    }
#endif
}

void IRAM_ATTR _startADC3PinConversionInline(){
    _startADC3PinConversionLowSide();
}

// Takes the buffered adc counts and returns the coresponding float voltage for a pin.
float IRAM_ATTR _readADCVoltageI2S(const int pin, const void *cs_params)
{
    float adc_voltage_conv = ((ESP32MCPWMCurrentSenseParams *)cs_params)->adc_voltage_conv;

    return i2s_adc_buffer[digitalPinToAnalogChannel(pin)] * adc_voltage_conv;
}

// Sets up the I2S peripheral and ADC. Can be run multiple times to configure multiple motors.
void* IRAM_ATTR _configureI2S(const bool lowside, const void* driver_params, const int pinA, const int pinB, const int pinC){
  sampleOnCommand = !lowside;

  mcpwm_unit_t unit = ((ESP32MCPWMDriverParams*)driver_params)->mcpwm_unit;

  if( _isset(pinA) ) channels[digitalPinToAnalogChannel(pinA)] = 1;// pinMode(pinA, GPIO_MODE_DISABLE);
  if( _isset(pinB) ) channels[digitalPinToAnalogChannel(pinB)] = 1; //pinMode(pinB, GPIO_MODE_DISABLE);
  if( _isset(pinC) ) channels[digitalPinToAnalogChannel(pinC)] = 1;// pinMode(pinC, GPIO_MODE_DISABLE);

  int activeChannels = 0;
  u_int some_channel = 0;
  for (int i = 0; i < ADC1_CHANNEL_MAX; i++)
  {
      activeChannels += channels[i];
      if (channels[i])
      {
          some_channel = channels[i];
      }
  }
  globalActiveChannels = activeChannels;

  ESP32MCPWMCurrentSenseParams* params = new ESP32MCPWMCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION),
    .mcpwm_unit = unit
  };

  periph_module_reset(PERIPH_I2S0_MODULE);
  periph_module_enable(PERIPH_I2S0_MODULE);

//   ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_1, (adc1_channel_t) some_channel));
  ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0));

  // Taken from https://github.com/pycom/esp-idf-2.0/blob/master/components/bootloader_support/src/bootloader_random.c
  // The set values are good I guess?
  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2S0_CLK_EN);
  CLEAR_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_FORCE_START_TOP);
  CLEAR_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_START_TOP);

  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 3, SENS_FORCE_XPD_SAR_S);
  SET_PERI_REG_MASK(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DIG_FORCE);
  CLEAR_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DIG_FORCE);
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR2_MUX);
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR_CLK_DIV, 4, SYSCON_SARADC_SAR_CLK_DIV_S);
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAMPLE_CYCLE, 16, SYSCON_SARADC_SAMPLE_CYCLE_S);

  SET_PERI_REG_BITS(SYSCON_SARADC_FSM_REG, SYSCON_SARADC_RSTB_WAIT, 8, SYSCON_SARADC_RSTB_WAIT_S);
  SET_PERI_REG_BITS(SYSCON_SARADC_FSM_REG, SYSCON_SARADC_START_WAIT, 10, SYSCON_SARADC_START_WAIT_S);
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_WORK_MODE, 0, SYSCON_SARADC_WORK_MODE_S);
  CLEAR_PERI_REG_MASK(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR_SEL);
  CLEAR_PERI_REG_MASK(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_DATA_SAR_SEL);

  SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_RX_BCK_DIV_NUM, 20, I2S_RX_BCK_DIV_NUM_S);

  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL_REG,SYSCON_SARADC_DATA_TO_I2S);
  CLEAR_PERI_REG_MASK(I2S_CONF2_REG(0), I2S_CAMERA_EN);
  SET_PERI_REG_MASK(I2S_CONF2_REG(0), I2S_LCD_EN);
  SET_PERI_REG_MASK(I2S_CONF2_REG(0), I2S_DATA_ENABLE);
  SET_PERI_REG_MASK(I2S_CONF_REG(0), I2S_RX_START);
  // End

  I2S0.conf_chan.rx_chan_mod = 2;

  I2S0.fifo_conf.rx_data_num = max(2, min(63, (globalActiveChannels * BUF_LEN) / 2));
  I2S0.fifo_conf.dscr_en = 0; // disable dma transfer.

  I2S0.int_ena.val = 0;

#if I2S_USE_INTERRUPT == true
  I2S0.int_ena.rx_take_data = 1;
// I2S0.int_ena.rx_rempty = 1;
#endif

    // ADC setting
    int channelnum = 0;
    for (size_t i = 0; i < ADC1_CHANNEL_MAX; i++)
    {
        if (channelnum <= 4 && channels[i] == 1)
        {
            SYSCON.saradc_sar1_patt_tab[0] |= ((i << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_11) << (3 - channelnum) * 8;
            channelnum++;
        }
        else if (channelnum > 4 && channels[i] == 1)
        {
            SYSCON.saradc_sar1_patt_tab[1] |= ((i << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_11) << (3 - channelnum) * 8;
            channelnum++;
        }
    }

  // Scan multiple channels.
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, channelnum - 1, SYSCON_SARADC_SAR1_PATT_LEN_S);

  if (!running)
  {
#if I2S_USE_INTERRUPT == true
      ESP_ERROR_CHECK(esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_IRAM, i2s_isr, NULL, NULL));
#endif

      running = true;
  }

  return params;
}

#endif