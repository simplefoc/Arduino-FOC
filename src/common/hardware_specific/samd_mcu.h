#pragma once
#include <Arduino.h>


typedef enum
{
	SPI_PAD_0_SCK_1_SS_2 = 0,
	SPI_PAD_2_SCK_3_SS_1,
	SPI_PAD_3_SCK_1_SS_2,
	SPI_PAD_0_SCK_3_SS_1
} SercomSpiDOPO;

typedef enum
{
    NO_SERCOM = -1,
	SERCOM_0 = 0,
	SERCOM_1,
	SERCOM_2,
	SERCOM_3,
    SERCOM_4,
    SERCOM_5,
} SercomChannel;

typedef struct _SercomConfig
{
    Sercom * sercom;
    IRQn_Type irq;
    uint8_t clockId;
}  SercomConfig;

SercomConfig getSercom(SercomChannel channel);

typedef enum
{
    NO_PAD = -1,
	PAD_0 = 0,
	PAD_1,
	PAD_2,
	PAD_3,
} SercomPad;

typedef enum
{
    No_VREF = -1,
    VREFA = 0,
    VREFB,
    VOUT,
} VRef;

typedef enum
{
  No_PTC_Channel=-1,
  PTC_Channel_X0 =0x00 + 0,
  PTC_Channel_X1 =0x00 + 1,
  PTC_Channel_X2 =0x00 + 2,
  PTC_Channel_X3 =0x00 + 3,
  PTC_Channel_X4 =0x00 + 4,
  PTC_Channel_X5 =0x00 + 5,
  PTC_Channel_X6 =0x00 + 6,
  PTC_Channel_X7 =0x00 + 7,
  PTC_Channel_X8 =0x00 + 8,
  PTC_Channel_X9 =0x00 + 9,
  PTC_Channel_X10=0x00 + 10,
  PTC_Channel_X11=0x00 + 11,
  PTC_Channel_X12=0x00 + 12,
  PTC_Channel_X13=0x00 + 13,
  PTC_Channel_X14=0x00 + 14,
  PTC_Channel_X15=0x00 + 15,
  PTC_Channel_Y0 =0x10 + 0,
  PTC_Channel_Y1 =0x10 + 1,
  PTC_Channel_Y2 =0x10 + 2,
  PTC_Channel_Y3 =0x10 + 3,
  PTC_Channel_Y4 =0x10 + 4,
  PTC_Channel_Y5 =0x10 + 5,
  PTC_Channel_Y6 =0x10 + 6,
  PTC_Channel_Y7 =0x10 + 7,
  PTC_Channel_Y8 =0x10 + 8,
  PTC_Channel_Y9 =0x10 + 9,
  PTC_Channel_Y10=0x10 + 10,
  PTC_Channel_Y11=0x10 + 11,
  PTC_Channel_Y12=0x10 + 12,
  PTC_Channel_Y13=0x10 + 13,
  PTC_Channel_Y14=0x10 + 14,
  PTC_Channel_Y15=0x10 + 15,
} PeripheralTouchChannel;

typedef enum
{
    NO_COM = -1,
	I2S_SD0 = 0,
    I2S_SD1,
	I2S_MCK0,
    I2S_MCK1,
	I2S_SCK0,
    I2S_SCK1,
	I2S_FS0,
    I2S_FS1,
    USB_SOF1kHz,
    USB_DM,
    USB_DP,
    SWCLK,
    SWDIO,

} ComChannel;

typedef enum 
{
    VDDANA,
    VDDIO
} VDDType;

typedef enum 
{
    NO_Type,
    I2C
} COMType;

typedef enum 
{
    NO_GCLK,
    GCLK_IO_0 = 0,
    GCLK_IO_1 = 1,
    GCLK_IO_2 = 2,
    GCLK_IO_3 = 3,
    GCLK_IO_4 = 4,
    GCLK_IO_5 = 5,
    GCLK_IO_6 = 6,
    GCLK_IO_7 = 7,
    AC_CMP_0 = 0x10 + 0, 
    AC_CMP_1 = 0x10 + 1, 
    AC_CMP_2 = 0x10 + 2, 
    AC_CMP_3 = 0x10 + 3, 
} GCLKChannel;

typedef struct _SamdPinDefinition
{
    int8_t SAMD21E_chip_pin;
    int8_t SAMD21G_chip_pin;
    int8_t SAMD21J_chip_pin;
    EPortType port;
    uint8_t port_pin;
    VDDType vdd;
    COMType is_i2c;
    EExt_Interrupts extrernal_interrupt;
    VRef vref;
    EAnalogChannel adc_channel;
    EAnalogChannel dac_channel;
    PeripheralTouchChannel ptc;
    SercomChannel sercom;
    SercomPad sercom_pad;
    SercomChannel sercom_alt;
    SercomPad sercom_pad_alt;
    ETCChannel tc_tcc;
    ETCChannel tcc;
    ComChannel com_channel;
    GCLKChannel gclk_channel;
} SamdPinDefinition;



extern const int8_t g_SamdMapPortPin[2][32];
extern const SamdPinDefinition g_SamdPinDefinitions[]; // *Warning! these pins indexed by the order of "Table 6-1. PORT Function Multiplexing" p21 of samd21 spec sheet
                                                                                                                                                                                   
const SamdPinDefinition * getSamdPinDefinition(int arduinoPin); 


#ifdef SIMPLEFOC_SAMD_DEBUG
template <typename T>
void debugPrint(T message){ SIMPLEFOC_SAMD_DEBUG_SERIAL.print(message);}
template <typename T>
void debugPrintln(T message){ SIMPLEFOC_SAMD_DEBUG_SERIAL.println(message);}
static char buffer[1000];
#define debugPrintf(args...) sprintf(buffer, args); debugPrint(buffer);
#else
#define debugPrintf(args...) ;
#define debugPrint(arg) ;
#define debugPrintln(arg) ;
#endif

SercomSpiClockMode from_SPI_MODE(int spi_mode);


extern bool g_EVSYSChannelInitialized[];
int initEVSYS(uint8_t evsysChannel, uint16_t EVSYS_ID_USER, uint16_t EVSYS_ID_GEN, uint16_t EVSYS_CHANNEL_PATH, uint16_t EVSYS_CHANNEL_EDGSEL, bool force = false);


extern bool g_DMACChannelInitialized[];
void initDMAC();
int initDMAChannel(uint8_t channel, DMAC_CHINTENSET_Type chintset, DMAC_CHCTRLB_Type chctrlb, const DmacDescriptor & descriptor, void (*interrupt_handler)(volatile DMAC_CHINTFLAG_Type &, volatile DMAC_CHCTRLA_Type &), bool force = false);
void trigDMACChannel(uint8_t channel);
uint32_t computeDSTADDR(uint8_t * startAddress, uint32_t STEPSEL, uint32_t STEPSIZE, uint32_t BEATSIZE, uint32_t BTCNT);