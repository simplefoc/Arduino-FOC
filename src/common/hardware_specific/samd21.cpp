#include "samd_mcu.h"

#ifdef _SAMD21_

#if defined __SAMD21J18A__
#define D_TC(TCC_VALUE) TCC_VALUE
#define D_AIN(AIN_VALUE) AIN_VALUE
#else
#define   TC6_CH0  = (6<<8)|(0)
#define   TC6_CH1  = (6<<8)|(1)
#define   TC7_CH0  = (7<<8)|(0)
#define   TC7_CH1  = (7<<8)|(1)
#define   D_TC(TCC_VALUE) NOT_ON_TIMER

#define   ADC_Channel8=8
#define   ADC_Channel9=9
#define   ADC_Channel10=10
#define   ADC_Channel11=11
#define   ADC_Channel12=12
#define   ADC_Channel13=13
#define   ADC_Channel14=14
#define   ADC_Channel15=15
#define   D_AIN(AIN_VALUE) No_ADC_Channel
#endif

#if defined __SAMD21J18A__
#define D_TC(TCC_VALUE) TCC_VALUE
#define D_AIN(AIN_VALUE) AIN_VALUE
#else
#define   TC6_CH0  = (6<<8)|(0)
#define   TC6_CH1  = (6<<8)|(1)
#define   TC7_CH0  = (7<<8)|(0)
#define   TC7_CH1  = (7<<8)|(1)
#define   D_TC(TCC_VALUE) NOT_ON_TIMER

#define   ADC_Channel8=8
#define   ADC_Channel9=9
#define   ADC_Channel10=10
#define   ADC_Channel11=11
#define   ADC_Channel12=12
#define   ADC_Channel13=13
#define   ADC_Channel14=14
#define   ADC_Channel15=15
#define   D_AIN(AIN_VALUE) No_ADC_Channel
#endif

const int8_t g_SamdMapPortPin[2][32] = 
{
//        0 ,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
/*PORTA*/{0 ,  1,  2,  3, 10, 11, 12, 13, 14, 15, 16, 17, 24, 25, 26, 27, 28, 29, 30, 31, 34, 35, 36, 37, 38, 39, -1, 42, 43, -1, 44, 45},
/*PORTB*/{48, 49, 50, 51,  4,  5,  6,  7,  8,  9, 18, 19, 20, 21, 22, 23, 32, 33, -1, -1, -1, -1, 40, 41, -1, -1, -1, -1, -1, -1, 46, 47}
};


const SamdPinDefinition g_SamdPinDefinitions[] = {                                                                                                         
{  1,  1 ,  1 ,  PORTA, 0 , VDDANA, NO_Type,  EXTERNAL_INT_0   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_1  , PAD_0    ,  TCC2_CH0      ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{  2,  2 ,  2 ,  PORTA, 1 , VDDANA, NO_Type,  EXTERNAL_INT_1   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_1  , PAD_1    ,  TCC2_CH1      ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{  3,  3 ,  3 ,  PORTA, 2 , VDDANA, NO_Type,  EXTERNAL_INT_2   , VOUT    ,  ADC_Channel0        ,  No_ADC_Channel  ,  PTC_Channel_Y0   , NO_SERCOM , NO_PAD, NO_SERCOM , NO_PAD   ,  NOT_ON_TIMER  ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{  4,  4 ,  4 ,  PORTA, 3 , VDDANA, NO_Type,  EXTERNAL_INT_3   , VREFA   ,  ADC_Channel1        ,  No_ADC_Channel  ,  PTC_Channel_Y1   , NO_SERCOM , NO_PAD, NO_SERCOM , NO_PAD   ,  NOT_ON_TIMER  ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{ -1, -1 ,  5 ,  PORTB, 4 , VDDANA, NO_Type,  EXTERNAL_INT_4   , No_VREF ,  D_AIN(ADC_Channel12),  No_ADC_Channel  ,  PTC_Channel_Y10  , NO_SERCOM , NO_PAD, NO_SERCOM , NO_PAD   ,  NOT_ON_TIMER  ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{ -1, -1 ,  6 ,  PORTB, 5 , VDDANA, NO_Type,  EXTERNAL_INT_5   , No_VREF ,  D_AIN(ADC_Channel13),  No_ADC_Channel  ,  PTC_Channel_Y11  , NO_SERCOM , NO_PAD, NO_SERCOM , NO_PAD   ,  NOT_ON_TIMER  ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{ -1, -1 ,  9 ,  PORTB, 6 , VDDANA, NO_Type,  EXTERNAL_INT_6   , No_VREF ,  D_AIN(ADC_Channel14),  No_ADC_Channel  ,  PTC_Channel_Y12  , NO_SERCOM , NO_PAD, NO_SERCOM , NO_PAD   ,  NOT_ON_TIMER  ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{ -1, -1 ,  10,  PORTB, 7 , VDDANA, NO_Type,  EXTERNAL_INT_7   , No_VREF ,  D_AIN(ADC_Channel15),  No_ADC_Channel  ,  PTC_Channel_Y13  , NO_SERCOM , NO_PAD, NO_SERCOM , NO_PAD   ,  NOT_ON_TIMER  ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{ -1,  7 ,  11,  PORTB, 8 , VDDANA, NO_Type,  EXTERNAL_INT_8   , No_VREF ,  ADC_Channel2        ,  No_ADC_Channel  ,  PTC_Channel_Y14  , NO_SERCOM , NO_PAD, SERCOM_4  , PAD_0    ,  TC4_CH0       ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{ -1,  8 ,  12,  PORTB, 9 , VDDANA, NO_Type,  EXTERNAL_INT_9   , No_VREF ,  ADC_Channel3        ,  No_ADC_Channel  ,  PTC_Channel_Y15  , NO_SERCOM , NO_PAD, SERCOM_4  , PAD_1    ,  TC4_CH1       ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{  5,  9 ,  13,  PORTA, 4 , VDDANA, NO_Type,  EXTERNAL_INT_4   , VREFB   ,  ADC_Channel4        ,  ADC_Channel0    ,  PTC_Channel_Y2   , NO_SERCOM , NO_PAD, SERCOM_0  , PAD_0    ,  TCC0_CH0      ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{  6,  10,  14,  PORTA, 5 , VDDANA, NO_Type,  EXTERNAL_INT_5   , No_VREF ,  ADC_Channel5        ,  ADC_Channel1    ,  PTC_Channel_Y3   , NO_SERCOM , NO_PAD, SERCOM_0  , PAD_1    ,  TCC0_CH1      ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{  7,  11,  15,  PORTA, 6 , VDDANA, NO_Type,  EXTERNAL_INT_6   , No_VREF ,  ADC_Channel6        ,  ADC_Channel2    ,  PTC_Channel_Y4   , NO_SERCOM , NO_PAD, SERCOM_0  , PAD_2    ,  TCC1_CH0      ,   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{  8,  12,  16,  PORTA, 7 , VDDANA, NO_Type,  EXTERNAL_INT_7   , No_VREF ,  ADC_Channel7        ,  ADC_Channel3    ,  PTC_Channel_Y5   , NO_SERCOM , NO_PAD, SERCOM_0  , PAD_3    ,  TCC1_CH1      ,   NOT_ON_TIMER ,  I2S_SD0    ,    NO_GCLK},
{ 11,  13,  17,  PORTA, 8 , VDDIO , NO_Type,  EXTERNAL_INT_NMI , No_VREF ,  ADC_Channel16       ,  No_ADC_Channel  ,  PTC_Channel_X0   , SERCOM_0  , PAD_0 , SERCOM_2  , PAD_0    ,  TCC0_CH0      ,   TCC1_CH2     ,  I2S_SD1    ,    NO_GCLK},
{ 12,  14,  18,  PORTA, 9 , VDDIO , NO_Type,  EXTERNAL_INT_9   , No_VREF ,  ADC_Channel17       ,  No_ADC_Channel  ,  PTC_Channel_X1   , SERCOM_0  , PAD_1 , SERCOM_2  , PAD_1    ,  TCC1_CH3      ,   TCC0_CH1     ,  I2S_MCK0   ,    NO_GCLK},
{ 13,  15,  19,  PORTA, 10, VDDIO , NO_Type,  EXTERNAL_INT_10  , No_VREF ,  ADC_Channel18       ,  No_ADC_Channel  ,  PTC_Channel_X2   , SERCOM_0  , PAD_2 , SERCOM_2  , PAD_2    ,  TCC0_CH2      ,   TCC1_CH0     ,  I2S_SCK0   ,    GCLK_IO_4},
{ 14,  16,  20,  PORTA, 11, VDDIO , NO_Type,  EXTERNAL_INT_11  , No_VREF ,  ADC_Channel19       ,  No_ADC_Channel  ,  PTC_Channel_X3   , SERCOM_0  , PAD_3 , SERCOM_2  , PAD_3    ,  TCC1_CH1      ,   NOT_ON_TIMER ,  I2S_FS0    ,    GCLK_IO_5},
{ -1,  19,  23,  PORTB, 10, VDDIO , NO_Type,  EXTERNAL_INT_10  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_4  , PAD_2    ,  TC5_CH0       ,   TCC0_CH4     ,  I2S_MCK1   ,    GCLK_IO_4},
{ -1,  20,  24,  PORTB, 11, VDDIO , NO_Type,  EXTERNAL_INT_11  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_4  , PAD_3    ,  TC5_CH1       ,   TCC0_CH5     ,  I2S_SCK1   ,    GCLK_IO_5},
{ -1,  -1,  25,  PORTB, 12, VDDIO , I2C    ,  EXTERNAL_INT_12  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X12  , SERCOM_4  , PAD_0 , NO_SERCOM , NO_PAD   ,  TC4_CH0       ,   TCC0_CH6     ,  I2S_FS1    ,    GCLK_IO_6},
{ -1,  -1,  26,  PORTB, 13, VDDIO , I2C    ,  EXTERNAL_INT_13  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X13  , SERCOM_4  , PAD_1 , NO_SERCOM , NO_PAD   ,  TC4_CH1       ,   TCC0_CH7     ,  NO_COM     ,    GCLK_IO_7},
{ -1,  -1,  27,  PORTB, 14, VDDIO , NO_Type,  EXTERNAL_INT_14  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X14  , SERCOM_4  , PAD_2 , NO_SERCOM , NO_PAD   ,  TC5_CH0       ,   NOT_ON_TIMER ,  NO_COM     ,    GCLK_IO_0},
{ -1,  -1,  28,  PORTB, 15, VDDIO , NO_Type,  EXTERNAL_INT_15  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X15  , SERCOM_4  , PAD_3 , NO_SERCOM , NO_PAD   ,  TC5_CH1       ,   NOT_ON_TIMER ,  NO_COM     ,    GCLK_IO_1},
{ -1,  21,  29,  PORTA, 12, VDDIO , I2C    ,  EXTERNAL_INT_12  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , SERCOM_2  , PAD_0 , SERCOM_4  , PAD_0    ,  TCC2_CH0      ,   TCC0_CH6     ,  NO_COM     ,    AC_CMP_0},
{ -1,  22,  30,  PORTA, 13, VDDIO , I2C    ,  EXTERNAL_INT_13  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , SERCOM_2  , PAD_1 , SERCOM_4  , PAD_1    ,  TCC2_CH1      ,   TCC0_CH7     ,  NO_COM     ,    AC_CMP_1},
{ 15,  23,  31,  PORTA, 14, VDDIO , NO_Type,  EXTERNAL_INT_14  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , SERCOM_2  , PAD_2 , SERCOM_4  , PAD_2    ,  TC3_CH0       ,   TCC0_CH4     ,  NO_COM     ,    GCLK_IO_0},
{ 16,  24,  32,  PORTA, 15, VDDIO , NO_Type,  EXTERNAL_INT_15  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , SERCOM_2  , PAD_3 , SERCOM_4  , PAD_3    ,  TC3_CH1       ,   TCC0_CH5     ,  NO_COM     ,    GCLK_IO_1},
{ 17,  25,  35,  PORTA, 16, VDDIO , I2C    ,  EXTERNAL_INT_0   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X4   , SERCOM_1  , PAD_0 , SERCOM_3  , PAD_0    ,  TCC2_CH0      ,   TCC0_CH6     ,  NO_COM     ,    GCLK_IO_2},
{ 18,  26,  36,  PORTA, 17, VDDIO , I2C    ,  EXTERNAL_INT_1   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X5   , SERCOM_1  , PAD_1 , SERCOM_3  , PAD_1    ,  TCC2_CH1      ,   TCC0_CH7     ,  NO_COM     ,    GCLK_IO_3},
{ 19,  27,  37,  PORTA, 18, VDDIO , NO_Type,  EXTERNAL_INT_2   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X6   , SERCOM_1  , PAD_2 , SERCOM_3  , PAD_2    ,  TC3_CH0       ,   TCC0_CH2     ,  NO_COM     ,    AC_CMP_0},
{ 20,  28,  38,  PORTA, 19, VDDIO , NO_Type,  EXTERNAL_INT_3   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X7   , SERCOM_1  , PAD_3 , SERCOM_3  , PAD_3    ,  TC3_CH1       ,   TCC0_CH3     ,  I2S_SD0    ,    AC_CMP_1},
{ -1,  -1,  39,  PORTB, 16, VDDIO , I2C    ,  EXTERNAL_INT_0   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , SERCOM_5  , PAD_0 , NO_SERCOM , NO_PAD   ,  D_TC(TC6_CH0),   TCC0_CH4     ,  I2S_SD1    ,    GCLK_IO_2},
{ -1,  -1,  40,  PORTB, 17, VDDIO , I2C    ,  EXTERNAL_INT_1   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , SERCOM_5  , PAD_1 , NO_SERCOM , NO_PAD   ,  D_TC(TC6_CH1),   TCC0_CH5     ,  I2S_MCK0   ,    GCLK_IO_3},
{ -1,  29,  41,  PORTA, 20, VDDIO , NO_Type,  EXTERNAL_INT_4   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X8   , SERCOM_5  , PAD_2 , SERCOM_3  , PAD_2    ,  D_TC(TC7_CH0),   TCC0_CH6     ,  I2S_SCK0   ,    GCLK_IO_4},
{ -1,  30,  42,  PORTA, 21, VDDIO , NO_Type,  EXTERNAL_INT_5   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X9   , SERCOM_5  , PAD_3 , SERCOM_3  , PAD_3    ,  D_TC(TC7_CH1),   TCC0_CH7     ,  I2S_FS0    ,    GCLK_IO_5},
{ 21,  31,  43,  PORTA, 22, VDDIO , I2C    ,  EXTERNAL_INT_6   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X10  , SERCOM_3  , PAD_0 , SERCOM_5  , PAD_0    ,  TC4_CH0       ,   TCC0_CH4     ,  NO_COM     ,    GCLK_IO_6},
{ 22,  32,  44,  PORTA, 23, VDDIO , I2C    ,  EXTERNAL_INT_7   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  PTC_Channel_X11  , SERCOM_3  , PAD_1 , SERCOM_5  , PAD_1    ,  TC4_CH1       ,   TCC0_CH5     ,  USB_SOF1kHz,    GCLK_IO_7},
{ 23,  33,  45,  PORTA, 24, VDDIO , NO_Type,  EXTERNAL_INT_12  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , SERCOM_3  , PAD_2 , SERCOM_5  , PAD_2    ,  TC5_CH0       ,   TCC1_CH2     ,  USB_DM     ,    NO_GCLK},
{ 24,  34,  46,  PORTA, 25, VDDIO , NO_Type,  EXTERNAL_INT_13  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , SERCOM_3  , PAD_3 , SERCOM_5  , PAD_3    ,  TC5_CH1       ,   TCC1_CH3     ,  USB_DP     ,    NO_GCLK},
{ -1,  37,  49,  PORTB, 22, VDDIO , NO_Type,  EXTERNAL_INT_6   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_5  , PAD_2    ,  D_TC(TC7_CH0),   NOT_ON_TIMER ,  NO_COM     ,    GCLK_IO_0},
{ -1,  38,  50,  PORTB, 23, VDDIO , NO_Type,  EXTERNAL_INT_7   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_5  , PAD_3    ,  D_TC(TC7_CH1),   NOT_ON_TIMER ,  NO_COM     ,    GCLK_IO_1},
{ 25,  39,  51,  PORTA, 27, VDDIO , NO_Type,  EXTERNAL_INT_15  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, NO_SERCOM , NO_PAD   ,  NOT_ON_TIMER  ,   NOT_ON_TIMER ,  NO_COM     ,    GCLK_IO_0},
{ 27,  41,  53,  PORTA, 28, VDDIO , NO_Type,  EXTERNAL_INT_8   , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, NO_SERCOM , NO_PAD   ,  NOT_ON_TIMER  ,   NOT_ON_TIMER ,  NO_COM     ,    GCLK_IO_0},
{ 31,  45,  57,  PORTA, 30, VDDIO , NO_Type,  EXTERNAL_INT_10  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_1  , PAD_2    ,  TCC1_CH0      ,   NOT_ON_TIMER ,  SWCLK      ,    GCLK_IO_0},
{ 32,  46,  58,  PORTA, 31, VDDIO , NO_Type,  EXTERNAL_INT_11  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_1  , PAD_3    ,  TCC1_CH1      ,   NOT_ON_TIMER ,  SWDIO      ,    NO_GCLK},
{ -1,  -1,  59,  PORTB, 30, VDDIO , I2C    ,  EXTERNAL_INT_14  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_5  , PAD_0    ,  TCC0_CH0      ,   TCC1_CH2     ,  NO_COM     ,    NO_GCLK},
{ -1,  -1,  60,  PORTB, 31, VDDIO , I2C    ,  EXTERNAL_INT_15  , No_VREF ,  No_ADC_Channel      ,  No_ADC_Channel  ,  No_PTC_Channel   , NO_SERCOM , NO_PAD, SERCOM_5  , PAD_1    ,  TCC0_CH1      ,   TCC1_CH3     ,  NO_COM     ,    NO_GCLK},
{ -1,  -1,  61,  PORTB, 0, VDDANA, NO_Type ,  EXTERNAL_INT_0   , No_VREF ,  D_AIN(ADC_Channel8) ,  No_ADC_Channel  ,  PTC_Channel_Y6   , NO_SERCOM , NO_PAD, SERCOM_5  , PAD_2     ,  D_TC(TC7_CH0),   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{ -1,  -1,  62,  PORTB, 1, VDDANA, NO_Type ,  EXTERNAL_INT_1   , No_VREF ,  D_AIN(ADC_Channel9) ,  No_ADC_Channel  ,  PTC_Channel_Y7   , NO_SERCOM , NO_PAD, SERCOM_5  , PAD_3     ,  D_TC(TC7_CH1),   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{ -1,  47,  63,  PORTB, 2, VDDANA, NO_Type ,  EXTERNAL_INT_2   , No_VREF ,  D_AIN(ADC_Channel10),  No_ADC_Channel  ,  PTC_Channel_Y8   , NO_SERCOM , NO_PAD, SERCOM_5  , PAD_0     ,  D_TC(TC6_CH0),   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
{ -1,  48,  64,  PORTB, 3, VDDANA, NO_Type ,  EXTERNAL_INT_3   , No_VREF ,  D_AIN(ADC_Channel11),  No_ADC_Channel  ,  PTC_Channel_Y9   , NO_SERCOM , NO_PAD, SERCOM_5  , PAD_1     ,  D_TC(TC6_CH1),   NOT_ON_TIMER ,  NO_COM     ,    NO_GCLK},
};               


SercomConfig getSercom(SercomChannel channel)
{
    switch(channel)
    {
        case SERCOM_0: return {.sercom =SERCOM0, .irq = SERCOM0_IRQn, .clockId = GCM_SERCOM0_CORE};
        case SERCOM_1: return {.sercom =SERCOM1, .irq = SERCOM1_IRQn, .clockId = GCM_SERCOM1_CORE}; 
        case SERCOM_2: return {.sercom =SERCOM2, .irq = SERCOM2_IRQn, .clockId = GCM_SERCOM2_CORE}; 
        case SERCOM_3: return {.sercom =SERCOM3, .irq = SERCOM3_IRQn, .clockId = GCM_SERCOM3_CORE}; 
        case SERCOM_4: return {.sercom =SERCOM4, .irq = SERCOM4_IRQn, .clockId = GCM_SERCOM4_CORE}; 
        case SERCOM_5: return {.sercom =SERCOM5, .irq = SERCOM5_IRQn, .clockId = GCM_SERCOM5_CORE};
        default:
        return {.sercom = nullptr, .irq = PendSV_IRQn, .clockId = 0}; 
    }
}


bool g_EVSYSChannelInitialized[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

int initEVSYS(uint8_t evsysChannel, uint16_t EVSYS_ID_USER, uint16_t EVSYS_ID_GEN, uint16_t EVSYS_CHANNEL_PATH, uint16_t EVSYS_CHANNEL_EDGSEL, bool force)
{
    /* Turn on the digital interface clock */

    if(g_EVSYSChannelInitialized[evsysChannel] && !force)
    {
        debugPrintf("initEVSYS() channel %d is already initialized", evsysChannel);
        return -1;
    }

    static bool PM_APBCMASK_EVSYS_initialized = false;
    if(!PM_APBCMASK_EVSYS_initialized)
    {
	    PM->APBCMASK.bit.EVSYS_ = 0b1;
        PM_APBCMASK_EVSYS_initialized = true;
    }
    
    if(evsysChannel > EVSYS_GCLK_ID_SIZE - 1)
    {
        debugPrintf("initEVSYS() channel %d out of bounds (max %d)", evsysChannel, EVSYS_GCLK_ID_SIZE);
        return -1;
    }
    if(EVSYS_ID_USER > EVSYS_ID_USER_PTC_STCONV)
    {
        debugPrintf("initEVSYS() EVSYS_ID_USER %d out of bounds (max %d)", EVSYS_ID_USER, EVSYS_ID_USER_PTC_STCONV);
        return -1;
    }
    if(EVSYS_ID_GEN > EVSYS_ID_GEN_PTC_WCOMP)
    {
        debugPrintf("initEVSYS() EVSYS_ID_GEN %d out of bounds (max %d)", EVSYS_ID_GEN, EVSYS_ID_GEN_PTC_WCOMP);
        return -1;
    }

    if(EVSYS_CHANNEL_PATH == EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val && EVSYS_CHANNEL_EDGSEL != EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val)
    {
        EVSYS_CHANNEL_EDGSEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val;
        debugPrintf("initEVSYS() warning, channel %d is configured with asynchronous path, forcing EDGSEL to 'NO_EVT_OUTPUT', note that interrupts aren't supported with that path", evsysChannel);
    }

    g_EVSYSChannelInitialized[evsysChannel] = true;

    /* Turn on the peripheral interface clock and select GCLK */
    GCLK_CLKCTRL_Type clkctrl;
    clkctrl.bit.WRTLOCK = 0;
    clkctrl.bit.CLKEN = 1;
    clkctrl.bit.ID = EVSYS_GCLK_ID_LSB + evsysChannel; //enable clock for channel 0
    clkctrl.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val; /* GCLK_GENERATOR_0 */
    GCLK->CLKCTRL.reg = clkctrl.reg;

    // event user (destination)
    EVSYS_USER_Type user;
    user.bit.CHANNEL = evsysChannel + 1; /* use channel 0 p421: "Note that to select channel n, the value (n+1) must be written to the USER.CHANNEL bit group." */
    user.bit.USER = EVSYS_ID_USER; 
    EVSYS->USER.reg = user.reg;


    // event generator (source)
	EVSYS_CHANNEL_Type channel;
	channel.bit.EDGSEL = EVSYS_CHANNEL_EDGSEL;
	channel.bit.PATH = EVSYS_CHANNEL_PATH; //--> EVSYS_ID_USER_ADC_SYNC: Asynchronous path only
	channel.bit.EVGEN = EVSYS_ID_GEN;
	channel.bit.SWEVT = 0b0;   /* no software trigger */
	channel.bit.CHANNEL = evsysChannel;
	EVSYS->CHANNEL.reg = channel.reg;


    return 0;


}

static DmacDescriptor descriptor_section[12] __attribute__ ((aligned (16)));
static volatile DmacDescriptor write_back[12] __attribute__ ((aligned (16)));
void initDMAC()
{
    // probably on by default
    PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
    PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;

    DMAC->BASEADDR.reg = (uint32_t)descriptor_section; // Descriptor Base Memory Address
    DMAC->WRBADDR.reg = (uint32_t)write_back; //Write-Back Memory Base Address


    DMAC_PRICTRL0_Type prictrl0{.reg = 0};

    prictrl0.bit.RRLVLEN0 = 0b1; //enable round-robin
    prictrl0.bit.RRLVLEN1 = 0b1; //enable round-robin
    prictrl0.bit.RRLVLEN2 = 0b1; //enable round-robin
    prictrl0.bit.RRLVLEN3 = 0b1; //enable round-robin

    DMAC->PRICTRL0.reg = prictrl0.reg;

    DMAC_CTRL_Type ctrl{.reg = 0};
    ctrl.bit.DMAENABLE = 0b1;
    ctrl.bit.LVLEN0 = 0b1;
    ctrl.bit.LVLEN1 = 0b1;
    ctrl.bit.LVLEN2 = 0b1;
    ctrl.bit.LVLEN3 = 0b1;
    ctrl.bit.CRCENABLE = 0b0;

    DMAC->CTRL.reg = ctrl.reg;

    NVIC_EnableIRQ( DMAC_IRQn ) ;
}

bool g_DMACChannelInitialized[] = {false, false, false, false, false, false, false, false, false, false, false, false};

static DMACInterruptCallback * DMAC_Handlers[12] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

int initDMAChannel(uint8_t channel, DMAC_CHINTENSET_Type chintset, DMAC_CHCTRLB_Type chctrlb, const DmacDescriptor & descriptor, DMACInterruptCallback * interrupt_handler, bool force) 
{

    if(channel > 11)
    {
        debugPrintf("initDMAChannel() channel %d out of bounds (max %d)", channel, 12);
        return -1;
    }

    if(g_DMACChannelInitialized[channel] && !force)
    {
        debugPrintf("initDMAChannel() channel %d is already initialized", channel);
        return -1;
    }

    g_DMACChannelInitialized[channel] = true;
    //select the channel
    DMAC->CHID.bit.ID = channel;

    // disable and reset the channel
    DMAC->CHCTRLA.bit.ENABLE = 0b0; //must be done **before** SWRST
    DMAC->CHCTRLA.bit.SWRST = 0b1;

    DMAC->CHINTENSET.reg = chintset.reg ; // enable all 3 interrupts
    DMAC->CHCTRLB.reg = chctrlb.reg;

    memcpy(&descriptor_section[channel], &descriptor, sizeof(DmacDescriptor));

    DMAC_Handlers[channel] = interrupt_handler;

    // start channel
    DMAC->CHCTRLA.bit.ENABLE = 0b1;


    return 0;
}


void trigDMACChannel(uint8_t channel)
{

  switch (channel)
  {
  case 0: DMAC->SWTRIGCTRL.bit.SWTRIG0 = 0b1; break;
  case 1: DMAC->SWTRIGCTRL.bit.SWTRIG1 = 0b1; break;
  case 2: DMAC->SWTRIGCTRL.bit.SWTRIG2 = 0b1; break;
  case 3: DMAC->SWTRIGCTRL.bit.SWTRIG3 = 0b1; break;
  case 4: DMAC->SWTRIGCTRL.bit.SWTRIG4 = 0b1; break;
  case 5: DMAC->SWTRIGCTRL.bit.SWTRIG5 = 0b1; break;
  case 6: DMAC->SWTRIGCTRL.bit.SWTRIG6 = 0b1; break;
  case 7: DMAC->SWTRIGCTRL.bit.SWTRIG7 = 0b1; break;
  case 8: DMAC->SWTRIGCTRL.bit.SWTRIG8 = 0b1; break;
  case 9: DMAC->SWTRIGCTRL.bit.SWTRIG9 = 0b1; break;
  case 10: DMAC->SWTRIGCTRL.bit.SWTRIG10 = 0b1; break;
  case 11: DMAC->SWTRIGCTRL.bit.SWTRIG11 = 0b1; break;

  default:
    debugPrintf("Error in adcToDMATransfer() [DMAC] Bad channel number %d");
  }
}


void DMAC_Handler() {

  DMAC->CHID.bit.ID = DMAC->INTPEND.bit.ID;

  DMACInterruptCallback *interrupt_handler = DMAC_Handlers[DMAC->CHID.bit.ID];

  if(interrupt_handler != nullptr)
    (*interrupt_handler)(DMAC->CHINTFLAG, DMAC->CHCTRLA);
}

typedef struct _InerruptHandlers
{
    uint8_t count = 0;
    TccInterruptCallback* handlers[MAX_HANDLERS];
} InerruptHandlers;

static InerruptHandlers TCC_Handlers[TCC_INST_NUM];


Tcc * addTCCHandler(uint8_t tccn, TccInterruptCallback * interrupt_handler)
{

    if(interrupt_handler == nullptr)
    {
        debugPrintln("initTCCHandler() Error: illegal nullptr handler");
        return nullptr;
    }

    if(tccn >= TCC_INST_NUM)
    {
        debugPrintf("initTCCHandler() Error: tccn %d is out of bounds (only &d TCC units)", tccn, TCC_INST_NUM);
        return nullptr;
    }

    if(TCC_Handlers[tccn].count < MAX_HANDLERS)
    {

        TCC_Handlers[tccn].handlers[TCC_Handlers[0].count] = interrupt_handler;
        TCC_Handlers[tccn].count++;
        switch(tccn)
        {
            case 0: if(TCC_Handlers[tccn].count == 0) NVIC_EnableIRQ( TCC0_IRQn ) ; return TCC0;
            case 1: if(TCC_Handlers[tccn].count == 0) NVIC_EnableIRQ( TCC1_IRQn ) ; return TCC1;
            case 2: if(TCC_Handlers[tccn].count == 0) NVIC_EnableIRQ( TCC2_IRQn ) ; return TCC2;
            default: return nullptr;
        }
        return 0;
    }
    // else
    debugPrintf("initTCCHandler() Error: maximum number of handlers (%d) reached for TCC (%d)", MAX_HANDLERS, tccn);
    return nullptr;
}

#define TCCN_Handler(tccn, tcc)\ 
    for(int i = 0; i < TCC_Handlers[tccn].count; i++)\
        (*TCC_Handlers[tccn].handlers[i])(tcc);


void TCC0_Handler() { TCCN_Handler(0, TCC0); }
void TCC1_Handler() { TCCN_Handler(1, TCC1); }
void TCC2_Handler() { TCCN_Handler(2, TCC2); }

#endif //_SAMD21_
