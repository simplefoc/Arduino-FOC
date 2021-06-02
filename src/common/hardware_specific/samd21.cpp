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

#endif //_SAMD21_
