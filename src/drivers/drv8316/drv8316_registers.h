

#ifndef SIMPLEFOC_DRV8316_REGISTERS
#define SIMPLEFOC_DRV8316_REGISTERS


#define IC_Status_ADDR   0x0
#define Status__1_ADDR   0x1
#define Status__2_ADDR   0x2
#define Control__1_ADDR  0x3
#define Control__2_ADDR  0x4
#define Control__3_ADDR  0x5
#define Control__4_ADDR  0x6
#define Control__5_ADDR  0x7
#define Control__6_ADDR  0x8
#define Control__10_ADDR 0xC

#define REG_LOCK_LOCK 0b110
#define REG_LOCK_UNLOCK 0b011
#define CLR_FAULT_CLR 0b1
#define OTW_REP_ENABLE 0b1
#define OTW_REP_DISABLE 0b0
#define SPI_FLT_REP_ENABLE 0b0
#define SPI_FLT_REP_DISABLE 0b1
#define OVP_EN_ENABLE 0b1
#define OVP_EN_DISABLE 0b0
#define OCP_CBC_ENABLE 0b1
#define OCP_CBC_DISABLE 0b0
#define DRV_OFF_ENABLE 0b1
#define DRV_OFF_DISABLE 0b0
#define EN_ASR_ENABLE 0b1
#define EN_ASR_DISABLE 0b0
#define EN_AAR_ENABLE 0b1
#define EN_AAR_DISABLE 0b0
#define BUCK_DIS_BUCK_DISABLE 0b1
#define BUCK_DIS_BUCK_ENABLE 0b0
#define BUCK_PS_DIS_DISABLE 0b1
#define BUCK_PS_DIS_ENABLE 0b0
#define DLYCMP_EN_ENABLE 0b1
#define DLYCMP_EN_DISABLE 0b0




typedef union {
	struct {
		uint8_t REG_LOCK:3;
		uint8_t :5;
	};
	uint8_t reg;
} Control__1;


typedef union {
	struct {
		uint8_t CLR_FLT:1;
		uint8_t PWM_MODE:2;
		uint8_t SLEW:2;
		uint8_t SDO_MODE:1;
		uint8_t :2;
	};
	uint8_t reg;
} Control__2;


typedef union {
	struct {
		uint8_t OTW_REP:1;
		uint8_t SPI_FLT_REP:1;
		uint8_t OVP_EN:1;
		uint8_t OVP_SEL:1;
		uint8_t PWM_100_DUTY_SEL:1;
		uint8_t :3;
	};
	uint8_t reg;
} Control__3;


typedef union {
	struct {
		uint8_t OCP_MODE:2;
		uint8_t OCP_LVL:1;
		uint8_t OCP_RETRY:1;
		uint8_t OCP_DEG:2;
		uint8_t OCP_CBC:1;
		uint8_t DRV_OFF:1;
	};
	uint8_t reg;
} Control__4;




typedef union {
	struct {
		uint8_t CSA_GAIN:2;
		uint8_t EN_ASR:1;
		uint8_t EN_AAR:1;
		uint8_t :2;
		uint8_t ILIM_RECIR:1;
		uint8_t :1;
	};
	uint8_t reg;
} Control__5;




typedef union {
	struct {
		uint8_t BUCK_DIS:1;
		uint8_t BUCK_SEL:2;
		uint8_t BUCK_CL:1;
		uint8_t BUCK_PS_DIS:1;
		uint8_t :2;
	};
	uint8_t reg;
} Control__6;




typedef union {
	struct {
		uint8_t DLY_TARGET:4;
		uint8_t DLYCMP_EN:1;
		uint8_t :3;
	};
	uint8_t reg;
} Control__10;




typedef union {
	struct {
		uint8_t FAULT:1;
		uint8_t OT:1;
		uint8_t OVP:1;
		uint8_t NPOR:1;
		uint8_t OCP:1;
		uint8_t SPI_FLT:1;
		uint8_t BK_FLT:1;
		uint8_t :1;
	};
	uint8_t reg;
} IC_Status;




typedef union {
	struct {
		uint8_t OCP_LA:1;
		uint8_t OCP_HA:1;
		uint8_t OCP_LB:1;
		uint8_t OCP_HB:1;
		uint8_t OCP_LC:1;
		uint8_t OCP_HC:1;
		uint8_t OTS:1;
		uint8_t OTW:1;
	};
	uint8_t reg;
} Status__1;




typedef union {
	struct {
		uint8_t SPI_ADDR_FLT:1;
		uint8_t SPI_SCLK_FLT:1;
		uint8_t SPI_PARITY:1;
		uint8_t VCP_UV:1;
		uint8_t BUCK_UV:1;
		uint8_t BUCK_OCP:1;
		uint8_t OTP_ERR:1;
		uint8_t :1;
	};
	uint8_t reg;
} Status__2;


#endif
