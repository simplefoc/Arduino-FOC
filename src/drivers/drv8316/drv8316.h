

#ifndef SIMPLEFOC_DRV8316
#define SIMPLEFOC_DRV8316


#include "Arduino.h"
#include <SPI.h>
#include <drivers/BLDCDriver3PWM.h>
#include <drivers/BLDCDriver6PWM.h>

#include "./drv8316_registers.h"


enum DRV8316_PWMMode {
	PWM6_Mode = 0b00,
	PWM6_CurrentLimit_Mode = 0b01,
	PWM3_Mode = 0b10,
	PWM3_CurrentLimit_Mode = 0b11
};


enum DRV8316_SDOMode {
	OpenDrain = 0b0,
	PushPull = 0b1
};


enum DRV8316_Slew {
	Slew_25Vus = 0b00,
	Slew_50Vus = 0b01,
	Slew_150Vus = 0b10,
	Slew_200Vus = 0b11
};


enum DRV8316_OVP {
	OVP_SEL_32V = 0b0,
	OVP_SEL_20V = 0b1
};


enum DRV8316_PWM100DUTY {
	FREQ_20KHz = 0b0,
	FREQ_40KHz = 0b1
};


enum DRV8316_OCPMode {
	Latched_Fault = 0b00,
	AutoRetry_Fault = 0b01,
	ReportOnly = 0b10,
	NoAction = 0b11
};


enum DRV8316_OCPLevel {
	Curr_16A = 0b0,
	Curr_24A = 0b1
};


enum DRV8316_OCPRetry {
	Retry5ms = 0b0,
	Retry500ms = 0b1
};


enum DRV8316_OCPDeglitch {
	Deglitch_0us2 = 0b00,
	Deglitch_0us6 = 0b01,
	Deglitch_1us1 = 0b10,
	Deglitch_1us6 = 0b11
};


enum DRV8316_CSAGain {
	Gain_0V15 = 0b00,
	Gain_0V1875 = 0b01,
	Gain_0V25 = 0b10,
	Gain_0V375 = 0b11
};


enum DRV8316_Recirculation {
	BrakeMode = 0b00, // FETs
	CoastMode = 0b01  // Diodes
};


enum DRV8316_BuckVoltage {
	VB_3V3 = 0b00,
	VB_5V = 0b01,
	VB_4V = 0b10,
	VB_5V7 = 0b11
};


enum DRV8316_BuckCurrentLimit {
	Limit_600mA = 0b00,
	Limit_150mA = 0b01
};


enum DRV8316_DelayTarget {
	Delay_0us = 0x0,
	Delay_0us4 = 0x1,
	Delay_0us6 = 0x2,
	Delay_0us8 = 0x3,
	Delay_1us = 0x4,
	Delay_1us2 = 0x5,
	Delay_1us4 = 0x6,
	Delay_1us6 = 0x7,
	Delay_1us8 = 0x8,
	Delay_2us = 0x9,
	Delay_2us2 = 0xA,
	Delay_2us4 = 0xB,
	Delay_2us6 = 0xC,
	Delay_2us8 = 0xD,
	Delay_3us = 0xE,
	Delay_3us2 = 0xF
};



class DRV8316ICStatus {
public:
	DRV8316ICStatus(IC_Status status) : status(status) {};
	~DRV8316ICStatus() {};

	bool isFault() { return status.FAULT==0b1; };
	bool isOverTemperature() { return status.OT==0b1; };
	bool isOverCurrent() { return status.OCP==0b1; };
	bool isOverVoltage() { return status.OVP==0b1; };
	bool isSPIError() { return status.SPI_FLT==0b1; };
	bool isBuckError() { return status.BK_FLT==0b1; };
	bool isPowerOnReset() { return status.NPOR==0b1; };

	IC_Status status;
};


class DRV8316Status1 {
public:
	DRV8316Status1(Status__1 status1) : status1(status1) {};
	~DRV8316Status1() {};


	bool isOverCurrent_Ah() { return status1.OCP_HA==0b1; };
	bool isOverCurrent_Al() { return status1.OCP_LA==0b1; };
	bool isOverCurrent_Bh() { return status1.OCP_HB==0b1; };
	bool isOverCurrent_Bl() { return status1.OCP_LB==0b1; };
	bool isOverCurrent_Ch() { return status1.OCP_HC==0b1; };
	bool isOverCurrent_Cl() { return status1.OCP_LC==0b1; };
	bool isOverTemperatureShutdown() { return status1.OTS==0b1; };
	bool isOverTemperatureWarning() { return status1.OTW==0b1; };

	Status__1 status1;
};


class DRV8316Status2 {
public:
	DRV8316Status2(Status__2 status2) : status2(status2) {};
	~DRV8316Status2() {};


	bool isOneTimeProgrammingError() { return status2.OTP_ERR==0b1; };
	bool isBuckOverCurrent() { return status2.BUCK_OCP==0b1; };
	bool isBuckUnderVoltage() { return status2.BUCK_UV==0b1; };
	bool isChargePumpUnderVoltage() { return status2.VCP_UV==0b1; };
	bool isSPIParityError() { return status2.SPI_PARITY==0b1; };
	bool isSPIClockFramingError() { return status2.SPI_SCLK_FLT==0b1; };
	bool isSPIAddressError() { return status2.SPI_ADDR_FLT==0b1; };

	Status__2 status2;
};



class DRV8316Status : public DRV8316ICStatus, public DRV8316Status1, public DRV8316Status2 {
	public:
		DRV8316Status(IC_Status status, Status__1 status1, Status__2 status2) : DRV8316ICStatus(status), DRV8316Status1(status1), DRV8316Status2(status2) {};
		~DRV8316Status() {};
};




class DRV8316Driver {

	public:
		DRV8316Driver(int cs, bool currentLimit = false, int nFault = NOT_SET) : currentLimit(currentLimit), cs(cs), nFault(nFault), spi(&SPI), settings(1000000, MSBFIRST, SPI_MODE1) {};
		virtual ~DRV8316Driver() {};

		virtual void init(SPIClass* _spi = &SPI);

		void clearFault(); // TODO check for fault condition methods

		DRV8316Status getStatus();

		bool isRegistersLocked();
		void setRegistersLocked(bool lock);

		DRV8316_PWMMode getPWMMode();
		void setPWMMode(DRV8316_PWMMode pwmMode);

		DRV8316_Slew getSlew();
		void setSlew(DRV8316_Slew slewRate);

		DRV8316_SDOMode getSDOMode();
		void setSDOMode(DRV8316_SDOMode sdoMode);

		bool isOvertemperatureReporting();
		void setOvertemperatureReporting(bool reportFault);

		bool isSPIFaultReporting();
		void setSPIFaultReporting(bool reportFault);

		bool isOvervoltageProtection();
		void setOvervoltageProtection(bool enabled);

		DRV8316_OVP getOvervoltageLevel();
		void setOvervoltageLevel(DRV8316_OVP voltage);

		DRV8316_PWM100DUTY getPWM100Frequency();
		void setPWM100Frequency(DRV8316_PWM100DUTY freq);

		DRV8316_OCPMode getOCPMode();
		void setOCPMode(DRV8316_OCPMode ocpMode);

		DRV8316_OCPLevel getOCPLevel();
		void setOCPLevel(DRV8316_OCPLevel amps);

		DRV8316_OCPRetry getOCPRetryTime();
		void setOCPRetryTime(DRV8316_OCPRetry ms);

		DRV8316_OCPDeglitch getOCPDeglitchTime();
		void setOCPDeglitchTime(DRV8316_OCPDeglitch ms);

		bool isOCPClearInPWMCycleChange();
		void setOCPClearInPWMCycleChange(bool enable);

		bool isDriverOffEnabled();
		void setDriverOffEnabled(bool enabled);

		DRV8316_CSAGain getCurrentSenseGain();
		void setCurrentSenseGain(DRV8316_CSAGain gain);

		bool isActiveSynchronousRectificationEnabled();
		void setActiveSynchronousRectificationEnabled(bool enabled);

		bool isActiveAsynchronousRectificationEnabled();
		void setActiveAsynchronousRectificationEnabled(bool enabled);

		DRV8316_Recirculation getRecirculationMode();
		void setRecirculationMode(DRV8316_Recirculation recirculationMode);

		bool isBuckEnabled();
		void setBuckEnabled(bool enabled);

		DRV8316_BuckVoltage getBuckVoltage();
		void setBuckVoltage(DRV8316_BuckVoltage volts);

		DRV8316_BuckCurrentLimit getBuckCurrentLimit();
		void setBuckCurrentLimit(DRV8316_BuckCurrentLimit mamps);

		bool isBuckPowerSequencingEnabled();
		void setBuckPowerSequencingEnabled(bool enabled);

		DRV8316_DelayTarget getDelayTarget();
		void setDelayTarget(DRV8316_DelayTarget us);

		bool isDelayCompensationEnabled();
		void setDelayCompensationEnabled(bool enabled);

	private:
		uint16_t readSPI(uint8_t addr);
		uint16_t writeSPI(uint8_t addr, uint8_t data);
		bool getParity(uint16_t data);

		bool currentLimit;
		int cs;
		int nFault;
		SPIClass* spi;
		SPISettings settings;
};



class DRV8316Driver3PWM : public DRV8316Driver, public BLDCDriver3PWM {

	public:
		DRV8316Driver3PWM(int phA,int phB,int phC, int cs, bool currentLimit = false, int en = NOT_SET, int nFault = NOT_SET) :
			DRV8316Driver(cs, currentLimit, nFault), BLDCDriver3PWM(phA, phB, phC, en) {};
		virtual ~DRV8316Driver3PWM() {};

		virtual void init(SPIClass* _spi = &SPI) override;

};



class DRV8316Driver6PWM : public DRV8316Driver, public BLDCDriver6PWM {

	public:
		DRV8316Driver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int cs, bool currentLimit = false, int en = NOT_SET, int nFault = NOT_SET) :
			DRV8316Driver(cs, currentLimit, nFault), BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, en) {};
		virtual ~DRV8316Driver6PWM() {};

		virtual void init(SPIClass* _spi = &SPI) override;

};


#endif
