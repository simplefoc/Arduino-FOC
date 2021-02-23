

#include "./drv8316.h"


void DRV8316Driver3PWM::init(SPIClass* _spi) {
	DRV8316Driver::init(_spi);
	setRegistersLocked(false);
	delayMicroseconds(1);
	DRV8316Driver::setPWMMode(DRV8316_PWMMode::PWM3_Mode);
	BLDCDriver3PWM::init();
};


void DRV8316Driver6PWM::init(SPIClass* _spi) {
	DRV8316Driver::init(_spi);
	setRegistersLocked(false);
	delayMicroseconds(1);
	DRV8316Driver::setPWMMode(DRV8316_PWMMode::PWM6_Mode); // default mode is 6-PWM
	BLDCDriver6PWM::init();
};






/*
 * SPI setup:
 *
 *  capture on falling, propagate on rising =
 *  MSB first
 *
 * 	16 bit words
 * 	 outgoing: R/W:1 addr:6 parity:1 data:8
 * 	 incoming: status:8 data:8
 *
 * 	 on reads, incoming data is content of register being read
 * 	 on writes, incomnig data is content of register being written
 *
 *
 */



void DRV8316Driver::init(SPIClass* _spi) {
	// TODO make SPI speed configurable
	spi = _spi;
	settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);

	//setup pins
	pinMode(cs, OUTPUT);
	digitalWrite(cs, 1); // switch off

	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();

	// TODO add interrupt handler on the nFault pin if configured
	// add configuration for how to handle faults... idea: interrupt handler calls a callback, depending on the type of fault
	// consider what would be a useful configuration in practice? What do we want to do on a fault, e.g. over-temperature for example?
};




bool DRV8316Driver::getParity(uint16_t data) {
	//PARITY = XNOR(CMD, A5..A0, D7..D0)
	uint8_t par = 0;
	for (int i=0;i<16;i++) {
		if (((data)>>i) & 0x0001)
			par+=1;
	}
	return (par&0x01)==0x01; // even number of bits means true
}




uint16_t DRV8316Driver::readSPI(uint8_t addr) {
	digitalWrite(cs, 0);
	spi->beginTransaction(settings);
	uint16_t data = (((addr<<1) | 0x80)<<8)|0x0000;
	if (getParity(data))
		data |= 0x0100;
	uint16_t result = spi->transfer16(data);
	spi->endTransaction();
	digitalWrite(cs, 1);
//	Serial.print("SPI Read Result: ");
//	Serial.print(data, HEX);
//	Serial.print(" -> ");
//	Serial.println(result, HEX);
	return result;
}


uint16_t DRV8316Driver::writeSPI(uint8_t addr, uint8_t value) {
	digitalWrite(cs, 0);
	spi->beginTransaction(settings);
	uint16_t data = ((addr<<1)<<8)|value;
	if (getParity(data))
		data |= 0x0100;
	uint16_t result = spi->transfer16(data);
	spi->endTransaction();
	digitalWrite(cs, 1);
//	Serial.print("SPI Write Result: ");
//	Serial.print(data, HEX);
//	Serial.print(" -> ");
//	Serial.println(result, HEX);
	return result;
}



DRV8316Status DRV8316Driver::getStatus() {
	IC_Status data;
	Status__1 data1;
	Status__2 data2;
	uint16_t result = readSPI(IC_Status_ADDR);
	data.reg = (result & 0x00FF);
	delayMicroseconds(1); // delay at least 400ns between operations
	result = readSPI(Status__1_ADDR);
	data1.reg = (result & 0x00FF);
	delayMicroseconds(1); // delay at least 400ns between operations
	result = readSPI(Status__2_ADDR);
	data2.reg = (result & 0x00FF);
	return DRV8316Status(data, data1, data2);
}








void DRV8316Driver::clearFault() {
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	data.CLR_FLT |= 1;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__2_ADDR, data.reg);
};








bool DRV8316Driver::isRegistersLocked(){
	uint16_t result = readSPI(Control__1_ADDR);
	Control__1 data;
	data.reg = (result & 0x00FF);
	return data.REG_LOCK==REG_LOCK_LOCK;
}
void DRV8316Driver::setRegistersLocked(bool lock){
	uint16_t result = readSPI(Control__1_ADDR);
	Control__1 data;
	data.reg = (result & 0x00FF);
	data.REG_LOCK = lock?REG_LOCK_LOCK:REG_LOCK_UNLOCK;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__1_ADDR, data.reg);
}



DRV8316_PWMMode DRV8316Driver::getPWMMode() {
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_PWMMode)data.PWM_MODE;
};
void DRV8316Driver::setPWMMode(DRV8316_PWMMode pwmMode){
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	data.PWM_MODE = pwmMode;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__2_ADDR, data.reg);
};



DRV8316_Slew DRV8316Driver::getSlew() {
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_Slew)data.SLEW;
};
void DRV8316Driver::setSlew(DRV8316_Slew slewRate) {
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	data.SLEW = slewRate;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__2_ADDR, data.reg);
};



DRV8316_SDOMode DRV8316Driver::getSDOMode() {
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_SDOMode)data.SDO_MODE;
};
void DRV8316Driver::setSDOMode(DRV8316_SDOMode sdoMode) {
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	data.SDO_MODE = sdoMode;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__2_ADDR, data.reg);
};



bool DRV8316Driver::isOvertemperatureReporting(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return data.OTW_REP==OTW_REP_ENABLE;
};
void DRV8316Driver::setOvertemperatureReporting(bool reportFault){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	data.OTW_REP = reportFault?OTW_REP_ENABLE:OTW_REP_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__3_ADDR, data.reg);
};



bool DRV8316Driver::isSPIFaultReporting(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return data.SPI_FLT_REP==SPI_FLT_REP_ENABLE;
}
void DRV8316Driver::setSPIFaultReporting(bool reportFault){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	data.SPI_FLT_REP = reportFault?SPI_FLT_REP_ENABLE:SPI_FLT_REP_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__3_ADDR, data.reg);
}



bool DRV8316Driver::isOvervoltageProtection(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return data.OVP_EN==OVP_EN_ENABLE;
};
void DRV8316Driver::setOvervoltageProtection(bool enabled){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	data.OVP_EN = enabled?OVP_EN_ENABLE:OVP_EN_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__3_ADDR, data.reg);
};



DRV8316_OVP DRV8316Driver::getOvervoltageLevel(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OVP)data.OVP_SEL;
};
void DRV8316Driver::setOvervoltageLevel(DRV8316_OVP voltage){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	data.OVP_SEL = voltage;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__3_ADDR, data.reg);
};



DRV8316_PWM100DUTY DRV8316Driver::getPWM100Frequency(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_PWM100DUTY)data.PWM_100_DUTY_SEL;
};
void DRV8316Driver::setPWM100Frequency(DRV8316_PWM100DUTY freq){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	data.PWM_100_DUTY_SEL = freq;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__3_ADDR, data.reg);
};



DRV8316_OCPMode DRV8316Driver::getOCPMode(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OCPMode)data.OCP_MODE;

};
void DRV8316Driver::setOCPMode(DRV8316_OCPMode ocpMode){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	data.OCP_MODE = ocpMode;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__4_ADDR, data.reg);
};



DRV8316_OCPLevel DRV8316Driver::getOCPLevel(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OCPLevel)data.OCP_LVL;
};
void DRV8316Driver::setOCPLevel(DRV8316_OCPLevel amps){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	data.OCP_LVL = amps;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__4_ADDR, data.reg);
};



DRV8316_OCPRetry DRV8316Driver::getOCPRetryTime(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OCPRetry)data.OCP_RETRY;
};
void DRV8316Driver::setOCPRetryTime(DRV8316_OCPRetry ms){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	data.OCP_RETRY = ms;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__4_ADDR, data.reg);
};



DRV8316_OCPDeglitch DRV8316Driver::getOCPDeglitchTime(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OCPDeglitch)data.OCP_DEG;
};
void DRV8316Driver::setOCPDeglitchTime(DRV8316_OCPDeglitch ms){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	data.OCP_DEG = ms;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__4_ADDR, data.reg);
};



bool DRV8316Driver::isOCPClearInPWMCycleChange(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return data.OCP_CBC==OCP_CBC_ENABLE;
};
void DRV8316Driver::setOCPClearInPWMCycleChange(bool enable){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	data.OCP_CBC = enable?OCP_CBC_ENABLE:OCP_CBC_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__4_ADDR, data.reg);
};



bool DRV8316Driver::isDriverOffEnabled(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return data.DRV_OFF==DRV_OFF_ENABLE;
};
void DRV8316Driver::setDriverOffEnabled(bool enabled){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	data.DRV_OFF = enabled?DRV_OFF_ENABLE:DRV_OFF_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__4_ADDR, data.reg);
};



DRV8316_CSAGain DRV8316Driver::getCurrentSenseGain(){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_CSAGain)data.CSA_GAIN;
};
void DRV8316Driver::setCurrentSenseGain(DRV8316_CSAGain gain){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	data.CSA_GAIN = gain;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__5_ADDR, data.reg);
};



bool DRV8316Driver::isActiveSynchronousRectificationEnabled(){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	return data.EN_ASR==EN_ASR_ENABLE;

};
void DRV8316Driver::setActiveSynchronousRectificationEnabled(bool enabled){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	data.EN_ASR = enabled?EN_ASR_ENABLE:EN_ASR_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__5_ADDR, data.reg);
};



bool DRV8316Driver::isActiveAsynchronousRectificationEnabled(){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	return data.EN_AAR==EN_AAR_ENABLE;
};
void DRV8316Driver::setActiveAsynchronousRectificationEnabled(bool enabled){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	data.EN_AAR = enabled?EN_AAR_ENABLE:EN_AAR_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__5_ADDR, data.reg);
};



DRV8316_Recirculation DRV8316Driver::getRecirculationMode(){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_Recirculation)data.ILIM_RECIR;
};
void DRV8316Driver::setRecirculationMode(DRV8316_Recirculation recirculationMode){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	data.ILIM_RECIR = recirculationMode;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__5_ADDR, data.reg);
};



bool DRV8316Driver::isBuckEnabled(){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	return data.BUCK_DIS==BUCK_DIS_BUCK_ENABLE;
};
void DRV8316Driver::setBuckEnabled(bool enabled){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	data.BUCK_DIS = enabled?BUCK_DIS_BUCK_ENABLE:BUCK_DIS_BUCK_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__6_ADDR, data.reg);
};



DRV8316_BuckVoltage DRV8316Driver::getBuckVoltage(){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_BuckVoltage)data.BUCK_SEL;
};
void DRV8316Driver::setBuckVoltage(DRV8316_BuckVoltage volts){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	data.BUCK_SEL = volts;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__6_ADDR, data.reg);
};



DRV8316_BuckCurrentLimit DRV8316Driver::getBuckCurrentLimit(){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_BuckCurrentLimit)data.BUCK_CL;
};
void DRV8316Driver::setBuckCurrentLimit(DRV8316_BuckCurrentLimit mamps){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	data.BUCK_CL = mamps;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__6_ADDR, data.reg);
};



bool DRV8316Driver::isBuckPowerSequencingEnabled(){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	return data.BUCK_PS_DIS==BUCK_PS_DIS_ENABLE;

};
void DRV8316Driver::setBuckPowerSequencingEnabled(bool enabled){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	data.BUCK_PS_DIS = enabled?BUCK_PS_DIS_ENABLE:BUCK_PS_DIS_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__6_ADDR, data.reg);
};



DRV8316_DelayTarget DRV8316Driver::getDelayTarget(){
	uint16_t result = readSPI(Control__10_ADDR);
	Control__10 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_DelayTarget)data.DLY_TARGET;
};
void DRV8316Driver::setDelayTarget(DRV8316_DelayTarget us){
	uint16_t result = readSPI(Control__10_ADDR);
	Control__10 data;
	data.reg = (result & 0x00FF);
	data.DLY_TARGET = us;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__10_ADDR, data.reg);
};



bool DRV8316Driver::isDelayCompensationEnabled(){
	uint16_t result = readSPI(Control__10_ADDR);
	Control__10 data;
	data.reg = (result & 0x00FF);
	return data.DLYCMP_EN==DLYCMP_EN_ENABLE;
};
void DRV8316Driver::setDelayCompensationEnabled(bool enabled){
	uint16_t result = readSPI(Control__10_ADDR);
	Control__10 data;
	data.reg = (result & 0x00FF);
	data.DLYCMP_EN = enabled?DLYCMP_EN_ENABLE:DLYCMP_EN_DISABLE;
	delayMicroseconds(1); // delay at least 400ns between operations
	result = writeSPI(Control__10_ADDR, data.reg);
};



