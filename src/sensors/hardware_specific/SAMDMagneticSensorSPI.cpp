#include "SAMDMagneticSensorSPI.h"
#include "../../common/hardware_specific/samd_mcu.h"

SAMDMagneticSensorSPI::SAMDMagneticSensorSPI(MagneticSensorSPIConfig_s config, int8_t tccN, int8_t dmaTX, int8_t dmaRX) 
: MagneticSensor(1 << config.bit_resolution), config(config), tccN(tccN), dmaTX(dmaTX), dmaRX(dmaRX), SPItransmitBuffer{0xFF, 0xFF}
{
    if(tccN > 0)
    {
      Tcc * tcc = addTCCHandler(tccN, this);
      tcc->INTENSET.bit.OVF = 0b1;
    }
}

void SAMDMagneticSensorSPI::tccHandler(Tcc * tcc)
{

}

void SAMDMagneticSensorSPI::operator()(Tcc * tcc)
{
    digitalWrite(this->spi->pinSS, LOW);
    
    trigDMACChannel(dmaTX);
    DMAC->CHID.reg = DMAC_CHID_ID(dmaTX);
    DMAC->CHCTRLA.bit.ENABLE = 0b1;
    DMAC->CHID.reg = DMAC_CHID_ID(dmaRX);
    DMAC->CHCTRLA.bit.ENABLE = 0b1;

    tcc->INTFLAG.bit.OVF = 0b1;
}

void SAMDMagneticSensorSPI::operator()(volatile DMAC_CHINTFLAG_Type &, volatile DMAC_CHCTRLA_Type &)
{
  if(DMAC->CHID.bit.ID == dmaTX)
    DMAC->CHCTRLA.bit.ENABLE = 0b0;

  if(DMAC->CHID.bit.ID == dmaRX)
  {
    DMAC->CHCTRLA.bit.ENABLE = 0b0;
    digitalWrite(this->spi->pinSS, HIGH);
  }
}

void SAMDMagneticSensorSPI::init(SAMDAdvancedSPI* _spi)
{

    spi = _spi;
    spi->init(from_SPI_MODE(config.spi_mode), config.clock_speed);
    MagneticSensor::init();
    if(tccN > 0)
    {
      initDMA();
    }
}

// function reading the raw counter of the magnetic sensor
uint32_t SAMDMagneticSensorSPI::getRawCount(uint64_t & timestamp_us){
  word command = config.angle_register;

  if (config.command_rw_bit > 0) 
    command |= (1 << config.command_rw_bit);

  if (config.command_parity_bit > 0) 
  	command |= ((word)spi->spiCalcEvenParity(command) << config.command_parity_bit); //Add a parity bit on the the MSB

  spi->transfer16(command);
  timestamp_us = _micros();
  delayMicroseconds(10);

  word register_value = spi->transfer16(command);
  
  register_value = register_value >> (1 + config.data_start_bit - config.bit_resolution);  //this should shift data to the rightmost bits of the word

  const static word data_mask = 0xFFFF >> (16 - config.bit_resolution);

  return register_value & data_mask;  // Return the data, stripping the non data (e.g parity) bits
}



void SAMDMagneticSensorSPI::initDMA() {


  DMAC_CHINTENSET_Type chinset{.reg = 0};
  chinset.bit.TCMPL = 0b1;  
  //SerialUSB.println("d");
  // configure the channel
  DMAC_CHCTRLB_Type chctrlb{.reg = 0};
  chctrlb.bit.LVL = DMAC_CHCTRLB_LVL_LVL0_Val;
  chctrlb.bit.EVIE = 0b0; //input (USER) event enabled?
  chctrlb.bit.EVOE = 0b0; //output (GEN) event enabled?
  chctrlb.bit.EVACT = DMAC_CHCTRLB_EVACT_NOACT_Val; //only used if EVIE is set
  chctrlb.bit.TRIGSRC = SERCOM4_DMAC_ID_TX;
  chctrlb.bit.TRIGACT = DMAC_CHCTRLB_TRIGACT_BEAT_Val; //block, beat or transaction


  volatile DMAC_BTCTRL_Type & btctrl_write = SPIdescriptors[0].BTCTRL;
  btctrl_write.bit.VALID = 0b1;                                     /*!< bit:      0  Descriptor Valid                   */
  
  //we want no events after the last adc read
  btctrl_write.bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE_Val;            /*!< bit:  1.. 2  Event Output Selection             */
  
  btctrl_write.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_NOACT_Val;       /*!< bit:  3.. 4  Block Action                       */
  btctrl_write.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_BYTE_Val;       /*!< bit:  8.. 9  Beat Size (byte, half-words, words) */
  btctrl_write.bit.SRCINC = 0b1;                                    /*!< bit:     10  Source Address Increment Enable    */
  btctrl_write.bit.DSTINC = 0b0;                                    /*!< bit:     11  Destination Address Increment Enable */
  btctrl_write.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;           /*!< bit:     12  Step Selection                     */
  btctrl_write.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val;          /*!< bit: 13..15  Address Increment Step Size        */

  SPIdescriptors[0].DESCADDR.reg = 0; //next descriptor
  SPIdescriptors[0].BTCNT.reg    = SPIbufferSize;
  SPIdescriptors[0].DSTADDR.reg  = (uint32_t)&spi->sercomCfg.sercom->SPI.DATA.reg;
  SPIdescriptors[0].SRCADDR.reg  = ((uint32_t)&SPItransmitBuffer[0]) + SPIbufferSize;


  ::initDMAChannel(dmaTX, chinset, chctrlb, SPIdescriptors[0], this);

  // configure the channel
  chctrlb.reg = 0;
  chctrlb.bit.LVL = DMAC_CHCTRLB_LVL_LVL0_Val;
  chctrlb.bit.EVIE = 0b0; //input (USER) event enabled?
  chctrlb.bit.EVOE = 0b0; //output (GEN) event enabled?
  chctrlb.bit.EVACT = DMAC_CHCTRLB_EVACT_NOACT_Val; //only used if EVIE is set
  chctrlb.bit.TRIGSRC = SERCOM4_DMAC_ID_RX;
  chctrlb.bit.TRIGACT = DMAC_CHCTRLB_TRIGACT_BEAT_Val; //block, beat or transaction

  // /////////////////////////////////////////////////
  // // read DMA

  volatile DMAC_BTCTRL_Type & btctrl_read = SPIdescriptors[1].BTCTRL;
  btctrl_read.bit.VALID = 0b1;                                     /*!< bit:      0  Descriptor Valid                   */
  
  //we want no events after the last adc read
  btctrl_read.bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE_Val;            /*!< bit:  1.. 2  Event Output Selection             */
  
  btctrl_read.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_NOACT_Val;       //isLast ? DMAC_BTCTRL_BLOCKACT_NOACT_Val : DMAC_BTCTRL_BLOCKACT_INT_Val;       /*!< bit:  3.. 4  Block Action                       */
  btctrl_read.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_BYTE_Val;       /*!< bit:  8.. 9  Beat Size (byte, half-words, words) */
  btctrl_read.bit.SRCINC = 0b0;                                    /*!< bit:     10  Source Address Increment Enable    */
  btctrl_read.bit.DSTINC = 0b1;                                    /*!< bit:     11  Destination Address Increment Enable */
  btctrl_read.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;           /*!< bit:     12  Step Selection                     */
  btctrl_read.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val;          /*!< bit: 13..15  Address Increment Step Size        */

  SPIdescriptors[1].DESCADDR.reg = 0;
  SPIdescriptors[1].BTCNT.reg    = SPIbufferSize;
  SPIdescriptors[1].DSTADDR.reg  = ((uint32_t)&SPIreceiveBuffer[0]) + SPIbufferSize;
  SPIdescriptors[1].SRCADDR.reg  = (uint32_t) &spi->sercomCfg.sercom->SPI.DATA.reg;

  ::initDMAChannel(dmaTX, chinset, chctrlb, SPIdescriptors[1], this);

}