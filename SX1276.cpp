#include "SX1276.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_BITRATE_H            0x02
#define REG_BITRATE_L            0x03
#define REG_FDEV_H               0x04
#define REG_FDEV_L               0x05
#define REG_FRF_H                0x06
#define REG_FRF_M                0x07
#define REG_FRF_L                0x08
#define REG_RXCFG                0x0D
#define REG_RSSI                 0x11
#define REG_FEI_H                0x1D
#define REG_FEI_L                0x1E
#define REG_PAYLOAD_LEN          0x32
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06



SX1276Class::SX1276Class() :
  _spiSettings(10E6, MSBFIRST, SPI_MODE0),
  _ss(SX1276_DEFAULT_SS_PIN), 
  _reset(SX1276_DEFAULT_RESET_PIN),
  _dio0(SX1276_DEFAULT_DIO0_PIN)
{
}

int SX1276Class::begin()
{
  pinMode(_ss, OUTPUT);
  pinMode(_reset, OUTPUT);
  pinMode(_dio0, INPUT);

  digitalWrite(_reset, LOW);
  delay(20);
  digitalWrite(_reset, HIGH);
  delay(50);
  digitalWrite(_ss, HIGH);

  SPI.begin();

  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12)
  { 
    return 0; 
  }
  uint8_t initRegs[] = { 0x01, 0x78, 0x02, 0xB8, 0xD8, 0xFC, 0x8B, 0xFC, 0x49, 0x2B, 0x23, 0x08, 0x02, 0x0A, 0xFF, 0xB9, 0x12, 0x01, 0x28, 0x0C, 0x12, 0x47, 0x32, 0x3E, 0x00, 0x00, 0x00, 0xFD, 0x38, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x05, 0x12, 0x3E, 0x6B, 0x3E, 0x55, 0x55, 0x55, 0x55, 0x55, 0x48, 0x40, 0x17, 0x00, 0x00, 0x0F, 0x00, 0x0C, 0x00, 0xF5, 0x20, 0x82, 0xE2, 0x02, 0xDA, 0x24, 0x00, 0x00, 0x12, 0x24, 0x2D, 0x00, 0x03, 0x00, 0x04, 0x23, 0x00, 0x09, 0x05, 0x84, 0x32, 0x2B, 0x14, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x0C, 0xD9, 0x07, 0x00, 0x5C, 0x78, 0x00, 0x1C, 0x0E, 0x5B, 0xCC, 0x0E, 0x7F, 0x50, 0x00, 0x00, 0x00, 0x00, 0x09, 0x3F, 0xB1, 0x0B };

  setSleep();
  for(int reg = 0; reg < sizeof(initRegs); reg++)
  {
    writeRegister(2 + reg, initRegs[reg]);
  }

  return 1;
}

int32_t SX1276Class::readFei()
{
  int16_t ret = 0;

  ret |= (readRegister(REG_FEI_H) << 8);
  ret |= (readRegister(REG_FEI_L) << 0);

  return (int64_t)ret * 32000000ULL / (1<<19);
}

uint8_t SX1276Class::readRssi()
{
  return readRegister(REG_RSSI);
}

uint8_t SX1276Class::readFifo()
{
  return readRegister(REG_FIFO);
}

void SX1276Class::writeBitrate(uint32_t rate)
{
  uint32_t regBitRate = 32000000ULL / rate;
  
  writeRegister(REG_BITRATE_H, regBitRate >> 8);
  writeRegister(REG_BITRATE_L, regBitRate >> 0);
}

void SX1276Class::writeFreqDev(uint32_t dev)
{
  uint32_t regFreqDev = (uint64_t)dev * (1<<19) / 32000000ULL;
  
  writeRegister(REG_FDEV_H, regFreqDev >> 8);
  writeRegister(REG_FDEV_L, regFreqDev >> 0);
}

void SX1276Class::writeFreq(uint64_t freq)
{
  uint32_t regFrf = freq * (1<<19) / 32000000;
  
  writeRegister(REG_FRF_H, regFrf >> 16);
  writeRegister(REG_FRF_M, regFrf >> 8);
  writeRegister(REG_FRF_L, regFrf >> 0);
}

void SX1276Class::writePayloadLength(uint8_t length)
{
  writeRegister(REG_PAYLOAD_LEN, length);
}

void SX1276Class::writeRxConfig(uint8_t cfg)
{
  writeRegister(REG_RXCFG, cfg);
}


void SX1276Class::setSleep()
{
  delayMicroseconds(100);
  writeRegister(REG_OP_MODE, MODE_SLEEP);
  delayMicroseconds(100);
  writeRegister(REG_OP_MODE, MODE_SLEEP);
}

void SX1276Class::setStandby()
{
  writeRegister(REG_OP_MODE, MODE_STDBY);
}

void SX1276Class::setRx()
{
  writeRegister(REG_OP_MODE, MODE_STDBY);
  writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);
}

void SX1276Class::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}


uint8_t SX1276Class::readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

void SX1276Class::writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

uint8_t SX1276Class::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer(address);
  response = SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
  return response;
}


