#ifndef SX1276_H
#define SX1276_H

#include <Arduino.h>
#include <SPI.h>

#define SX1276_DEFAULT_SS_PIN     SS
#define SX1276_DEFAULT_RESET_PIN  RST_LoRa
#define SX1276_DEFAULT_DIO0_PIN   DIO0

#define PA_OUTPUT_PA_BOOST_PIN  1
#define PA_OUTPUT_RFO_PIN       0



class SX1276Class {
public:
  SX1276Class();

  int begin();
  void init();
  void setPins(int ss = SX1276_DEFAULT_SS_PIN, int reset = SX1276_DEFAULT_RESET_PIN, int dio0 = SX1276_DEFAULT_DIO0_PIN);
  void setSPIFrequency(uint32_t frequency);

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  void writePayloadLength(uint8_t length);
  void writeBitrate(uint32_t rate);
  void writeFreqDev(uint32_t dev);
  void writeRxBw(uint32_t bw);
  void writeAfcBw(uint32_t bw);
  void writeAfc(uint32_t freq);
  void writeFreq(uint64_t freq);
  void writeRxConfig(uint8_t cfg);
  
  int32_t readAfc();
  int32_t readFei();
  uint8_t readRssi();
  uint8_t readFifo();
  void setStandby();
  void setSleep();
  void setRx();
  
private:
  uint32_t delta(uint32_t ref, uint32_t value);
  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
};


#endif

