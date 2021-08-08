/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 by g3gg0.de
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * crossfire sniffing receiver
 *
 */

/* needs 
    #define CONFIG_DISABLE_HAL_LOCKS 1
   in
     C:\Users\<username>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\cores\esp32\esp32-hal.h
   else you will get crashes
*/

#if !defined(CONFIG_DISABLE_HAL_LOCKS)
#error "please add  '#define CONFIG_DISABLE_HAL_LOCKS 1'  at about line 66 of esp32-hal.h else this code will crash"
#endif

#include <FS.h>
#include <SPIFFS.h>

#include "SSD1306Wire.h" 
#include "SX1276.h" 
#include "Plot.h"

#include "esp32-hal-timer.h"

#define COUNT(x) (sizeof(x)/sizeof((x)[0]))
#define CONFIG_MAGIC 0xC0FFFEF5

typedef struct
{
  uint8_t entries;
  uint8_t slots[4];
} timeslotInfo_t;

typedef struct 
{
  uint32_t magic;
  uint8_t hoppingSequence[150];
} receiver_config;

receiver_config config;

SX1276Class SX1276;
SSD1306Wire *Display;

uint16_t channelValues[16];

bool currentDirectionDown = false;
uint16_t currentChannel = 0;
uint16_t currentLength = 0;
int state = 0;
uint64_t startTimestamp = 0;
uint64_t prevTimestamp = 0;
int checkChan = 0;
int retries = 0;

//uint32_t frequencyRegVals[] = { 0xD70AD0, 0xD71B74, 0xD72C18, 0xD73CBB, 0xD74D60, 0xD75E03, 0xD76EA7, 0xD77F4B, 0xD78FEF, 0xD7A092, 0xD7B137, 0xD7C1DA, 0xD7D27E, 0xD7E322, 0xD7F3C6, 0xD80469, 0xD8150E, 0xD825B1, 0xD83655, 0xD846F9, 0xD8579D, 0xD86840, 0xD878E5, 0xD88988, 0xD89A2C, 0xD8AAD0, 0xD8BB74, 0xD8CC17, 0xD8DCBC, 0xD8ED5F, 0xD8FE03, 0xD90EA7, 0xD91F4B, 0xD92FEE, 0xD94093, 0xD95136, 0xD961DA, 0xD9727E, 0xD98322, 0xD993C5, 0xD9A46A, 0xD9B50D, 0xD9C5B2, 0xD9D655, 0xD9E6F9, 0xD9F79D, 0xDA0841, 0xDA18E4, 0xDA2989, 0xDA3A2C, 0xDA4AD0, 0xDA5B74, 0xDA6C18, 0xDA7CBB, 0xDA8D60, 0xDA9E03, 0xDAAEA7, 0xDABF4B, 0xDACFEF, 0xDAE092, 0xDAF137, 0xDB01DA, 0xDB127E, 0xDB2322, 0xDB33C6, 0xDB4469, 0xDB550E, 0xDB65B1, 0xDB7655, 0xDB86F9, 0xDB979D, 0xDBA840, 0xDBB8E5, 0xDBC988, 0xDBDA2C, 0xDBEAD0, 0xDBFB74, 0xDC0C17, 0xDC1CBC, 0xDC2D5F, 0xDC3E03, 0xDC4EA7, 0xDC5F4B, 0xDC6FEE, 0xDC8093, 0xDC9136, 0xDCA1DA, 0xDCB27E, 0xDCC322, 0xDCD3C5, 0xDCE46A, 0xDCF50D, 0xDD05B2, 0xDD1655, 0xDD26F9, 0xDD379D, 0xDD4841, 0xDD58E4, 0xDD6989, 0xDD7A2C };
uint32_t frequencies[] = { 860168945, 860428955, 860688964, 860948913, 861208984, 861468933, 861728942, 861988952, 862248962, 862508911, 862768981, 863028930, 863288940, 863548950, 863808960, 864068908, 864328979, 864588928, 864848938, 865108947, 865368957, 865628906, 865888977, 866148925, 866408935, 866668945, 866928955, 867188903, 867448974, 867708923, 867968933, 868228942, 868488952, 868748901, 869008972, 869268920, 869528930, 869788940, 870048950, 870308898, 870568969, 870828918, 871088989, 871348938, 871608947, 871868957, 872128967, 872388916, 872648986, 872908935, 873168945, 873428955, 873688964, 873948913, 874208984, 874468933, 874728942, 874988952, 875248962, 875508911, 875768981, 876028930, 876288940, 876548950, 876808960, 877068908, 877328979, 877588928, 877848938, 878108947, 878368957, 878628906, 878888977, 879148925, 879408935, 879668945, 879928955, 880188903, 880448974, 880708923, 880968933, 881228942, 881488952, 881748901, 882008972, 882268920, 882528930, 882788940, 883048950, 883308898, 883568969, 883828918, 884088989, 884348938, 884608947, 884868957, 885128967, 885388916, 885648986, 885908935,  };
//uint8_t hoppingSequence[] = { 0, 6, 28, 11, 39, 23, 17, 9, 37, 1, 48, 38, 31, 3, 49, 46, 25, 15, 44, 8, 27, 47, 24, 2, 29, 10, 43, 40, 26, 42, 7, 4, 19, 33, 12, 5, 36, 22, 45, 14, 20, 35, 32, 13, 16, 30, 21, 0, 18, 28, 11, 41, 23, 17, 34, 37, 1, 6, 38, 31, 39, 49, 46, 9, 15, 44, 48, 27, 47, 3, 2, 29, 25, 43, 40, 8, 42, 7, 24, 19, 33, 10, 5, 36, 26, 45, 14, 4, 35, 32, 12, 16, 30, 22, 0, 18, 20, 11, 41, 13, 17, 34, 21, 1, 6, 28, 31, 39, 23, 46, 9, 37, 44, 48, 38, 47, 3, 49, 29, 25, 15, 40, 8, 27, 7, 24, 2, 33, 10, 43, 36, 26, 42, 14, 4, 19, 32, 12, 5, 30, 22, 45, 18, 20, 35, 41, 13, 16, 34, 21 };
uint8_t hoppingPos = 0;
uint32_t cfgBitrate = 85000;
uint32_t cfgFreqDev = 42300;
uint32_t cfgChanDist = 260010;
uint64_t cfgStartFreq = 860168000ULL;

float freqCorrectionUp = 0;
float freqCorrectionDown = 0;

int16_t statRssiUp = 0;
int16_t statRssiDown = 0;
int16_t scanRssi = 0;
uint64_t lastPacketTimestamp = 0;

timeslotInfo_t timeslotMap[50];
static const float packet_ms = 6666.666666666f * 1.00101f;
hw_timer_t * scheduleTimer = NULL;

uint32_t zeroSyncTime[10];
uint32_t zeroSyncTimePos = 0;
uint64_t prevSyncStart = 0;
uint32_t channelExpects[10];
bool synced = false;
uint32_t losses = 0;


uint32_t statUpScheduled = 0;
uint32_t statUpLoss = 0;
uint32_t statUpReceived = 0;
float statUpHeadroom = 0;
uint32_t statDownScheduled = 0;
uint32_t statDownLoss = 0;
uint32_t statDownReceived = 0;
float statDownHeadroom = 0;
float statAverageScheduleLength = 0;
uint32_t statResyncs = 0;
uint32_t learnSchedLength = 6676;

PlotClass *plotRssiUp;
PlotClass *plotRssiDown;
PlotClass *plotFCorrUp;
PlotClass *plotFCorrDown;
PlotClass *plotFreq;
PlotClass *plotSchedLen;
PlotClass *plotLossUp;
PlotClass *plotLossDown;
PlotClass *plotScan;


void IRAM_ATTR setChan(uint16_t chan, bool downlink)
{
  uint16_t chanAbs = chan + (downlink ? 50 : 0);
  uint32_t freq = cfgStartFreq + chanAbs * cfgChanDist + (downlink ? freqCorrectionDown : freqCorrectionUp);

  currentChannel = chan;
  currentDirectionDown = downlink;
  currentLength = downlink ? 0x0D : 0x17;
  
  SX1276.setStandby();
  SX1276.writePayloadLength(currentLength);
  SX1276.writeBitrate(cfgBitrate);
  SX1276.writeFreqDev(cfgFreqDev);
  SX1276.writeFreq(freq);

  /* AgcAutoOn and AGC PreambleDetect RxTrigger */
  SX1276.writeRxConfig(0x08 | 6);

  SX1276.setRx();
}

bool IRAM_ATTR calc_channel(uint32_t *slotDelta, int32_t *remainder, uint64_t timestamp, uint64_t startTimestamp)
{
  uint64_t delta = timestamp - startTimestamp;
  float distance = (float)delta / packet_ms;
  
  *slotDelta = (int)(distance + 0.5f) % 150;
  *remainder = 100.0f * (distance - (int)(distance + 0.5f));

  if(*remainder > 1 || *remainder < -1)
  {
    return false;
  }
  return true;
}

void IRAM_ATTR addDelta(uint8_t channel, uint64_t timestamp, uint64_t startTimestamp)
{
  uint64_t delta = timestamp - startTimestamp;
  uint32_t slotDelta = 0;
  int32_t remainder = 0;

  bool match = calc_channel(&slotDelta, &remainder, timestamp, startTimestamp);
  
  if(remainder > 10 || remainder < -10)
  {
    Serial.printf("OFF : Channel #%2u %3u (%7llu us, remainder %3i, %llu %llu)\n", channel, slotDelta, delta, remainder, timestamp, startTimestamp);
    return;
  }
  timeslotInfo_t *slot = &timeslotMap[channel];

  for(int pos = 0; pos < slot->entries; pos++)
  {
    if(slot->slots[pos] == slotDelta)
    {
      Serial.printf("SKIP: Channel #%2u %3u (%7llu us, remainder %3i) - already contained\n", channel, slotDelta, delta, remainder);
      return;
    }
  }

  if(slot->entries >= 4)
  {
    Serial.printf("FAIL: Channel #%2u %3u (%7llu us, remainder %3i) - already enough\n", channel, slotDelta, delta, remainder);
    return;
  }
  
  slot->slots[slot->entries++] = slotDelta;
  Serial.printf("ADD : Channel #%2u %3u (%7llu us, remainder %3i, %llu %llu)\n", channel, slotDelta, delta, remainder, timestamp, startTimestamp);
}

void IRAM_ATTR timer_isr_body(uint64_t timestamp)
{
  const char *dir = currentDirectionDown ? "D" : "U";

  if(lastPacketTimestamp == 0)
  {
    losses++;
  }
  else
  {
    losses = 0;
  }
  
  if(losses > 200)
  {
    statResyncs++;
    state = 200;
    Serial.printf("Lost Sync\n");
    return;
  }
  
  uint32_t lastPacketDelta = timestamp - lastPacketTimestamp;

  if(currentDirectionDown)
  {
    setChan(config.hoppingSequence[hoppingPos++], false);
    hoppingPos %= 150;
    timerAlarmWrite(scheduleTimer, prevTimestamp + learnSchedLength, false);

    statDownScheduled++;
    if(lastPacketTimestamp == 0)
    {
      plotLossDown->AddSample(1);
      statDownLoss++;
    }
    else
    {
      plotLossDown->AddSample(0);
      statDownHeadroom = (63.0f * statDownHeadroom + lastPacketDelta) / 64.0f;
      statDownReceived++;
    }
  }
  else
  {
    uint64_t prevTimestampOld = prevTimestamp;
    bool lastSuccessful = false;
    
    setChan(currentChannel, true);
    prevTimestamp += learnSchedLength;
    timerAlarmWrite(scheduleTimer, prevTimestamp + 2650, false);

    statUpScheduled++;
    if(lastPacketTimestamp == 0)
    {
      plotLossUp->AddSample(1);
      statUpLoss++;
      lastSuccessful = false;
    }
    else
    {
      plotLossUp->AddSample(0);
      statUpHeadroom = (63.0f * statUpHeadroom + lastPacketDelta) / 64.0f;
      statUpReceived++;
      lastSuccessful = true;
    }

    if(lastPacketDelta < 180)
    {
      prevTimestamp += 10;
    }
    else if(lastPacketDelta > 220)
    {
      prevTimestamp -= 10;
    }

    uint32_t scheduleLength = prevTimestamp - prevTimestampOld;

    if(scheduleLength > 6500 && scheduleLength < 6800 && lastSuccessful)
    {
      if(statAverageScheduleLength < 1)
      {
        statAverageScheduleLength = scheduleLength;
      }
      statAverageScheduleLength = (63.0f * statAverageScheduleLength + scheduleLength) / 64.0f;

      plotFreq->AddSample(1000000.0f / scheduleLength);
      plotSchedLen->AddSample(statAverageScheduleLength);
      
      static uint32_t update_delay = 0;
      if(update_delay++ > 10000)
      {
        update_delay = 0;
        if(learnSchedLength < statAverageScheduleLength)
        {
          learnSchedLength++;
        }
        else if(learnSchedLength > statAverageScheduleLength)
        {
          learnSchedLength--;
        }
      }
    }
  }
  lastPacketTimestamp = 0;
  timerAlarmEnable(scheduleTimer);
  //Serial.printf("Lost %s %d times, next #%d %s\n", dir, losses, currentChannel, currentDirectionDown ? "D" : "U");
}

#define SKIP_SPACE(l) do { while(*line && *line == ' ') {line++;} } while (0)

void IRAM_ATTR parseChannels(uint8_t *buffer)
{
  uint32_t chanBits = 0;
  uint32_t channel = 0;
  uint32_t value = 0;
  bool upperEight = buffer[0] & 0x20;

  /* go through all payload bytes */
  for (int bitPos = 0; bitPos < 11; bitPos++)
  {
    /* fetch 8 bits */
    value |= ((uint32_t)buffer[1 + bitPos]) << chanBits;
    chanBits += 8;

    /* when we got enough (10) bits, treat this as a sample */
    if(chanBits >= 10)
    {
      int chanOffset = 0;
      
      /* handle channel multiplexing */
      if(channel > 3 && upperEight) 
      {
        chanOffset = 4;
      }
      channelValues[chanOffset + channel++] = (value & 0x3FF);
      /* keep remaining bits */
      value >>= 10;
      chanBits -= 10;
    }
  }
}

typedef struct 
{
  uint32_t filled = 0;
  uint32_t maxLen = 0;
  uint8_t receivedBuffer[256];
} telemetryBuffer;

void IRAM_ATTR telemetryAdd(uint32_t direction, uint8_t *buffer, uint32_t length)
{
  static telemetryBuffer buffers[2];
  const char *dir = direction ? "D" : "U";
  
  if(buffers[direction].filled == 0)
  {
    if(buffer[0] == 0xDA && buffer[1] == 0x09)
    {
      int thisLen = length - 3;
      
      buffers[direction].maxLen = buffer[2];
      buffers[direction].filled = 0;
      
      memcpy(&buffers[direction].receivedBuffer[buffers[direction].filled], &buffer[3], thisLen);
      buffers[direction].filled += thisLen;
    }
    /*
      DA 0F 02 22 47 64 9B
             |  |  |  |  \_ crc
             |  |  |  \_ TQly
             |  |  \_ TSNR
             |  \_ TRSS
             \_ RFMD
    */
  }
  else
  {
    memcpy(&buffers[direction].receivedBuffer[buffers[direction].filled], buffer, length);
    buffers[direction].filled += length;
  }

  if(buffers[direction].maxLen > 0 && buffers[direction].filled >= buffers[direction].maxLen)
  {
    bool parsed = false;
    uint8_t *receivedBuffer = buffers[direction].receivedBuffer;

    switch(receivedBuffer[0])
    {
      /*
        0x08 Battery sensor
        Payload:
        uint16_t    Voltage ( mV * 100 )
        uint16_t    Current ( mA * 100 )
        uint24_t    Fuel ( drawn mAh )
        uint8_t     Battery remaining ( percent )
      */
      case 0x08:
      {
        uint16_t voltage = (receivedBuffer[1] << 8) | receivedBuffer[2];
        uint16_t current = (receivedBuffer[3] << 8) | receivedBuffer[4];
        uint32_t drawn = (receivedBuffer[5] << 16) | (receivedBuffer[6] << 8) | receivedBuffer[7];
        uint8_t remain = receivedBuffer[8];

        Serial.printf("%s: Battery: %2.1fV, %2.1fmA, %dmAh, %d%%\n", dir, (float)voltage / 10.0f, (float)current / 10.0f, drawn, remain);
        parsed = true;
        break;
      }
      
      case 0x28:
      {
        Serial.printf("%s: %02X <- %02X discover\n", dir, receivedBuffer[1], receivedBuffer[2]);
        parsed = true;
        break;
      }
      
      case 0x29:
      {
        Serial.printf("%s: %02X <- %02X response '%s'\n", dir, receivedBuffer[1], receivedBuffer[2], &receivedBuffer[3]);
        parsed = true;
        break;
      }
      
      /*
        0x2B config entry response
        2B [dst] [src] [ID] [cont] [parentID] [flags] [name] 00
        
        [dst] EA = Transmitter
              EC = Receiver
        [cont] = 0 is last packet, else number of following
        [flags] 0x0B = has children list after name, terminated with 0xFF
                0x09 = has option list after name, 00, default value somehow
                0x08 = unknown
                0x0D = executable
                0x80 = hidden
      */
      case 0x2B:
      {
        static uint8_t lastId = 0xFF;
        
        if(receivedBuffer[3] != lastId)
        {
          lastId = receivedBuffer[3];
          switch(receivedBuffer[6] & 0x0F)
          {
            case 0x08:
              Serial.printf("%s: Menu #%d - %s\n", dir, receivedBuffer[3], &receivedBuffer[7]);
              break;
            case 0x09:
              Serial.printf("%s: Menu #%d - %s options: %s\n", dir, receivedBuffer[3], &receivedBuffer[7], &receivedBuffer[7+1+strlen((char*)&receivedBuffer[7])]);
              break;
            case 0x0B:
              Serial.printf("%s: Menu #%d - %s (children)\n", dir, receivedBuffer[3], &receivedBuffer[7]);
              break;
            case 0x0C:
              Serial.printf("%s: Menu #%d - ", dir, receivedBuffer[3]);
              Serial.printf("%s ", &receivedBuffer[7]);
              Serial.printf("%s\n", &receivedBuffer[7+1+strlen((char*)&receivedBuffer[7])]);
              break;
            case 0x0D:
              Serial.printf("%s: Menu #%d - %s (exec)\n", dir, receivedBuffer[3], &receivedBuffer[7]);
              break;
          }
        }
        parsed = true;
        break;
      }
    }

    if(!parsed)
    {
      char buf[512];
      
      for(int pos = 0; pos < buffers[direction].filled; pos++)
      {
        sprintf(&buf[pos*3], "%02X ", buffers[direction].receivedBuffer[pos]);
      }
      for(int pos = 0; pos < buffers[direction].filled; pos++)
      {
        char ch = '.';
        if(buffers[direction].receivedBuffer[pos] >= ' ' && buffers[direction].receivedBuffer[pos] <= 'z')
        {
          ch = buffers[direction].receivedBuffer[pos];
        }
        sprintf(&buf[buffers[direction].filled*3 + pos], "%c", ch);
      }
      Serial.printf("%s: Telemetry (%u): %s\n", dir, buffers[direction].filled, buf);
    }

    buffers[direction].maxLen = 0;
    buffers[direction].filled = 0;
  }
}

void IRAM_ATTR parseDownlink(uint8_t *buffer)
{
  bool first = buffer[0] & 0x80;
  uint8_t type = buffer[0] & 0x0F;
  char buf[128];
  
  switch(type)
  {
    case 1:
      Serial.printf("D: type 0x%02X\n", buffer[0]);
      break;

    case 3:
      //parseChannels(buffer);
      break;
      
    case 0x0B:
    {
      uint8_t len = (buffer[2] & 0xF0) >> 4;

      /* as we dont check CRC, invalid packets cause trouble */
      //telemetryAdd(0, &buffer[3], len);
      
      break;
    }

    default:
      for(int pos = 0; pos < 0x0D; pos++)
      {
        sprintf(&buf[pos*3], "%02X ", buffer[pos]);
      }
      for(int pos = 0; pos < 0x0D; pos++)
      {
        char ch = '.';
        if(buffer[pos] >= ' ' && buffer[pos] < 'z')
        {
          ch = buffer[pos];
        }
        sprintf(&buf[0x0D*3 + pos], "%c", ch);
      }
      Serial.printf("D: %s\n", buf);
      break;
  }
}

void IRAM_ATTR parseUplink(uint8_t *buffer)
{
  bool first = buffer[0] & 0x80;
  uint8_t type = buffer[0] & 0x0F;
  uint8_t lenExtra = (buffer[11] & 0xF0) >> 4;
  uint8_t flagExtra = (buffer[11] & 0x0F);
  char buf[128];

  switch(type)
  {
    case 1:
      Serial.printf("U: 0x%02X at %d\n", buffer[0], hoppingPos);
      parseChannels(buffer);
      break;
      
    case 3:
      parseChannels(buffer);
      break;

    default:
      for(int pos = 0; pos < 0x17; pos++)
      {
        sprintf(&buf[pos*3], "%02X ", buffer[pos]);
      }
      Serial.printf("U: %s\n", buf);
      break;
  }

  if(lenExtra)
  {
    for(int pos = 0; pos < lenExtra + 1; pos++)
    {
      sprintf(&buf[pos*3], "%02X ", buffer[11 + pos]);
    }
    for(int pos = 0; pos < lenExtra + 1; pos++)
    {
      char ch = '.';
      if(buffer[11 + pos] >= ' ' && buffer[11 + pos] < 'z')
      {
        ch = buffer[pos];
      }
      sprintf(&buf[(lenExtra + 1)*3 + pos], "%c", ch);
    }
    
    /* as we dont check CRC, invalid packets cause trouble */
    //telemetryAdd(1, &buffer[12], lenExtra);
    Serial.printf("U: %s\n", buf);
  }
}

uint8_t IRAM_ATTR chanPercent(int channel)
{
  uint32_t value = channelValues[channel];

  if(value < 0xA1)
  {
    value = 0xA1;
  }
  value -= 0xA1;
  if(value > 0x2BB)
  {
    value = 0x2BB;
  }

  return (value * 100) / 0x2BB;
}

void IRAM_ATTR fifo_isr_body(uint64_t timestamp)
{
  bool restart = false;
  bool scanDownlink = false;

  uint8_t rssi = SX1276.readRssi();
  int32_t fei = SX1276.readFei();
  
  switch(state)
  {
    case 401:
    case 402:
    {
      uint8_t receive_buffer[64];
      char buf[128];
      
      if(rssi < scanRssi)
      {
        scanRssi = rssi;
      }
      
      for(int fifoPos = 0; fifoPos < currentLength; fifoPos++)
      {
        receive_buffer[fifoPos] = SX1276.readFifo();
      }
      for(int pos = 0; pos < 0x17; pos++)
      {
        sprintf(&buf[pos*3], "%02X ", receive_buffer[pos]);
      }
    
      Serial.printf("ch#%02d: %s, RSSI -%03d, FEI %i\n", currentChannel, buf, rssi/2, fei);
      SX1276.setRx();
      break;
    }
     
    case 201:
    {
      Serial.printf("Sync to slot 0\n");
      setChan(0, false);
      zeroSyncTimePos = 0;
      state++;

      /* reset statistics - except resyncs */
      statUpScheduled = 0;
      statUpLoss = 0;
      statUpReceived = 0;
      statUpHeadroom = 0;
      statDownScheduled = 0;
      statDownLoss = 0;
      statDownReceived = 0;
      statDownHeadroom = 0;
      statAverageScheduleLength = 0;
      learnSchedLength = 6676;
      break;
    }
    
    case 202:
    {
      /*  find the channel 0 packet which is our hopping sequence start position by the time delta 
       *  these numbers are for sure not generic, so maybe use some other way to detect a distinguishable
       *  pattern that gives us the start of the hopping sequence
       */
      if(prevTimestamp != 0)
      {
        uint32_t delta = (uint32_t)(timestamp - prevTimestamp);
        
        if(delta > 373000 && delta < 374000)
        {
          prevTimestamp = timestamp + 200;
          startTimestamp = prevTimestamp;
          freqCorrectionDown = 0;
          freqCorrectionUp = 0;
          hoppingPos = 1;
          setChan(0, true);
          timerAlarmWrite(scheduleTimer, prevTimestamp + 2700, false);
          timerAlarmEnable(scheduleTimer);
          Serial.printf("Synced\n");
          state++;
          break;
        }
      }

      prevTimestamp = timestamp;
      SX1276.setRx();
      break;
    }

    case 203:
    {
      lastPacketTimestamp = timestamp;

      uint8_t receive_buffer[64];
      
      for(int fifoPos = 0; fifoPos < currentLength; fifoPos++)
      {
        receive_buffer[fifoPos] = SX1276.readFifo();
      }
    
      if(!currentDirectionDown)
      {
        digitalWrite(LED, HIGH);
        statRssiUp = (31.0f*statRssiUp + rssi) / 32.0f;
        plotRssiUp->AddSample(rssi * -0.5f);
        freqCorrectionUp += fei / 1000.0f;
        plotFCorrUp->AddSample(fei);
        
        parseUplink(receive_buffer);
      }
      else
      {
        digitalWrite(LED, LOW);
        statRssiDown = (31.0f*statRssiDown + rssi) / 32.0f;
        plotRssiDown->AddSample(rssi * -0.5f);
        freqCorrectionDown += fei / 1000.0f;
        plotFCorrDown->AddSample(fei);
        
        parseDownlink(receive_buffer);
      }
      break;
    }

    
    case 101:
    {
      Serial.printf("Sync to slot 0\n");
      setChan(0, scanDownlink);
      zeroSyncTimePos = 0;
      state++;
      break;
    }

    case 102:
    {
      if(synced)
      {
        uint32_t channel = 0;
        int32_t remainder = 0;
        bool match = calc_channel(&channel, &remainder, timestamp, prevSyncStart);
        
        if(zeroSyncTimePos == 0)
        {
          Serial.printf("<-- %7u --> ", (uint32_t)(timestamp - prevSyncStart));
        }
        Serial.printf("dist ch%u (%u)\n", channel, remainder);
        
        for(int pos = 0; pos < 3; pos++)
        {
          if(channelExpects[(zeroSyncTimePos + pos)%3] == channel)
          {
            zeroSyncTimePos += pos;
            zeroSyncTimePos %= 3;
            break;
          }
          Serial.printf("skipped one slot(?)\n");
        }
      }

      zeroSyncTime[zeroSyncTimePos++] = timestamp;
      
      if(zeroSyncTimePos >= 3)
      {
        Serial.printf("channel map: ");

        for(int pos = 0; pos < 3; pos++)
        {
          uint32_t channel = 0;
          int32_t remainder = 0;
          
          bool match = calc_channel(&channel, &remainder, zeroSyncTime[pos], zeroSyncTime[0]);
          Serial.printf("ch%u (%u) ", channel, remainder);

          channelExpects[pos] = channel;
        }
        Serial.printf("\n");     

        prevSyncStart = zeroSyncTime[0];
        synced = true;
        setChan(0, scanDownlink);
        zeroSyncTimePos = 0;
        break;
      }
      setChan(0, scanDownlink);
      break;
    }
      
    case 1:
      //Serial.printf("Sync\n");
      setChan(0, scanDownlink);
      prevTimestamp = 0;
      state++;
      break;
      
    case 2:
    {
      /*  find the channel 0 packet which is our hopping sequence start position by the time delta 
       *  these numbers are for sure not generic, so maybe use some other way to detect a distinguishable
       *  pattern that gives us the start of the hopping sequence
       */
      if(prevTimestamp != 0)
      {
        uint32_t delta = (uint32_t)(timestamp - prevTimestamp);
        
        if(delta > 373000 && delta < 374000)
        {
          startTimestamp = timestamp;
          setChan(checkChan, scanDownlink);
          Serial.printf("Channel %d is next\n", checkChan);
          state++;
          break;
        }
      }

      prevTimestamp = timestamp;
      SX1276.setRx();
      break;
    }

    case 3:
    {
      addDelta(checkChan, timestamp, startTimestamp);

      if(retries++ >= 3*20 || timeslotMap[checkChan].entries >= 3)
      {
        retries = 0;
        checkChan++;
        checkChan %= 50;
        
        /* done */
        if(checkChan == 0)
        {
          state++;
          break;
        }

        state = 2;
        setChan(0, scanDownlink);
        break;
      }

      SX1276.setRx();
      break;
    }
  }
}


void IRAM_ATTR fifoReadyIsr()
{
  uint64_t timestamp_dummy = timerRead(scheduleTimer);
  uint64_t timestamp = timerRead(scheduleTimer);
  
  /* https://esp32.com/viewtopic.php?t=1292 */
  static uint32_t cp0_regs[18];
  uint32_t cp_state = xthal_get_cpenable(); // get FPU state
  
  if(cp_state)
  {
    xthal_save_cp0(cp0_regs); // Save FPU registers
  }
  else
  {
    xthal_set_cpenable(1); // enable FPU
  }
  
  fifo_isr_body(timestamp);
  
  if(cp_state)
  {
    xthal_restore_cp0(cp0_regs); // Restore FPU registers
  }
  else
  {
    xthal_set_cpenable(0); // turn it back off
  }
}

void IRAM_ATTR timer_isr()
{
  uint64_t timestamp_dummy = timerRead(scheduleTimer);
  uint64_t timestamp = timerRead(scheduleTimer);
  
  /* https://esp32.com/viewtopic.php?t=1292 */
  static uint32_t cp0_regs[18];
  uint32_t cp_state = xthal_get_cpenable(); // get FPU state
  
  if(cp_state)
  {
    xthal_save_cp0(cp0_regs); // Save FPU registers
  }
  else
  {
    xthal_set_cpenable(1); // enable FPU
  }
  
  timer_isr_body(timestamp);
  
  if(cp_state)
  {
    xthal_restore_cp0(cp0_regs); // Restore FPU registers
  }
  else
  {
    xthal_set_cpenable(0); // turn it back off
  }
}

void save_config()
{
  File file = SPIFFS.open("/config.bin", "w");
  if(!file)
  {
    Serial.println("[E] failed to open file for writing");
  }
  else
  {
    file.write((const uint8_t *)&config, sizeof(config));
    file.close();
    Serial.println("[i] wrote config to SPIFFS");
  }
}

void load_config()
{
  File file = SPIFFS.open("/config.bin", "r");
  if(!file)
  {
    Serial.println("[E] failed to open config file for reading");
  }
  else
  {
    file.read((uint8_t *)&config, sizeof(receiver_config));
    file.close();
    Serial.println("[i] read config from SPIFFS");
  }
  
  if(config.magic != CONFIG_MAGIC)
  {
    Serial.println("[i] incorrect magics or version, reinit");
    config.magic = CONFIG_MAGIC;
    
    memset((void *)&config.hoppingSequence, 0x00, sizeof(config.hoppingSequence));
    save_config();
  }
}

void loop_core1(void *unused);

void setup()
{ 
  Serial.begin(250000);

  Serial.printf("\n\n\n");

  Serial.printf("[i] SDK:          '%s'\n", ESP.getSdkVersion());
  Serial.printf("[i] CPU Speed:    %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("[i] Chip Id:      %06X\n", ESP.getEfuseMac());
  Serial.printf("[i] Flash Mode:   %08X\n", ESP.getFlashChipMode());
  Serial.printf("[i] Flash Size:   %08X\n", ESP.getFlashChipSize());
  Serial.printf("[i] Flash Speed:  %d MHz\n", ESP.getFlashChipSpeed() / 1000000);
  Serial.printf("[i] Heap          %d/%d\n", ESP.getFreeHeap(), ESP.getHeapSize());
  Serial.printf("[i] SPIRam        %d/%d\n", ESP.getFreePsram(), ESP.getPsramSize());
  Serial.printf("\n");
  
  Serial.printf("[i] Starting on core %d\n", xPortGetCoreID());
  Serial.printf("[i]   Setup SPIFFS\n");
  if(!SPIFFS.begin(true))
  {
      Serial.println("      [E] mount failed");
  }
  Serial.printf("[i]   Load config\n");
  load_config();

  Serial.printf("[i]   Setup SX1276\n");
  if(!SX1276.begin())
  {
      Serial.println("[      [E] init failed");
  }
 
  Serial.printf("[i]   Setup Display\n");
  
  Display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_128_64);
  Display->init();
  Display->setContrast(5, 5, 0);
  Display->flipScreenVertically();  
  Display->setFont(ArialMT_Plain_10);
  Display->clear();
  Display->display();
  

  Serial.printf("[i]   Setup Timers/Interrupts\n");
  pinMode(DIO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(DIO0), fifoReadyIsr, RISING);

  scheduleTimer = timerBegin(2, 80, true);
  timerAttachInterrupt(scheduleTimer, &timer_isr, true);
  
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  Serial.printf("[i]   Setup Plots\n");
  plotRssiUp = new PlotClass(Display, "RSSI up", 16, 64);
  plotRssiDown = new PlotClass(Display, "RSSI dn", 16, 64);
  plotFCorrUp = new PlotClass(Display, "Corr up", 56, 64);
  plotFCorrDown = new PlotClass(Display, "Corr dn", 56, 64);
  plotSchedLen = new PlotClass(Display, "PktDly", 56, 64);
  plotFreq = new PlotClass(Display, "PktRate", 56, 64);
  plotLossUp = new PlotClass(Display, "Loss up", 56, 64);
  plotLossDown = new PlotClass(Display, "Loss dn", 56, 64);
  plotScan = new PlotClass(Display, "Scan", 1, 50);
  Serial.println("Setup done");

  /* start with the hop-following sniff code */
  state = 200;
}


void loop()
{
  char msg[32];
  static int lastTime = 0;
  static int buttonPressTime = 0;
  static int lastButtonPressTime = 0;
  static int currentDisplay = 0;
  static float lastContrast = 0;
  int displayCount = 9;
  uint32_t currentTime = millis();

  const char *menu_entries[] = { "Hop Scan", "Band Scan", "Follow", "Analyze", "----" };
  static int menu_pos = 0;
  static uint32_t currentScanChan = 0;
  

  bool buttonPressed = digitalRead(0) == LOW;

  buttonPressed |= chanPercent(8) > 80;

  float contrast = 255;
  
  if(currentTime - lastButtonPressTime > 10000)
  {
    contrast = 0;
  }

  if((int)contrast != (int)lastContrast)
  {
    lastContrast = (contrast + 15.0f*lastContrast) / 16.0f;
    Display->setBrightness(lastContrast);
  }

  
  /* when button gets pressed, save timestamp */
  if(buttonPressed && buttonPressTime == 0)
  {
    lastButtonPressTime = currentTime;
    buttonPressTime = currentTime;
  }
  
  /* if button is or was pressed */
  if(buttonPressTime > 0)
  {
    int pressDuration = currentTime - buttonPressTime;

    /* no matter if pressed, when it was longer than two seconds */
    if(pressDuration > 1000)
    {
      buttonPressTime = -1;
      if(state == 300)
      {
        switch(menu_pos)
        {
          case 0:
            /* start hopping scan */
            state = 0;
            break;
            
          case 1:
            /* start band scan */
            state = 400;
            break;
            
          case 2:
            /* follow hopping */
            state = 200;
            break;
            
          case 3:
            /* analyze channel 0 */
            state = 100;
            break;
        }
      }
      else
      {
        state = 300;
        digitalWrite(LED,LOW);
        timerAlarmDisable(scheduleTimer);
      }
    }
    else if(!buttonPressed && pressDuration > 100)
    {
      if(state == 300)
      {
        menu_pos = (menu_pos + 1) % COUNT(menu_entries);
      }
      else if(state == 401)
      {
        state = 402;
        currentScanChan = 0;
        setChan(currentScanChan, false);
        scanRssi = 200;
      }
      else if(state == 402)
      {
        state = 402;
        currentScanChan = (currentScanChan + 1) % 50;
        setChan(currentScanChan, false);
        scanRssi = 200;
      }
      else
      {
        currentDisplay = (currentDisplay + 1) % displayCount;
      }
      buttonPressTime = -1;
    }
  }

  /* when start time marked as "reset", reset if button is released again */
  if(buttonPressTime < 0)
  {
    if(!buttonPressed)
    {
      buttonPressTime = 0;
    }
  }
  
  switch(state)
  {
    case 400:
      for(int pos = 0; pos < 50; pos++)
      {
        plotScan->SetSample(pos, -100);
      }
      freqCorrectionUp = 0;
      freqCorrectionDown = 0;
      state++;
      break;
      
    case 401:
    {
      if(currentTime - lastTime > 100 || scanRssi != 200)
      {
        float oldValue = plotScan->GetSample(currentScanChan);
        float value = scanRssi * -0.5f;
        if(oldValue < value)
        {
          plotScan->SetSample(currentScanChan, value);
        }
        currentScanChan = (currentScanChan + 1) % 50;
        setChan(currentScanChan, false);
        scanRssi = 200;
        Display->clear();
        plotScan->DrawFullPlot(0, 0, 54, 60);
        char msg[33];
        snprintf(msg, 32, "ch#%d", currentScanChan);
        Display->setFont(ArialMT_Plain_16);
        Display->drawString(68, 48, msg);
        Display->display();
      }
      break;
    }
      
    case 402:
    {
      if(currentTime - lastTime > 50)
      {
        float oldValue = plotScan->GetSample(currentScanChan);
        float value = scanRssi * -0.5f;
        if(oldValue < value)
        {
          plotScan->SetSample(currentScanChan, value);
        }
        Display->clear();
        plotScan->DrawFullPlot(0, 0, 54, 60);
        char msg[33];
        snprintf(msg, 32, "ch#%d %c", currentScanChan, (scanRssi == 200)?' ':'*');
        Display->setFont(ArialMT_Plain_16);
        Display->drawString(68, 48, msg);
        Display->display();
        scanRssi = 200;
      }
      break;
    }
      
    case 300:
      Display->setFont(ArialMT_Plain_24);
      Display->clear();
      Display->drawString(0, 0, "Menu");
      Display->setFont(ArialMT_Plain_10);
      for(int pos = 0; pos < COUNT(menu_entries); pos++)
      {
        if(pos > 3)
        {
          break;
        }
        int entry = menu_pos + pos;
        const char *menu_name = menu_entries[entry % COUNT(menu_entries)];
        if(entry == menu_pos)
        {
          Display->drawString(8, 26 + 1 + pos * 10, menu_name);
          Display->drawString(8+1, 26 + pos * 10, menu_name);
          Display->drawString(8+1, 26 + 1 + pos * 10, menu_name);
        }
        Display->drawString(8, 26 + pos * 10, menu_name);
      }
      Display->display();
      break;
      
    case 200:
      Display->setFont(ArialMT_Plain_24);
      Display->clear();
      Display->drawString(0, 0, "Search...");
      Display->display();

      Serial.printf("Init hopping follow\n");

      setChan(0, false);
      state++;
      break;

    case 202:
    {
      Display->setFont(ArialMT_Plain_24);
      Display->clear();
      Display->drawString(0, 0, "Sync...");
      Display->display();
      
      break;
    }

    case 203:
    {
      if(0 && currentTime - lastTime > 500)
      {
        lastTime = currentTime;
        if(statUpScheduled && statDownScheduled)
        {
          Serial.printf("Statistics:\n");
          Serial.printf("   Up scheduled: %6u\n", statUpScheduled);
          Serial.printf("           recv: %6u\n", statUpReceived);
          Serial.printf("           loss: %6u (%d%%)\n", statUpLoss, 100*statUpLoss/statUpScheduled);
          Serial.printf("           head:   %4.0f\n", statUpHeadroom);
          Serial.printf("        fr.corr:   %4i\n", (int16_t)freqCorrectionUp);
          Serial.printf("            dBm:  -%2.1f\n", (float)statRssiUp / 2);
          Serial.printf("   Dn scheduled: %6u\n", statDownScheduled);
          Serial.printf("           recv: %6u\n", statDownReceived);
          Serial.printf("           loss: %6u (%d%%)\n", statDownLoss, 100*statDownLoss/statDownScheduled);
          Serial.printf("           head:   %4.0f\n", statDownHeadroom);
          Serial.printf("        fr.corr:   %4i\n", (int16_t)freqCorrectionDown);
          Serial.printf("            dBm:  -%2.1f\n", (float)statRssiDown / 2);
          Serial.printf("  Sched len avg:   %4.0f\n", statAverageScheduleLength);
          Serial.printf("  Sched len set:   %4u\n", learnSchedLength);
          Serial.printf("        Resyncs: %6u\n", statResyncs);
          Serial.printf("\n");
        }
      }

      Display->clear();
      switch(currentDisplay)
      {
        /* show values */
        case 0:
        {
          if(statUpScheduled && statDownScheduled)
          {
            Display->setFont(ArialMT_Plain_24);
            Display->drawString(0, 0, "CRSF");
            
            Display->setFont(ArialMT_Plain_10);
            for(int chan = 0; chan < 12; chan++)
            {
              uint8_t pct = chanPercent(chan);
              sprintf(msg, "%d %3d%%", chan, pct);
              Display->drawString(3 + 43*(chan/4), 24 + (chan%4)*10, msg);
            }
            sprintf(msg, "Ch %d %c", currentChannel, currentDirectionDown ? 'D':'U');
            Display->drawString(68, 0, msg);
            sprintf(msg, "-%2u -%2u", statRssiUp / 2, statRssiDown / 2);
            Display->drawString(68, 11, msg);
            Display->drawLine(0, 24, 0, 64);
            Display->drawLine(43, 24, 43, 64);
            Display->drawLine(86, 24, 86, 64);
            Display->drawLine(127, 24, 127, 64);
          }
          else
          {  
            Display->setFont(ArialMT_Plain_24);
            Display->drawString(0, 0, "Waiting");
          }
          break;
        }

        case 1:
          plotRssiUp->DrawFullPlot(0, 0, 60, 60);
          break;

        case 2:
          plotRssiDown->DrawFullPlot(0, 0, 60, 60);
          break;

        case 3:
          plotFCorrUp->DrawFullPlot(0, 0, 60, 60);
          break;

        case 4:
          plotFCorrDown->DrawFullPlot(0, 0, 60, 60);
          break;

        case 5:
          plotLossUp->DrawFullPlot(0, 0, 60, 60, false);
          break;

        case 6:
          plotLossDown->DrawFullPlot(0, 0, 60, 60, false);
          break;

        case 7:
          plotFreq->DrawFullPlot(0, 0, 60, 60, false);
          break;

        case 8:
          plotSchedLen->DrawFullPlot(0, 0, 60, 60);
          break;
      }
      Display->display();
      
      break;
    }
      
    case 100:
      Display->setFont(ArialMT_Plain_10);
      //Display->drawString(0, 0, "Init channel 0 analysis");
      Display->display();
      Serial.printf("Init channel 0 analysis\n");

      setChan(0, false);
      state++;
      break;
      
    case 0:
    {
      Display->setFont(ArialMT_Plain_24);
      Display->clear();
      Display->drawString(0, 0, "Hop scan");
      Display->setFont(ArialMT_Plain_10);
      Display->drawString(0, 30, "Initializing...");
      Display->display();
      
      Serial.printf("Init hopping scan\n");
      addDelta(0, 0, 0);

      SX1276.setStandby();
      SX1276.setStandby();
      setChan(0, false);
      SX1276.setRx();
      state++;
      break;
    }
      
    case 3:
    {
      Display->setFont(ArialMT_Plain_24);
      Display->clear();
      Display->drawString(0, 0, "Hop scan");
      Display->setFont(ArialMT_Plain_10);
      sprintf(msg, "Channel #%u (%u/3)", checkChan, timeslotMap[checkChan].entries);
      Display->drawString(0, 26, msg);
      Display->drawString(0, 38, "Map:");

      int mapX = 8;
      int mapY = 50;

      Display->drawRect(mapX, mapY, 2 + 50 * 2 + 2, 2 + 3 * 2 + 2);
      for(int chan = 0; chan < 50; chan++)
      {
        for(int entry = 0; entry < timeslotMap[chan].entries; entry++)
        {
          int hop = timeslotMap[chan].slots[entry];
          int x = mapX + 2 + (hop % 50) * 2;
          int y = mapY + 2 + (hop / 50) * 2;
          Display->setPixel(x, y);
          Display->setPixel(x+1, y);
          Display->setPixel(x, y+1);
          Display->setPixel(x+1, y+1);
        }
      }
      
      Display->display();
      
      break;
    }
    
    case 4:
    {
      Serial.printf("\n");
      Serial.printf("-----------------------\n");
      Serial.printf(" Unsorted hopping list\n");
      Serial.printf("-----------------------\n");
      for(int chan = 0; chan < COUNT(timeslotMap); chan++)
      {
        for(int pos = 0; pos < timeslotMap[chan].entries; pos++)
        {
          Serial.printf("slot #%03d channel #%02d\n", timeslotMap[chan].slots[pos], chan);
        }
      }
      Serial.printf("\n");
      Serial.printf("-----------------------\n");
      Serial.printf("  Sorted hopping list\n");
      Serial.printf("-----------------------\n");
      Serial.printf("{ ");
      for(int hop = 0; hop < 150; hop++)
      {
        for(int chan = 0; chan < COUNT(timeslotMap); chan++)
        {
          for(int pos = 0; pos < timeslotMap[chan].entries; pos++)
          {
            if(hop == timeslotMap[chan].slots[pos])
            {
              Serial.printf("%d%s", chan, (hop != 149)?", ":" ");
              config.hoppingSequence[hop] = chan;
            }
          }
        }
      }
      Serial.printf("}");
      save_config();
      state = 200;
      break;
    }
  }
}

