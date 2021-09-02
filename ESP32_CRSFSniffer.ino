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

/*

    Rx  03 AA FC F7 1F 80 A1 84 12 4A 28 00 00 00 00 00 00 00 00 00 00 32 E2
        || |                           | || |                        | \___/
        || |                           | || |    telemetry payload   |  CRC
        || |   80 bits                 | ||  \______________________/
        || |    = 10 bits * 8 sticks   | | \_ telemetry flags?   
        || |                           |  \__ telemetry length     
        ||  \_________________________/        
        ||
        | \_ packet type: 3 normal FSK uplink data
        |                 1 50Hz mode uplink data?
        |                  
         \__ flags: 2 sticks 8-11



    Tx  03 75 64 00 00 00 00 00 00 00 00 00 0F
        || |   | |                        |  \_ CRC
        || |   | |    telemetry payload   |
        ||  \_/   \______________________/
        ||   |
        ||    \_ cycling metadata (RSSI etc)
        ||
        | \_ packet type: 3 normal FSK downlink data
        |                 1 50Hz mode downlink data? 
        |                 B telemetry payload  
        |
         \__ flags: 2 did not receive uplink for 8 times
                    8 metadata cycle start
                   

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
#define CONFIG_MAGIC 0xC0FFFEF6

typedef struct
{
  uint8_t entries;
  uint8_t slots[4];
} timeslotInfo_t;

typedef struct 
{
  uint32_t magic;
  uint8_t hoppingSequence[150];
  uint64_t startFreq;
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

uint8_t hoppingPos = 0;
uint32_t cfgBitrate = 85000;
uint32_t cfgFreqDev = 42300;
uint32_t cfgChanDist = 260010;
uint64_t cfgStartFreq = 860000000ULL;
bool enableAfc = false;

float freqCorrectionUp = 0;
float freqCorrectionDown = 0;

int16_t statRssiUp = 0;
int16_t statRssiDown = 0;
int16_t scanRssi = 0;
uint64_t lastPacketTimestamp = 0;

timeslotInfo_t timeslotMap[50];
float packet_ms = 6666.666666666f * 1.00101f;
hw_timer_t * scheduleTimer = NULL;

uint32_t zeroSyncTime[10];
uint32_t zeroSyncTimePos = 0;
uint64_t prevSyncStart = 0;
uint32_t channelExpects[10];
bool synced = false;
uint32_t losses = 0;

uint64_t freqScanFreq = 859500000ULL;
uint64_t freqScanFreqCur = 0;
uint32_t freqScanBw = 10000;


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
  uint32_t freq = config.startFreq + chanAbs * cfgChanDist + (downlink ? freqCorrectionDown : freqCorrectionUp);

  currentChannel = chan;
  currentDirectionDown = downlink;
  currentLength = downlink ? 0x0D : 0x17;
  
  SX1276.setStandby();
  SX1276.writeAfc(0);
  SX1276.writeFreq(freq);
  SX1276.writePayloadLength(currentLength);
  SX1276.writeRegister(0x0C, 0x23);

  /* AgcAutoOn and AGC PreambleDetect RxTrigger */
  SX1276.writeRxConfig(0x08 | 6 | (enableAfc ? 0x10 : 0));

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

int32_t IRAM_ATTR addDelta(uint8_t channel, uint64_t timestamp, uint64_t startTimestamp)
{
  uint64_t delta = timestamp - startTimestamp;
  uint32_t slotDelta = 0;
  int32_t remainder = 0;

  bool match = calc_channel(&slotDelta, &remainder, timestamp, startTimestamp);
  
  if(remainder > 10 || remainder < -10)
  {
    Serial.printf("OFF : Channel #%2u %3u (%7llu us, remainder %3i, %llu %llu)\n", channel, slotDelta, delta, remainder, timestamp, startTimestamp);
    return remainder;
  }
  timeslotInfo_t *slot = &timeslotMap[channel];

  for(int pos = 0; pos < slot->entries; pos++)
  {
    if(slot->slots[pos] == slotDelta)
    {
      Serial.printf("SKIP: Channel #%2u %3u (%7llu us, remainder %3i) - already contained\n", channel, slotDelta, delta, remainder);
      return remainder;
    }
  }

  if(slot->entries >= 4)
  {
    Serial.printf("FAIL: Channel #%2u %3u (%7llu us, remainder %3i) - already enough\n", channel, slotDelta, delta, remainder);
    return remainder;
  }
  
  slot->slots[slot->entries++] = slotDelta;
  Serial.printf("ADD : Channel #%2u %3u (%7llu us, remainder %3i, %llu %llu)\n", channel, slotDelta, delta, remainder, timestamp, startTimestamp);
  
  return remainder;
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
  bool alternate = buffer[0] & 0x20;
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
  int32_t afc = SX1276.readAfc();
  
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
        freqCorrectionUp += afc / 1000.0f;
        plotFCorrUp->AddSample(afc);
        
        parseUplink(receive_buffer);
      }
      else
      {
        digitalWrite(LED, LOW);
        statRssiDown = (31.0f*statRssiDown + rssi) / 32.0f;
        plotRssiDown->AddSample(rssi * -0.5f);
        freqCorrectionDown += afc / 1000.0f;
        plotFCorrDown->AddSample(afc);
        
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
      prevTimestamp = timestamp;
      Serial.printf("Sync at %d kHz fei %i afc %i\n", (uint32_t)(config.startFreq / 1000), fei, afc);

      if(!retries)
      {
        config.startFreq += afc;
        enableAfc = false;
      }
      retries++;

      if(retries > 2)
      {
        Serial.printf("Sync at %d kHz -> proceed\n", (uint32_t)(config.startFreq / 1000));
        prevTimestamp = 0;
        state++;
      }
      setChan(0, scanDownlink);
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
          //Serial.printf("delta %d\n", delta);
        
        if(delta > 365000 && delta < 374000)
        {
          startTimestamp = timestamp;
          setChan(checkChan, scanDownlink);
          Serial.printf("Channel %d is next\n", checkChan);
          state++;
          break;
        }
      }

      prevTimestamp = timestamp;
      setChan(0, scanDownlink);
      break;
    }

    case 3:
    {
      int32_t remainder = addDelta(checkChan, timestamp, startTimestamp);

      /* try to compensate clock drift */
      if(remainder != 0 && abs(remainder) < 10)
      {
        //packet_ms *= 1.0f + remainder / 100000.0f;
        startTimestamp += (remainder * packet_ms) / 100;
        //Serial.printf("packet_ms: %2.2f\n", packet_ms);
      }

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

        /* when the drift gets to high resync to compensate clock drift. only checks last packet in this channel, so leave some margin */
        if(/*(checkChan % 10) == 0 || */ abs(remainder) > 6)
        {
          state = 2;
          setChan(0, scanDownlink);
        }
        else
        {
          setChan(checkChan, scanDownlink);
        }
        break;
      }

      setChan(checkChan, scanDownlink);
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
    config.startFreq = cfgStartFreq;
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
  
  SX1276.writeBitrate(cfgBitrate);
  SX1276.writeFreqDev(cfgFreqDev);
 
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
  plotScan = new PlotClass(Display, "Scan", 1, 100);
  Serial.println("Setup done");

  /* start with the hop-following sniff code */
  state = 0;
}


void loop()
{
  char msg[32];
  static int lastTime = 0;
  static int lastUpdateTime = 0;
  static int buttonPressTime = 0;
  static int lastButtonPressTime = 0;
  static int currentDisplay = 0;
  static float lastContrast = 0;
  int displayCount = 9;
  uint32_t currentTime = millis();

  const char *menu_entries[] = { "Display", "Scan 868", "Scan 868 CE (ToDo)", "Scan 915", "Scan 915 AU (ToDo)", "Band Scan", "Freq Scan", "Analyze", "----" };
  static int menu_pos = 0;
  static uint32_t currentScanChan = 0;
  

  bool buttonPressed = digitalRead(0) == LOW;

  buttonPressed |= chanPercent(8) > 80;

  /* fade out display to prevent burn-in */
  float contrast = 255;
  
  if(currentTime - lastButtonPressTime > 60000)
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
            /* follow hopping */
            state = 200;
            break;
            
          case 1:
            /* start hopping scan */
            cfgChanDist = 260000;
            cfgStartFreq = 860000000;
            state = 0;
            break;
            
          case 2:
            /* start hopping scan */
            cfgChanDist = 112000;
            cfgStartFreq = 860000000;
            state = 0;
            break;
            
          case 3:
            /* start hopping scan */
            cfgChanDist = 260000;
            cfgStartFreq = 900000000;
            state = 0;
            break;
            
          case 4:
            /* start hopping scan */
            cfgChanDist = 112000;
            cfgStartFreq = 915000000;
            state = 0;
            break;
            
          case 5:
            /* start band scan */
            state = 400;
            break;
            
          case 6:
            /* start freq scan */
            state = 500;
            break;
            
          case 7:
            /* analyze channel 0 */
            state = 100;
            break;
            
          case 8:
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
    case 500:
      freqCorrectionUp = 0;
      freqCorrectionDown = 0;
      currentScanChan = 0;
      state++;
      break;

    case 501:
    {
      if(currentScanChan >= plotScan->ValueCount)
      {
        currentScanChan = 0;
      }
      freqScanFreqCur = freqScanFreq + currentScanChan * freqScanBw;
      SX1276.setStandby();
      SX1276.writeRxBw(freqScanBw);
      SX1276.writeRxConfig(0);
      SX1276.writeRegister(0x0E, 3);
      SX1276.writeFreq(freqScanFreqCur);
      SX1276.setRx();
      //plotScan->SetSample(currentScanChan, -100);
      state++;
      break;
    }

    case 502:
    {
      float oldValue = plotScan->GetSample(currentScanChan);
      float value = SX1276.readRssi() * -0.5f;
      if(oldValue < value)
      {
        plotScan->SetSample(currentScanChan, value);
      }
      
      if(currentTime - lastTime > 20)
      {
        lastTime = currentTime;
        currentScanChan++;
        state = 501;
      }

      if(currentTime - lastUpdateTime > 100)
      {
        lastUpdateTime = currentTime;
        Display->clear();
        plotScan->DrawFullPlot(0, 0, 100, 60, true, false);
        char msg[33];
        snprintf(msg, 32, "%d kHz", (uint32_t)( freqScanFreqCur /1000));
        Display->setFont(ArialMT_Plain_10);
        Display->drawString(64, 46, msg);
        Display->display();
      }
      
      break;
    }

      
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
        int entry = (menu_pos + pos + COUNT(menu_entries) - 1) % COUNT(menu_entries);
        const char *menu_name = menu_entries[entry];
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

      SX1276.init();

      setChan(0, false);
      state++;
      break;

    case 202:
    {
      Display->setFont(ArialMT_Plain_24);
      Display->clear();
      Display->drawString(0, 0, "Sync...");
      Display->display();
      enableAfc = true;
      
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
      SX1276.init();
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

      memset(&timeslotMap, 0x00, sizeof(timeslotMap));
      addDelta(0, 0, 0);

      freqCorrectionUp = 0;
      freqCorrectionDown = 0;
      SX1276.init();
      
      SX1276.writeAfcBw(100000);
      enableAfc = true;
      config.startFreq = cfgStartFreq;

      retries = 0;
      state++;
      
      setChan(0, false);
      break;
    }

    case 1:
    {
      bool skip = false;
      
      if(!retries)
      {
        if((currentTime - lastTime) > 500)
        {
           skip = true;
        }
      }
      else
      {
        if((currentTime - lastTime) > 2000)
        {
          Serial.printf("timed out\n");
          skip = true;
        }
      }
      
      if(skip)
      {
        lastTime = currentTime;

        config.startFreq += 50000;
        enableAfc = true;
        retries = 0;
        setChan(0, false);
        
        char msg[33];
        snprintf(msg, 32, "Check %d.%03d MHz", (uint32_t)(config.startFreq /1000000), (uint32_t)(config.startFreq /1000) % 1000);
        Display->setFont(ArialMT_Plain_24);
        Display->clear();
        Display->drawString(0, 0, "Hop scan");
        Display->setFont(ArialMT_Plain_10);
        Display->drawString(0, 30, msg);
        Display->display();

        Serial.printf("%s\n", msg);
      }
      break;
    }
      
    case 3:
    {
      Display->setFont(ArialMT_Plain_24);
      Display->clear();
      Display->drawString(0, 0, "Hop scan");
      Display->setFont(ArialMT_Plain_10);
      sprintf(msg, "Channel #%u (%u/3)", (checkChan+1), timeslotMap[checkChan].entries);
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

