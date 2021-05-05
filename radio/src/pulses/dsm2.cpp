/*
*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */




#include "opentx.h"

// SRXL control bits
#define SRXL_NORMAL_CHANS           16

#define SRXL_FRAME_SIZE             35

// 16 channel v2 of protocol
#define SRXL_FRAME_BEGIN_BYTE       0xA2


//TODO: wat is chan center ??
#define SRXL_CHAN_CENTER            992

#define BITLEN_SRXL                 (9*2) //115200 Baud , ~9us per bit, not sure why it's doubled


//PPM_PIN_SERIAL is not defined in current build, does an external reciver need to be on it to define this?
#if defined(PPM_PIN_SERIAL)
void putDsm2SerialBit(uint8_t bit)
{
  extmodulePulsesData.dsm2.serialByte >>= 1;
  if (bit & 1) {
    extmodulePulsesData.dsm2.serialByte |= 0x80;
  }
  if (++extmodulePulsesData.dsm2.serialBitCount >= 8) {
    *extmodulePulsesData.dsm2.ptr++ = extmodulePulsesData.dsm2.serialByte;
    extmodulePulsesData.dsm2.serialBitCount = 0;
  }
}

void sendByteDsm2(uint8_t b)     // max 10changes 0 10 10 10 10 1
{
  putDsm2SerialBit(0);           // Start bit
  for (uint8_t i=0; i<8; i++) {  // 8 data Bits
    putDsm2SerialBit(b & 0x80);
    b >>= 1;
  }

  putDsm2SerialBit(1);           // Stop bit
}

void putDsm2Flush()
{
  for (int i=0; i<16; i++) {
    putDsm2SerialBit(1);         // 16 extra stop bits
  }
}
#else
    
//This function is called when a bit transitions from low to high and vice versa
//v is always a multiple of BITLEN_SRXL, 16, 32, 48, etc.
//this v value essentially coincides with the pulse length, 1 bit has a pulse length of 16 not sure why it's not 8
void _send_1(uint8_t v)
  /* this looks doubious and in my logic analyzer
     output the low->high is about 2 ns late */
{
  //if (extmodulePulsesData.dsm2.index & 1)
    //v += 2;
  //else
    //v -= 2;

  *extmodulePulsesData.dsm2.ptr++ = v - 1;
  extmodulePulsesData.dsm2.index += 1;
}

//TODO: bits are reversed in oscilloscope, is this correct?, 0xA2 looks like 0x45
void sendByteDsm2(uint8_t b) // max 10 changes 0 10 10 10 10 1
{
  bool    lev = 0;
  uint8_t len = BITLEN_SRXL; // max val: 8*16 < 256
  for (uint8_t i=0; i<=8; i++) { // 8Bits + Stop=1
    bool nlev = b & 1; // lsb first
    if (lev == nlev) {
      len += BITLEN_SRXL;
    }
    else {
      _send_1(len); 
      len  = BITLEN_SRXL;
      lev  = nlev;
    }
    b = (b>>1) | 80; // shift in stop bit
  }
  _send_1(len); // stop bit (len is already BITLEN_SRXL)
}

void putDsm2Flush()
{
  if (extmodulePulsesData.dsm2.index & 1)
    *extmodulePulsesData.dsm2.ptr++ = 60000; //is this an arbitrarily large value? 
  else
    *(extmodulePulsesData.dsm2.ptr - 1) = 60000;
}
#endif


static uint16_t srxlCrc16(uint8_t* packet)
{
    uint16_t crc = 0;                      //seeds with 0
    uint8_t length = SRXL_FRAME_SIZE - 2;  //exclude 2 crc bytes at end of packet from length
    
    for(uint8_t i = 0; i < length; ++i)
    {
        crc = crc ^ ((uint16_t)packet[i] << 8);
        for(int b = 0; b < 8; b++)
        {
            if(crc & 8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}



void setupPulsesDSM2()
{
#if defined(PPM_PIN_SERIAL)
  extmodulePulsesData.dsm2.serialByte = 0 ;
  extmodulePulsesData.dsm2.serialBitCount = 0 ;
#else
  extmodulePulsesData.dsm2.index = 0;
#endif

  extmodulePulsesData.dsm2.ptr = extmodulePulsesData.dsm2.pulses;
  

  uint8_t SRXLData[SRXL_FRAME_SIZE];
  
  // Start Byte
  SRXLData[0] = SRXL_FRAME_BEGIN_BYTE;
  for (int i=0; i<SRXL_NORMAL_CHANS; i++) {
    int channel = g_model.moduleData[EXTERNAL_MODULE].channelsStart+i;
    int value = channelOutputs[channel] + 2 * PPM_CH_CENTER(channel) - 2*PPM_CENTER;
    
    //TODO: Do I need to do anything with the value or does the reciver do the work to calculate the pwm_out value?
    //in 4095 steps, 0x000 -> 0xFFF, pwm pulse 800us -> 2200us, do I need to center at 1500us?
 
    SRXLData[2*i+1] = ((value >> 8) & 0xff); //sends MSB first,
    SRXLData[2*i+2] = (value & 0xff);
  }
  uint16_t crc = srxlCrc16(SRXLData);
  
  sendByteDsm2(SRXL_FRAME_BEGIN_BYTE);
  
  for(int i=0; i < SRXL_NORMAL_CHANS; i++){
    sendByteDsm2(SRXLData[2*i+1]);
    sendByteDsm2(SRXLData[2*i+2]);
  }
  sendByteDsm2(uint8_t((crc >> 8) & 0xff));
  sendByteDsm2(uint8_t(crc & 0xff));

  putDsm2Flush();
}