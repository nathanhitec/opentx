/*
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

//Telem in/out not implemented for GCSV2
Fifo<uint8_t, TELEMETRY_FIFO_SIZE> telemetryFifo;
uint32_t telemetryErrors = 0;
static void telemetryInitDirPin()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = TELEMETRY_DIR_GPIO_PIN;
    GPIO_Init(TELEMETRY_DIR_GPIO, &GPIO_InitStructure);
    TELEMETRY_DIR_INPUT();
}

void telemetryPortInit(uint32_t baudrate, uint8_t mode)
{
    telemetryInitDirPin();
}


void telemetryPortInvertedInit(uint32_t baudrate)
{
   
}

void telemetryPortInvertedRxBit()
{
   
}

void telemetryPortSetDirectionOutput()
{

}

void sportWaitTransmissionComplete()
{
   
}

void telemetryPortSetDirectionInput()
{
   
}

void sportSendByte(uint8_t byte)
{
    
}

void sportStopSendByteLoop()
{
  
}

void sportSendByteLoop(uint8_t byte)
{
   
}

void sportSendBuffer(const uint8_t* buffer, uint32_t count)
{
   
}

extern "C" void TELEMETRY_DMA_TX_IRQHandler(void)
{
   
}

#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)
extern "C" void TELEMETRY_USART_IRQHandler(void)
{
   
}

void check_telemetry_exti()
{
    
}

#if defined(TELEMETRY_EXTI_IRQHandler)
extern "C" void TELEMETRY_EXTI_IRQHandler(void)
{
   
}
#endif

extern "C" void TELEMETRY_TIMER_IRQHandler()
{
   
}

bool telemetryGetByte(uint8_t* byte)
{
    return false;
}

void telemetryClearFifo()
{

}