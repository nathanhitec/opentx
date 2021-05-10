#include "opentx.h"

ModuleFifo debugeFifo;

void debugHardwareInit()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //TODO: figure out correct settings for the pins
    GPIO_InitStructure.GPIO_Pin = DEBUG_SWDIO_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(DEBUG_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DEBUG_SWDCLK_GPIO_PIN;
    GPIO_Init(DEBUG_GPIO, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(DEBUG_GPIO, DEBUG_SWDIO_GPIO_PIN_SOURCE, GPIO_AF_SWJ);
    GPIO_PinAFConfig(DEBUG_GPIO, DEBUG_SWDCLK_GPIO_PIN_SOURCE, GPIO_AF_SWJ);




    /*
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN | DEBUG_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DEBUG_USART_GPIO, &GPIO_InitStructure);

    GPIO_PinAFConfig(DEBUG_USART_GPIO, DEBUG_USART_TX_GPIO_PinSource, DEBUG_USART_GPIO_AF);
    GPIO_PinAFConfig(DEBUG_USART_GPIO, DEBUG_USART_RX_GPIO_PinSource, DEBUG_USART_GPIO_AF);
    
    // UART config
    USART_DeInit(DEBUG_USART);
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(DEBUG_USART, &USART_InitStructure);
    USART_Cmd(DEBUG_USART, ENABLE);

    debugeFifo.clear();

    USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(DEBUG_USART, 6);
    NVIC_EnableIRQ(DEBUG_USART_IRQn);
    */
}

//TODO: add debug support on the pi



