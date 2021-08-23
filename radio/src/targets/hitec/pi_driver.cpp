

#include "opentx.h"


void initPiUART() {
	//Set AF config for tx UART on pin
	USART_DeInit(PI_USART);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = PI_USART_GPIO_TX_PIN | PI_USART_GPIO_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(PI_USART_GPIO, &GPIO_InitStructure);
	GPIO_PinAFConfig(PI_USART_GPIO, PI_USART_GPIO_RX_PinSource, PI_USART_GPIO_AF);
	GPIO_PinAFConfig(PI_USART_GPIO, PI_USART_GPIO_TX_PinSource, PI_USART_GPIO_AF);

	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(PI_USART, &USART_InitStructure);

	USART_Cmd(PI_USART, ENABLE);
}

//TODO: uart is echoing in tasks.cpp, delete in actual build
