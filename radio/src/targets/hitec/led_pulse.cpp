

#include "opentx.h"
#include "board.h"


void initPulse(){

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = LED_PULSE_GPIO_PIN;
  GPIO_Init(LED_PULSE_GPIO, &GPIO_InitStructure);
}

void sendPulse(){
	GPIO_ToggleBits(LED_PULSE_GPIO, LED_PULSE_GPIO_PIN);
}