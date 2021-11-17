

#include "opentx.h"
#include "fastmavlink/minimal/mavlink.h"
#include "board.h"

uint8_t *heartbeat_buff;
uint8_t *override_radio_buff;

void initPiUART() {
	//Create buffers in ram so compiler doesn't put them in CCM, CCM can't use DMA
	heartbeat_buff = (uint8_t*) malloc(1024*sizeof(uint8_t)); 
	override_radio_buff = (uint8_t*) malloc(1024*sizeof(uint8_t));
	//Set AF config for tx UART on pin
	USART_DeInit(PI_USART);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = PI_USART_GPIO_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(PI_USART_GPIO, &GPIO_InitStructure);
	GPIO_PinAFConfig(PI_USART_GPIO, PI_USART_GPIO_TX_PinSource, PI_USART_GPIO_AF);

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(PI_USART, &USART_InitStructure);
	USART_Cmd(PI_USART, ENABLE);

	USART_ITConfig(PI_USART, USART_IT_TC, ENABLE);
	NVIC_EnableIRQ(PI_USART_IRQn);
	NVIC_SetPriority(PI_USART_IRQn, 6);
}


void MavlinkSendBuffer(const uint8_t * data, uint16_t size)
{
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(PI_USART_TX_DMA_STREAM);
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = PI_USART_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = CONVERT_PTR_UINT(&PI_USART->DR);
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = size;
  DMA_InitStructure.DMA_Memory0BaseAddr = CONVERT_PTR_UINT(data);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  

  DMA_Init(PI_USART_TX_DMA_STREAM, &DMA_InitStructure);
  DMA_Cmd(PI_USART_TX_DMA_STREAM, ENABLE);
	USART_DMACmd(PI_USART, USART_DMAReq_Tx, ENABLE);

	//Might not be needed
  while(DMA_GetCmdStatus(PI_USART_TX_DMA_STREAM) == DISABLE){};

  DMA_ITConfig(PI_USART_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  NVIC_EnableIRQ(PI_DMA_STREAM_IRQn);
  NVIC_SetPriority(PI_DMA_STREAM_IRQn, 7);
}

extern "C" void PI_USART_IRQHandler()
{
	if(USART_GetITStatus(PI_USART, USART_IT_TC)){
			USART_ClearITPendingBit(PI_USART, USART_IT_TC);
	}
}


extern "C" void PI_DMA_STREAM_IRQHandler()
{
  if (DMA_GetITStatus(PI_USART_TX_DMA_STREAM, PI_DMA_FLAG_TC)){
    DMA_ClearITPendingBit(PI_USART_TX_DMA_STREAM, PI_DMA_FLAG_TC);
  }
}


void sendHeartbeat(){
	
	GPIO_ToggleBits(LED_BLUE_GPIO, LED_BLUE_GPIO_PIN);

	uint8_t sysid = 245;
	fmav_status_t status;
	uint16_t size;

	
	size = fmav_msg_heartbeat_pack_to_frame_buf(heartbeat_buff, sysid, MAV_COMPONENT::MAV_COMP_ID_USER1,
											 MAV_TYPE::MAV_TYPE_GCS,
											 MAV_AUTOPILOT::MAV_AUTOPILOT_INVALID,
											 MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED,
											 0,
											 MAV_STATE::MAV_STATE_ACTIVE,
											 &status);

	MavlinkSendBuffer(heartbeat_buff, size);
}





//TODO: Might be optimization opportunities for data transmission
//Probably want to move to circular buffer so I don't have to keep initializing 
// and deintializing array very quickly, wasting cpu cycles
//Don't think we need to use the FIFO, might be able to take advantage of burst mode

//TODO: for multiple messages trying to send at different intervals, add queue? 
//When only heartbeat sending the rate is steady
//messages can become corrupted without some type of queue
//Might not need if heartbeat is not needed
//Sometimes rcoverride conflicts with heartbeat message can cause bad serial data momentarily
//bad data send ~10s, seems to be ok when no heartbeats are trying to send

//TODO: test with tablet, send missions, get params etc
//TODO: take new controller out to completely test
//TODO: make whatever changes needed for commands, push repo for pddl
//TODO: make optimizations above

void sendRCChannelsOverMavlink(uint16_t* channel_data) {

	uint8_t sysid = 255;
	uint8_t target_sys = 1;
	uint16_t size;
	fmav_status_t status;


    size = fmav_msg_rc_channels_override_pack_to_frame_buf(override_radio_buff, sysid, MAV_COMPONENT::MAV_COMP_ID_USER1, 
    											target_sys, MAV_COMPONENT::MAV_COMP_ID_AUTOPILOT1,
    															channel_data[0],
    															channel_data[1],
    															channel_data[2],
    															channel_data[3],
    															channel_data[4],
    															channel_data[5],
    															channel_data[6],
    															channel_data[7],
    															channel_data[8],
    															channel_data[9],
    															channel_data[10],
    															channel_data[11],
    															channel_data[12],
    															channel_data[13],
    															channel_data[14],
    															channel_data[15],
    															channel_data[16],
    															channel_data[17],
    															&status);
    MavlinkSendBuffer(override_radio_buff,  size);
}
