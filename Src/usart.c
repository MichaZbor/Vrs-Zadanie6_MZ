/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
uint8_t bufferUSART2dma[DMA_USART2_BUFFER_SIZE]; // Memory buffer used directly by DMA for USART Rx
static void (* USART2_ProcessData)(uint8_t data) = 0;


/* Register callback */
void USART2_RegisterCallback(void *callback){
        if(callback != 0)
        	USART2_ProcessData = callback;
}


/* USER CODE END 0 */

/* USART2 init function */

void MX_USART2_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */

  /* USART2_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses( DMA1, LL_DMA_CHANNEL_6,
		  LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_RECEIVE),
		  (uint32_t) bufferUSART2dma, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, DMA_USART2_BUFFER_SIZE);

  /* //silenced because rx is not used
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
  LL_USART_EnableDMAReq_RX(USART2);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_6);*/

  /* USART2_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_7,LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT));
  LL_USART_EnableDMAReq_TX(USART2);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_7);


  USART_InitStruct.BaudRate = USART_BAUD_RATE;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);

  LL_USART_DisableIT_CTS(USART2); // to indicate whether the USART is ready to send data
  LL_USART_EnableIT_IDLE(USART2); // triggered when the USART detects an idle line condition, which occurs when there is no data transmission or reception for a frame time

  LL_USART_Enable(USART2);

  /*// USART2 interrupt Init  // silenced because of not use
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);*/

}

/* USER CODE BEGIN 1 */
// Send data stored in buffer with DMA
void USART2_PutBuffer(uint8_t *buffer, uint8_t length){
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_7, (uint32_t)buffer);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, length);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);
}


/*
void USART2_CheckDmaReception(void)
{
	//type your implementation here
	static uint8_t old_pos = 0;
	static uint8_t max_stable_pos = DMA_USART2_BUFFER_SIZE-(EXPECTED_SIZE_PER_MESSAGE/2*3);

	if (USART2_ProcessData == 0) return; // check validity of function

	uint8_t pos = DMA_USART2_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6);

	if (pos != old_pos){
		for (uint8_t cycle = old_pos; cycle<=pos; cycle++) USART2_ProcessData(bufferUSART2dma[cycle]);

		old_pos = pos;
		if (old_pos >= max_stable_pos){
			old_pos = 0;
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6); // Disable the DMA channel
			memset(bufferUSART2dma, 0, DMA_USART2_BUFFER_SIZE); // Clear the buffer
		    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, DMA_USART2_BUFFER_SIZE); // Reset the DMA data length register
		    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6); // Re-enable the DMA channel
		}
	}

}

void resetBuffer(){
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, DMA_USART2_BUFFER_SIZE);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
}

*/
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
