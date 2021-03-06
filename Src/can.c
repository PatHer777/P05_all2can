/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
CAN_FilterTypeDef	sFilterConfig;
CAN_TxHeaderTypeDef	TxHeader;
CAN_RxHeaderTypeDef	RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
#define clutch 	0
#define upshift		1
#define downshift 	2
#define greenled	7
//uint8_t newCanMessage = 0;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(CAN_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN_TX_IRQn);
    HAL_NVIC_SetPriority(CAN_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan1)
{
	//HAL_GPIO_TogglePin(LED_Blue_GPIO_Port,LED_Blue_Pin);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {}
	newCanMessage();// = 1;

}

void JDO_SendCan(void)
{
	HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
	//HAL_Delay(500);
	TxData[7]=TxData[7]+1;
 //   HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET); //Geaenderte Zeile
//	HAL_GPIO_TogglePin(GPIOA, DO0_Pin);
	//GPIO_Out2_Pin
}

void JDO_GetCan(void)
{

	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {}
	switch(RxHeader.StdId){
		case (0x101):
			if(RxData[0] & 1<<clutch){ //clutch
				HAL_GPIO_WritePin(GPIOA, clutch_out_Pin, 1); //cluch_out_Pin = Pin1 = DO0_Pin (in earlier version)
				//Kuppling dauerhaft offen
			}
			if( !(RxData[0] & 1<<clutch) ){
				HAL_GPIO_WritePin(GPIOA, clutch_out_Pin, 0);
				//Kuppling dauerhaft geschlossen
			}
			if(RxData[0] & 1<<upshift){	//hochschalten 2
				HAL_GPIO_WritePin(GPIOA, ignitioncut_out_Pin, 1); //invertet, 'cause pull up in hardware
				osDelay(10);
				HAL_GPIO_WritePin(GPIOA, upshift_out_Pin, 1); // Pin2
				osDelay(125); //125 aktueller   //75war mal ganz ok
				HAL_GPIO_WritePin(GPIOA, upshift_out_Pin, 0);
				HAL_GPIO_WritePin(GPIOA, ignitioncut_out_Pin, 0); // Pin4 //gearcut+
			}
			else if(RxData[0] & 1<<downshift){	//Runter schalten 4
				HAL_GPIO_WritePin(GPIOA, clutch_out_Pin, 1);
				osDelay(20);
				HAL_GPIO_WritePin(GPIOA, downshift_out_Pin, 1); //Pin3
				osDelay(275); // 275 besser //175  klappt
				HAL_GPIO_WritePin(GPIOA, downshift_out_Pin, 0);
				if( !(RxData[0] & 1<<clutch) ){
					HAL_GPIO_WritePin(GPIOA, clutch_out_Pin, 0);
					//Kupplung wird geschlossen, wenn das Kupplungsbit nicht gesetzt ist
				}
			}
			if(RxData[0] & 1<<greenled)  //128
				HAL_GPIO_WritePin(GPIOB, DO7_Pin, 1); //Gr�nes Licht
			else
				HAL_GPIO_WritePin(GPIOB, DO7_Pin, 0); //Gr�nes Licht aus
	break;

	}

}

void JDO_CanInit(void)
{
	/* Filter noch mal �berpr�fen!!! */
	sFilterConfig.FilterBank=0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh=0x0101<<5; //0x0700;
	sFilterConfig.FilterIdLow=0x0000<<5;
	sFilterConfig.FilterMaskIdHigh=0x1FFF<<5;//0x1FFF<<5;//muss hier geshifted werden???
	sFilterConfig.FilterMaskIdLow=0;//0x1FFF<<5;
	sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	sFilterConfig.FilterActivation=ENABLE;
	sFilterConfig.SlaveStartFilterBank=14;
	if(HAL_CAN_ConfigFilter(&hcan,&sFilterConfig)!=HAL_OK)
	{/* Filter configuration Error */
	  Error_Handler();
	}

	if(HAL_CAN_Start(&hcan)!=HAL_OK)
	{/* Start Error */
	  Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING |CAN_IT_TX_MAILBOX_EMPTY)!=HAL_OK)
	{/* Notification Error */
	  Error_Handler();
	}

	TxHeader.StdId=0x321;
	TxHeader.ExtId=0x01;
	TxHeader.RTR=CAN_RTR_DATA;
	TxHeader.IDE=CAN_ID_STD;
	TxHeader.DLC=8;
	TxHeader.TransmitGlobalTime=DISABLE;
	TxData[0]=1;
	TxData[1]=2;
	TxData[2]=3;
	TxData[3]=4;
	TxData[4]=5;
	TxData[5]=6;
	TxData[6]=7;
	TxData[7]=8;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
