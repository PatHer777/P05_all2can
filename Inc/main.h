/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED_Y_Pin GPIO_PIN_0
#define LED_Y_GPIO_Port GPIOF
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOF
#define DI0_Pin GPIO_PIN_0
#define DI0_GPIO_Port GPIOA
#define DI1_Pin GPIO_PIN_1
#define DI1_GPIO_Port GPIOA
#define DI2_Pin GPIO_PIN_2
#define DI2_GPIO_Port GPIOA
#define DI3_Pin GPIO_PIN_3
#define DI3_GPIO_Port GPIOA
#define DI4_Pin GPIO_PIN_4
#define DI4_GPIO_Port GPIOA
#define DI5_Pin GPIO_PIN_5
#define DI5_GPIO_Port GPIOA
#define DI6_Pin GPIO_PIN_0
#define DI6_GPIO_Port GPIOB
#define DI7_Pin GPIO_PIN_1
#define DI7_GPIO_Port GPIOB
#define clutch_out_Pin GPIO_PIN_8
#define clutch_out_GPIO_Port GPIOA
#define upshift_out_Pin GPIO_PIN_9
#define upshift_out_GPIO_Port GPIOA
#define downshift_out_Pin GPIO_PIN_10
#define downshift_out_GPIO_Port GPIOA
#define ignitioncut_out_Pin GPIO_PIN_15
#define ignitioncut_out_GPIO_Port GPIOA
#define DO4_Pin GPIO_PIN_3
#define DO4_GPIO_Port GPIOB
#define DO5_Pin GPIO_PIN_4
#define DO5_GPIO_Port GPIOB
#define DO6_Pin GPIO_PIN_5
#define DO6_GPIO_Port GPIOB
#define DO7_Pin GPIO_PIN_6
#define DO7_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
