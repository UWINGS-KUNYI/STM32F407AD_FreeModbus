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
  * Copyright (c) 2018 STMicroelectronics International N.V.
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

#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define D_IN2_Pin GPIO_PIN_2
#define D_IN2_GPIO_Port GPIOE
#define D_IN1_Pin GPIO_PIN_3
#define D_IN1_GPIO_Port GPIOE
#define ADC_CS_Pin GPIO_PIN_15
#define ADC_CS_GPIO_Port GPIOC
#define D_OUT1_Pin GPIO_PIN_10
#define D_OUT1_GPIO_Port GPIOF
#define D_OUT2_Pin GPIO_PIN_0
#define D_OUT2_GPIO_Port GPIOC
#define D_OUT3_Pin GPIO_PIN_2
#define D_OUT3_GPIO_Port GPIOC
#define D_OUT4_Pin GPIO_PIN_3
#define D_OUT4_GPIO_Port GPIOC
#define D_OUT5_Pin GPIO_PIN_0
#define D_OUT5_GPIO_Port GPIOA
#define D_OUT6_Pin GPIO_PIN_3
#define D_OUT6_GPIO_Port GPIOA
#define D_OUT7_Pin GPIO_PIN_4
#define D_OUT7_GPIO_Port GPIOA
#define D_OUT8_Pin GPIO_PIN_5
#define D_OUT8_GPIO_Port GPIOA
#define D_OUT9_Pin GPIO_PIN_6
#define D_OUT9_GPIO_Port GPIOA
#define RS485RE_Pin GPIO_PIN_11
#define RS485RE_GPIO_Port GPIOF
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define DA_DATA_Pin GPIO_PIN_6
#define DA_DATA_GPIO_Port GPIOG
#define DA_CLK_Pin GPIO_PIN_7
#define DA_CLK_GPIO_Port GPIOG
#define RS485RE3_Pin GPIO_PIN_8
#define RS485RE3_GPIO_Port GPIOG
#define W25Q128_CS_Pin GPIO_PIN_8
#define W25Q128_CS_GPIO_Port GPIOC
#define DA_LOAD_Pin GPIO_PIN_11
#define DA_LOAD_GPIO_Port GPIOA
#define DA_LD_Pin GPIO_PIN_12
#define DA_LD_GPIO_Port GPIOA
#define APP_RUN_Pin GPIO_PIN_15
#define APP_RUN_GPIO_Port GPIOA
#define D_IN8_Pin GPIO_PIN_2
#define D_IN8_GPIO_Port GPIOD
#define ETH_RESET_Pin GPIO_PIN_3
#define ETH_RESET_GPIO_Port GPIOD
#define D_IN7_Pin GPIO_PIN_6
#define D_IN7_GPIO_Port GPIOD
#define D_IN6_Pin GPIO_PIN_7
#define D_IN6_GPIO_Port GPIOD
#define D_IN5_Pin GPIO_PIN_9
#define D_IN5_GPIO_Port GPIOG
#define D_IN4_Pin GPIO_PIN_12
#define D_IN4_GPIO_Port GPIOG
#define D_IN3_Pin GPIO_PIN_15
#define D_IN3_GPIO_Port GPIOG

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
