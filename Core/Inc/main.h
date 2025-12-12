/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RS485_EN_Pin GPIO_PIN_1
#define RS485_EN_GPIO_Port GPIOA
#define EA_TXD1_Pin GPIO_PIN_4
#define EA_TXD1_GPIO_Port GPIOC
#define EA_RXD1_Pin GPIO_PIN_5
#define EA_RXD1_GPIO_Port GPIOC
#define EA_TXD2_Pin GPIO_PIN_10
#define EA_TXD2_GPIO_Port GPIOB
#define EA_RXD2_Pin GPIO_PIN_11
#define EA_RXD2_GPIO_Port GPIOB
#define Motor_TXD1_Pin GPIO_PIN_10
#define Motor_TXD1_GPIO_Port GPIOC
#define Motor_RXD1_Pin GPIO_PIN_11
#define Motor_RXD1_GPIO_Port GPIOC
#define Motor_TXD2_Pin GPIO_PIN_12
#define Motor_TXD2_GPIO_Port GPIOC
#define Motor_RXD2_Pin GPIO_PIN_2
#define Motor_RXD2_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
