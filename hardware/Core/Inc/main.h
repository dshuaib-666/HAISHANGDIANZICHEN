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
#include "stm32f4xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define FORMAT_Pin GPIO_PIN_1
#define FORMAT_GPIO_Port GPIOF
#define START_ADC_Pin GPIO_PIN_3
#define START_ADC_GPIO_Port GPIOF
#define END_ADC_Pin GPIO_PIN_5
#define END_ADC_GPIO_Port GPIOF
#define ADC_CLK_Pin GPIO_PIN_0
#define ADC_CLK_GPIO_Port GPIOC
#define ADC_DATA_Pin GPIO_PIN_2
#define ADC_DATA_GPIO_Port GPIOC
#define Key_Select_Pin GPIO_PIN_8
#define Key_Select_GPIO_Port GPIOD
#define Key_End_Pin GPIO_PIN_10
#define Key_End_GPIO_Port GPIOD
#define Key_Start_Pin GPIO_PIN_12
#define Key_Start_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

// USB功能控制 - 启用USB功能，支持动态检测USB连接状�??
#define ENABLE_USB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
