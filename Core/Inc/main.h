/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define ADC_Pin GPIO_PIN_1
#define ADC_GPIO_Port GPIOA
#define OUT_ON_Pin GPIO_PIN_12
#define OUT_ON_GPIO_Port GPIOB
#define OUT_OFF_Pin GPIO_PIN_13
#define OUT_OFF_GPIO_Port GPIOB
#define REV_ON_Pin GPIO_PIN_14
#define REV_ON_GPIO_Port GPIOB
#define REV_OFF_Pin GPIO_PIN_15
#define REV_OFF_GPIO_Port GPIOB
#define TXD_Pin GPIO_PIN_9
#define TXD_GPIO_Port GPIOA
#define RXD_Pin GPIO_PIN_10
#define RXD_GPIO_Port GPIOA
#define ADC_ON_Pin GPIO_PIN_4
#define ADC_ON_GPIO_Port GPIOB
#define ADC_OFF_Pin GPIO_PIN_5
#define ADC_OFF_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_6
#define EN_GPIO_Port GPIOB
#define DIAG_Pin GPIO_PIN_7
#define DIAG_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_8
#define STEP_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_9
#define DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
