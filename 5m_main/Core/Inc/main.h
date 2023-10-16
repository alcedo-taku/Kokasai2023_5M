/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define G5_Pin GPIO_PIN_0
#define G5_GPIO_Port GPIOF
#define G4_Pin GPIO_PIN_1
#define G4_GPIO_Port GPIOF
#define G3_Pin GPIO_PIN_0
#define G3_GPIO_Port GPIOC
#define G2_Pin GPIO_PIN_1
#define G2_GPIO_Port GPIOC
#define G1_Pin GPIO_PIN_2
#define G1_GPIO_Port GPIOC
#define G10_Pin GPIO_PIN_3
#define G10_GPIO_Port GPIOC
#define G9_Pin GPIO_PIN_0
#define G9_GPIO_Port GPIOA
#define G8_Pin GPIO_PIN_1
#define G8_GPIO_Port GPIOA
#define G7_Pin GPIO_PIN_2
#define G7_GPIO_Port GPIOA
#define G6_Pin GPIO_PIN_3
#define G6_GPIO_Port GPIOA
#define L9_Pin GPIO_PIN_6
#define L9_GPIO_Port GPIOA
#define L10_Pin GPIO_PIN_7
#define L10_GPIO_Port GPIOA
#define L7_Pin GPIO_PIN_4
#define L7_GPIO_Port GPIOC
#define L8_Pin GPIO_PIN_5
#define L8_GPIO_Port GPIOC
#define L5_Pin GPIO_PIN_0
#define L5_GPIO_Port GPIOB
#define L6_Pin GPIO_PIN_1
#define L6_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_6
#define L3_GPIO_Port GPIOC
#define L4_Pin GPIO_PIN_7
#define L4_GPIO_Port GPIOC
#define L1_Pin GPIO_PIN_8
#define L1_GPIO_Port GPIOC
#define L2_Pin GPIO_PIN_9
#define L2_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_2
#define LED0_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
