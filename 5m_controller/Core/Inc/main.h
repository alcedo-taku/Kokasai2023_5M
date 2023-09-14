/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define PF6_Pin GPIO_PIN_13
#define PF6_GPIO_Port GPIOC
#define PA8_Pin GPIO_PIN_14
#define PA8_GPIO_Port GPIOC
#define PD8_Pin GPIO_PIN_15
#define PD8_GPIO_Port GPIOC
#define PB15_Pin GPIO_PIN_0
#define PB15_GPIO_Port GPIOF
#define PB14_Pin GPIO_PIN_1
#define PB14_GPIO_Port GPIOF
#define joystick_1A_Pin GPIO_PIN_0
#define joystick_1A_GPIO_Port GPIOA
#define joystick_1B_Pin GPIO_PIN_1
#define joystick_1B_GPIO_Port GPIOA
#define joystick_2A_Pin GPIO_PIN_2
#define joystick_2A_GPIO_Port GPIOA
#define joystick_2B_Pin GPIO_PIN_3
#define joystick_2B_GPIO_Port GPIOA
#define PA4_Pin GPIO_PIN_4
#define PA4_GPIO_Port GPIOA
#define PA5_Pin GPIO_PIN_5
#define PA5_GPIO_Port GPIOA
#define External_LED_1_Pin GPIO_PIN_6
#define External_LED_1_GPIO_Port GPIOA
#define LED_R_D_Pin GPIO_PIN_0
#define LED_R_D_GPIO_Port GPIOB
#define LED_R_L_Pin GPIO_PIN_1
#define LED_R_L_GPIO_Port GPIOB
#define PB2_Pin GPIO_PIN_2
#define PB2_GPIO_Port GPIOB
#define External_LED_2_Pin GPIO_PIN_8
#define External_LED_2_GPIO_Port GPIOE
#define PE9_Pin GPIO_PIN_9
#define PE9_GPIO_Port GPIOE
#define radio_Pin GPIO_PIN_14
#define radio_GPIO_Port GPIOB
#define LED_L_D_Pin GPIO_PIN_9
#define LED_L_D_GPIO_Port GPIOA
#define LED_L_L_Pin GPIO_PIN_10
#define LED_L_L_GPIO_Port GPIOA
#define PF7_Pin GPIO_PIN_6
#define PF7_GPIO_Port GPIOF
#define PC13_Pin GPIO_PIN_7
#define PC13_GPIO_Port GPIOF
#define LED_L_U_Pin GPIO_PIN_15
#define LED_L_U_GPIO_Port GPIOA
#define LED_L_R_Pin GPIO_PIN_3
#define LED_L_R_GPIO_Port GPIOB
#define LED_R_U_Pin GPIO_PIN_4
#define LED_R_U_GPIO_Port GPIOB
#define LED_R_R_Pin GPIO_PIN_5
#define LED_R_R_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
