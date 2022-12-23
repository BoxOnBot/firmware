/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define U4_Pin GPIO_PIN_11
#define U4_GPIO_Port GPIOC
#define U5_Pin GPIO_PIN_0
#define U5_GPIO_Port GPIOA
#define U6_Pin GPIO_PIN_1
#define U6_GPIO_Port GPIOA
#define U9_Pin GPIO_PIN_6
#define U9_GPIO_Port GPIOA
#define U10_Pin GPIO_PIN_7
#define U10_GPIO_Port GPIOA
#define U11_Pin GPIO_PIN_0
#define U11_GPIO_Port GPIOB
#define U12_Pin GPIO_PIN_1
#define U12_GPIO_Port GPIOB
#define U7_Pin GPIO_PIN_10
#define U7_GPIO_Port GPIOB
#define U8_Pin GPIO_PIN_11
#define U8_GPIO_Port GPIOB
#define U3_Pin GPIO_PIN_8
#define U3_GPIO_Port GPIOA
#define U2_Pin GPIO_PIN_9
#define U2_GPIO_Port GPIOA
#define U1_Pin GPIO_PIN_10
#define U1_GPIO_Port GPIOA
#define TRIG_PIN_Pin GPIO_PIN_4
#define TRIG_PIN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
