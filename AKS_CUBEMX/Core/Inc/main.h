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
#include "stm32f1xx_hal.h"

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
#define Rel2Signal_Pin GPIO_PIN_13
#define Rel2Signal_GPIO_Port GPIOC
#define Rel1Signal_Pin GPIO_PIN_14
#define Rel1Signal_GPIO_Port GPIOC
#define EN_Pin GPIO_PIN_15
#define EN_GPIO_Port GPIOC
#define P2_Pin GPIO_PIN_0
#define P2_GPIO_Port GPIOA
#define P6_Pin GPIO_PIN_1
#define P6_GPIO_Port GPIOA
#define P3_Pin GPIO_PIN_4
#define P3_GPIO_Port GPIOA
#define Silecek1_Pin GPIO_PIN_6
#define Silecek1_GPIO_Port GPIOA
#define Silecek2_Pin GPIO_PIN_7
#define Silecek2_GPIO_Port GPIOA
#define Deadman_Pin GPIO_PIN_0
#define Deadman_GPIO_Port GPIOB
#define Brake_Pin GPIO_PIN_1
#define Brake_GPIO_Port GPIOB
#define P1_Pin GPIO_PIN_12
#define P1_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_8
#define BTN2_GPIO_Port GPIOA
#define BTN3_Pin GPIO_PIN_9
#define BTN3_GPIO_Port GPIOA
#define BTN4_Pin GPIO_PIN_10
#define BTN4_GPIO_Port GPIOA
#define BTN5_Pin GPIO_PIN_11
#define BTN5_GPIO_Port GPIOA
#define BTN6_Pin GPIO_PIN_12
#define BTN6_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_15
#define BTN1_GPIO_Port GPIOA
#define Relay2_Pin GPIO_PIN_4
#define Relay2_GPIO_Port GPIOB
#define Latch_Pin GPIO_PIN_6
#define Latch_GPIO_Port GPIOB
#define Relay1_Pin GPIO_PIN_7
#define Relay1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
