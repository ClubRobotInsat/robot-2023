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
#include "stm32g4xx_hal.h"
#include "../../Drivers/STEPPER/Inc/controller.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Enable_Stepper_Pin GPIO_PIN_0
#define Enable_Stepper_GPIO_Port GPIOA
#define Reset_Stepper_Pin GPIO_PIN_1
#define Reset_Stepper_GPIO_Port GPIOA
#define USM0_Stepper_Pin GPIO_PIN_2
#define USM0_Stepper_GPIO_Port GPIOA
#define USM1_Stepper_Pin GPIO_PIN_3
#define USM1_Stepper_GPIO_Port GPIOA
#define Home_Stepper_Pin GPIO_PIN_4
#define Home_Stepper_GPIO_Port GPIOA
#define STEP_Stepper_Pin GPIO_PIN_5
#define STEP_Stepper_GPIO_Port GPIOA
#define DIR__Stepper_Pin GPIO_PIN_6
#define DIR__Stepper_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
