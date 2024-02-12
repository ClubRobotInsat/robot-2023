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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MOTOR_DC/motor_dc.h"
#include "Encoder/Encoder.h"
#include "MOVEMENT/movement.h"
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
#define PWM_Motor_1_Pin GPIO_PIN_0
#define PWM_Motor_1_GPIO_Port GPIOA
#define PWM_Motor_2_Pin GPIO_PIN_1
#define PWM_Motor_2_GPIO_Port GPIOA
#define DIR_Motor_1_Pin GPIO_PIN_4
#define DIR_Motor_1_GPIO_Port GPIOA
#define Aplus_1_Pin GPIO_PIN_6
#define Aplus_1_GPIO_Port GPIOA
#define Bplus_1_Pin GPIO_PIN_7
#define Bplus_1_GPIO_Port GPIOA
#define Index_Encoder_2_Pin GPIO_PIN_8
#define Index_Encoder_2_GPIO_Port GPIOA
#define Index_Encoder_2_EXTI_IRQn EXTI9_5_IRQn
#define DIR_Motor_2_Pin GPIO_PIN_9
#define DIR_Motor_2_GPIO_Port GPIOA
#define Index_Encoder_1_Pin GPIO_PIN_5
#define Index_Encoder_1_GPIO_Port GPIOB
#define Index_Encoder_1_EXTI_IRQn EXTI9_5_IRQn
#define Aplus_2_Pin GPIO_PIN_6
#define Aplus_2_GPIO_Port GPIOB
#define Bplus_2_Pin GPIO_PIN_7
#define Bplus_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
