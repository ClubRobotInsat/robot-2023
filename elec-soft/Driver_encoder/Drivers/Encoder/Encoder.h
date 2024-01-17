/**
  ******************************************************************************
  * @file           : Encoder.h
  * @brief          : Driver contains the functions to use 2 encoders
  * @date			: Jan 17, 2024
  * @author			: TANG Huong Cam
  ******************************************************************************
  * @attention
  * Necessary configurations :
  *
  * Timer / TIM3 : Encoder Mode
  * Timer / TIM4 : Encoder Mode
  *
  * GPIO :
  * 	PB3 : GPIO_EXTI3
  * 	PA8 : GPIO_EXTI8
  *
  * System Core / NVIC :
  * 	EXTI line3 interrupt 		: true
  *		EXTI line[9:5] interrupts	: true
  ******************************************************************************
  */

/*

 * */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_


#include "stm32g4xx_hal.h"


/**
 * @fn void Encoder_Init(TIM_HandleTypeDef*, TIM_HandleTypeDef*)
 * @brief Initialize the encoders
 *
 * @param Encoder_Left_HTimer3
 * @param Encoder_Right_HTimer4
 */
void Encoder_Init (TIM_HandleTypeDef * Encoder_Left_HTimer3, TIM_HandleTypeDef * Encoder_Right_HTimer4/*,UART_HandleTypeDef * Uart2*/);

/**
 * @fn float Encoder_Left_Get_Distance(void)
 * @brief Get the current distance of the left encoder from the last position where the record started
 *
 * @return
 */
float Encoder_Left_Get_Distance (void);

/**
 * @fn float Encoder_Right_Get_Distance(void)
 * @brief Get the current distance of the right encoder from the last position where the record started
 *
 * @return
 */
float Encoder_Right_Get_Distance (void);

/**
 * @fn void Encoder_Start_Record(void)
 * @brief Reset and start a new record
 *
 */
void Encoder_Start_Record (void);


#endif /* ENCODER_ENCODER_H_ */
