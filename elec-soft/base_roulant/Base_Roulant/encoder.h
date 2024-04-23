/**
  ******************************************************************************
  * @file           : Encoder.h
  * @brief          : Driver used to control 2 encoders
  * @date			: Feb 23, 2024
  * @author			: TANG Huong Cam
  ******************************************************************************
  * @attention
  * ***Hardware Connection***
  *
  * Encoder 1 : (TIM3)
  * 	A+ : PA6
  * 	V+ : +5V, 0V : GND
  * 	B+ : PA7(not use), 0+ : PB5 (not use)
  *
  * Encoder 2 : (TIM4)
  * 	A+ : PB6
  *     V+ : +5V, 0V : GND
  *     B+ : PB7(not use), 0+ : PA8(not use)
  *
  * ***Necessary configurations CUBE IDE:***
  *
  * Timer / TIM3 : CH1 capture direct mode
  * Timer / TIM4 : CH1 capture direct mode
  *
  * GPIO :
  * 	PB5 : GPIO_EXTI5 (not use)
  * 	PA8 : GPIO_EXTI8 (not use)
  *
  * System Core / NVIC :
  *		EXTI line[9:5] interrupts	: true (not use)
  *		TIM3 global interrupt		: true
  *		TIM4 global interrupt		: true
  *
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
 * @param Encoder_Left_HTimer3
 * @param Encoder_Right_HTimer4
 */
void Encoder_Init (TIM_HandleTypeDef * Encoder_Left_HTimer3, TIM_HandleTypeDef * Encoder_Right_HTimer4);


/**
 * @fn void Encoder_Start_Record(void)
 * @brief Reset and start a new distance record
 *
 */
void Encoder_Start_Record_Distance (void);



/**
 * @fn float Encoder_Left_Get_Distance(void)
 * @brief Get the distance of the left encoder from the last position where the record started
 * @return Distance in centimeter : float
 */
float Encoder_Left_Get_Distance (void);



/**
 * @fn float Encoder_Right_Get_Distance(void)
 * @brief Get the distance of the right encoder from the last position where the record started
 * @return Distance in centimeter : float
 */
float Encoder_Right_Get_Distance (void);



/**
 * @fn float Encoder_GetLeftSpeed(void)
 * @brief Get the speed of the left motor
 * @return Speed's Left Motor in millimeter/s : float
 */
float Encoder_Left_Get_Speed(void);

/**
 * @fn float Encoder_GetRightSpeed(void)
 * @brief Get the speed of the right motor
 * @return Speed's Right Motor in millimeter/s : float
 */
float Encoder_Right_Get_Speed(void);


void Encoder_IC_Interrupt_Handler(TIM_HandleTypeDef *htim);
void Encoder_Overflow_Interrupt_Handler(TIM_HandleTypeDef *htim);



#endif

