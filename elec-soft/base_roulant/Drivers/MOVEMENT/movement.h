/*
 * movement.h
 *
 *  Created on: Feb 1, 2024
 *      Author: hoang
 */

#ifndef MOUVEMENT_MOVEMENT_H_
#define MOUVEMENT_MOVEMENT_H_

#include "stm32g4xx_hal.h"
#include "Encoder/Encoder.h"
#include "MOTOR_DC/motor_dc.h"


#define MOVEMENT_MAX_VITESSE					60
#define MOVEMENT_MIN_VITESSE					10

#define MOVEMENT_MAX_DISTANCE					150 //in centimeter

/**
 * @fn void Movement_Init(Motor_Config, Motor_Config, TIM_HandleTypeDef*, TIM_HandleTypeDef*)
 * @brief
 *
 * @param motor_R
 * @param motor_L
 * @param htim_EncoderL
 * @param htim_EncoderR
 */
void Movement_Init(Motor_Config motor_R, Motor_Config motor_L,
				   TIM_HandleTypeDef * htim_EncoderL,TIM_HandleTypeDef * htim_EncoderR);
int Movement_Regulation(void);


/**
 * @fn void Movement_Start(int)
 * @brief
 *
 * @param distance : in cm
 */
void Movement_Start(int distance);

#endif /* MOUVEMENT_MOVEMENT_H_ */
