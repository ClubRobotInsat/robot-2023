/*
 * motor_dc.c
 *
 *  Created on: Feb 1, 2024
 *      Author: lilultime
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "motor_dc.h"



Motor_Config Motor_Init(GPIO_TypeDef  * DIR_Port, uint16_t DIR_Pin, TIM_HandleTypeDef * TIM, uint32_t TIM_Channel){
    Motor_Config motor;
    /*  Attach GPIO and TIM to Motor*/
    motor.DIR_Port = DIR_Port;
    motor.DIR_Pin = DIR_Pin;
    motor.TIM = TIM;
    motor.TIM_Channel = TIM_Channel;

    /* Set the duty cycle = 0 at initialize*/
	__HAL_TIM_SET_COMPARE(TIM,TIM_Channel,0);

	/* Set the directional pin to 0 (Counter Clock-Wise) */
	HAL_GPIO_WritePin(DIR_Port, DIR_Pin, MOTOR_DIRECTION_CCW);
	HAL_TIM_Base_Start(TIM);
	return motor;
}

void Motor_Start(Motor_Config motor){
	HAL_TIM_PWM_Start(motor.TIM, motor.TIM_Channel);
}

void Motor_Stop(Motor_Config motor){
	HAL_TIM_PWM_Stop(motor.TIM, motor.TIM_Channel);
}

void Motor_Set_Speed(Motor_Config motor, uint8_t duty_cycle){

	if ((duty_cycle >= 0) || (duty_cycle <= 100)){
		//duty_cycle = duty_cycle - 1;
		__HAL_TIM_SET_COMPARE(motor.TIM,motor.TIM_Channel,duty_cycle);
	}
	else {
		/* Error handler to  add*/
	}
}


void Motor_Set_Direction(Motor_Config motor, uint8_t direction){
	if ((direction == MOTOR_DIRECTION_CW) || (direction == MOTOR_DIRECTION_CCW)){
		HAL_GPIO_WritePin(motor.DIR_Port, motor.DIR_Pin, direction);
	}
	else {
		/* Error handler to  add*/
	}
}

void Motor_Toggle_Direction(Motor_Config motor){
	HAL_GPIO_TogglePin(motor.DIR_Port, motor.DIR_Pin);
}

uint8_t Motor_Get_Speed(Motor_Config motor){
	return __HAL_TIM_GET_COMPARE(motor.TIM, motor.TIM_Channel);
}

uint8_t Motor_Get_Direction(Motor_Config motor){
	return HAL_GPIO_ReadPin(motor.DIR_Port, motor.DIR_Pin);
}
