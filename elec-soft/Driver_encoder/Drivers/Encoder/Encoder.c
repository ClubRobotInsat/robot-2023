/*
 * Encoder.c
 *
 *  Created on: Jan 4, 2024
 *      Author: TANG Huong Cam
 */

#include "Encoder.h"

#define ENCODER_MAX_COUNTER     2048

#define ENCODER_LEFT_TIMER      htim3
#define ENCODER_RIGHT_TIMER		htim4
#define ENCODER_LEFT_INDEX_PIN  GPIO_PIN_3
#define ENCODER_RIGHT_INDEX_PIN GPIO_PIN_8

#define PERIMETER_WHEEL_IN_CM 	15

uint16_t Number_Turned_Rounds;

// EXTI External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// If The INT Source Is EXTI Line3 (B3 Pin)
    if(GPIO_Pin == ENCODER_LEFT_INDEX_PIN)
    {
		//uint8_t MSG[5] = {'\0'};
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
		//sprintf(MSG, "%d", (__HAL_TIM_GET_COUNTER(&htim3)));
		//HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		__HAL_TIM_SET_COUNTER(&ENCODER_LEFT_TIMER,(abs(ENCODER_MAX_COUNTER - __HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIMER))));
		Number_Turned_Rounds++; //Increase the number of the turned rounds of the encoder
    }

	// If The INT Source Is EXTI Line8 (A8 Pin)
    else if(GPIO_Pin == ENCODER_RIGHT_INDEX_PIN){
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    	__HAL_TIM_SET_COUNTER(&ENCODER_RIGHT_TIMER,(abs(ENCODER_MAX_COUNTER - __HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIMER))));
    }
}

void Encoder_Init (void){
	HAL_TIM_Encoder_Start(&ENCODER_LEFT_TIMER, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&ENCODER_RIGHT_TIMER, TIM_CHANNEL_ALL);
	Number_Turned_Rounds = 0;
}

float Encoder_Left_Get_Distance (void){
	float Current_Counter_Distance = (float)(PERIMETER_WHEEL_IN_CM*(float)(__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIMER)/ENCODER_MAX_COUNTER));
	return PERIMETER_WHEEL_IN_CM * Number_Turned_Rounds + Current_Counter_Distance;
}

float Encoder_Right_Get_Distance (void){
	float Current_Counter_Distance = (float)(PERIMETER_WHEEL_IN_CM*(float)(__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIMER)/ENCODER_MAX_COUNTER));
	return PERIMETER_WHEEL_IN_CM * Number_Turned_Rounds + Current_Counter_Distance;
}

void Encoder_Start_Record (void){
	__HAL_TIM_SET_COUNTER(&ENCODER_RIGHT_TIMER,0);
	__HAL_TIM_SET_COUNTER(&ENCODER_LEFT_TIMER,0);
	Number_Turned_Rounds = 0;
}
