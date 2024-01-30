/*
 * Encoder.c
 *
 *  Created on: Jan 4, 2024
 *      Author: TANG Huong Cam
 */

#include "Encoder.h"

#define ENCODER_COUNT_1_ROUND     			2048			//Reference to Datasheet of Encoder 2400 1024
#define ENCODER_1_ROUND_INCERTITUDE			100				//Error when the index rises for 1 round (encoder turned in the same direction)
#define ENCODER_ZERO_INCERTITUDE			50				//Error when the index rises for back to 0 (encoder turned in the other direction)
#define ARR_TIMER							65534
#define ENCODER_LEFT_INDEX_PIN  			GPIO_PIN_5		//Pin for the index of the left encoder
#define ENCODER_RIGHT_INDEX_PIN 			GPIO_PIN_8		//Pin for the index of the right encoder
#define PERIMETER_WHEEL_IN_CM 				25.748			//Wheel perimeter in cm

TIM_HandleTypeDef * ENCODER_LEFT_TIMER;
TIM_HandleTypeDef * ENCODER_RIGHT_TIMER;

int Number_Turned_Rounds_Left;
int Number_Turned_Rounds_Right;
float First_Index_Distance_Left;							//Distance calculated when the first left index rises
float First_Index_Distance_Right;							//Distance calculated when the first right index rises

// Handler GPIOs for the index (EXTI External Interrupt ISR Handler CallBackFunction)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Handler for the left encoder's index (If The INT Source Is EXTI Line5 (B5 Pin))
    if(GPIO_Pin == ENCODER_LEFT_INDEX_PIN)
    {
    	int left_counter = __HAL_TIM_GET_COUNTER(ENCODER_LEFT_TIMER);
		// Check the counter's  value
		if(ENCODER_COUNT_1_ROUND - ENCODER_1_ROUND_INCERTITUDE <= left_counter && left_counter <= ENCODER_COUNT_1_ROUND) // if encoder turned one round in clockwise
		{
			Number_Turned_Rounds_Left++; 							//Increase the number of the turned rounds of the encoder
			__HAL_TIM_SET_COUNTER(ENCODER_LEFT_TIMER,0); 			//Reset the counter's value for clockwise
		}

		else if(ARR_TIMER - ENCODER_COUNT_1_ROUND <= left_counter && left_counter <= ARR_TIMER - ENCODER_COUNT_1_ROUND + ENCODER_1_ROUND_INCERTITUDE) // if encoder turned one round in counter clockwise
		{
			Number_Turned_Rounds_Left--;							//Increase the number of the turned rounds of the encoder
			__HAL_TIM_SET_COUNTER(ENCODER_LEFT_TIMER,ARR_TIMER);	//Reset the counter's value for counter clockwise
		}

		else if (!(First_Index_Distance_Left) && (ENCODER_ZERO_INCERTITUDE <= left_counter && left_counter <= ENCODER_COUNT_1_ROUND - ENCODER_1_ROUND_INCERTITUDE)) // if the first index in clockwise
		{
			First_Index_Distance_Left = (float)(((float)left_counter/(float)ENCODER_COUNT_1_ROUND)*PERIMETER_WHEEL_IN_CM);
			__HAL_TIM_SET_COUNTER(ENCODER_LEFT_TIMER,0); 			//Reset the counter's value for clockwise

		}

		else if (!(First_Index_Distance_Left) && (ARR_TIMER - ENCODER_COUNT_1_ROUND + ENCODER_1_ROUND_INCERTITUDE <= left_counter && left_counter <= ARR_TIMER - ENCODER_ZERO_INCERTITUDE)) // if the first index in clockwise
		{
			First_Index_Distance_Left = (float)(((float)(left_counter - ARR_TIMER)/(float)ENCODER_COUNT_1_ROUND)*PERIMETER_WHEEL_IN_CM);
			__HAL_TIM_SET_COUNTER(ENCODER_LEFT_TIMER,ARR_TIMER);	//Reset the counter's value for counter clockwise

		}
		else	// if encoder's index goes back to 0 when it changes the direction
		{
			__HAL_TIM_SET_COUNTER(ENCODER_LEFT_TIMER,0);
		}

    }

	// Handler for the right encoder's index (If The INT Source Is EXTI Line8 (B8 Pin))
    else if(GPIO_Pin == ENCODER_RIGHT_INDEX_PIN){

    	int right_counter = __HAL_TIM_GET_COUNTER(ENCODER_RIGHT_TIMER);

		if(ENCODER_COUNT_1_ROUND - ENCODER_1_ROUND_INCERTITUDE <= right_counter && right_counter <= ENCODER_COUNT_1_ROUND) // if encoder turned one round in clockwise
		{
			Number_Turned_Rounds_Right++; 							//Increase the number of the turned rounds of the encoder
			__HAL_TIM_SET_COUNTER(ENCODER_RIGHT_TIMER,0); 			//Reset the counter's value for clockwise
		}

		else if(ARR_TIMER - ENCODER_COUNT_1_ROUND <= right_counter && right_counter <= ARR_TIMER - ENCODER_COUNT_1_ROUND + ENCODER_1_ROUND_INCERTITUDE) // if encoder turned one round in counter clockwise
		{
			Number_Turned_Rounds_Right--;							//Increase the number of the turned rounds of the encoder
			__HAL_TIM_SET_COUNTER(ENCODER_RIGHT_TIMER,ARR_TIMER);	//Reset the counter's value for counter clockwise
		}

		else if (!(First_Index_Distance_Right) && (ENCODER_ZERO_INCERTITUDE <= right_counter && right_counter <= ENCODER_COUNT_1_ROUND - ENCODER_1_ROUND_INCERTITUDE)) // if the first index in clockwise
		{
			First_Index_Distance_Right = (float)(((float)right_counter/(float)ENCODER_COUNT_1_ROUND)*PERIMETER_WHEEL_IN_CM);
			__HAL_TIM_SET_COUNTER(ENCODER_RIGHT_TIMER,0); 			//Reset the counter's value for clockwise

		}

		else if (!(First_Index_Distance_Right) && (ARR_TIMER - ENCODER_COUNT_1_ROUND + ENCODER_1_ROUND_INCERTITUDE <= right_counter && right_counter <= ARR_TIMER - ENCODER_ZERO_INCERTITUDE)) // if the first index in clockwise
		{
			First_Index_Distance_Right = (float)(((float)(right_counter - ARR_TIMER)/(float)ENCODER_COUNT_1_ROUND)*PERIMETER_WHEEL_IN_CM);
			__HAL_TIM_SET_COUNTER(ENCODER_RIGHT_TIMER,ARR_TIMER);	//Reset the counter's value for counter clockwise

		}
		else // if encoder's index goes back to 0 when it changes the direction
		{
			__HAL_TIM_SET_COUNTER(ENCODER_RIGHT_TIMER,0);
		}

    }
}

void Encoder_Init (TIM_HandleTypeDef * Encoder_Left_HTimer3, TIM_HandleTypeDef * Encoder_Right_HTimer4){
	ENCODER_LEFT_TIMER = Encoder_Left_HTimer3;
	ENCODER_RIGHT_TIMER = Encoder_Right_HTimer4;

	HAL_TIM_Encoder_Start(ENCODER_LEFT_TIMER, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(ENCODER_RIGHT_TIMER, TIM_CHANNEL_ALL);
}

float Encoder_Left_Get_Distance (void){
	float Current_Counter_Distance;
	int Current_Counter = __HAL_TIM_GET_COUNTER(ENCODER_LEFT_TIMER);

	if ( Current_Counter <= ENCODER_COUNT_1_ROUND)
	{
		Current_Counter_Distance = (float)(PERIMETER_WHEEL_IN_CM*(float)((float)Current_Counter/(float)ENCODER_COUNT_1_ROUND));
	}
	else
	{
		Current_Counter_Distance = (float)(PERIMETER_WHEEL_IN_CM*(float)((float)(Current_Counter - ARR_TIMER)/(float)ENCODER_COUNT_1_ROUND));
	}
	return PERIMETER_WHEEL_IN_CM * Number_Turned_Rounds_Left + Current_Counter_Distance + First_Index_Distance_Left;

}

float Encoder_Right_Get_Distance (void){
	float Current_Counter_Distance;
	int Current_Counter = __HAL_TIM_GET_COUNTER(ENCODER_RIGHT_TIMER);

	if ( Current_Counter <= ENCODER_COUNT_1_ROUND)
	{
		Current_Counter_Distance = (float)(PERIMETER_WHEEL_IN_CM*(float)((float)Current_Counter/(float)ENCODER_COUNT_1_ROUND));
	}
	else
	{
		Current_Counter_Distance = (float)(PERIMETER_WHEEL_IN_CM*(float)((float)(Current_Counter - ARR_TIMER)/(float)ENCODER_COUNT_1_ROUND));
	}
	return PERIMETER_WHEEL_IN_CM * Number_Turned_Rounds_Right + Current_Counter_Distance + First_Index_Distance_Right;
}


void Encoder_Start_Record (void){
	__HAL_TIM_SET_COUNTER(ENCODER_RIGHT_TIMER,0);
	__HAL_TIM_SET_COUNTER(ENCODER_LEFT_TIMER,0);
	Number_Turned_Rounds_Left = 0;
	Number_Turned_Rounds_Right = 0;
	First_Index_Distance_Left = 0;
	First_Index_Distance_Right = 0;
}
