/**
  ******************************************************************************
  * @file           : Encoder.h
  * @brief          : Driver used to control 2 encoders
  * @date			: Feb 23, 2024
  * @author			: TANG Huong Cam
  ******************************************************************************
*/

#include "encoder.h"

#define ENCODER_TIMER_PRESCALER				127
#define ENCODER_TIMER_AUTORELOAD			65535
#define ENCODER_FREQ_CLOCKSOURCE			16000000UL
#define ENCODER_RESOLUTION     				1024			//Reference to Datasheet of Encoder 2400 1024
#define PERIMETER_WHEEL_IN_MM 				251.327			//Wheel perimeter in mm

#define ENCODERS_MAX_OVF					6

#define ENCODER_SPEED_IN_MM_PER_PULSE		(float)PERIMETER_WHEEL_IN_MM/(float)ENCODER_RESOLUTION	// in mm/pulse

//Timer frequency (before ARR) = FCPU/ENCODER_PRECALER (pulses/s)
#define ENCODER_FREQ_TIM 					(float)ENCODER_FREQ_CLOCKSOURCE/(float)ENCODER_TIMER_PRESCALER

//Constant to calculate the car speed = (Timer frequency before ARR)*(Number of rising pulses per mm) (pulses*mm/s)
#define ENCODER_CAL_SPEED  					ENCODER_FREQ_TIM * ENCODER_SPEED_IN_MM_PER_PULSE




TIM_HandleTypeDef * ENCODER_LEFT_TIMER;
TIM_HandleTypeDef * ENCODER_RIGHT_TIMER;

/**
 * Variables to stock the encoder position
 *
 * Position > 0 : move forward
 * Position < 0 : move backward
 */
float position_left = 0;
float position_right = 0;

/**
 * Variables to stock the timer counter value
 */
int prev_ccr_left = 0;
int prev_ccr_right = 0;
int delta_ccr_left = 0;
int delta_ccr_right = 0;


/**
 * Variables to stock the number of counter overflow
 */
int OVF_cnt_left = 0;
int OVF_cnt_right = 0;


/**
 * Variables to stock the encoder speed
 */
float speed_left = 0;
float speed_right = 0;

/**
 * Time counter (increase each mili-seconde by HAL_SYSTICK_Callback)
 */
int start_time = 0;

void Encoder_Init (TIM_HandleTypeDef * Encoder_Left_HTimer3, TIM_HandleTypeDef * Encoder_Right_HTimer4){

	ENCODER_LEFT_TIMER = Encoder_Left_HTimer3;
	ENCODER_RIGHT_TIMER = Encoder_Right_HTimer4;

	HAL_TIM_IC_Start_IT(ENCODER_RIGHT_TIMER,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(ENCODER_LEFT_TIMER,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(ENCODER_RIGHT_TIMER);
	HAL_TIM_Base_Start_IT(ENCODER_LEFT_TIMER);

}


void Encoder_Start_Record_Distance (void){
	position_left = 0;
	position_right = 0;
}

void Encoder_IC_Interrupt_Handler(TIM_HandleTypeDef *htim)
{
	if (htim == ENCODER_LEFT_TIMER) {

		position_left += ENCODER_SPEED_IN_MM_PER_PULSE;
    	int ccr_left = HAL_TIM_ReadCapturedValue(ENCODER_LEFT_TIMER,TIM_CHANNEL_1);
    	delta_ccr_left = ccr_left - prev_ccr_left + OVF_cnt_left*__HAL_TIM_GET_AUTORELOAD(ENCODER_LEFT_TIMER);
		prev_ccr_left = ccr_left;
		OVF_cnt_left=0;
	}
    if (htim == ENCODER_RIGHT_TIMER) {

		position_right += ENCODER_SPEED_IN_MM_PER_PULSE;
    	int ccr_right = HAL_TIM_ReadCapturedValue(ENCODER_RIGHT_TIMER,TIM_CHANNEL_1);
    	delta_ccr_right = ccr_right - prev_ccr_right + OVF_cnt_right*__HAL_TIM_GET_AUTORELOAD(ENCODER_RIGHT_TIMER);
		prev_ccr_right = ccr_right;
		OVF_cnt_right=0;

    }

}


void Encoder_Overflow_Interrupt_Handler(TIM_HandleTypeDef *htim)
{
	if (htim == ENCODER_LEFT_TIMER)
	{
		OVF_cnt_left++;
		if(OVF_cnt_left > ENCODERS_MAX_OVF) delta_ccr_left = -1;
	}

    if (htim == ENCODER_RIGHT_TIMER)
	{
		OVF_cnt_right++;
		if(OVF_cnt_right > ENCODERS_MAX_OVF) delta_ccr_right = -1;
	}

}


float Encoder_Left_Get_Speed(void){
	if(delta_ccr_left > 0)
	{
		return ENCODER_CAL_SPEED /delta_ccr_left;
	}
	else
	{
		return 0;
	}
}

float Encoder_Right_Get_Speed(void){
	if(delta_ccr_right > 0)
	{
		return ENCODER_CAL_SPEED /delta_ccr_right;
	}
	else
	{
		return 0;
	}
}

float Encoder_Left_Get_Distance (void){
	return position_left;
}

float Encoder_Right_Get_Distance (void){
	return position_right;
}





