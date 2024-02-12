/*
 * movement.c
 *
 *  Created on: Feb 1, 2024
 *      Author: hoang
 */
#include "MOVEMENT/movement.h"
#include <stdlib.h>

int proportional_gain;

float distance_target;
float val_enc_L;
float val_enc_R;
float err_R;
float err_L;

Motor_Config motor_Right;
Motor_Config motor_Left;

void Movement_Init(Motor_Config motor_R, Motor_Config motor_L, TIM_HandleTypeDef * htim_EncoderL,TIM_HandleTypeDef * htim_EncoderR)
{
	motor_Right = motor_R;
	motor_Left = motor_L;
	Motor_Start(motor_Right);
	Motor_Start(motor_Left);
	Encoder_Init(htim_EncoderL,htim_EncoderR);
	HAL_Delay(2000);

}

void Movement_Set_Speed(Motor_Config motor, int speed){

	if(speed < 0)
	{
		Motor_Set_Direction(motor, MOTOR_DIRECTION_CW);
	}
	else
	{
		Motor_Set_Direction(motor, MOTOR_DIRECTION_CCW);
	}

	speed = abs(speed) + MOVEMENT_MIN_VITESSE;

	if(speed > MOVEMENT_MAX_VITESSE)
	{
		speed = MOVEMENT_MAX_VITESSE;
	}
	else if(speed < MOVEMENT_MIN_VITESSE)
	{
		speed = MOVEMENT_MIN_VITESSE;
	}

	Motor_Set_Speed(motor, speed);

}
void Movement_Stop(void)
{
	Motor_Stop(motor_Left);
	Motor_Stop(motor_Right);

}

void Movement_Set_Distance(int distance)
{
	if(distance > MOVEMENT_MAX_DISTANCE)
	{
		distance_target = MOVEMENT_MAX_DISTANCE;
	}
	else
	{
		distance_target = distance;
		proportional_gain = MOVEMENT_MAX_VITESSE/distance_target;
	}
}

int Movement_Regulation(void)
{
	val_enc_L = Encoder_Left_Get_Distance();
	val_enc_R = Encoder_Right_Get_Distance();
	err_L = distance_target - val_enc_L;
	err_R = distance_target + val_enc_R;

	if (err_L > 0 || err_R > 0)
	{
		if(abs(err_L))
		{
			Movement_Set_Speed(motor_Left,err_L*proportional_gain);
		}
		if (abs(err_R))
		{
			Movement_Set_Speed(motor_Right,err_R*proportional_gain);
		}
		return 1;
	}

	else
	{
		Movement_Stop();
		return 0;
	}

}

void Movement_Start(int distance)
{
	Encoder_Start_Record();
	Movement_Set_Distance(distance);
	Movement_Set_Speed(motor_Left,MOVEMENT_MAX_VITESSE);
	Movement_Set_Speed(motor_Right,MOVEMENT_MAX_VITESSE);
	//while (Movement_Regulation()){}
	Movement_Regulation();
}
