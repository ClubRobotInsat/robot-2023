/*
 * movement.c
 *
 *  Created on: Feb 1, 2024
 *      Author: hoang
 */
#include "MOVEMENT/movement.h"


float distance_target;
float val_enc_L;
float val_enc_R;
float err_R;
float err_L;

Motor_Config motor_Right;
Motor_Config motor_Left;

void Movement_Init (Motor_Config motor_R, Motor_Config motor_L)
{
	motor_Righ = motor_R;
	motor_Left = motor_L;
	Encoder_Start_Record();
	Motor_Start(motor_Righ);
	Motor_Start(motor_Left);
	HAL_Delay(2000);

}

void Movement_Set_Speed(int speed){

	if(speed > MOVEMENT_MAX_VITESSE)
	{
		speed = MOVEMENT_MAX_VITESSE;
	}
	else if(speed < MOVEMENT_MIN_VITESSE)
	{
		speed = MOVEMENT_MIN_VITESSE;
	}
	else
	{
		Motor_Set_Direction(motor_Left, MOTOR_DIRECTION_CW);
		Motor_Set_Direction(motor_Righ, MOTOR_DIRECTION_CW);
		Motor_Set_Speed(motor_Left, speed);
		Motor_Set_Speed(motor_Righ, speed);
	}

}
void Movement_Stop(void)
{
	Motor_Stop(motor_Left);
	Motor_Stop(motor_Righ);

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
	}
}

void Movement_Regulation(void)
{
	val_enc_L = Encoder_Left_Get_Distance();
	val_enc_R = Encoder_Right_Get_Distance();
	err_l = target + val_enc_L;
	err_r = target - val_enc_R;

	if (distance_target > 0)
	{

	}
	else if (distance_target < 0)
	{

	}
	else
	{
		Movement_Stop();
	}

}
