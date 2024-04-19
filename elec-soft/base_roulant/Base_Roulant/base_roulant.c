/**
 * @file base_roulant.c
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Code to control a DC Motor combined with a encoder
 * @version 0.1
 * @date 2023-12-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "base_roulant.h"

/* Define ------------------------------------------------------------------- */

#define BR_SPEED_LIMIT_PWM	90.0
#define BR_SPEED_LIMIT_MM_S	900.0
#define BR_SPEED_CORRECTOR_THRESHOLD  30.0			// uncertainty : +/- 30 mm/s
#define BR_SPEED_CORRECTOR_KP			1.0
#define BR_SPEED_CORRECTOR_KPD			0.9
#define BR_SPEED_CORRECTOR_KPI			0.003
/* Global variables ----------------------------------------------------------*/
Motor_Config motor_left;
Motor_Config motor_right;
TIM_HandleTypeDef * Timer_Regulate;

const float SPEED_TO_PWM_CONVERSION = BR_SPEED_LIMIT_PWM/BR_SPEED_LIMIT_MM_S;

float targetSpeedLeft; 		//	mm/s
float targetSpeedRight; 	//	mm/s
float oldSpeedLeft;			//	mm/s
float oldSpeedRight;		//	mm/s
float regulatedSpeedLeft;	//	mm/s
float regulatedSpeedRight;	//	mm/s
float applySpeedLeft;		//	mm/s
float applySpeedRight;		//	mm/s

// Derivative term
float dkLeft;
float dkRight;

// Integral term
float ikLeft;
float ikRight;
/**
 * @brief Error Handler for the base roulant
 *
 * @param error Error code
 */
void BR_errorHandler(BR_Error_t error){

}

void BR_init(TIM_HandleTypeDef * TIM_Regulate,TIM_HandleTypeDef * TIM_Motor_Left, uint32_t TIM_Channel_Motor_Left, TIM_HandleTypeDef * TIM_Motor_Right, uint32_t TIM_Channel_Motor_Right, GPIO_TypeDef * DIR_Port_Left, uint16_t DIR_Pin_Left, GPIO_TypeDef * DIR_Port_Right, uint16_t DIR_Pin_Right,  TIM_HandleTypeDef * TIM_Encoder_Left, TIM_HandleTypeDef * TIM_Encoder_Right){
    /* Initialize the encoders */
    Encoder_Init(TIM_Encoder_Left, TIM_Encoder_Right);
    /* Initialize the motors */
    Motor_Init(&motor_left, DIR_Port_Left, DIR_Pin_Left, TIM_Motor_Left, TIM_Channel_Motor_Left);
    Motor_Init(&motor_right, DIR_Port_Right, DIR_Pin_Right, TIM_Motor_Right, TIM_Channel_Motor_Right);

    Timer_Regulate = TIM_Regulate;
    HAL_Delay(100);
}

void BR_startAllMotors(void){
    Motor_Start(&motor_left);
    Motor_Start(&motor_right);
    HAL_TIM_Base_Start_IT(Timer_Regulate);
	BR_setSpeed(0.0);
	regulatedSpeedLeft=0.0;
	regulatedSpeedRight=0.0;
	oldSpeedLeft = 0.0;
	oldSpeedRight = 0.0;
	applySpeedLeft = 0.0;
	applySpeedRight = 0.0;
	dkLeft = 0.0;
	dkRight = 0.0;
	ikLeft = 0.0;
	ikRight = 0.0;
}

void BR_stopAllMotors(void){
	BR_setSpeed(0.0);
	regulatedSpeedLeft=0.0;
	regulatedSpeedRight=0.0;
	oldSpeedLeft = 0.0;
	oldSpeedRight = 0.0;
	applySpeedLeft = 0.0;
	applySpeedRight = 0.0;
	dkRight = 0.0;
	dkLeft = 0.0;
	ikLeft = 0.0;
	ikRight = 0.0;
    Motor_Stop(&motor_left);
    Motor_Stop(&motor_right);
}

void BR_setDirection(BR_Motor_ID_t motor, BR_Direction_t direction){   
    if (direction != BR_DIRECTION_CCW && direction != BR_DIRECTION_CW){
        /* Error handler to add*/
        BR_errorHandler(BR_ERROR_DIRECTION);
        return;
    }

    if (motor == BR_MOTOR_LEFT){
        Motor_Set_Direction(&motor_left, direction);
    }
    else if (motor == BR_MOTOR_RIGHT){
        Motor_Set_Direction(&motor_right, direction);
    } 
    else {
        /* Error handler to add*/
        BR_errorHandler(BR_ERROR_MOTOR_ID);
    }
}

void BR_setPWM(BR_Motor_ID_t motor, uint8_t pwm){
    if (pwm > 100 || pwm < 0){
        /* Error handler to add*/
        BR_errorHandler(BR_ERROR_PWM);
        return;
    }

    if (motor == BR_MOTOR_LEFT){
        Motor_Set_Speed(&motor_left, pwm);
    }
    else if (motor == BR_MOTOR_RIGHT){
        Motor_Set_Speed(&motor_right, pwm);
    } 
    else {
        /* Error handler to add*/
        BR_errorHandler(BR_ERROR_MOTOR_ID);
    }
}

float BR_getSpeed(BR_Motor_ID_t motor){
    if (motor == BR_MOTOR_LEFT){
        return Encoder_Left_Get_Speed();
    }
    else if (motor == BR_MOTOR_RIGHT){
        return Encoder_Right_Get_Speed();
    } 
    else {
        /* Error handler to add*/
        BR_errorHandler(BR_ERROR_MOTOR_ID);
        return 0;
    }
}

uint8_t BR_getPWM(BR_Motor_ID_t motor){
    if (motor == BR_MOTOR_LEFT){
        return Motor_Get_Duty_Cycle(&motor_left);
    }
    else if (motor == BR_MOTOR_RIGHT){
        return Motor_Get_Duty_Cycle(&motor_right);
    } 
    else {
        /* Error handler to add*/
        BR_errorHandler(BR_ERROR_MOTOR_ID);
        return 0;
    }
}

BR_Direction_t BR_getDirection(BR_Motor_ID_t motor){
    if (motor == BR_MOTOR_LEFT){
        return Motor_Get_Direction(&motor_left);
    }
    else if (motor == BR_MOTOR_RIGHT){
        return Motor_Get_Direction(&motor_right);
    } 
    else {
        /* Error handler to add*/
        BR_errorHandler(BR_ERROR_MOTOR_ID);
        return 0;
    }
}

void BR_startRecordDistance(void){
    Encoder_Start_Record_Distance();
}

float BR_getDistance(BR_Motor_ID_t motor){
    if (motor == BR_MOTOR_LEFT){
        return Encoder_Left_Get_Distance();
    }
    else if (motor == BR_MOTOR_RIGHT){
        return Encoder_Right_Get_Distance();
    } 
    else {
        /* Error handler to add*/
        BR_errorHandler(BR_ERROR_MOTOR_ID);
        return 0;
    }
}


void BR_setSpeed(float speed) {
	if(speed < 0.0)
	{
		BR_stopAllMotors();
		return;
	}

	if(speed > BR_SPEED_LIMIT_MM_S)
	{
		speed = BR_SPEED_LIMIT_MM_S;
	}

	targetSpeedLeft = speed;
	targetSpeedRight = speed;
}

void BR_regulateSpeed(void){
    float currentSpeedLeft = BR_getSpeed(BR_MOTOR_LEFT);
    float currentSpeedRight = BR_getSpeed(BR_MOTOR_RIGHT);
    float errorLeft;
    float errorRight;


	if((currentSpeedLeft < 0.0) ||(currentSpeedRight < 0.0))
	{	
        //detect invalid speed readings
		currentSpeedLeft = 0.0;
		currentSpeedRight = 0.0;
	}

	errorLeft = targetSpeedLeft - currentSpeedLeft;
	errorRight = targetSpeedRight - currentSpeedRight;

	dkLeft = BR_SPEED_CORRECTOR_KPD*(dkLeft - currentSpeedLeft + oldSpeedLeft);
	ikLeft = ikLeft + BR_SPEED_CORRECTOR_KPI*(errorLeft + applySpeedLeft - regulatedSpeedLeft);
	if(errorLeft > BR_SPEED_CORRECTOR_THRESHOLD || errorLeft < -BR_SPEED_CORRECTOR_THRESHOLD)   //error margin
	{
		regulatedSpeedLeft =  ikLeft + BR_SPEED_CORRECTOR_KP*errorLeft + dkLeft;
	}

	dkRight = BR_SPEED_CORRECTOR_KPD*(dkRight - currentSpeedRight + oldSpeedRight);
	ikRight = ikRight + BR_SPEED_CORRECTOR_KPI*(errorRight + applySpeedRight - regulatedSpeedRight);
	if(errorRight > BR_SPEED_CORRECTOR_THRESHOLD || errorRight < -BR_SPEED_CORRECTOR_THRESHOLD)
	{
		regulatedSpeedRight = ikRight + BR_SPEED_CORRECTOR_KP*errorRight + dkRight;
	}


    // Apply regulated speed to motors
	if(regulatedSpeedLeft < 0.0){
		applySpeedLeft = 0.0;
    }
	else if(regulatedSpeedLeft > BR_SPEED_LIMIT_MM_S){
		applySpeedLeft = BR_SPEED_LIMIT_MM_S;
    }
	else
	{
		applySpeedLeft = regulatedSpeedLeft;
	}

	if(regulatedSpeedRight < 0.0){
		applySpeedRight = 0.0;
    }
	else if(regulatedSpeedRight > BR_SPEED_LIMIT_MM_S){
		applySpeedRight = BR_SPEED_LIMIT_MM_S;
    }
	else
	{
		applySpeedRight = regulatedSpeedRight;
	}

	BR_setPWM(BR_MOTOR_LEFT, (int)(applySpeedLeft * SPEED_TO_PWM_CONVERSION));
	BR_setPWM(BR_MOTOR_RIGHT, (int)(applySpeedRight * SPEED_TO_PWM_CONVERSION));


   // Update old speed
	oldSpeedLeft = currentSpeedLeft;
	oldSpeedRight = currentSpeedRight;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	Encoder_IC_Interrupt_Handler(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	Encoder_Overflow_Interrupt_Handler(htim);
	if(htim == Timer_Regulate){
		BR_regulateSpeed();
	}
}



