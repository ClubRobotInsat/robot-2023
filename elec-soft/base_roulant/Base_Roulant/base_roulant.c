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

#define BR_SPEED_LIMIT_PWM				90.0
#define BR_SPEED_LIMIT_MM_S				900.0
#define BR_SPEED_CORRECTOR_THRESHOLD  	5.0			// uncertainty : +/- 5 mm/s
#define BR_SPEED_STOP_THRESHOLD			1.0 		// Command under 10.0 mm/s, send 0% PWM (no regulation)
#define BR_SPEED_CORRECTOR_KP			0.8
#define BR_SPEED_CORRECTOR_KPD			0.0
#define BR_SPEED_CORRECTOR_KPI			0.003		// Ancien : 0.003
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

// Derivative term for speed correction
float dkLeft;
float dkRight;

// Integral term for speed correction
float ikLeft;
float ikRight;

uint8_t enable_flag = 0;

/* ID of STM for CAN */
/* 
 * CAN ID of the STM32 <-- CHANGE THIS ACCORDING TO THE STM32
 * ID for CDF2024 :
 * 2 : Base Roulante (Left + Right) (Vert + Rouge)
 * 3 : Base Roulante 2 (Front + Rear) (Blue + Jaune)
 */
/* !!!!!!!!!!!!!!!!! */
#define CAN_ID_STM 2

/*********************/

/**
 * @brief Error Handler for the base roulant
 *
 * @param error Error code
 */
void BR_errorHandler(BR_Error_t error);

/**
 * @brief Execute the command from CAN
 */
void BR_executeCommandFromCAN(void);

void BR_init(TIM_HandleTypeDef * TIM_Regulate,TIM_HandleTypeDef * TIM_Motor_Left, uint32_t TIM_Channel_Motor_Left, TIM_HandleTypeDef * TIM_Motor_Right, uint32_t TIM_Channel_Motor_Right, GPIO_TypeDef * DIR_Port_Left, uint16_t DIR_Pin_Left, GPIO_TypeDef * DIR_Port_Right, uint16_t DIR_Pin_Right,  TIM_HandleTypeDef * TIM_Encoder_Left, TIM_HandleTypeDef * TIM_Encoder_Right, FDCAN_HandleTypeDef * hfdcan){
    /* Initialize the encoders */
    Encoder_Init(TIM_Encoder_Left, TIM_Encoder_Right);
    /* Initialize the motors */
    Motor_Init(&motor_left, DIR_Port_Left, DIR_Pin_Left, TIM_Motor_Left, TIM_Channel_Motor_Left);
    Motor_Init(&motor_right, DIR_Port_Right, DIR_Pin_Right, TIM_Motor_Right, TIM_Channel_Motor_Right);
    Timer_Regulate = TIM_Regulate;
    
	/* Init CAN interface */
    CAN_initInterface(hfdcan, CAN_ID_STM);
    CAN_filterConfig();
    CAN_setReceiveCallback(BR_executeCommandFromCAN);
    CAN_start();
    CAN_sendBackPing(CAN_ID_MASTER);
    HAL_Delay(100);

	/* Start all motors */
	BR_startAllMotors();
}

void BR_startAllMotors(void){
	enable_flag = 1;

	/* Init regulation loop variables */
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

    Motor_Start(&motor_left);
    Motor_Start(&motor_right);
    Motor_Set_Direction(&motor_left, MOTOR_DIRECTION_CW);
    Motor_Set_Direction(&motor_right, MOTOR_DIRECTION_CCW);

	/* Start motors at speed 0.0 */
	HAL_TIM_Base_Start_IT(Timer_Regulate);
	BR_setSpeed(BR_MOTOR_BROADCAST, 0.0);
}

void BR_stopAllMotors(void){
	enable_flag = 0;
	Motor_Stop(&motor_left);
    Motor_Stop(&motor_right);
	BR_setSpeed(BR_MOTOR_BROADCAST,0.0);
	HAL_TIM_Base_Stop_IT(Timer_Regulate);
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
	else if (motor == BR_MOTOR_BROADCAST){
		Motor_Set_Direction(&motor_left, direction);
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
	else if (motor == BR_MOTOR_BROADCAST){
		Motor_Set_Speed(&motor_left, pwm);
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
        return BR_DIRECTION_ERROR;
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


void BR_setSpeed(BR_Motor_ID_t motor,float speed) {
	/* Handle negative speed */
	if (speed >= 0.0){
		BR_setDirection(motor, BR_DIRECTION_CW);
	} else {
		BR_setDirection(motor, BR_DIRECTION_CCW);
		speed = -speed;
	}

	if(speed > BR_SPEED_LIMIT_MM_S)
	{
		speed = BR_SPEED_LIMIT_MM_S;
	}

	switch(motor){
		case BR_MOTOR_LEFT:
			targetSpeedLeft = speed;
			//BR_setPWM(BR_MOTOR_LEFT, (int)(speed * SPEED_TO_PWM_CONVERSION));
			break;
		case BR_MOTOR_RIGHT:
			targetSpeedRight = speed;
			//BR_setPWM(BR_MOTOR_RIGHT, (int)(speed * SPEED_TO_PWM_CONVERSION));
			break;
		case BR_MOTOR_BROADCAST:
			targetSpeedLeft = speed;
			targetSpeedRight = speed;
			//BR_setPWM(BR_MOTOR_BROADCAST, (int)(speed * SPEED_TO_PWM_CONVERSION));
			break;
		default:
			break;
	}
}



void BR_regulateSpeed(void){
	float currentSpeedLeft;
	float currentSpeedRight;
    float errorLeft;
    float errorRight;

    if (enable_flag != 0){

		currentSpeedLeft = BR_getSpeed(BR_MOTOR_LEFT);
		currentSpeedRight = BR_getSpeed(BR_MOTOR_RIGHT);
		if((currentSpeedLeft < 0.0) ||(currentSpeedRight < 0.0))
		{
			//detect invalid speed readings, encoder return -1.0 if error
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
		if ((regulatedSpeedLeft >= 0.0) && (regulatedSpeedLeft <= BR_SPEED_LIMIT_MM_S))
		{
			applySpeedLeft = regulatedSpeedLeft;
		} else if(regulatedSpeedLeft > BR_SPEED_LIMIT_MM_S){
			applySpeedLeft = BR_SPEED_LIMIT_MM_S;
		} else if(regulatedSpeedLeft < 0.0){
			applySpeedLeft = 0.0;
		}

		if ((regulatedSpeedRight >= 0.0) && (regulatedSpeedRight <= BR_SPEED_LIMIT_MM_S))
		{
			applySpeedRight = regulatedSpeedRight;
		} else if(regulatedSpeedRight > BR_SPEED_LIMIT_MM_S){
			applySpeedRight = BR_SPEED_LIMIT_MM_S;
		} else if(regulatedSpeedRight < 0.0){
			applySpeedRight = 0.0;
		}

		if(targetSpeedLeft > BR_SPEED_STOP_THRESHOLD)
		{
			BR_setPWM(BR_MOTOR_LEFT, (int)(applySpeedLeft * SPEED_TO_PWM_CONVERSION));
		}
		else
		{
			BR_setPWM(BR_MOTOR_LEFT, 0);
			ikLeft = 0;
		}

		if(targetSpeedRight > BR_SPEED_STOP_THRESHOLD)
		{
			BR_setPWM(BR_MOTOR_RIGHT, (int)(applySpeedRight * SPEED_TO_PWM_CONVERSION));
		}
		else
		{
			BR_setPWM(BR_MOTOR_RIGHT, 0);
			ikRight = 0;
		}


	   // Update old speed
		oldSpeedLeft = currentSpeedLeft;
		oldSpeedRight = currentSpeedRight;
    }
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


void BR_getCMDfromCAN(void){
    CAN_read();
}

void BR_executeCommandFromCAN(void){
	uint8_t * cmd = CAN_getRXData();
    uint8_t dataToRaspi[8] = {0,0,0,0,0,0,0,0};
    uint32_t speed_uninterpreted;
    float speed_float;
    float distance;
    uint32_t speed_mem = *((uint32_t*)&speed_float);

    uint32_t distance_mem = *((uint32_t*)&distance);
    BR_Motor_ID_t idMotor;

    uint32_t test_data = 0;
	switch (cmd[0]){
		case 0:
			BR_stopAllMotors();
			break;
		case 1:
			CAN_sendBackPing(CAN_ID_MASTER);
			break;
		case 2:
			BR_startAllMotors();
			break;
		case 3:
			idMotor = cmd[1];
			// speed en float 32, bigEndian
			speed_uninterpreted = (((cmd[2] & 0x000000FF)<< 24) + (cmd[3]<<16) + (cmd[4]<<8) + cmd[5]);
			// Convert to float
			speed_float = *((float*)&speed_uninterpreted);
			// Set speed
			BR_setSpeed(idMotor,speed_float);
			/*
			test_data = (uint32_t) speed_float;
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = cmd[1];

			dataToRaspi[2] = (test_data >> 24) & 0xFF;
			dataToRaspi[3] = (test_data >> 16) & 0xFF ;
			dataToRaspi[4] = (test_data >> 8) & 0xFF;
			dataToRaspi[5] = test_data & 0xFF;

			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			*/
			break;
		case 4:
			idMotor = cmd[1];
			// speed en float 32, bigEndian
			speed_uninterpreted = (((cmd[2] & 0x000000FF)<< 24) + (cmd[3]<<16) + (cmd[4]<<8) + cmd[5]);
			// Convert to float
			speed_float = *((float*)&speed_uninterpreted);
			// Set speed
			BR_setSpeed(idMotor,speed_float);

			test_data = (uint32_t) speed_float;
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = cmd[1];

			dataToRaspi[2] = (test_data >> 24) & 0xFF;
			dataToRaspi[3] = (test_data >> 16) & 0xFF ;
			dataToRaspi[4] = (test_data >> 8) & 0xFF;
			dataToRaspi[5] = test_data & 0xFF;

			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 5:
			//BR_setDirection(cmd[1], cmd[2]);
			break;
		case 6:
			// Get speed from encoder
			speed_float = BR_getSpeed(cmd[1]);
			speed_mem = *((uint32_t*)&speed_float);

			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = cmd[1];

			// speed en float 32, bigEndian
			dataToRaspi[2] = (speed_mem >> 24) & 0xFF;
			dataToRaspi[3] = (speed_mem >> 16) & 0xFF ;
			dataToRaspi[4] = (speed_mem >> 8) & 0xFF;
			dataToRaspi[5] = speed_mem & 0xFF;

			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 7:

			distance = BR_getDistance(cmd[1]);
			distance_mem = *((uint32_t*)&distance);

			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = cmd[1];
			// bigEndian : https://en.wikipedia.org/wiki/Endianness
			dataToRaspi[2] = (distance_mem >> 24) & 0xFF;
			dataToRaspi[3] = (distance_mem >> 16) & 0xFF;
			dataToRaspi[4] = (distance_mem >> 8) & 0xFF;
			dataToRaspi[5] = distance_mem & 0xFF;

			CAN_send(dataToRaspi, 2, CAN_ID_MASTER);
			break;
		case 8:
			BR_startRecordDistance();
			break;
		default :
			break;
	}
}

void BR_errorHandler(BR_Error_t error){
    /* Error handling to implement */
}
