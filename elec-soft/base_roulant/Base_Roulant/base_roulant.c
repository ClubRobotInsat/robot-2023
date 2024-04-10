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


/* Global variables ----------------------------------------------------------*/
Motor_Config motor_left;
Motor_Config motor_right;

/**
 * @brief Error Handler for the base roulant
 *
 * @param error Error code
 */
void BR_errorHandler(BR_Error_t error){

}

void BR_init(TIM_HandleTypeDef * TIM_Motor_Left, uint32_t TIM_Channel_Motor_Left, TIM_HandleTypeDef * TIM_Motor_Right, uint32_t TIM_Channel_Motor_Right, GPIO_TypeDef * DIR_Port_Left, uint16_t DIR_Pin_Left, GPIO_TypeDef * DIR_Port_Right, uint16_t DIR_Pin_Right,  TIM_HandleTypeDef * TIM_Encoder_Left, TIM_HandleTypeDef * TIM_Encoder_Right){
    /* Initialize the encoders */
    Encoder_Init(TIM_Encoder_Left, TIM_Encoder_Right);
    /* Initialize the motors */
    Motor_Init(&motor_left, DIR_Port_Left, DIR_Pin_Left, TIM_Motor_Left, TIM_Channel_Motor_Left);
    Motor_Init(&motor_right, DIR_Port_Right, DIR_Pin_Right, TIM_Motor_Right, TIM_Channel_Motor_Right);
    HAL_Delay(100);
}

void BR_startAllMotors(void){
    Motor_Start(&motor_left);
    Motor_Start(&motor_right);
}

void BR_stopAllMotors(void){
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
