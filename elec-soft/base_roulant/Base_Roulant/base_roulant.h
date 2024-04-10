/**
 * @file base_roulant.h
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Header file for the base roulant module
 * @version 0.1
 * @date 2021-03-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BASE_ROULANT_H
#define BASE_ROULANT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ---------------------------------*/
#include "stm32g4xx_hal.h"
#include "motor_dc.h"
#include <encoder.h>


/* ---------------------------Pin configurations --------------------------------- */

#define PWM_MOTOR_1_Pin GPIO_PIN_0
#define PWM_MOTOR_1_GPIO_Port GPIOA
#define PWM_MOTOR_2_Pin GPIO_PIN_1
#define PWM_MOTOR_2_GPIO_Port GPIOA
#define DIR_MOTOR_1_Pin GPIO_PIN_4
#define DIR_MOTOR_1_GPIO_Port GPIOA
#define A_ENCODER_1_Pin GPIO_PIN_6
#define A_ENCODER_1_GPIO_Port GPIOA
#define DIR_MOTOR_2_Pin GPIO_PIN_9
#define DIR_MOTOR_2_GPIO_Port GPIOA
#define A_ENCODER_2_Pin GPIO_PIN_6
#define A_ENCODER_2_GPIO_Port GPIOB



/* ---------------------------Exported Types --------------------------------- */
typedef enum {
    BR_MOTOR_LEFT = 0,
    BR_MOTOR_RIGHT = 1
} BR_Motor_ID_t;

typedef enum {
    BR_DIRECTION_CW = MOTOR_DIRECTION_CW,
    BR_DIRECTION_CCW = MOTOR_DIRECTION_CCW,
} BR_Direction_t;

typedef enum {
    BR_ERROR_NONE = 0,
    BR_ERROR_MOTOR_ID = 1,
    BR_ERROR_DIRECTION = 2,
    BR_ERROR_PWM = 3,
} BR_Error_t;

/* ----------------------------- Initialization ------------------------------------------*/
/**
 * @brief Initialize the base roulant
 * 
 * @param TIM_Motor_Left Timer for the left motor
 * @param TIM_Channel_Motor_Left Timer Channel for the left motor
 * @param TIM_Motor_Right Timer for the right motor
 * @param TIM_Channel_Motor_Right Timer Channel for the right motor
 * @param TIM_Encoder_Left Timer for the left encoder
 * @param TIM_Encoder_Right Timer for the right encoder
 * @param DIR_Port_Left GPIO Port for the direction pin of the left motor
 * @param DIR_Pin_Left GPIO Pin for the direction pin of the left motor
 * @param DIR_Port_Right GPIO Port for the direction pin of the right motor
 * @param DIR_Pin_Right GPIO Pin for the direction pin of the right motor
 */
void BR_init(TIM_HandleTypeDef * TIM_Motor_Left,
            uint32_t TIM_Channel_Motor_Left,     
            TIM_HandleTypeDef * TIM_Motor_Right,
            uint32_t TIM_Channel_Motor_Right, 
            GPIO_TypeDef * DIR_Port_Left, 
            uint16_t DIR_Pin_Left, 
            GPIO_TypeDef * DIR_Port_Right, 
            uint16_t DIR_Pin_Right,
            TIM_HandleTypeDef * TIM_Encoder_Left, 
            TIM_HandleTypeDef * TIM_Encoder_Right);

/* ----------------------------- API ------------------------------------------*/
/**
 * @brief Start all the motors at speed 0
 */
void BR_startAllMotors(void);

/**
 * @brief Stop all the motors
 */
void BR_stopAllMotors(void);

/**
 * @brief Set the direction of the motor
 * 
 * @param motor ID of the motor
 * @param direction Direction of the motor
 */
void BR_setDirection(BR_Motor_ID_t motor, BR_Direction_t direction);

/**
 * @brief Set the PWM of the motor
 * 
 * @param motor ID of the motor
 * @param pwm PWM value - 0 to 100
 */
void BR_setPWM(BR_Motor_ID_t motor, uint8_t pwm);

/**
 * @brief Get the speed of the motor
 * 
 * @param motor ID of the motor
 * @return float Speed of the motor in cm/s
 */
float BR_getSpeed(BR_Motor_ID_t motor);

/**
 * @brief Get the PWM of the motor
 * 
 * @param motor ID of the motor
 * @return uint8_t PWM value - 0 to 100
 */
uint8_t BR_getPWM(BR_Motor_ID_t motor);

/**
 * @brief Get the direction of the motor
 * 
 * @param motor ID of the motor
 * @return BR_Direction_t Direction of the motor
 */
BR_Direction_t BR_getDirection(BR_Motor_ID_t motor);

/**
 * @brief Start recording the distance
 * 
 */
void BR_startRecordDistance(void);

/**
 * @brief Get the distance of the motor
 * 
 * @param motor ID of the motor
 * @return float Distance in cm
 */
float BR_getDistance(BR_Motor_ID_t motor); 

#ifdef __cplusplus
}
#endif

#endif
