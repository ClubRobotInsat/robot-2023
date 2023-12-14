/**
 * @file motor_dc.h
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Header for DC Motor Control
 * @version 0.1
 * @date 2023-12-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTOR_DC
#define MOTOR_DC

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ---------------------------------*/
#include "stm32g4xx_hal.h"

/*	######################################################################################
 * WARNING : This driver assume that the frequency of PWM signal for DC Motor is 20kHz !!!
 * #######################################################################################
*/

/* Exported define --------------------------*/
/* Motor direction*/
#define MOTOR_DIRECTION_CCW (GPIO_PIN_RESET)
#define MOTOR_DIRECTION_CW  (GPIO_PIN_SET)

/**
 * @brief Structure containing configuration for a motor
 * 
 */
typedef struct {
    /* Peripheral configuration */
    GPIO_TypeDef  * DIR_Port;
    uint16_t DIR_Pin;
    TIM_HandleTypeDef * TIM;
    uint32_t TIM_Channel;
} Motor_Config;

/**
  * @brief  Different Errors Types may occurs
  */
typedef enum {
	Wrong_Direction,
	Wrong_Percentage_Of_Duty_Cycle
} CONTROLLER_Error;

/**
 * @brief Initialize a the motor, duty cycle is set to 0, direction to CCW
 * 
 * @param DIR_Port 
 * @param DIR_Pin 
 * @param TIM 
 * @param TIM_channel 
 * @return Motor_Config 
 */
Motor_Config Motor_Init(GPIO_TypeDef  * DIR_Port, uint16_t DIR_Pin, TIM_HandleTypeDef * TIM, uint32_t TIM_Channel);

/**
 * @brief Start the motor, need to set speed
 *
 * @param motor
 */
void Motor_Start(Motor_Config motor);

/**
 * @brief Stop the motor
 *
 * @param motor
 */
void Motor_Stop(Motor_Config motor);


/**
 * @brief Set speed for DC Motor
 * 
 * @param motor 
 * @param duty_cycle Duty_cycle of PWM, must be between 0 and 100
 */
void Motor_Set_Speed(Motor_Config motor, uint8_t duty_cycle);

/**
 * @brief Set the direction of the motor
 * 
 * @param motor 
 * @param direction Direction of DC Motor, must be one of the following value
 *                    - MOTOR_DIRECTION_CW
 *                    - MOTOR_DIRECTION_CCW
 */
void Motor_Set_Direction(Motor_Config motor, uint8_t direction);

/**
 * @brief Get the current duty cycle of PWM sent to DC motor
 * 
 * @param motor 
 * @return uint8_t PWM duty cycle
 */
uint8_t Motor_Get_Speed(Motor_Config motor);

/**
 * @brief Get the current direction of the motor
 * 
 * @param motor 
 * @return uint8_t Direction of DC Motor, can be one of the following value
 *                    - MOTOR_DIRECTION_CW
 *                    - MOTOR_DIRECTION_CCW
 */
uint8_t Motor_Get_Direction(Motor_Config motor);

#ifdef __cplusplus
}
#endif

#endif
