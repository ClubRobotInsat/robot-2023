/**
 * @file herkulex.c
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Code to control the arm of robot CDF2024
 * @version 0.1
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

/* Includes */
#include <stdbool.h>
#include "herkulex.h"
#include "bras.h"

/* ID of Herkulex servos */
#define BRAS_EPAULE_ID 2
#define BRAS_COUDE_ID 11
#define BRAS_POIGNET_ID 1

/* Position of each arm configuration*/
const uint16_t position_home[3] = {171, 856, 745};
const uint16_t position_ready[3] = {317, 580, 669};
const uint16_t position_putInStock[3] = {343, 884, 765};
const uint16_t position_getInStock[3] = {512, 512, 512};

Herkulex_Struct bras;
Pince_StructTypeDef pince;

uint16_t position_actuel[3] = {0};

void BRAS_init(UART_HandleTypeDef * huart, TIM_HandleTypeDef * pinceTim, uint32_t pinceChannel){
    Herkulex_initCommunication(&bras, huart);
    pince.pinceTimer = pinceTim;
    pince.pinceChannel = pinceChannel;
    Herkulex_reboot(&bras, BRAS_EPAULE_ID);
    Herkulex_reboot(&bras, BRAS_COUDE_ID);
    Herkulex_reboot(&bras, BRAS_POIGNET_ID);
    Herkulex_initServos(&bras);
    Herkulex_setLed(&bras, HERKULEX_BROADCAST_ID, HERKULEX_LED_PINK);
    HAL_TIM_Base_Start(pince.pinceTimer);
    HAL_TIM_PWM_Start(pince.pinceTimer, pince.pinceChannel);
}

void BRAS_moveHomePosition(){
	uint8_t result;
    BRAS_grab();
    result = Herkulex_moveOne(&bras, BRAS_COUDE_ID, position_home[1], HERKULEX_LED_PINK);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, BRAS_EPAULE_ID, position_home[0], HERKULEX_LED_PINK);
    HAL_Delay(500);
    
    Herkulex_moveOne(&bras, BRAS_POIGNET_ID, position_home[2], HERKULEX_LED_PINK);
    HAL_Delay(500);
}

void BRAS_moveReadyPosition(){
    Herkulex_moveOne(&bras, BRAS_EPAULE_ID, position_ready[0], HERKULEX_LED_GREEN);
    HAL_Delay(500);
    
    Herkulex_moveOne(&bras, BRAS_POIGNET_ID, position_ready[2], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, BRAS_COUDE_ID, position_ready[1], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    BRAS_release();    
}

void BRAS_movePutInStock(){
    Herkulex_moveOne(&bras, BRAS_EPAULE_ID, position_putInStock[0], HERKULEX_LED_GREEN);
    HAL_Delay(500);
    
    Herkulex_moveOne(&bras, BRAS_COUDE_ID, position_putInStock[1], HERKULEX_LED_GREEN);
    HAL_Delay(500);
    
    Herkulex_moveOne(&bras, BRAS_POIGNET_ID, position_putInStock[2], HERKULEX_LED_GREEN);
    HAL_Delay(500);
}

void BRAS_grab(void){
    // Apply 3% of duty cycle to the pince
    __HAL_TIM_SET_COMPARE(pince.pinceTimer, pince.pinceChannel, 3);
}

void BRAS_release(void){
    // Apply 6% of duty cycle to the pince
    __HAL_TIM_SET_COMPARE(pince.pinceTimer, pince.pinceChannel, 6);
}

void BRAS_getPosition(void){
    Herkulex_getPosition(&bras, BRAS_EPAULE_ID, &position_actuel[0]);
    Herkulex_getPosition(&bras, BRAS_COUDE_ID, &position_actuel[1]);
    Herkulex_getPosition(&bras, BRAS_POIGNET_ID, &position_actuel[2]);
}
