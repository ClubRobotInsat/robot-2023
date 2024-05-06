/**
 * @file chaine.c
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Code to control the chaine of robot CDF2024
 * @version 0.1
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

/* Includes */
#include <stdbool.h>
#include "herkulex.h"
#include "chaine.h"

/* ID of Herkulex servos */
#define CHAINE_HERKULEX_ID 10

/* Global variables */

/* Private variables */
Herkulex_Struct chaine;
const uint16_t CHAINE_SPEED_STOP = 0;
uint16_t CHAINE_pos_actuel = 0;
uint16_t CHAINE_speed = 500;

void CHAINE_init(UART_HandleTypeDef * huart){
    Herkulex_initCommunication(&chaine, huart);
    Herkulex_reboot(&chaine, CHAINE_HERKULEX_ID);
    Herkulex_initServos(&chaine);
    Herkulex_torqueON(&chaine, CHAINE_HERKULEX_ID);
    Herkulex_moveOne(&chaine, CHAINE_HERKULEX_ID, 21, HERKULEX_LED_GREEN);
    HAL_Delay(500);
    Herkulex_changeMode(&chaine, CHAINE_HERKULEX_ID, HERKULEX_MODE_CONTINUOUS);
}

void CHAINE_nextPosition(){
	Herkulex_torqueON(&chaine, CHAINE_HERKULEX_ID);
	Herkulex_rotateOne(&chaine, CHAINE_HERKULEX_ID, CHAINE_speed, HERKULEX_LED_PINK);
	//HAL_Delay(5000);
	//Herkulex_torqueOFF(&chaine, CHAINE_HERKULEX_ID);
	//Herkulex_rotateOne(&chaine, CHAINE_HERKULEX_ID, CHAINE_SPEED_STOP, HERKULEX_LED_BLUE);
}

void CHAINE_previousPosition(){
	Herkulex_torqueON(&chaine, CHAINE_HERKULEX_ID);
	Herkulex_rotateOne(&chaine, CHAINE_HERKULEX_ID, (-1)*CHAINE_speed, HERKULEX_LED_PINK);
	HAL_Delay(1000);
	Herkulex_torqueOFF(&chaine, CHAINE_HERKULEX_ID);
	Herkulex_rotateOne(&chaine, CHAINE_HERKULEX_ID, CHAINE_SPEED_STOP, HERKULEX_LED_GREEN);
}

