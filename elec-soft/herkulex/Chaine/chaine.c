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
#define CHAINE_ID 10

/* Global variables */

/* Private variables */
Herkulex_Struct chaine;
const uint16_t CHAINE_speed_stop = 0;
const uint16_t CHAINE_speed = 500;
uint16_t CHAINE_pos_actuel = 0;
const uint16_t CHAINE_target[6] = {0,0,0,0,0,0};

void CHAINE_init(UART_HandleTypeDef * huart){
    Herkulex_initCommunication(&chaine, huart);
    Herkulex_reboot(&chaine, CHAINE_ID);
    Herkulex_initServos(&chaine);
    Herkulex_setLed(&chaine, CHAINE_ID, HERKULEX_LED_PINK);
    Herkulex_changeMode(&chaine, CHAINE_ID, HERKULEX_MODE_CONTINUOUS);
    Herkulex_torqueOFF(&chaine, CHAINE_ID);
    Herkulex_rotateOne(&chaine, CHAINE_ID, CHAINE_speed, HERKULEX_LED_PINK);
}

void CHAINE_spin(){
    Herkulex_torqueON(&chaine, CHAINE_ID);  
    Herkulex_rotateOne(&chaine, CHAINE_ID, CHAINE_speed, HERKULEX_LED_PINK);
}

void CHAINE_stop(){
    Herkulex_torqueOFF(&chaine, CHAINE_ID);
    Herkulex_rotateOne(&chaine, CHAINE_ID, CHAINE_speed_stop, HERKULEX_LED_PINK);
}

