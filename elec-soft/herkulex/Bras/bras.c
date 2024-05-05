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
#include "can_stm32.h"

/* ID of Herkulex servos */
#define EPAULE_HERKULEX_ID 2
#define COUDE_HERKULEX_ID 11
#define POIGNET_HERKULEX_ID 1

/* ID of servo in arm configuration (not Herkulex ID) */
#define EPAULE_ID 0
#define COUDE_ID 1
#define POIGNET_ID 2

/* CAN ID of HERKULEX board */
#define CAN_ID_STM 4

/* Position of each arm configuration*/
const uint16_t position_home[3] = {709, 921, 845};
const uint16_t position_ready[3] = {1003, 960, 825};
const uint16_t position_findObjetLow[3] = {812, 680, 259};
const uint16_t position_findObjetHigh[3] = {818, 770, 260};
const uint16_t position_putInStock[3] = {830, 980, 800};
const uint16_t position_goToStock[3] = {912, 900, 865};
const uint16_t position_takeStock[3] = {780, 900, 865};
const uint16_t position_getStockOut[3] = {790, 990, 865};
const uint16_t position_placeObjet[3] = {832, 694, 298};


Herkulex_Struct bras;
Pince_StructTypeDef pince;

uint16_t position_actuel[3] = {0};

void BRAS_init(UART_HandleTypeDef * huart, TIM_HandleTypeDef * pinceTim, uint32_t pinceChannel, FDCAN_HandleTypeDef * hfdcan){
	CAN_initInterface(hfdcan, 4);
	CAN_filterConfig();
	CAN_setReceiveCallback(BRAS_getCMDfromCAN);
	CAN_start();
    Herkulex_initCommunication(&bras, huart);
    pince.pinceTimer = pinceTim;
    pince.pinceChannel = pinceChannel;
    Herkulex_reboot(&bras, EPAULE_HERKULEX_ID);
    Herkulex_reboot(&bras, COUDE_HERKULEX_ID);
    Herkulex_reboot(&bras, POIGNET_HERKULEX_ID);
    Herkulex_initServos(&bras);
    Herkulex_setLed(&bras, HERKULEX_BROADCAST_ID, HERKULEX_LED_PINK);
    HAL_TIM_Base_Start(pince.pinceTimer);
    HAL_TIM_PWM_Start(pince.pinceTimer, pince.pinceChannel);
    //Herkulex_torqueOFF(&bras, HERKULEX_BROADCAST_ID);
}

void BRAS_moveHomePosition(){
	uint8_t result;
    BRAS_grab();
    HAL_Delay(100);
    result = Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_home[COUDE_ID], HERKULEX_LED_PINK);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_home[POIGNET_ID], HERKULEX_LED_PINK);
    HAL_Delay(500);
    
    Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_home[EPAULE_ID], HERKULEX_LED_PINK);
    HAL_Delay(500);

    //Herkulex_torqueOFF(&bras, HERKULEX_BROADCAST_ID);

}

void BRAS_moveReadyPosition(){
	BRAS_release();
	HAL_Delay(200);

    Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_ready[EPAULE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);
    
    Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_ready[POIGNET_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_ready[COUDE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

}

void BRAS_moveFindObjetLow(){
    Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_findObjetLow[POIGNET_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_findObjetLow[EPAULE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_findObjetLow[COUDE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    BRAS_release();
}

void BRAS_moveFindObjetHigh(){
    Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_findObjetHigh[EPAULE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_findObjetHigh[POIGNET_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_findObjetHigh[COUDE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);
}

void BRAS_movePutInStock(){
    Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_putInStock[COUDE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_putInStock[POIGNET_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_putInStock[EPAULE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    BRAS_release();
    HAL_Delay(1000);

//
//    Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_home[COUDE_ID], HERKULEX_LED_GREEN);
//    HAL_Delay(500);
}

void BRAS_moveGetFromStock(){
	BRAS_release();
	HAL_Delay(500);

	Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_goToStock[EPAULE_ID], HERKULEX_LED_GREEN);
	HAL_Delay(1000);

	Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_goToStock[POIGNET_ID], HERKULEX_LED_GREEN);
	HAL_Delay(1000);

	Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_goToStock[COUDE_ID], HERKULEX_LED_GREEN);
	HAL_Delay(1000);

	Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_takeStock[COUDE_ID], HERKULEX_LED_GREEN);
	HAL_Delay(1000);

	Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_takeStock[POIGNET_ID], HERKULEX_LED_GREEN);
	HAL_Delay(1000);

	Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_takeStock[EPAULE_ID], HERKULEX_LED_GREEN);
	HAL_Delay(1000);

	BRAS_grab();
	HAL_Delay(1000);

	Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_getStockOut[COUDE_ID], HERKULEX_LED_GREEN);
	HAL_Delay(1000);

	Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_getStockOut[POIGNET_ID], HERKULEX_LED_GREEN);
	HAL_Delay(1000);

	Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_getStockOut[EPAULE_ID], HERKULEX_LED_GREEN);
	HAL_Delay(1000);
}

void BRAS_movePlaceObject(void){
    Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_findObjetLow[EPAULE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_findObjetLow[POIGNET_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_findObjetLow[COUDE_ID], HERKULEX_LED_GREEN);
    HAL_Delay(500);

    BRAS_release();
    HAL_Delay(1000);
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
    Herkulex_getPosition(&bras, EPAULE_HERKULEX_ID, &position_actuel[EPAULE_ID]);
    Herkulex_getPosition(&bras, COUDE_HERKULEX_ID, &position_actuel[COUDE_ID]);
    Herkulex_getPosition(&bras, POIGNET_HERKULEX_ID, &position_actuel[POIGNET_ID]);
}

void BRAS_getCMDfromCAN(void){
	uint8_t * cmd = CAN_getRXData();
    uint8_t dataToRaspi[8] = {0,0,0,0,0,0,0,0};
	switch (cmd[0]){
		case 0:
			Herkulex_torqueOFF(&bras, HERKULEX_BROADCAST_ID);
			break;
		case 1:
			CAN_sendBackPing(CAN_ID_MASTER);
			break;
		case 2:
			Herkulex_torqueON(&bras, HERKULEX_BROADCAST_ID);
			break;
		case 17:
			BRAS_grab();
			break;
		case 18:
			BRAS_release();
			break;
		case 19:
			BRAS_moveHomePosition();
			break;
		case 20:
			BRAS_moveReadyPosition();
			break;
		case 21:
			BRAS_moveFindObjetLow();
			break;
		case 22:
			BRAS_moveFindObjetHigh();
			break;
		case 23:
			BRAS_movePutInStock();
			break;
		case 24:
			BRAS_moveGetFromStock();
			break;
		case 25:
			BRAS_movePlaceObject();
			break;
		default :
			break;
	}
}
