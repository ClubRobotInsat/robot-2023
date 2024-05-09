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
#include "chaine.h"
/* ID of Herkulex servos */
#define EPAULE_HERKULEX_ID 2
#define COUDE_HERKULEX_ID 11
#define POIGNET_HERKULEX_ID 1

/* ID of servo in arm configuration (not Herkulex ID) */
//#define EPAULE_ID 0
//#define COUDE_ID 1
//#define POIGNET_ID 2

/* CAN ID of HERKULEX board */
#define CAN_ID_STM 4

/* Position of each arm configuration*/
const uint16_t position_home[3] = {709, 921, 845};
const uint16_t position_ready[3] = {834, 909, 652};	//1003, 960, 825
const uint16_t position_findObjetLow[3] = {812, 680, 259};
const uint16_t position_findObjetHigh[3] = {818, 770, 260};
const uint16_t position_putInStock[3] = {831, 949, 814};	//830 980 800
const uint16_t position_goToStock[3] = {912, 900, 865};
const uint16_t position_takeStock[3] = {780, 900, 865};
const uint16_t position_getStockOut[3] = {790, 990, 865};
const uint16_t position_placeObjet[3] = {832, 694, 298};

typedef enum {
    EPAULE_ID = 0,
    COUDE_ID = 1,
 	POIGNET_ID = 2,
} BRAS_Joint_ID_t;

Herkulex_Struct bras;
Pince_StructTypeDef pince;

uint16_t position_actuel[3] = {0};
uint16_t position_target[3] = {0};

void BRAS_errorHandler(void){

}

void BRAS_execCMD(void);

void BRAS_init(UART_HandleTypeDef * huartArm, UART_HandleTypeDef * huartChaine, TIM_HandleTypeDef * pinceTim, uint32_t pinceChannel, FDCAN_HandleTypeDef * hfdcan){
	CAN_initInterface(hfdcan, 4);
	CAN_filterConfig();
	CAN_setReceiveCallback(BRAS_execCMD);
	CAN_start();
	CHAINE_init(huartChaine);
    Herkulex_initCommunication(&bras, huartArm);
    pince.pinceTimer = pinceTim;
    pince.pinceChannel = pinceChannel;
    Herkulex_reboot(&bras, EPAULE_HERKULEX_ID);
    HAL_Delay(400);
    Herkulex_reboot(&bras, COUDE_HERKULEX_ID);
    HAL_Delay(400);
    Herkulex_reboot(&bras, POIGNET_HERKULEX_ID);
    HAL_Delay(400);
    Herkulex_initServos(&bras);
    Herkulex_setLed(&bras, HERKULEX_BROADCAST_ID, HERKULEX_LED_PINK);
    HAL_TIM_Base_Start(pince.pinceTimer);
    HAL_TIM_PWM_Start(pince.pinceTimer, pince.pinceChannel);
    //Herkulex_torqueOFF(&bras, HERKULEX_BROADCAST_ID);
}

void BRAS_reboot(uint8_t herkulexID){
	Herkulex_reboot(&bras, herkulexID);
	HAL_Delay(400);

	Herkulex_clearError(&bras, herkulexID);
	HAL_Delay(100);

	Herkulex_setACK(&bras, HERKULEX_ACK_REPLY_READ);
	HAL_Delay(100);

	Herkulex_torqueON(&bras, herkulexID);
	HAL_Delay(100);
}

uint8_t BRAS_checkError(uint8_t herkulexID, uint8_t stateDesired){
	uint8_t status = 255;
	uint8_t attemp = 0;

	while ((status != stateDesired) && attemp < 3){
		Herkulex_getStatusError(&bras, herkulexID, &status);
		if (status != stateDesired){
			BRAS_reboot(herkulexID);
		}
		attemp++;
	}
	if (status != stateDesired){
		return 0;
	}
	Herkulex_setLed(&bras, herkulexID, HERKULEX_LED_BLUE);
	return 1;
}

uint8_t BRAS_checkInPosition(uint8_t herkulexID){
	uint8_t status = 255;
	uint8_t attemp = 0;
	uint8_t stateDesired = HERKULEX_DETAIL_MOTOR_ON | HERKULEX_DETAIL_INPOSITION;
	while ((status != stateDesired) && attemp < 10){
		Herkulex_getStatusDetail(&bras, herkulexID, &status);
		if (status != stateDesired){
			HAL_Delay(100);
		}
		attemp++;
	}
	if (status != stateDesired){
		return 0;
	}
	Herkulex_setLed(&bras, herkulexID, HERKULEX_LED_GREEN2);
	return 1;
}

uint8_t BRAS_doAction(BRAS_Joint_ID_t joint, uint16_t goal_position){
	uint8_t herkulex_id;
	switch (joint){
		case EPAULE_ID:
			herkulex_id = EPAULE_HERKULEX_ID;
			break;
		case COUDE_ID:
			herkulex_id = COUDE_HERKULEX_ID;
			break;
		case POIGNET_ID:
			herkulex_id = POIGNET_HERKULEX_ID;
			break;
		default:
			break;
	}
	if (BRAS_checkError(herkulex_id, HERKULEX_STATUS_OK) == 0){
		return 0;
	}
	Herkulex_torqueON(&bras, herkulex_id);
	Herkulex_moveOne(&bras, herkulex_id, goal_position, HERKULEX_LED_GREEN);
	HAL_Delay(100);
	
//	if (BRAS_checkInPosition(herkulex_id) == 0){
//		return 0;
//	}
	HAL_Delay(500);
	Herkulex_torqueBREAK(&bras, herkulex_id);
	return 1;
}

uint8_t BRAS_moveHomePosition(){
    BRAS_grab();
    HAL_Delay(100);

	if (BRAS_doAction(COUDE_ID, position_home[COUDE_ID]) == 0){
		return 0;
	}

	if (BRAS_doAction(POIGNET_ID, position_home[POIGNET_ID]) == 0){
		return 0;
	}

	if (BRAS_doAction(EPAULE_ID, position_home[EPAULE_ID]) == 0){
		return 0;
	}
    return 1;
}

uint8_t BRAS_moveReadyPosition(){
//	uint8_t result = 0;
//	BRAS_release();
//	HAL_Delay(200);

	BRAS_grab();
	HAL_Delay(200);

	if (BRAS_doAction(EPAULE_ID, position_ready[EPAULE_ID])){
	return 0;
	}
	if (BRAS_doAction(POIGNET_ID, position_ready[POIGNET_ID])){
	return 0;
	}
	if (BRAS_doAction(COUDE_ID, position_ready[COUDE_ID])){
	return 0;
	}
	return 1;

//    if (Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_ready[POIGNET_ID], HERKULEX_LED_BLUE) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_ready[EPAULE_ID], HERKULEX_LED_GREEN) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_ready[COUDE_ID], HERKULEX_LED_GREEN2) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    result = 1;
//    return result;
}

uint8_t BRAS_moveFindObjetLow(){
//	uint8_t result = 0;

	if (BRAS_doAction(POIGNET_ID, position_findObjetLow[POIGNET_ID])){
	return 0;
	}
	if (BRAS_doAction(EPAULE_ID, position_findObjetLow[EPAULE_ID])){
	return 0;
	}
	if (BRAS_doAction(COUDE_ID, position_findObjetLow[COUDE_ID])){
	return 0;
	}
	BRAS_release();
	return 1;
//
//    if (Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_findObjetLow[POIGNET_ID], HERKULEX_LED_GREEN) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_findObjetLow[EPAULE_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_findObjetLow[COUDE_ID], HERKULEX_LED_GREEN) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    BRAS_release();
//
//    result = 1;
//    return result;
}

uint8_t BRAS_moveFindObjetHigh(){
//	uint8_t result = 0;

	if (BRAS_doAction(POIGNET_ID, position_findObjetHigh[POIGNET_ID])){
	return 0;
	}
	if (BRAS_doAction(EPAULE_ID, position_findObjetHigh[EPAULE_ID])){
	return 0;
	}
	if (BRAS_doAction(COUDE_ID, position_findObjetHigh[COUDE_ID])){
	return 0;
	}
	BRAS_release();
	return 1;

//    if (Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_findObjetHigh[POIGNET_ID], HERKULEX_LED_GREEN) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_findObjetHigh[EPAULE_ID], HERKULEX_LED_GREEN) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_findObjetHigh[COUDE_ID], HERKULEX_LED_GREEN) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    result = 1;
//    return result;
}

uint8_t BRAS_movePutInStock(){
//	uint8_t result = 0;

	 if (BRAS_doAction(COUDE_ID, position_putInStock[COUDE_ID])){
	 	return 0;
	 }
	 if (BRAS_doAction(POIGNET_ID, position_putInStock[POIGNET_ID])){
	 	return 0;
	 }
	 if (BRAS_doAction(EPAULE_ID, position_putInStock[EPAULE_ID])){
	 	return 0;
	 }
	 BRAS_release();
	 HAL_Delay(1000);
	 return 1;
//
//    if (Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_putInStock[COUDE_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_putInStock[POIGNET_ID], HERKULEX_LED_GREEN) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_putInStock[EPAULE_ID], HERKULEX_LED_GREEN) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//	 BRAS_release();
//	 HAL_Delay(1000);
//
//    result = 1;
//    return result;

//
//    Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_home[COUDE_ID], HERKULEX_LED_GREEN);
//    HAL_Delay(500);
}

uint8_t BRAS_moveGetFromStock(){
//	uint8_t result = 0;
	BRAS_release();
	HAL_Delay(500);

	if (BRAS_doAction(EPAULE_ID, position_goToStock[EPAULE_ID])){
	return 0;
	}
	if (BRAS_doAction(POIGNET_ID, position_goToStock[POIGNET_ID])){
	return 0;
	}
	if (BRAS_doAction(COUDE_ID, position_goToStock[COUDE_ID])){
	return 0;
	}
	
	if (BRAS_doAction(COUDE_ID, position_takeStock[COUDE_ID])){
	return 0;
	}
	if (BRAS_doAction(POIGNET_ID, position_takeStock[POIGNET_ID])){
	return 0;
	}
	if (BRAS_doAction(EPAULE_ID, position_takeStock[EPAULE_ID])){
	return 0;
	}
	BRAS_grab();
	HAL_Delay(1000);

	if (BRAS_doAction(COUDE_ID, position_getStockOut[COUDE_ID])){
	return 0;
	}
	if (BRAS_doAction(POIGNET_ID, position_getStockOut[POIGNET_ID])){
	return 0;
	}
	if (BRAS_doAction(EPAULE_ID, position_getStockOut[EPAULE_ID])){
	return 0;
	}
	return 1;



//	if (Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_goToStock[EPAULE_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//	HAL_Delay(1000);
//
//	if (Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_goToStock[POIGNET_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//	HAL_Delay(1000);
//
//	if (Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_goToStock[COUDE_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//	HAL_Delay(1000);
//
//	if (Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_takeStock[COUDE_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//	HAL_Delay(1000);
//
//	if (Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_takeStock[POIGNET_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//	HAL_Delay(1000);
//
//	if (Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_takeStock[EPAULE_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//	HAL_Delay(1000);
//
//	BRAS_grab();
//	HAL_Delay(1000);
//
//	if (Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_getStockOut[COUDE_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//	HAL_Delay(1000);
//
//	if (Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_getStockOut[POIGNET_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//	HAL_Delay(1000);
//
//	if (Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_getStockOut[EPAULE_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//	HAL_Delay(1000);
//
//	result = 1;
//	return result;
}

uint8_t BRAS_movePlaceObject(void){
//	uint8_t result = 0;

	if (BRAS_doAction(EPAULE_ID, position_findObjetLow[EPAULE_ID])){
	return 0;
	}
	if (BRAS_doAction(POIGNET_ID, position_findObjetLow[POIGNET_ID])){
	return 0;
	}
	if (BRAS_doAction(COUDE_ID, position_findObjetLow[COUDE_ID])){
	return 0;
	}
	BRAS_release();
	HAL_Delay(1000);
	return 1;

//    if (Herkulex_moveOne(&bras, EPAULE_HERKULEX_ID, position_findObjetLow[EPAULE_ID], HERKULEX_LED_GREEN) != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, POIGNET_HERKULEX_ID, position_findObjetLow[POIGNET_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    if (Herkulex_moveOne(&bras, COUDE_HERKULEX_ID, position_findObjetLow[COUDE_ID], HERKULEX_LED_GREEN)  != HERKULEX_STATUS_OK){
//    	BRAS_errorHandler();
//    	return result;
//    }
//    HAL_Delay(500);
//
//    BRAS_release();
//    HAL_Delay(1000);
//
//    result = 1;
//    return result;
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

void BRAS_readMsgFromCAN(void){
	CAN_read();
}

void BRAS_execCMD(void){
	uint8_t * cmd = CAN_getRXData();
    uint8_t dataToRaspi[8] = {0,0,0,0,0,0,0,0};
    uint8_t result = 0;
    uint8_t data;
	switch (cmd[0]){
		case 0:
			Herkulex_torqueBREAK(&bras, HERKULEX_BROADCAST_ID);
			break;
		case 1:
			CAN_sendBackPing(CAN_ID_MASTER);
			break;
		case 2:
			Herkulex_torqueON(&bras, HERKULEX_BROADCAST_ID);
			break;
		case 3:
			Herkulex_torqueOFF(&bras, HERKULEX_BROADCAST_ID);
			break;
		case 4:
			BRAS_getPosition();
			dataToRaspi[0] = cmd[0];
			dataToRaspi[2] = (position_actuel[0] >> 8) & 0xFF;
			dataToRaspi[3] = position_actuel[0] & 0xFF;
			dataToRaspi[4] = (position_actuel[1] >> 8) & 0xFF;
			dataToRaspi[5] = position_actuel[1] & 0xFF;
			dataToRaspi[6] = (position_actuel[2] >> 8) & 0xFF;
			dataToRaspi[7] = position_actuel[2] & 0xFF;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 13:
			switch (cmd[1]){
				case EPAULE_ID :
					BRAS_reboot(EPAULE_HERKULEX_ID);
					break;
				case COUDE_ID :
					BRAS_reboot(COUDE_HERKULEX_ID);
					break;
				case POIGNET_ID :
					BRAS_reboot(POIGNET_HERKULEX_ID);
					break;
				default :
					break;
			}
		case 15 :
			switch (cmd[1]){
				case EPAULE_ID :
					Herkulex_getStatusError(&bras, EPAULE_HERKULEX_ID, &data);
					break;
				case COUDE_ID :
					Herkulex_getStatusError(&bras, COUDE_HERKULEX_ID, &data);
					break;
				case POIGNET_ID :
					Herkulex_getStatusError(&bras, POIGNET_HERKULEX_ID, &data);
					break;
				default :
					break;
			}
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = cmd[1];
			dataToRaspi[2] = data;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 16 :
			switch (cmd[1]){
				case EPAULE_ID :
					Herkulex_getStatusDetail(&bras, EPAULE_HERKULEX_ID, &data);
					break;
				case COUDE_ID :
					Herkulex_getStatusDetail(&bras, COUDE_HERKULEX_ID, &data);
					break;
				case POIGNET_ID :
					Herkulex_getStatusDetail(&bras, POIGNET_HERKULEX_ID, &data);
					break;
				default :
					break;
			}
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = cmd[1];
			dataToRaspi[2] = data;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 17:
			BRAS_grab();
			result = 1;
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = result;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 18:
			BRAS_release();
			result = 1;
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = result;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 19:
			result = BRAS_moveHomePosition();
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = result;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 20:
			result = BRAS_moveReadyPosition();
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = result;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 21:
			result = BRAS_moveFindObjetLow();
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = result;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 22:
			result = BRAS_moveFindObjetHigh();
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = result;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 23:
			result = BRAS_movePutInStock();
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = result;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 24:
			result = BRAS_moveGetFromStock();
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = result;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 25:
			result = BRAS_movePlaceObject();
			dataToRaspi[0] = cmd[0];
			dataToRaspi[1] = result;
			CAN_send(dataToRaspi, 1, CAN_ID_MASTER);
			break;
		case 26:
			CHAINE_spin();
			break;
		case 27:
			CHAINE_stop();
			break;
		default :
			break;
	}
}
