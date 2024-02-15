/**
 * @file herkulex.h
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Library to control a Herkulex servo motor
 * @version 0.1
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

/* Includes */
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "herkulex.h"

/* Private define */
#define SEND_TIMEOUT 3000	/* Timeout duration when send data through serial port*/
#define RECEIVE_TIMEOUT 3000	/* Timeout duration when receive data through serial port*/


/* Global variable */

bool messageReceived = false;

/****************************************************************************************************************
 *
 *
 * 	Private functions
 *
 *
 ****************************************************************************************************************/
/**
 * @fn void HAL_UART_RxCpltCallback(UART_HandleTypeDef*)
 * @brief Callback function when completing reception from RX USART
 *
 * @param huart	UART handler
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	messageReceived = true;
}

/**
 * @fn void HKL_sendData(Herkulex_Struct*)
 * @brief Send data via  USART TX to Herkulex servo
 *
 * @param servos	Herkulex_Struct handler
 */
void HKL_sendData(Herkulex_Struct * servos){
	HAL_StatusTypeDef uart_status = HAL_OK;

	__HAL_UART_FLUSH_DRREGISTER(servos->huart);
	do{
		uart_status = HAL_UART_Transmit(servos->huart, servos->msg.dataEx, servos->msg.pSize, SEND_TIMEOUT);
	} while (uart_status == HAL_BUSY);
}

/**
 * @fn void HKL_readData(Herkulex_Struct*, uint8_t*, uint8_t)
 * @brief Configure DMA to store data received from Herkulex in a buffer
 *
 * @param servos Herkulex_Struct handler
 * @param pData Pointer to buffer to store data
 * @param size	Number of bytes to store
 */
void HKL_readData(Herkulex_Struct * servos, uint8_t *pData, uint8_t size){
	messageReceived = false;
	HAL_UART_Receive_DMA(servos->huart, pData, size);
}

/**
 * @fn bool HKL_waitToReceiveData()
 * @brief Wait until received data from Herkulex
 *
 * @return	Whether a data is received
 */
bool HKL_waitToReceiveData(){
	uint32_t tickstart;

	tickstart = HAL_GetTick();
	while ((messageReceived == false) && (HAL_GetTick() < (RECEIVE_TIMEOUT + tickstart))){
		//wait until receive a new message or timeout
	}
	return messageReceived;
}

/**
 * @fn void HKL_addData(Herkulex_Struct*, uint8_t, uint8_t, uint8_t, uint8_t)
 * @brief To Do
 *
 * @param servos
 * @param GoalLSB
 * @param GoalMSB
 * @param set
 * @param servoID
 */
void HKL_addData(Herkulex_Struct * servos, uint8_t GoalLSB, uint8_t GoalMSB, uint8_t set, uint8_t servoID){}

/**
 * @fn uint8_t HKL_checksum1(Herkulex_Message_Struct*)
 * @brief Checksum 1 to verify data of Herkulex
 *
 * @param package Herkulex_Message_Struct handler
 * @return Result of Checksum1
 */
uint8_t HKL_checksum1(Herkulex_Message_Struct * msg){
	uint8_t XOR = 0;
	XOR = XOR ^ msg->pSize;
	XOR = XOR ^ msg->pID;
	XOR = XOR ^ msg->cmd;
	for (int i = 0; i < msg->lenghtString; i++)
	{
		XOR = XOR ^ msg->data[i];
	}
	return (XOR&0xFE);
}

/**
 * @fn uint8_t HKL_checksum2(uint8_t)
 * @brief Checksum2 to verify data of Herkulex
 *
 * @param XOR data to perform Checksum2
 * @return Result of Checksum2
 */
static inline uint8_t HKL_checksum2(uint8_t XOR){
	return (~XOR)&0xFE;
}
/****************************************************************************************************************
 *
 *
 * 	Exported functions
 *
 *
 ****************************************************************************************************************/


void HKL_begin(Herkulex_Struct * servos, UART_HandleTypeDef * huart){
	servos->huart = huart;
	servos->msg.conta = 0;
	HAL_Delay(100);
}

void  HKL_init(Herkulex_Struct * servos){
	HKL_reboot(servos, BROADCAST_ID);
	HAL_Delay(500);
	HKL_clearError(servos, BROADCAST_ID);
	HAL_Delay(500);

	HKL_ACK(servos, ACK_Reply_Read);
	HAL_Delay(500);

	HKL_torqueON(servos, BROADCAST_ID);
	HAL_Delay(500);
}

void  HKL_clearError(Herkulex_Struct * servos, uint8_t servoID){

	servos->msg.pSize = 0x0B;
	servos->msg.pID   = servoID;
	servos->msg.cmd   = HRAMWRITE;
	servos->msg.data[0] = 0x30;               // RAM Address
	servos->msg.data[1] = 0x02;               // Length
	servos->msg.data[2] = 0x00;               // Write error=0
	servos->msg.data[3] = 0x00;               // Write detail error=0

	servos->msg.lenghtString = 4;             // lenghtData

	servos->msg.ck1 = HKL_checksum1(&(servos->msg));
	servos->msg.ck2 = HKL_checksum2(servos->msg.ck1);

	servos->msg.dataEx[0] = 0xFF;
	servos->msg.dataEx[1] = 0xFF;
	servos->msg.dataEx[2] = servos->msg.pSize;
	servos->msg.dataEx[3] = servos->msg.pID;
	servos->msg.dataEx[4] = servos->msg.cmd;
	servos->msg.dataEx[5] = servos->msg.ck1;
	servos->msg.dataEx[6] = servos->msg.ck2;
	servos->msg.dataEx[7] = servos->msg.data[0];
	servos->msg.dataEx[8] = servos->msg.data[1];
	servos->msg.dataEx[9] = servos->msg.data[2];
	servos->msg.dataEx[10]= servos->msg.data[3];

	HKL_sendData(servos);
}

void  HKL_ACK(Herkulex_Struct * servos, uint8_t valueACK){

	servos->msg.pSize = 0x0A;
	servos->msg.pID   = 0xFE;
	servos->msg.cmd   = HRAMWRITE;
	servos->msg.data[0] = 0x34;               // RAM Address
	servos->msg.data[1] = 0x01;               // Length
	servos->msg.data[2] = valueACK;           // Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
	servos->msg.lenghtString = 3;             // lenghtData

	servos->msg.ck1 = HKL_checksum1(&(servos->msg));
	servos->msg.ck2 = HKL_checksum2(servos->msg.ck1);

	servos->msg.dataEx[0] = 0xFF;
	servos->msg.dataEx[1] = 0xFF;
	servos->msg.dataEx[2] = servos->msg.pSize;
	servos->msg.dataEx[3] = servos->msg.pID;
	servos->msg.dataEx[4] = HRAMWRITE;
	servos->msg.dataEx[5] = servos->msg.ck1;
	servos->msg.dataEx[6] = servos->msg.ck2;
	servos->msg.dataEx[7] = servos->msg.data[0];
	servos->msg.dataEx[8] = servos->msg.data[1];
	servos->msg.dataEx[9] = servos->msg.data[2];

	HKL_sendData(servos);

}

uint16_t  HKL_stat(Herkulex_Struct * servos, uint8_t servoID){

	uint8_t checksum = 0;
	uint8_t buffer[9] = {0};		// Herkulex will send 9 bytes

	// Send request to Herkulex
	servos->msg.pSize    = 0x07;
	servos->msg.pID      = servoID;
	servos->msg.cmd      = HSTAT;

	servos->msg.ck1 		= (servos->msg.pSize^servos->msg.pID^servos->msg.cmd)&0xFE;
	servos->msg.ck2		= (~(servos->msg.pSize^servos->msg.pID^servos->msg.cmd))&0xFE ;

	servos->msg.dataEx[0] = 0xFF;
	servos->msg.dataEx[1] = 0xFF;
	servos->msg.dataEx[2] = servos->msg.pSize;
	servos->msg.dataEx[3] = servos->msg.pID;
	servos->msg.dataEx[4] = servos->msg.cmd;
	servos->msg.dataEx[5] = servos->msg.ck1;
	servos->msg.dataEx[6] = servos->msg.ck2;


	HKL_readData(servos, buffer, 9);
	HAL_UART_Receive_DMA(servos->huart, buffer, 9);

	HKL_sendData(servos);

	// Wait for Herkulex to response

	if (HKL_waitToReceiveData()){		// read 9 bytes from serial
		checksum = (buffer[2]^buffer[3]^buffer[4]^buffer[7]^buffer[8]) & 0xFE;

		if (checksum != buffer[5]){
			return -1; 				//checksum1 not true
		}
		if (HKL_checksum2(checksum) != buffer[6]){
			return -2;				//checksum2 not true
		}
		return (buffer[7] << 8) + buffer[8];			// return Status Error (first byte) and Status Detail (second byte)
	} else {
		return -3;					//Message not received (timeout)
	};
}

void  HKL_torqueON(Herkulex_Struct * servos, uint8_t servoID){

	servos->msg.pSize = 0x0A;
	servos->msg.lenghtString = 0x03;
	servos->msg.pID   = servoID;
	servos->msg.cmd   = HRAMWRITE;
	servos->msg.data[0]=0x34;               // Address
	servos->msg.data[1]=0x01;               // Data Length
	servos->msg.data[2]=0x60;               // 0x60=Torque ON

	servos->msg.ck1 = HKL_checksum1(&(servos->msg));
	servos->msg.ck2 = HKL_checksum2(servos->msg.ck1);

	servos->msg.dataEx[0] = 0xFF;
	servos->msg.dataEx[1] = 0xFF;
	servos->msg.dataEx[2] = servos->msg.pSize;
	servos->msg.dataEx[3] = servoID;
	servos->msg.dataEx[4] = HRAMWRITE;
	servos->msg.dataEx[5] = servos->msg.ck1;
	servos->msg.dataEx[6] = servos->msg.ck2;
	servos->msg.dataEx[7] = servos->msg.data[0];
	servos->msg.dataEx[8] = servos->msg.data[1];
	servos->msg.dataEx[9] = servos->msg.data[2];

	HKL_sendData(servos);
}

void  HKL_moveOne(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint16_t pTime, uint8_t iLed){

	if (Goal > 1023 || Goal < 0) return;            // goal not correct
	if ((pTime <0) || (pTime > 2856)) return;		// execution time not correct

	// Position definition
	uint8_t posLSB = Goal & 0X00FF;								// MSB Position
	uint8_t posMSB = (Goal & 0XFF00) >> 8;						// LSB Position

	// Led
	uint8_t iBlue=0;
	uint8_t iGreen=0;
	uint8_t iRed=0;
	switch (iLed) {
	case 1:
		iGreen=1;
		break;
	case 2:
		iBlue=1;
		break;
	case 3:
		iRed=1;
		break;
	}
	uint8_t LedValue = iGreen*4 + iBlue*8 + iRed*16;	// Assign led value

	servos->msg.playTime = (uint8_t)((float)pTime/11.2);			// Execution time

	servos->msg.pSize = 0x0C;
	servos->msg.cmd   = HSJOG;

	servos->msg.data[0] = posLSB;
	servos->msg.data[1] = posMSB;
	servos->msg.data[2] = LedValue;
	servos->msg.data[3] = servoID;

	servos->msg.pID = servoID ^ servos->msg.playTime;

	servos->msg.lenghtString = 4;

	servos->msg.ck1 = HKL_checksum1(&(servos->msg));
	servos->msg.ck2 = HKL_checksum2(servos->msg.ck1);

	servos->msg.pID = servoID;

	servos->msg.dataEx[0] = 0xFF;
	servos->msg.dataEx[1] = 0xFF;
	servos->msg.dataEx[2] = servos->msg.pSize;
	servos->msg.dataEx[3] = servos->msg.pID;
	servos->msg.dataEx[4] = servos->msg.cmd;
	servos->msg.dataEx[5] = servos->msg.ck1;
	servos->msg.dataEx[6] = servos->msg.ck2;
	servos->msg.dataEx[7] = servos->msg.playTime;
	servos->msg.dataEx[8] = servos->msg.data[0];
	servos->msg.dataEx[9] = servos->msg.data[1];
	servos->msg.dataEx[10] = servos->msg.data[2];
	servos->msg.dataEx[11] = servos->msg.data[3];

	HKL_sendData(servos);
}

void  HKL_rotate(Herkulex_Struct * servos, uint8_t servoID, uint16_t speed, uint16_t pTime, uint8_t iLed){
	uint8_t goalSpeedSign;
	uint8_t speedGoalLSB; 		       // MSB speedGoal
	uint8_t speedGoalMSB;
	uint8_t iBlue=0;
	uint8_t iGreen=0;
	uint8_t iRed=0;

	if (speed > 1023 || speed < 0) return;            // goal not correct
	if ((pTime <0) || (pTime > 2856)) return;		// execution time not correct



	if (speed < 0) {
	    goalSpeedSign = (-1)* speed ;
	    goalSpeedSign |= 0x4000;  //bit n14
	} else {
		goalSpeedSign = speed;
	}
	speedGoalLSB=goalSpeedSign & 0X00FF;
	speedGoalMSB=(goalSpeedSign & 0xFF00) >> 8;


	// Led
	switch (iLed) {
	case 1:
		iGreen=1;
		break;
	case 2:
		iBlue=1;
		break;
	case 3:
		iRed=1;
		break;
	}
	uint8_t LedValue = iGreen*4 + iBlue*8 + iRed*16;		// Assign led value

	servos->msg.playTime = (uint8_t)((float)pTime/11.2);			// Execution time

	servos->msg.pSize = 0x0C;
	servos->msg.cmd   = HSJOG;

	servos->msg.data[0] = speedGoalLSB;
	servos->msg.data[1] = speedGoalMSB;
	servos->msg.data[2] = LedValue;
	servos->msg.data[3] = servoID;

	servos->msg.pID = servoID ^ servos->msg.playTime;

	servos->msg.lenghtString = 4;

	servos->msg.ck1 = HKL_checksum1(&(servos->msg));
	servos->msg.ck2 = HKL_checksum2(servos->msg.ck1);

	servos->msg.pID = servoID;

	servos->msg.dataEx[0] = 0xFF;
	servos->msg.dataEx[1] = 0xFF;
	servos->msg.dataEx[2] = servos->msg.pSize;
	servos->msg.dataEx[3] = servos->msg.pID;
	servos->msg.dataEx[4] = servos->msg.cmd;
	servos->msg.dataEx[5] = servos->msg.ck1;
	servos->msg.dataEx[6] = servos->msg.ck2;
	servos->msg.dataEx[7] = servos->msg.playTime;
	servos->msg.dataEx[8] = servos->msg.data[0];
	servos->msg.dataEx[9] = servos->msg.data[1];
	servos->msg.dataEx[10] = servos->msg.data[2];
	servos->msg.dataEx[11] = servos->msg.data[3];

	HKL_sendData(servos);
}

void  HKL_reboot(Herkulex_Struct * servos, uint8_t servoID){
	servos->msg.pSize = 0x07;
	servos->msg.pID   = servoID;
	servos->msg.cmd   = HREBOOT;
	servos->msg.ck1 = (servos->msg.pSize^servos->msg.pID^servos->msg.cmd)&0xFE;
	servos->msg.ck2 = (~(servos->msg.pSize^servos->msg.pID^servos->msg.cmd))&0xFE ; ;

	servos->msg.dataEx[0] = 0xFF;
	servos->msg.dataEx[1] = 0xFF;
	servos->msg.dataEx[2] = servos->msg.pSize;
	servos->msg.dataEx[3] = servos->msg.pID;
	servos->msg.dataEx[4] = servos->msg.cmd;
	servos->msg.dataEx[5] = servos->msg.ck1;
	servos->msg.dataEx[6] = servos->msg.ck2;

	HKL_sendData(servos);
}

