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

/* TIMEOUT limit for serial communication */
#define HERKULEX_SEND_TIMEOUT 500	/* Timeout duration when send data through serial port*/
#define HERKULEX_RECEIVE_TIMEOUT 500	/* Timeout duration when receive data through serial port*/



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
 * @fn void Herkulex_sendData(Herkulex_Struct*, uint8_t*, uint8_t)
 * @brief Send data to Herkulex
 *
 * @param servos Herkulex_Struct handler
 * @param package Pointer to data to send
 * @param size	Number of bytes to send
 */
void Herkulex_sendData(Herkulex_Struct * servos, uint8_t size){
	HAL_UART_Transmit(servos->huart, servos->package, size, HERKULEX_SEND_TIMEOUT);
}

/**
 * @fn void Herkulex_readData(Herkulex_Struct*, uint8_t*, uint8_t)
 * @brief Configure DMA to store data received from Herkulex in a buffer
 *
 * @param servos Herkulex_Struct handler
 * @param pBuffer Pointer to buffer to store data
 * @param size	Number of bytes to store
 */
void Herkulex_startListenData(Herkulex_Struct * servos, uint8_t *pBuffer, uint8_t size){
	messageReceived = false;
	//HAL_UART_Receive_DMA(servos->huart, pBuffer, size);
	HAL_UART_Receive_IT(servos->huart, pBuffer, size);
}

/**
 * @fn bool Herkulex_waitToReceiveData()
 * @brief Wait until received data from Herkulex
 *
 * @return	Whether a data is received
 */
bool Herkulex_waitToReceiveData(){
	uint32_t tickstart;

	tickstart = HAL_GetTick();
	while ((messageReceived == false) && (HAL_GetTick() < (HERKULEX_RECEIVE_TIMEOUT + tickstart))){
		//wait until receive a new message or timeout
	}
	return messageReceived;
}

/**
 * @fn uint8_t Herkulex_checksum1(uint8_t*)
 * @brief Checksum 1 to verify data of Herkulex
 *
 * @param msg uint8_t* pointer to message
 * @return Result of Checksum1
 */
uint8_t Herkulex_checksum1(uint8_t * msg, uint8_t lenghtData){
	uint8_t XOR = msg[2]^msg[3]^msg[4];
	for (int i = 0; i < lenghtData; i++)
	{
		XOR = XOR ^ msg[i+7];
	}
	return (XOR&0xFE);
}

/**
 * @fn uint8_t Herkulex_checksum2(uint8_t)
 * @brief Checksum 2 to verify data of Herkulex
 *
 * @param XOR Checksum1
 * @return Result of Checksum2
 */
static inline uint8_t Herkulex_checksum2(uint8_t XOR){
	return (~XOR)&0xFE;
}

/**
 * @fn void Herkulex_addData(Herkulex_Struct*, uint8_t, uint8_t, uint8_t, uint8_t)
 * @brief To Do
 *
 * @param servos
 * @param GoalLSB
 * @param GoalMSB
 * @param set
 * @param servoID
 */
void Herkulex_addData(Herkulex_Struct * servos, uint8_t GoalLSB, uint8_t GoalMSB, uint8_t set, uint8_t servoID){}


/****************************************************************************************************************
 *
 *
 * 	API functions
 *
 *
 ****************************************************************************************************************/

/*	****************************************************************************************************************
	Functions to Set up the servos
*************************************************************************************************************** */

void Herkulex_initCommunication(Herkulex_Struct * servos, UART_HandleTypeDef * huart){
	servos->huart = huart;
	__HAL_UART_FLUSH_DRREGISTER(huart);
	//servos->conta = 0;
	HAL_Delay(100);
}

void  Herkulex_initServos(Herkulex_Struct * servos){
	Herkulex_reboot(servos, HERKULEX_BROADCAST_ID);
	HAL_Delay(100);
	Herkulex_clearError(servos, HERKULEX_BROADCAST_ID);
	HAL_Delay(100);

	Herkulex_setACK(servos, HERKULEX_ACK_REPLY_READ);
	HAL_Delay(100);

	Herkulex_torqueON(servos, HERKULEX_BROADCAST_ID);
	HAL_Delay(100);
}

void  Herkulex_reboot(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	package_sizeToSend = HMB_reboot(servos->package, servoID);
	Herkulex_sendData(servos, package_sizeToSend);
}

void  Herkulex_setACK(Herkulex_Struct * servos, uint8_t valueACK){
	uint8_t package_sizeToSend;
	package_sizeToSend = HMB_ramWrite(servos->package, HERKULEX_BROADCAST_ID, RAM_ACK_POLICY, &valueACK, 1);
	Herkulex_sendData(servos, package_sizeToSend);
}

void  Herkulex_clearError(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	uint8_t data[2] = {0x00, 0x00};
	package_sizeToSend = HMB_ramWrite(servos->package, servoID, RAM_STATUS_ERROR, data, 2);
	Herkulex_sendData(servos, package_sizeToSend);
}

/*	****************************************************************************************************************
	Functions to Move the servos
*************************************************************************************************************** */

void  Herkulex_torqueON(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	uint8_t data = 0x60;			// Torque ON

	package_sizeToSend = HMB_ramWrite(servos->package, servoID, RAM_TORQUE_CONTROL, &data, 1);
	Herkulex_sendData(servos, package_sizeToSend);
}

void  Herkulex_torqueOFF(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	uint8_t data = 0x00;			// Torque OFF
	package_sizeToSend = HMB_ramWrite(servos->package, servoID, RAM_TORQUE_CONTROL, &data, 1);
	Herkulex_sendData(servos, package_sizeToSend);
}

void Herkulex_changeMode(Herkulex_Struct * servos, uint8_t servoID, uint8_t mode){
	Herkulex_torqueOFF(servos, servoID);
	if (mode == 0){
  		Herkulex_moveOne(servos, servoID, 200, HERKULEX_LED_GREEN);
	} else {
		Herkulex_rotateOne(servos, servoID, 200, HERKULEX_LED_GREEN);
	}
  	Herkulex_torqueON(servos, servoID);
}

void  Herkulex_torqueBREAK(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	uint8_t data = 0x40;			// Torque BREAK

	package_sizeToSend = HMB_ramWrite(servos->package, servoID, RAM_TORQUE_CONTROL, &data, 1);
	Herkulex_sendData(servos, package_sizeToSend);
}

void Herkulex_moveOne(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint8_t iLed){
	uint8_t package_sizeToSend;
	uint8_t posLSB;
	uint8_t posMSB;
	uint16_t pTime = 60;
	uint8_t iBlue=0;
	uint8_t iGreen=0;
	uint8_t iRed=0;

	if (Goal > 1023 || Goal < 0) return;            // goal not correct
	if ((pTime <0) || (pTime > 2856)) return;		// execution time not correct

	// Position definition
	posLSB = Goal & 0X00FF;								// MSB Position
	posMSB = (Goal & 0XFF00) >> 8;						// LSB Position

	// Led assignment
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
	package_sizeToSend = HMB_sJog(servos->package, servoID, pTime, posLSB, posMSB, iBlue*8 + iGreen*4 + iRed*16);	
	Herkulex_sendData(servos, package_sizeToSend);
}

void  Herkulex_rotateOne(Herkulex_Struct * servos, uint8_t servoID, int16_t speed, uint8_t iLed){
	uint8_t package_sizeToSend;
	uint16_t goalSpeedSign;
	uint8_t speedGoalLSB;
	uint8_t speedGoalMSB;
	uint16_t pTime = 60;
	uint8_t iBlue=0;
	uint8_t iGreen=0;
	uint8_t iRed=0;

	if (speed > 1023 || speed < -1023) return;            // goal not correct
	if ((pTime < -1) || (pTime > 2856)) return;		// execution time not correct

	if (speed < 0) {
			    goalSpeedSign = (-1) * speed ;
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
	package_sizeToSend = HMB_sJog(servos->package, servoID, pTime, speedGoalLSB, speedGoalMSB, 2+iBlue*8 + iGreen*4 + iRed*16);
	Herkulex_sendData(servos, package_sizeToSend);
}

/*	****************************************************************************************************************
	Functions to Get Info from the servos
*************************************************************************************************************** */
float Herkulex_getAngle(Herkulex_Struct * servos, uint8_t servoID){
	uint16_t pos = Herkulex_getPosition(servos, servoID);
	if (pos == -1) return -201;
	if (pos == -2) return -202;
	if (pos == -3) return -203;
	return (pos-520)*0.325;
}

uint16_t Herkulex_getPosition(Herkulex_Struct * servos, uint8_t servoID){
	/*
	uint8_t package_sizeToSend;
	uint8_t checksum = 0;
	uint8_t buffer[13] = {0};		// Herkulex will send 13 bytes, including 6 bytes of data

	// Build request to Herkulex
	package_sizeToSend = HMB_ramRead(servos->package, servoID, RAM_ABSOLUTE_POSITION, 2);

	// Start listen to incoming data
	Herkulex_startListenData(servos, buffer, 13);

	// Send request to Herkulex
	Herkulex_sendData(servos, package_sizeToSend);

	// Wait for Herkulex to response
	if (Herkulex_waitToReceiveData()){		// read 13 bytes from serial
		messageReceived = false;
		__HAL_UART_FLUSH_DRREGISTER(servos->huart);
		checksum = Herkulex_checksum1(buffer, 6);

		if (checksum != buffer[5]){
			return -1; 				//checksum1 not true
		}
		if (((~checksum)&0xFE) != buffer[6]){
			return -2;				//checksum2 not true
		}
		return (((buffer[10]&0x03)<<8) | buffer[9]);
	} else {
		return -3;					//Message not received (timeout)
	};
	*/
	return (Herkulex_readRAM(servos, servoID, RAM_ABSOLUTE_POSITION, 2)&0x3FF);
}

uint8_t  Herkulex_getStatusError(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	uint8_t checksum = 0;
	uint8_t buffer[9] = {0};		// Herkulex will send 9 bytes

	// Build request to Herkulex
	package_sizeToSend = HMB_stat(servos->package, servoID);

	// Start listen to incoming data
	Herkulex_startListenData(servos, buffer, 9);

	// Send request to Herkulex
	Herkulex_sendData(servos, package_sizeToSend);

	// Wait for Herkulex to response

	if (Herkulex_waitToReceiveData()){		// read 9 bytes from serial
		messageReceived = false;
		__HAL_UART_FLUSH_DRREGISTER(servos->huart);
		checksum = (buffer[2]^buffer[3]^buffer[4]^buffer[7]^buffer[8]) & 0xFE;

		if (checksum != buffer[5]){
			return -1; 				//checksum1 not true
		}
		if (((~checksum)&0xFE) != buffer[6]){
			return -2;				//checksum2 not true
		}
		return buffer[7];			// return Status Error
	} else {
		return -3;					//Message not received (timeout)
	};
}

uint8_t Herkulex_getCurrentMode(Herkulex_Struct * servos, uint8_t servoID){
	return Herkulex_readRAM(servos, servoID, RAM_CURRENT_CONTROL_MODE, 1);
}

uint16_t Herkulex_readRAM(Herkulex_Struct * servos, uint8_t servoID, Herkulex_RAM_Address_t addr, uint8_t lengthToRead){
	uint8_t size_receivedPackage = 11+lengthToRead;
	uint8_t size_receivedData = 4+lengthToRead;
	uint8_t package_sizeToSend;
	uint8_t checksum = 0;
	uint8_t buffer[13] = {0};

	// Build request to Herkulex
	package_sizeToSend = HMB_ramRead(servos->package, servoID , addr, lengthToRead);

	// Start listen to incoming data
	Herkulex_startListenData(servos, buffer, size_receivedPackage);

	// Send request to Herkulex
	Herkulex_sendData(servos, package_sizeToSend);

	// Wait for Herkulex to response
	if (Herkulex_waitToReceiveData()){		
		messageReceived = false;
		__HAL_UART_FLUSH_DRREGISTER(servos->huart);
		checksum = Herkulex_checksum1(buffer, size_receivedData);

		if (checksum != buffer[5]){
			return -1; 				//checksum1 not true
		}
		if (((~checksum)&0xFE) != buffer[6]){
			return -2;				//checksum2 not true
		}
		if (lengthToRead == 1){
			return (buffer[9]);
		}
		else{
			return ((buffer[10]<<8) | buffer[9]);	// only handle 2 bytes, add handle for more bytes later
		}
			
	} else {
		return -3;					//Message not received (timeout)
	};
}
