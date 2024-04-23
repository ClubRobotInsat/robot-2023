/**
 * @file herkulex.c
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
#define HERKULEX_RECEIVE_TIMEOUT 1000	/* Timeout duration when receive data through serial port*/



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
 * @return	Whether the data is sent successfully
 */
Herkulex_StatusTypedef Herkulex_sendData(Herkulex_Struct * servos, uint8_t size){
	return (HAL_UART_Transmit(servos->huart, servos->package, size, HERKULEX_SEND_TIMEOUT) && 0x1);
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

Herkulex_StatusTypedef  Herkulex_initServos(Herkulex_Struct * servos){
	uint8_t status;
	status = Herkulex_reboot(servos, HERKULEX_BROADCAST_ID);
	HAL_Delay(100);
	status = Herkulex_clearError(servos, HERKULEX_BROADCAST_ID);
	HAL_Delay(100);

	status = Herkulex_setACK(servos, HERKULEX_ACK_REPLY_READ);
	HAL_Delay(100);

	status = Herkulex_torqueON(servos, HERKULEX_BROADCAST_ID);
	HAL_Delay(100);
	return status;
}

Herkulex_StatusTypedef  Herkulex_reboot(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	package_sizeToSend = HMB_reboot(servos->package, servoID);
	return Herkulex_sendData(servos, package_sizeToSend);
}

Herkulex_StatusTypedef  Herkulex_setACK(Herkulex_Struct * servos, uint8_t valueACK){
	if (valueACK > 0x02) return Herkulex_ErrorWrongParameter;		// ACK value not correct, see HERKULEX_ACK_MODE
	uint8_t package_sizeToSend;
	package_sizeToSend = HMB_ramWrite(servos->package, HERKULEX_BROADCAST_ID, RAM_ACK_POLICY, &valueACK, 1);
	return Herkulex_sendData(servos, package_sizeToSend);
}

Herkulex_StatusTypedef  Herkulex_clearError(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	uint8_t data[2] = {0x00, 0x00};
	package_sizeToSend = HMB_ramWrite(servos->package, servoID, RAM_STATUS_ERROR, data, 2);
	return Herkulex_sendData(servos, package_sizeToSend);
}

/*	****************************************************************************************************************
	Functions to Move the servos
*************************************************************************************************************** */

Herkulex_StatusTypedef  Herkulex_torqueON(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	uint8_t data = 0x60;			// Torque ON

	package_sizeToSend = HMB_ramWrite(servos->package, servoID, RAM_TORQUE_CONTROL, &data, 1);
	return Herkulex_sendData(servos, package_sizeToSend);
}

Herkulex_StatusTypedef  Herkulex_torqueOFF(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	uint8_t data = 0x00;			// Torque OFF
	package_sizeToSend = HMB_ramWrite(servos->package, servoID, RAM_TORQUE_CONTROL, &data, 1);
	return Herkulex_sendData(servos, package_sizeToSend);
}

Herkulex_StatusTypedef Herkulex_changeMode(Herkulex_Struct * servos, uint8_t servoID, uint8_t mode){
	if (mode > 1) return Herkulex_ErrorWrongParameter;		// Mode not correct, see HERKULEX_MODE
	uint8_t status;
	status = Herkulex_torqueOFF(servos, servoID);
	if (mode == 0){
  		status = Herkulex_moveOne(servos, servoID, 200, HERKULEX_LED_GREEN);
	} else if (mode == 1) {
		status = Herkulex_rotateOne(servos, servoID, 200, HERKULEX_LED_GREEN);
	}
  	status = Herkulex_torqueON(servos, servoID);
	return status;
}

Herkulex_StatusTypedef  Herkulex_torqueBREAK(Herkulex_Struct * servos, uint8_t servoID){
	uint8_t package_sizeToSend;
	uint8_t data = 0x40;			// Torque BREAK

	package_sizeToSend = HMB_ramWrite(servos->package, servoID, RAM_TORQUE_CONTROL, &data, 1);
	return Herkulex_sendData(servos, package_sizeToSend);
}

Herkulex_StatusTypedef Herkulex_moveOne(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint8_t iLed){
	uint8_t package_sizeToSend;
	uint8_t posLSB;
	uint8_t posMSB;
	uint16_t pTime = 60;

	if (Goal > 1023 || Goal < 0) return Herkulex_ErrorWrongParameter;            // goal not correct

	// Position definition
	posLSB = Goal & 0X00FF;								// MSB Position
	posMSB = (Goal & 0XFF00) >> 8;						// LSB Position

	package_sizeToSend = HMB_sJog(servos->package, servoID, pTime, posLSB, posMSB, iLed << 2);	
	return Herkulex_sendData(servos, package_sizeToSend);
}

Herkulex_StatusTypedef Herkulex_rotateOne(Herkulex_Struct * servos, uint8_t servoID, int16_t speed, uint8_t iLed){
	uint8_t package_sizeToSend;
	uint16_t goalSpeedSign;
	uint8_t speedGoalLSB;
	uint8_t speedGoalMSB;
	uint16_t pTime = 60;


	if (speed > 1023 || speed < -1023) return Herkulex_ErrorWrongParameter;            // goal not correct

	if (speed < 0) {
			    goalSpeedSign = (-1) * speed ;
			    goalSpeedSign |= 0x4000;  				// set the sign bit to 1
	} else {
			goalSpeedSign = speed;
	}

	speedGoalLSB=goalSpeedSign & 0X00FF;
	speedGoalMSB=(goalSpeedSign & 0xFF00) >> 8;

	// setValue include the Led color and the Mode, see p.48 of the manual
	package_sizeToSend = HMB_sJog(servos->package, servoID, pTime, speedGoalLSB, speedGoalMSB, 2+(iLed << 2));
	return Herkulex_sendData(servos, package_sizeToSend);
}

/*	****************************************************************************************************************
	Functions to Get Info from the servos
*************************************************************************************************************** */
Herkulex_StatusTypedef Herkulex_getAngle(Herkulex_Struct * servos, uint8_t servoID, float * ptrResult){
	uint16_t pos;
	uint8_t status;
	status = Herkulex_getPosition(servos, servoID, &pos);
	if (status != Herkulex_OK) return Herkulex_ErrorReceiveFailed;
	(*ptrResult) = (pos-520)*0.325;
	return status;
}

Herkulex_StatusTypedef Herkulex_getPosition(Herkulex_Struct * servos, uint8_t servoID, uint16_t * ptrResult){
	uint8_t status = Herkulex_readRAM(servos, servoID, RAM_ABSOLUTE_POSITION, 2, ptrResult);
	(*ptrResult) = (*ptrResult) & 0x3FF;
	return status;
}

Herkulex_StatusTypedef  Herkulex_getStatusError(Herkulex_Struct * servos, uint8_t servoID, uint8_t * ptrResult){
	uint8_t package_sizeToSend;
	uint8_t checksum = 0;
	uint8_t buffer[9] = {0};		// Herkulex will send 9 bytes

	// Build request to Herkulex
	package_sizeToSend = HMB_stat(servos->package, servoID);

	// Start listen to incoming data
	Herkulex_startListenData(servos, buffer, 9);

	// Send request to Herkulex
	if (Herkulex_sendData(servos, package_sizeToSend) == Herkulex_ErrorSendFailed){
		return Herkulex_ErrorSendFailed;
	}
	// Wait for Herkulex to response
	if (Herkulex_waitToReceiveData()){		// read 9 bytes from serial
		messageReceived = false;
		__HAL_UART_FLUSH_DRREGISTER(servos->huart);
		checksum = (buffer[2]^buffer[3]^buffer[4]^buffer[7]^buffer[8]) & 0xFE;

		if ((checksum != buffer[5]) && (((~checksum)&0xFE) != buffer[6])){
			return Herkulex_ErrorReceiveFailed; 				//checksum not true
		}
		(*ptrResult) = buffer[7];			// return Status Error
		return Herkulex_OK;			// return Status Error
	} else {
		return Herkulex_ErrorReceiveFailed;					//Message not received (timeout)
	};
}

Herkulex_StatusTypedef  Herkulex_getStatusDetail(Herkulex_Struct * servos, uint8_t servoID, uint8_t * ptrResult){
	uint8_t package_sizeToSend;
	uint8_t checksum = 0;
	uint8_t buffer[9] = {0};		// Herkulex will send 9 bytes

	// Build request to Herkulex
	package_sizeToSend = HMB_stat(servos->package, servoID);

	// Start listen to incoming data
	Herkulex_startListenData(servos, buffer, 9);

	// Send request to Herkulex
	if (Herkulex_sendData(servos, package_sizeToSend) == Herkulex_ErrorSendFailed){
		return Herkulex_ErrorSendFailed;
	}
	// Wait for Herkulex to response
	if (Herkulex_waitToReceiveData()){		// read 9 bytes from serial
		messageReceived = false;
		__HAL_UART_FLUSH_DRREGISTER(servos->huart);
		checksum = (buffer[2]^buffer[3]^buffer[4]^buffer[7]^buffer[8]) & 0xFE;

		if ((checksum != buffer[5]) && (((~checksum)&0xFE) != buffer[6])){
			return Herkulex_ErrorReceiveFailed; 				//checksum not true
		}
		(*ptrResult) = buffer[8];			// return Status Detail
		return Herkulex_OK;			
	} else {
		return Herkulex_ErrorReceiveFailed;					//Message not received (timeout)
	};
}

Herkulex_StatusTypedef Herkulex_getCurrentMode(Herkulex_Struct * servos, uint8_t servoID, uint8_t * ptrResult){
	return Herkulex_readRAM(servos, servoID, RAM_CURRENT_CONTROL_MODE, 1, (uint16_t *)ptrResult);
}

/*	****************************************************************************************************************
	Functions to Configure the servo
*************************************************************************************************************** */


Herkulex_StatusTypedef Herkulex_setLed(Herkulex_Struct * servos, uint8_t servoID, uint8_t valueLed){
	return Herkulex_writeRAM(servos, servoID, RAM_LED_CONTROL, &valueLed, 1);
}


/*	****************************************************************************************************************
	Functions to Read the servo RAM and EEPROM
*************************************************************************************************************** */

Herkulex_StatusTypedef Herkulex_readRAM(Herkulex_Struct * servos, uint8_t servoID, Herkulex_RAM_Address_t addr, uint8_t lengthToRead, uint16_t * ptrResult){
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
	if (Herkulex_sendData(servos, package_sizeToSend) == Herkulex_ErrorSendFailed){
		return Herkulex_ErrorSendFailed;
	}

	// Wait for Herkulex to response
	if (Herkulex_waitToReceiveData()){		
		messageReceived = false;				// reset flag
		__HAL_UART_FLUSH_DRREGISTER(servos->huart);
		checksum = Herkulex_checksum1(buffer, size_receivedData);

		if ((checksum != buffer[5]) && (((~checksum)&0xFE) != buffer[6])){
			return Herkulex_ErrorReceiveFailed; 				//checksum not true
		}
		if (lengthToRead == 1){
			(*ptrResult) = (buffer[9]);
		}
		else{
			(*ptrResult) = ((buffer[10]<<8) | buffer[9]);	// only handle 2 bytes, add handle for more bytes later
		}
		return Herkulex_OK;
			
	} else {
		return Herkulex_ErrorReceiveFailed;					//Message not received (timeout)
	};
}

/*	****************************************************************************************************************
	Functions to Write to the servo RAM and EEPROM
*************************************************************************************************************** */
Herkulex_StatusTypedef Herkulex_writeRAM(Herkulex_Struct * servos, uint8_t servoID, Herkulex_RAM_Address_t RAMaddress, uint8_t * data, uint8_t lengthToWrite){	
	uint8_t package_sizeToSend;
	package_sizeToSend = HMB_ramWrite(servos->package, HERKULEX_BROADCAST_ID, RAMaddress, data, lengthToWrite);
	return Herkulex_sendData(servos, package_sizeToSend);
}