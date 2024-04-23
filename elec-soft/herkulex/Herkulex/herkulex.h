/**
 * @file herkulex.h
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Header of library to control a Herkulex servo motor
 * @version 0.2 : Add function to write to RAM 
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

/**************************************************
 * Librairy for Herkulex DRS-0101 Servo motor.
 * This librairy is mostly based on the Arduino librairy
 * of the Herkulex made by Alessandro Giacomel on 09/12/2012.
 * Check here for his website: http://robottini.altervista.org
 *
 * The manual for the servo is available here :
 * https://cdn.robotshop.com/media/d/das/rb-das-05/pdf/_eng_herkulex_manual_20140218.pdf
 **************************************************/

/**************************************************
 * What a Herkulex can do?
 *
 * A intelligent servo-motor that can be controlled through UART communication.
 * Each UART communication line can have up to 253 servos connected, each have a dedicated ID.
 * 	- User can control a motor in continuous (function HERKULEX_Rotate)
 *    or positional mode (HERKULEX_MoveToPosition or HERKULEX_MoveToAngle).
 *
 *	- User can also controlled all motors to do a same action by using the broadcast ID.
 *
 * 	- User can also make multiple motors to perform different action SIMULTANUOUSLY (function add... then ActionAll)
 *
 */

/**************************************************
 * USART configuration for Herkulex :
 * 		- Baud rate : 115000
 * 		- Data bit : 8
 * 		- Stop bit : 1
 * 		- Parity : None
 **************************************************/

#ifndef HERKULEX_H_
#define HERKULEX_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32g4xx_hal.h"
#include "herkulexMessageBuilder.h"
/* HERKULEX_LED - See Manual p29 */
#define HERKULEX_LED_GREEN 	 	0x01
#define HERKULEX_LED_BLUE     	0x02
#define HERKULEX_LED_CYAN		0x03
#define HERKULEX_LED_RED		0x04
#define HERKULEX_LED_GREEN2		0x05
#define HERKULEX_LED_PINK  		0x06
#define HERKULEX_LED_WHITE 		0x07



/* HERKULEX_STATUS_ERROR - See Manual p39 */
#define HERKULEX_STATUS_OK					 0x00
#define HERKULEX_ERROR_INPUT_VOLTAGE 		 0x01
#define HERKULEX_ERROR_POS_LIMIT			 0x02
#define HERKULEX_ERROR_TEMPERATURE_LIMIT	 0x04
#define HERKULEX_ERROR_INVALID_PKT			 0x08
#define HERKULEX_ERROR_OVERLOAD			 	0x10
#define HERKULEX_ERROR_DRIVER_FAULT    		0x20
#define HERKULEX_ERROR_EEPREG_DISTORT	 	0x40

/* HERKULEX_STATUS_DETAIL */
#define HERKULEX_DETAIL_MOVING				 0x01
#define HERKULEX_DETAIL_INPOSITION			 0x02
#define HERKULEX_DETAIL_CHECKSUM_ERROR		 0x04
#define HERKULEX_DETAIL_UNKNOWN_COMMAND		 0x08
#define HERKULEX_DETAIL_EXCEED_REG_RANGE	 0x10
#define HERKULEX_DETAIL_GARBAGE_CMD		 	 0x20
#define HERKULEX_DETAIL_MOTOR_ON			 0x40

/* HERKULEX_BROADCAST_ID */
#define HERKULEX_BROADCAST_ID	0xFE

/* HERKULEX_MODE */
#define HERKULEX_MODE_POSITION		0x00
#define HERKULEX_MODE_CONTINUOUS	0x01

/* HERKULEX_ACK_MODE */
#define HERKULEX_ACK_NO_REPLY		0x00
#define HERKULEX_ACK_REPLY_READ		0x01
#define HERKULEX_ACK_REPLY_ALWAYS	0x02

/* HERKULEX_NUMBER_MAX */
#define HERKULEX_NBR_SERVOS	 		10					// Number of servos to manipulate simultaneously 	<---- change this for more servos!
#define HERKULEX_DATA_ACTION_ALL  	 5*HERKULEX_NBR_SERVOS		// Size of package to manipulate all motors simultaneously
#define HERKULEX_PACKAGE_SIZE		 HERKULEX_DATA_ACTION_ALL+8	// Size of package to send to servos

/* ------------------------------------------------------------------------------------
							Exported types
---------------------------------------------------------------------------------------*/

/**
 * @struct Herkulex_Struct
 * @brief  Structure to manipulate up to NBR_SERVOS on a serial link
 *
 */
typedef struct {
	UART_HandleTypeDef * huart;		/* Handle for UART connection */
	uint8_t package[HERKULEX_PACKAGE_SIZE];	/* Package to send to servos */
	//uint8_t conta;		/* counter for simultaneous actions */
	//uint8_t moveData[DATA_ACTION_ALL];	/* data for simultaneous actions*/
} Herkulex_Struct;

typedef enum {
	Herkulex_OK = 0x00,
	Herkulex_ErrorSendFailed = 0x01,
	Herkulex_ErrorReceiveFailed = 0x02,
	Herkulex_ErrorWrongParameter = 0x03,
} Herkulex_StatusTypedef;

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

/**
 * @fn void Herkulex_initCommunication(Herkulex_Struct*, UART_HandleTypeDef*)
 * @brief Setup a UART link to Herkulex
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param huart UART handler
 */
void Herkulex_initCommunication(Herkulex_Struct * servos, UART_HandleTypeDef * huart);

/**
 * @fn void Herkulex_init(Herkulex_Struct*)
 * @brief Initialize all motors, make sure to reboot each servo with their proper ID before initialize.
 *
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_initServos(Herkulex_Struct * servos);

/**
 * @brief Reboot a servo. The broadcast ID doesn't work for this function.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to reboot
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_reboot(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn void Herkulex_setACK(Herkulex_Struct*, uint8_t)
 * @brief Set ACK policy for servo on RAM registry
 *
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param valueACK ACK policy to set, can be one on the list: HERKULEX_ACK_MODE
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_setACK(Herkulex_Struct * servos, uint8_t valueACK);

/**
 * @fn void Herkulex_clearError(Herkulex_Struct*, uint8_t)
 * @brief Clear error of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to clear error
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_clearError(Herkulex_Struct * servos, uint8_t servoID);

/*
*************TO DO*********************
*/

/**
 * @fn void Herkulex_factoryReset(Herkulex_Struct*, uint8_t)
 * @brief Factory reset a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to factory reset
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef Herkulex_factoryReset(Herkulex_Struct * servos, uint8_t servoID);
/*	****************************************************************************************************************
	Functions to Move on the servos
*************************************************************************************************************** */

/**
 * @fn void Herkulex_torqueON(Herkulex_Struct*, uint8_t)
 * @brief Turn on the torque of a servo. See manual p28 for more details.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to turn on the torque
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_torqueON(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn void Herkulex_torqueOFF(Herkulex_Struct*, uint8_t)
 * @brief Turn off the torque of a servo. See manual p28 for more details.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to turn off the torque
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_torqueOFF(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn void Herkulex_torqueBREAK(Herkulex_Struct*, uint8_t)
 * @brief Break the torque of a servo. See manual p28 for more details.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to break the torque
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_torqueBREAK(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn void Herkulex_changeMode(Herkulex_Struct*, uint8_t, uint8_t)
 * @brief Change the operating mode of a servo
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to change the mode
 * @param mode New operating mode to set, can be one of the following values:
 *             - HERKULEX_MODE_POSITION: Position control mode
 *             - HERKULEX_MODE_CONTINUOUS: Speed control mode
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef Herkulex_changeMode(Herkulex_Struct * servos, uint8_t servoID, uint8_t mode);


/**
 * @fn void Herkulex_moveSpeed(Herkulex_Struct*, uint8_t, uint16_t, uint16_t, uint8_t)
 * @brief Move a servo to a position with a speed
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to move
 * @param Goal Position to move, can be from 0 to 1023
 * @param iLed LED to turn on, can be one on the list: HERKULEX_LED
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_moveOne(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint8_t iLed);

/**
 * @fn void Herkulex_rotate(Herkulex_Struct*, uint8_t, int16_t, uint16_t, uint8_t)
 * @brief Rotate a servo with a speed
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to rotate
 * @param speed Speed to rotate, can be from -1023 to 1023
 * @param iLed LED to turn on, can be one on the list: HERKULEX_LED
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_rotateOne(Herkulex_Struct * servos, uint8_t servoID, int16_t speed, uint8_t iLed);

/*	****************************************************************************************************************
	Functions to Get Info from the servos
*************************************************************************************************************** */
/**
 * @fn uint16_t Herkulex_getAngle(Herkulex_Struct*, uint8_t, float*)
 * @brief Get angle of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get angle
 * @param ptrResult Pointer to the result
 * @return Herkulx_StatusTypedef Status of the function
 */ 
Herkulex_StatusTypedef Herkulex_getAngle(Herkulex_Struct * servos, uint8_t servoID, float * ptrResult);

/**
 * @fn uint16_t Herkulex_getPosition(Herkulex_Struct*, uint8_t, uint16_t*)
 * @brief Get position of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get position
 * @param ptrResult Pointer to the result
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef   Herkulex_getPosition(Herkulex_Struct * servos, uint8_t servoID, uint16_t * ptrResult);

/**
 * @brief Get status error of a servo(Herkulex_Struct*, uint8_t)
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get status error
 * @param ptrResult Pointer to the status error of the servo. Can be one on the list: HERKULEX_STATUS_ERROR
 * Check the manual at page 39 for more details.
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef  Herkulex_getStatusError(Herkulex_Struct * servos, uint8_t servoID, uint8_t * ptrResult);

/**
 * @fn uint8_t Herkulex_getStatusDetail(Herkulex_Struct*, uint8_t)
 * @brief Get status detail of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get status detail
 * @param ptrResult Pointer to the status detail of the servo. Can be one on the list: HERKULEX_STATUS_DETAIL
 * Check the manual at page 39 for more details.
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef Herkulex_getStatusDetail(Herkulex_Struct * servos, uint8_t servoID, uint8_t * ptrResult);

/**
 * @fn uint8_t Herkulex_getCurrentMode(Herkulex_Struct*, uint8_t)
 * @brief Get current mode of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get current mode
 * @param ptrResult Pointer to the current mode of the servo
 * 				  	can be one on the list:
 * 						0x00 : Position Control Mode
 * 						0x01 : Continuous Rotation Mode
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef Herkulex_getCurrentMode(Herkulex_Struct * servos, uint8_t servoID, uint8_t * ptrResult);

/**
 * @fn uint8_t Herkulex_getID(Herkulex_Struct*, uint8_t)
 * @brief Get ID of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get ID
 * @return uint8_t ID of the servo
 */
//uint8_t Herkulex_getID(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn uint8_t Herkulex_getID_EEP(Herkulex_Struct*, uint8_t)
 * @brief Get ID of a servo from EEPROM
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get ID
 * @return uint8_t ID of the servo
 */
//uint8_t Herkulex_getID_EEP(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn uint8_t Herkulex_getACKpolicy(Herkulex_Struct*, uint8_t)
 * @brief Get ACK policy of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get ACK policy
 * @return uint8_t ACK policy of the servo
 * 				   can be one on the list: HERKULEX_ACK_MODE
 */
//uint8_t Herkulex_getACKpolicy(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn uint8_t Herkulex_getTorquePolicy(Herkulex_Struct*, uint8_t)
 * @brief Get Torque policy of a servo, see manual p28 for more details
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get Torque policy
 * @return uint8_t Torque policy of the servo, can be one on the list:
 * 						0x00 : Torque OFF
 * 						0x40 : Torque BREAK
 * 						0x60 : Torque ON
 */
//uint8_t Herkulex_getTorquePolicy(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn uint16_t Herkulex_getMinPosition(Herkulex_Struct*, uint8_t)
 * @brief Get min position of a servo. 
 * 		  When requested position angle is less than the minimum position value, 
 * 		  "Exceed Allowed POT Limit" error is thrown.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get min position
 * @return uint16_t Min position of the servo. Default is 0x15 (-159.8 degree)
 */
//uint16_t Herkulex_getMinPosition(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn uint16_t Herkulex_getMaxPosition(Herkulex_Struct*, uint8_t)
 * @brief Get max position of a servo. 
 * 		  When requested position angle is more than the maximum position value, 
 * 		  "Exceed Allowed POT Limit" error is thrown.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get max position
 * @return uint16_t Max position of the servo. Default is 0x3EA (159.8 degree)
 */
//uint16_t Herkulex_getMaxPosition(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn uint8_t Herkulex_getLed(Herkulex_Struct*, uint8_t)
 * @brief Get LED of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get LED
 * @return uint8_t LED of the servo, can be one on the list: HERKULEX_LED
 */
//uint8_t Herkulex_getLed(Herkulex_Struct * servos, uint8_t servoID);


/*	****************************************************************************************************************
	Functions to Configure the servo
*************************************************************************************************************** */
/**
 * @fn uint8_t Herkulex_setID(Herkulex_Struct*, uint8_t, uint8_t)
 * @brief Set ID of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param ID_Old Old ID of the servo
 * @param ID_New New ID of the servo
 */
//void Herkulex_setID(Herkulex_Struct * servos, uint8_t ID_Old, uint8_t ID_New);

/**
 * @fn uint8_t Herkulex_setID_EEP(Herkulex_Struct*, uint8_t, uint8_t)
 * @brief Set ID of a servo in EEPROM
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param ID_Old Old ID of the servo
 * @param ID_New New ID of the servo
 */
//void Herkulex_setID_EEP(Herkulex_Struct * servos, uint8_t ID_Old, uint8_t ID_New);

/**
 * @fn void Herkulex_setMinPosition(Herkulex_Struct*, uint8_t, uint16_t)
 * @brief Set min position of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to set min position
 * @param minPosition Min position to set, can be from 0 to 1023
 */
//void Herkulex_setMinPosition(Herkulex_Struct * servos, uint8_t servoID, uint16_t minPosition);

/**
 * @fn void Herkulex_setMaxPosition(Herkulex_Struct*, uint8_t, uint16_t)
 * @brief Set max position of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to set max position
 * @param maxPosition Max position to set, can be from 0 to 1023
 */
//void Herkulex_setMaxPosition(Herkulex_Struct * servos, uint8_t servoID, uint16_t maxPosition);

/**
 * @fn void Herkulex_setLed(Herkulex_Struct*, uint8_t, uint8_t)
 * @brief Set LED of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to set LED
 * @param valueLed LED to set, can be one on the list: HERKULEX_LED
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef Herkulex_setLed(Herkulex_Struct * servos, uint8_t servoID, uint8_t valueLed);

/*	****************************************************************************************************************
	Functions to Read the servo RAM and EEPROM
*************************************************************************************************************** */
/**
 * @fn uint16_t Herkulex_readRAM(Herkulex_Struct*, uint8_t, uint8_t)
 * @brief Read a RAM registry of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to read RAM
 * @param RAMaddress Address of the RAM registry to read. Can be a value on the list: Herkulex_RAM_Address_t
 * @param lengthToRead Number of bytes to read
 * @param ptrResult Pointer to the value of the RAM registry readed
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef Herkulex_readRAM(Herkulex_Struct * servos, uint8_t servoID, Herkulex_RAM_Address_t addr, uint8_t lengthToRead, uint16_t * ptrResult);

//uint16_t Herkulex_readEEP(Herkulex_Struct * servos, uint8_t servoID, Herkulex_EEP_Address_t EEPaddress);

/*	****************************************************************************************************************
	Functions to Write to the servo RAM and EEPROM
*************************************************************************************************************** */
/**
 * @fn void Herkulex_writeRAM(Herkulex_Struct*, uint8_t, uint8_t, uint8_t)
 * @brief Write a RAM registry of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to write RAM
 * @param RAMaddress Address of the RAM registry to write. Can be a value on the list: Herkulex_RAM_Address_t
 * @param data Pointer to data to write in the RAM registry
 * @param lengthToWrite Number of bytes to write
 * @return Herkulex_StatusTypedef Status of the function
 */
Herkulex_StatusTypedef Herkulex_writeRAM(Herkulex_Struct * servos, uint8_t servoID, Herkulex_RAM_Address_t RAMaddress, uint8_t * data, uint8_t lengthToWrite);

/**
 * @fn void Herkulex_writeEEP(Herkulex_Struct*, uint8_t, uint8_t, uint8_t)
 * @brief Write a EEPROM registry of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to write EEPROM
 * @param EEPaddress Address of the EEPROM registry to write. Can be a value on the list: Herkulex_EEP_Address_t
 * @param writeByte Value to write in the EEPROM registry
 */
//void Herkulex_writeEEP(Herkulex_Struct * servos, uint8_t servoID, Herkulex_EEP_Address_t EEPaddress, uint8_t writeByte);

/*-----------------------------------------------------------------------------------------*/
// TODO
/*
uint8_t  Herkulex_model(Herkulex_Struct * servos);



void  Herkulex_moveAll(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint8_t iLed);
void  Herkulex_moveSpeedAll(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint8_t iLed);
void  Herkulex_moveAllAngle(Herkulex_Struct * servos, uint8_t servoID, float angle, uint8_t iLed);
void  Herkulex_actionAll(Herkulex_Struct * servos, uint8_t pTime);
*/
#ifdef __cplusplus
extern "C" }
#endif

#endif /* HERKULEX_H */
