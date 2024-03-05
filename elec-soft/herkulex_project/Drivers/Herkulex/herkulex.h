/**
 * @file herkulex.h
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Header of library to control a Herkulex servo motor
 * @version 0.1
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

/* HERKULEX_LED - See Manual p29 */
#define HERKULEX_LED_GREEN 	 0x01
#define HERKULEX_LED_BLUE     0x02
#define HERKULEX_LED_CYAN     0x03
#define HERKULEX_LED_RED    	 0x04
#define HERKULEX_LED_GREEN2 	 0x05
#define HERKULEX_LED_PINK     0x06
#define HERKULEX_LED_WHITE    0x07

/* HERKULEX_STATUS_ERROR - See Manual p39 */
#define HERKULEX_STATUS_OK					 0x00
#define HERKULEX_ERROR_INPUT_VOLTAGE 		 0x01
#define HERKULEX_ERROR_POS_LIMIT			 0x02
#define HERKULEX_ERROR_TEMPERATURE_LIMIT	 0x04
#define HERKULEX_ERROR_INVALID_PKT			 0x08
#define HERKULEX_ERROR_OVERLOAD			 0x10
#define HERKULEX_ERROR_DRIVER_FAULT    	0x20
#define HERKULEX_ERROR_EEPREG_DISTORT	 	0x40

/* HERKULEX_BROADCAST_ID */
#define HERKULEX_BROADCAST_ID	0xFE


/* HERKULEX_ACK_MODE */
#define HERKULEX_ACK_NO_REPLY		0x00
#define HERKULEX_ACK_REPLY_READ		0x01
#define HERKULEX_ACK_REPLY_ALWAYS	0x02

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


/****************************************************************************************************************
 *
 *
 * 	API functions
 *
 *
 ****************************************************************************************************************/

/*	****************************************************************************************************************
	Functions for Set up the servos
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
 * @brief Initialize all motors, make sure to reboot each servo with their proper ID before initialize
 *
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 */
void  Herkulex_initServos(Herkulex_Struct * servos);

/**
 * @brief Reboot a servo.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to reboot
 */
void  Herkulex_reboot(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn void Herkulex_setACK(Herkulex_Struct*, uint8_t)
 * @brief Set ACK policy for servo on RAM registry
 *
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param valueACK ACK policy to set, can be one on the list: HERKULEX_ACK_MODE
 */
void  Herkulex_setACK(Herkulex_Struct * servos, uint8_t valueACK);

/**
 * @brief Clear error of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to clear error
 */
void  Herkulex_clearError(Herkulex_Struct * servos, uint8_t servoID);

/*	****************************************************************************************************************
	Functions for Move on the servos
*************************************************************************************************************** */

/**
 * @fn void Herkulex_torqueON(Herkulex_Struct*, uint8_t)
 * @brief Turn on the torque of a servo. See manual p28 for more details.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to turn on the torque
 */
void  Herkulex_torqueON(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn void Herkulex_torqueOFF(Herkulex_Struct*, uint8_t)
 * @brief Turn off the torque of a servo. See manual p28 for more details.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to turn off the torque
 */
void  Herkulex_torqueOFF(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn void Herkulex_torqueBREAK(Herkulex_Struct*, uint8_t)
 * @brief Break the torque of a servo. See manual p28 for more details.
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to break the torque
 */
void  Herkulex_torqueBREAK(Herkulex_Struct * servos, uint8_t servoID);

/**
 * @fn void Herkulex_moveSpeed(Herkulex_Struct*, uint8_t, uint16_t, uint16_t, uint8_t)
 * @brief Move a servo to a position with a speed
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to move
 * @param Goal Position to move, can be from 0 to 1023
 * @param pTime Time to move in ms, can be from 0 to 2856
 * @param iLed LED to turn on, can be one on the list: HERKULEX_LED
 */
void  Herkulex_moveOne(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint16_t pTime, uint8_t iLed);

/**
 * @fn void Herkulex_rotate(Herkulex_Struct*, uint8_t, int16_t, uint16_t, uint8_t)
 * @brief Rotate a servo with a speed
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to rotate
 * @param speed Speed to rotate, can be from -1023 to 1023
 * @param pTime Time to rotate in ms, can be from 0 to 2856
 * @param iLed LED to turn on, can be one on the list: HERKULEX_LED
 */
void  Herkulex_rotateOne(Herkulex_Struct * servos, uint8_t servoID, int16_t speed, uint16_t pTime, uint8_t iLed);

/*	****************************************************************************************************************
	Functions for Configure the servos
*************************************************************************************************************** */

/**
 * @brief Get status of a servo
 * 
 * @param servos Herkulex_Struct Handler for all servos on a serial link
 * @param servoID ID of the servo to get status
 * @return uint16_t Status of the servo. 
 * 			| Byte 1 	   | Byte 0 	   |
 * 			| Status Error | Status Detail |
 * Check the manual at page 39 for more details.
 */
uint16_t  Herkulex_getStatus(Herkulex_Struct * servos, uint8_t servoID);

// TODO
uint8_t  Herkulex_model(Herkulex_Struct * servos);

// TODO
void  Herkulex_set_ID(Herkulex_Struct * servos, uint8_t ID_Old, uint8_t ID_New);



void  Herkulex_moveAll(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint8_t iLed);
void  Herkulex_moveSpeedAll(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint8_t iLed);
void  Herkulex_moveAllAngle(Herkulex_Struct * servos, uint8_t servoID, float angle, uint8_t iLed);
void  Herkulex_actionAll(Herkulex_Struct * servos, uint8_t pTime);



void  Herkulex_moveOneAngle(Herkulex_Struct * servos, uint8_t servoID, float angle, uint16_t pTime, uint8_t iLed);

uint16_t   Herkulex_getPosition(Herkulex_Struct * servos, uint8_t servoID);
float Herkulex_getAngle(Herkulex_Struct * servos, uint8_t servoID);
uint16_t   Herkulex_getSpeed(Herkulex_Struct * servos, uint8_t servoID);

void  Herkulex_setLed(Herkulex_Struct * servos,uint8_t servoID, uint8_t valueLed);

void  Herkulex_writeRegistryRAM(Herkulex_Struct * servos, uint8_t servoID, uint8_t address, uint8_t writeByte);
void  Herkulex_writeRegistryEEP(Herkulex_Struct * servos, uint8_t servoID, uint8_t address, uint8_t writeByte);


#ifdef __cplusplus
extern "C" }
#endif

#endif /* HERKULEX_H_ */
