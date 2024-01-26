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


/* Exported define --------------------------*/
#define NBR_SERVOS	 10				// Number of servos to manipulate simultaneously 	<---- change this for more servos!

#define DATA_SIZE	 30				// Size of buffer for input data

#define DATA_ACTION_ALL  	 5*NBR_SERVOS	// Size of package to manipulate all motors simultaneously


/* SERVO HERKULEX COMMAND - See Manual p40 */
#define HEEPWRITE    0x01 	//Rom (EEP) write
#define HEEPREAD     0x02 	//Rom (EEP) read
#define HRAMWRITE	 0x03 	//Ram write
#define HRAMREAD	 0x04 	//Ram read
#define HIJOG		 0x05 	//Write n servo with different timing
#define HSJOG		 0x06 	//Write n servo with same time
#define HSTAT	 	 0x07 	//Read error
#define HROLLBACK	 0x08 	//Back to factory value
#define HREBOOT	 	 0x09 	//Reboot

/* HERKULEX LED - See Manual p29 */
#define LED_GREEN 	 0x01
#define LED_BLUE     0x02
#define LED_CYAN     0x03
#define LED_RED    	 0x04
#define LED_GREEN2 	 0x05
#define LED_PINK     0x06
#define LED_WHITE    0x07

/* HERKULEX STATUS ERROR - See Manual p39 */
#define H_STATUS_OK					 0x00
#define H_ERROR_INPUT_VOLTAGE 		 0x01
#define H_ERROR_POS_LIMIT			 0x02
#define H_ERROR_TEMPERATURE_LIMIT	 0x04
#define H_ERROR_INVALID_PKT			 0x08
#define H_ERROR_OVERLOAD			 0x10
#define H_ERROR_DRIVER_FAULT    	0x20
#define H_ERROR_EEPREG_DISTORT	 	0x40

/* HERKULEX Broadcast Servo ID */
#define BROADCAST_ID	0xFE


/* HERKULEX Ack mode */
#define ACK_No_Reply		0x00
#define ACK_Reply_Read		0x01
#define ACK_Reply_Always	0x02

typedef struct {
	uint8_t pSize;		/* Packet Size */
	uint8_t pID;
	uint8_t cmd;

	uint8_t lenghtString;

	uint8_t ck1;		/* Check Sum1 */
	uint8_t ck2;		/* Check Sum2 */


	uint8_t XOR;		/* Use for Check Sum */
	uint8_t playTime; 	/* Execution time*/

	uint8_t data[DATA_SIZE];		/* Input data to servos*/
	uint8_t dataEx[DATA_ACTION_ALL+8];

	uint8_t conta;		/* counter for simultaneous actions */
	uint8_t moveData[DATA_ACTION_ALL];	/* data for simultaneous actions*/

} Herkulex_Message_Struct;

/**
 * @struct Herkulex_Struct
 * @brief  Structure to manipulate up to NBR_SERVOS
 *
 */
typedef struct {
	UART_HandleTypeDef * huart;	/* Handle for UART connection */
	Herkulex_Message_Struct msg;
} Herkulex_Struct;



/* Exported functions --------------------------*/
/**
 * @fn void HKL_begin(Herkulex_Struct*, UART_HandleTypeDef*)
 * @brief Setup a UART link to Herkulex
 * 
 * @param servos Herkulex_Struct handler
 * @param huart UART handler
 */
void HKL_begin(Herkulex_Struct * servos, UART_HandleTypeDef * huart);

/**
 * @fn void HKL_init(Herkulex_Struct*)
 * @brief Initialize all motors
 *
 * @param servos
 */
void  HKL_init(Herkulex_Struct * servos);
void  HKL_ACK(Herkulex_Struct * servos, uint8_t valueACK);
void  HKL_clearError(Herkulex_Struct * servos, uint8_t servoID);
uint16_t  HKL_stat(Herkulex_Struct * servos, uint8_t servoID);

uint8_t  HKL_model(Herkulex_Struct * servos);
void  HKL_set_ID(Herkulex_Struct * servos, uint8_t ID_Old, uint8_t ID_New);


void  HKL_torqueON(Herkulex_Struct * servos, uint8_t servoID);
void  HKL_torqueOFF(Herkulex_Struct * servos, uint8_t servoID);

void  HKL_moveAll(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint8_t iLed);
void  HKL_moveSpeedAll(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint8_t iLed);
void  HKL_moveAllAngle(Herkulex_Struct * servos, uint8_t servoID, float angle, uint8_t iLed);
void  HKL_actionAll(Herkulex_Struct * servos, uint8_t pTime);

void  HKL_rotate(Herkulex_Struct * servos, uint8_t servoID, uint16_t speed, uint16_t pTime, uint8_t iLed);
void  HKL_moveOne(Herkulex_Struct * servos, uint8_t servoID, uint16_t Goal, uint16_t pTime, uint8_t iLed);
void  HKL_moveOneAngle(Herkulex_Struct * servos, uint8_t servoID, float angle, uint16_t pTime, uint8_t iLed);

uint16_t   HKL_getPosition(Herkulex_Struct * servos, uint8_t servoID);
float HKL_getAngle(Herkulex_Struct * servos, uint8_t servoID);
uint16_t   HKL_getSpeed(Herkulex_Struct * servos, uint8_t servoID);

void  HKL_reboot(Herkulex_Struct * servos, uint8_t servoID);
void  HKL_setLed(Herkulex_Struct * servos,uint8_t servoID, uint8_t valueLed);

void  HKL_writeRegistryRAM(Herkulex_Struct * servos, uint8_t servoID, uint8_t address, uint8_t writeByte);
void  HKL_writeRegistryEEP(Herkulex_Struct * servos, uint8_t servoID, uint8_t address, uint8_t writeByte);


#ifdef __cplusplus
extern "C" }
#endif

#endif /* HERKULEX_H_ */
