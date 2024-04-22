/**
 * @file herkulexMessageBuilder.h
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief  This file contains the function to build the message to send to the Herkulex servo motor
 * @version 0.2 : Basics functions to build the message - lack of HEEPWRITE, HEEPREAD, HROLLBACK
 * @date 2024-02-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef HERKULEX_MESSAGE_HMBER_H_
#define HERKULEX_MESSAGE_HMBER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32g4xx_hal.h"

/* Exported define ------------------------------------------------------------*/
#define DATA_SIZE	 30				// Size of buffer for input data

/* Exported types ------------------------------------------------------------*/

/**
 * @struct Herkulex_Message_Struct
 * @brief  Structure to create message to Herkulex
 *
 */
typedef struct {
	uint8_t pSize;				/* Packet Size */
	uint8_t pID;
	uint8_t cmd;

	uint8_t ck1;				/* Check Sum1 */
	uint8_t ck2;				/* Check Sum2 */

	uint8_t XOR;				/* Use for Check Sum */
	uint8_t playTime; 			/* Execution time*/

	uint8_t lenghtData;			/* Lenght of input data to send to servo*/
	uint8_t data[DATA_SIZE];	/* Input data to servos*/
} Herkulex_Message_Struct;

typedef enum {
	RAM_ID = 0x00,
	RAM_ACK_POLICY = 0x01,
	RAM_LED_POLICY = 0x02,
	RAM_TORQUE_POLICY = 0x03,
	RAM_MAX_TEMPERATURE = 0x05,
	RAM_MIN_VOLTAGE = 0x06,
	RAM_MAX_VOLTAGE = 0x07,
	RAM_ACCELERATION_RATIO = 0x08,
	RAM_MAX_ACCELERATION_TIME = 0x09,
	RAM_DEAD_ZONE = 0x0A,
	RAM_SATURATOR_OFFSET = 0x0B,
	RAM_SATURATOR_SLOPE = 0x0C,
	RAM_PWM_OFFSET = 0x0D,
	RAM_MIN_PWM = 0x0E,
	RAM_MAX_PWM = 0x0F,
	RAM_OVERLOAD_PWM_THRESHOLD = 0x12,
	RAM_MIN_POSITION = 0x14,
	RAM_MAX_POSITION = 0x16,
	RAM_POSITION_KP = 0x18,
	RAM_POSITION_KD = 0x1A,
	RAM_POSITION_KI = 0x1C,
	RAM_POSITION_FEEDFORWARD_1ST_GAIN = 0x1E,
	RAM_POSITION_FEEDFORWARD_2ND_GAIN = 0x20,
	RAM_LED_BLINK_PERIOD = 0x26,
	RAM_ADC_FAULT_CHECK_PERIOD = 0x27,
	RAM_PACKET_GARBAGE_CHECK_PERIOD = 0x28,
	RAM_STOP_DETECTION_PERIOD = 0x29,
	RAM_OVERLOAD_DETECTION_PERIOD = 0x2A,
	RAM_STOP_THRESHOLD = 0x2B,
	RAM_INPOSITION_MARGIN = 0x2C,
	RAM_CALIBRATION_DIFFERENCE = 0x2F,
	RAM_STATUS_ERROR = 0x30,
	RAM_STATUS_DETAIL = 0x31,
	RAM_TORQUE_CONTROL = 0x34,
	RAM_LED_CONTROL = 0x35,
	RAM_VOLTAGE = 0x36,
	RAM_TEMPERATURE = 0x37,
	RAM_CURRENT_CONTROL_MODE = 0x38,		//????
	RAM_TICK = 0x39,
	RAM_CALIBRATED_POSITION = 0x38,			//????
	RAM_ABSOLUTE_POSITION = 0x3A,
	RAM_DIFFERENTIAL_POSITION = 0x3C,
	RAM_PWM = 0x3E,
	RAM_ABSOLUTE_GOAL_POSITION = 0x42,
	RAM_ABSOLUTE_DESIRED_TRAJECTORY_POSITION = 0x44,
	RAM_DESIRED_VELOCITY = 0x46,
} Herkulex_RAM_Address_t;

typedef enum {
	EEP_MODEL_NO1 = 0x00,
	EEP_MODEL_NO2 = 0x01,
	EEP_VERSION1 = 0x02,
	EEP_VERSION2 = 0x03,
	EEP_BAUD_RATE = 0x04,
	EEP_ID = 0x06,
	EEP_ACK_POLICY = 0x07,
	EEP_LED_POLICY = 0x08,
	EEP_TORQUE_POLICY = 0x09,
	EEP_MAX_TEMPERATURE = 0x0B,
	EEP_MIN_VOLTAGE = 0x0C,
	EEP_MAX_VOLTAGE = 0x0D,
	EEP_ACCELERATION_RATIO = 0x0E,
	EEP_MAX_ACCELERATION_TIME = 0x0F,
	EEP_DEAD_ZONE = 0x10,
	EEP_SATURATOR_OFFSET = 0x11,
	EEP_SATURATOR_SLOPE = 0x12,
	EEP_PWM_OFFSET = 0x14,
	EEP_MIN_PWM = 0x15,
	EEP_MAX_PWM = 0x16,
	EEP_OVERLOAD_PWM_THRESHOLD = 0x18,
	EEP_MIN_POSITION = 0x1A,
	EEP_MAX_POSITION = 0x1C,
	EEP_POSITION_KP = 0x1E,
	EEP_POSITION_KD = 0x20,
	EEP_POSITION_KI = 0x22,
	EEP_POSITION_FEEDFORWARD_1ST_GAIN = 0x24,
	EEP_POSITION_FEEDFORWARD_2ND_GAIN = 0x26,
	EEP_LED_BLINK_PERIOD = 0x2C,
	EEP_ADC_FAULT_CHECK_PERIOD = 0x2D,
	EEP_PACKET_GARBAGE_CHECK_PERIOD = 0x2E,
	EEP_STOP_DETECTION_PERIOD = 0x2F,
	EEP_OVERLOAD_DETECTION_PERIOD = 0x30,
	EEP_STOP_THRESHOLD = 0x31,
	EEP_INPOSITION_MARGIN = 0x32,
	EEP_CALIBRATION_DIFFERENCE = 0x35,
} Herkulex_EEP_Address_t;

/* --------------------------------------------------------------------------
                                API functions
----------------------------------------------------------------------------- */

/* --------------------------- HREBOOT ------------------------------------ */

/**
 * @brief Message for HREBOOT command
 * 
 * @param pPackage package to write in the message
 * @param servoID  ID of the servo
 * @return uint8_t size of the package to send
 */
uint8_t HMB_reboot(uint8_t *pPackage, uint8_t servoID);

/* --------------------------- HRAMWRITE ------------------------------------ */
/**
 * @brief Message for HRAMWRITE command
 * 
 * @param pPackage package to write in the message
 * @param servoID  ID of the servo
 * @param startAddress  Start address to write
 * @param data  Data to write
 * @param lenghtData  Lenght of data to write
 * @return uint8_t size of the package to send
 */
uint8_t HMB_ramWrite(uint8_t *pPackage, uint8_t servoID, Herkulex_RAM_Address_t startAddress, uint8_t *data, uint8_t lenghtData);

/* --------------------------- HRAMREAD ------------------------------------ */
/**
 * @brief Message for HRAMREAD command
 * 
 * @param pPackage package to write in the message
 * @param servoID  ID of the servo
 * @param startAddress  Start address to read
 * @param lenghtData  Lenght of data to read
 * @return uint8_t size of the package to send
 */
uint8_t HMB_ramRead(uint8_t *pPackage, uint8_t servoID, Herkulex_RAM_Address_t startAddress, uint8_t lenghtData);

/* --------------------------- HSJOG --------------------------------------- */
/**
 * @brief Message for HSJOG command
 * 
 * @param pPackage package to write in the message
 * @param servoID  ID of the servo
 * @param pTime  Execution time
 * @param goalMSB  Goal position MSB
 * @param goalLSB  Goal position LSB
 * @param setValue  2 bytes od Set, see manual p.48 for more details
 * @return uint8_t size of the package to send
 */
uint8_t HMB_sJog(uint8_t *pPackage, uint8_t servoID, uint8_t pTime, uint8_t goalMSB, uint8_t goalLSB, uint8_t setValue);

/* --------------------------- HSTAT ------------------------------------ */
/**
 * @brief Message for HSTAT command
 * 
 * @param pPackage package to write in the message
 * @param servoID  ID of the servo
 * @return uint8_t size of the package to send
 */ 
uint8_t HMB_stat(uint8_t *pPackage, uint8_t servoID);

/* --------------------------- HEEPWRITE ------------------------------------ */
/**
 * @brief Message for HEEPWRITE command
 * 
 * @param pPackage package to write in the message
 * @param servoID  ID of the servo
 * @param startAddress  Start address to write
 * @param data  Data to write
 * @param lenghtData  Lenght of data to write
 * @return uint8_t size of the package to send
 */
uint8_t HMB_eepWrite(uint8_t *pPackage, uint8_t servoID, Herkulex_EEP_Address_t startAddress, uint8_t *data, uint8_t lenghtData);
/* --------------------------- HEEPREAD ------------------------------------ */
/**
 * @brief Message for HEEPREAD command
 * 
 * @param pPackage package to write in the message
 * @param servoID  ID of the servo
 * @param startAddress  Start address to read
 * @param lenghtData  Lenght of data to read
 * @return uint8_t size of the package to send
 */
uint8_t HMB_eepRead(uint8_t *pPackage, uint8_t servoID, Herkulex_EEP_Address_t startAddress, uint8_t lenghtData);
/* --------------------------- HROLLBACK ------------------------------------ */
/**
 * @brief Message for HROLLBACK command
 * 
 * @param pPackage package to write in the message
 * @param servoID  ID of the servo
 * @return uint8_t size of the package to send
 */
uint8_t HMB_roolback(uint8_t *pPackage, uint8_t servoID);

#ifdef __cplusplus
extern "C" }
#endif

#endif /* HERKULEX_MESSAGE_HMBER_H_ */
