/**
 * @file can_stm32.h
 * @brief Header file for the STM32 CAN interface.
 *
 * This file contains the declarations and definitions for the STM32 CAN interface.
 * It provides functions and macros to interact with the CAN peripheral on STM32 microcontrollers 
 * using MCP2551 Can Transceiver.
 *
 * @author Ronan Bonnet, Liam Chrisment, Triet Nguyen
 * @date 2024-04-09
 * @version 1.0
 */

#ifndef CAN_STM32_H_
#define CAN_STM32_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Include */
#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
/*
* The configuration of the CAN peripheral on the STM32 is done using the STM32CubeMX software.
* STM32 Configurations required (with MCP2551):
* - Clock Divider : Divide kernel clock by 2
* - Frame Format : Classic mode
* - Mode : Normal mode
* - Auto Retransmission : Disable
* - Transmit Pause : Disable
* - Protocol Exception : Disable
* - Nominal Sync Jump Width : 1
* - Data Prescaler : 1
* - Data Sync Jump Width : 1
* - Data Time Seg1 : 1
* - Data Time Seg2 : 1
* - Std Filters Nbr : 1
* - Ext Filters Nbr : 0
* - Tx Fifo Queue Mode : FIFO mode
* - Nominal Prescaler : 8
* - Nominal Time Seg1 : 3
* - Nominal Time Seg2 : 4
*/

/**
 * Message structure for CAN communication.
 * |                       Header (11 bits)                 |                              Data (8 bytes)                              |
 * | Priority (3 bits) | Dest ID (4 bits) | Src ID (4 bits) | Command ID (1 byte) | 1st Param (1 byte) | Optionnal Parameter (6 bytes) |
 */

/*
 * CAN ID of the STM32 <-- CHANGE THIS ACCORDING TO THE STM32
 * ID for CDF2024 :
 * 0 : Urgency
 * 1 : Raspy
 * 2 : Base Roulante (Left + Right)
 * 3 : Base Roulante 2 (Front + Rear)
 * 4 : Bras + Storage
 * 5 : Sensors
 */
extern uint8_t CAN_ID_STM;

/**
 * ID of the Raspberry PI
 */
#define CAN_ID_MASTER 1

/**
 * @brief Pass the CAN handle to the interface.
 * 
 * @param hfdcan Handle of the CAN peripheral.
 * @return None
 */
void CAN_initInterface(FDCAN_HandleTypeDef * hfdcan, uint8_t idSTM);

/**
 * @brief Configure the filter for the CAN peripheral. Required after the initialization of the CAN peripheral.
 * 
 * @return None
 */
void CAN_filterConfig(void);

/**
 * @brief Set the callback function to be called when a message is received.
 * 
 * @param callback Callback function to be called.
 * 
 * @return None
 */
void CAN_setReceiveCallback(void (*callback)(void));

/**
 * @brief Start the CAN peripheral.
 * 
 * @return None
 */
void CAN_start(void);

/**
 * @brief Make the header of the CAN message.
 * 
 * @param priority Priority of the message.
 * @param destID ID of the receiver.
 */
void CAN_makeHeader(uint8_t priority, uint8_t destID);

/**
 * @brief Send a CAN message.
 * 
 * @param data Data to send.
 * @param priority Priority of the message.
 * @param destID ID of the receiver.
 */
void CAN_send(uint8_t* data, uint8_t priority, uint8_t destID);

/**
 * @brief Call the receive callback function when a message is received. If FIFO is empty, do nothing.
 * 
 * @return None
 */
void CAN_read(void);

/**
 * @brief Get the adresse of the received data.
 * 
 * @return uint8_t* Pointer to the data of the received message.
 */
uint8_t * CAN_getRXData(void);

/**
 * @brief Decode the source ID of the received message.
 * 
 * @return uint8_t Source ID of the received message.
 */
uint8_t CAN_decodeIDSrc(void);

/**
 * @brief Send a ping message. !!!DANGER!!! Risk of deprecated address. Need to fix.
 * 
 * @param destID ID of the receiver.
 * 
 * @return None
 */
void CAN_sendBackPing(uint8_t destID);

#ifdef __cplusplus
}
#endif

#endif /* CAN_STM32_H_ */
