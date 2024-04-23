/* Includes */
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "herkulexMessageBuilder.h"


/* Private defines -----------------------------------------------*/

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

/* Private function -----------------------------------------------*/

/**
 * @fn uint8_t HMB_checksum1(Herkulex_Message_Struct*)
 * @brief Checksum 1 to verify data of Herkulex
 *
 * @param pPackage Herkulex_Message_Struct handler
 * @return Result of Checksum1
 */
uint8_t HMB_checksum1(Herkulex_Message_Struct * msg){
	uint8_t XOR = 0;
	XOR = XOR ^ msg->pSize;
	XOR = XOR ^ msg->pID;
	XOR = XOR ^ msg->cmd;
	for (int i = 0; i < msg->lenghtData; i++)
	{
		XOR = XOR ^ msg->data[i];
	}
	return (XOR&0xFE);
}

/**
 * @fn uint8_t HMB_checksum2(uint8_t)
 * @brief Checksum 2 to verify data of Herkulex
 *
 * @param XOR Checksum1
 * @return Result of Checksum2
 */
static inline uint8_t HMB_checksum2(uint8_t XOR){
	return (~XOR)&0xFE;
}

/* -------------------------------------------------------------------------
* 	                            API functions
* ------------------------------------------------------------------------- */


/* --------------------------- HREBOOT ------------------------------------ */

uint8_t HMB_reboot(uint8_t *pPackage, uint8_t servoID){
    Herkulex_Message_Struct msg;
    msg.pSize = 0x07;
    msg.pID = servoID;
    msg.cmd = HREBOOT;
	msg.ck1 = (msg.pSize^msg.pID^msg.cmd)&0xFE;
	msg.ck2 = (~(msg.pSize^msg.pID^msg.cmd))&0xFE ; ;

	pPackage[0] = 0xFF;
	pPackage[1] = 0xFF;
	pPackage[2] = msg.pSize;
	pPackage[3] = msg.pID;
	pPackage[4] = msg.cmd;
	pPackage[5] = msg.ck1;
	pPackage[6] = msg.ck2;

    return msg.pSize;
}

/* --------------------------- HRAMWRITE ------------------------------------ */
uint8_t HMB_ramWrite(uint8_t *pPackage, uint8_t servoID, Herkulex_RAM_Address_t startAddress, uint8_t *data, uint8_t lenghtDataToWrite){
    Herkulex_Message_Struct msg;

    msg.pSize = 0x09 + lenghtDataToWrite;
	msg.pID   = servoID;
	msg.cmd   = HRAMWRITE;
	msg.data[0] = startAddress;               
	msg.data[1] = lenghtDataToWrite;               
	
	for (int i = 0; i < lenghtDataToWrite; i++)
	{
		msg.data[i+2] = data[i];
	}
	      
	msg.lenghtData = 2 + lenghtDataToWrite;

	msg.ck1 = HMB_checksum1(&(msg));
	msg.ck2 = HMB_checksum2(msg.ck1);

	pPackage[0] = 0xFF;
	pPackage[1] = 0xFF;
	pPackage[2] = msg.pSize;
	pPackage[3] = msg.pID;
	pPackage[4] = HRAMWRITE;
	pPackage[5] = msg.ck1;
	pPackage[6] = msg.ck2;
	pPackage[7] = msg.data[0];
	pPackage[8] = msg.data[1];
	
	for (int i = 0; i < lenghtDataToWrite; i++){
		pPackage[i+9] = msg.data[i+2];
	}

    return msg.pSize;
}
/* --------------------------- HRAMREAD ------------------------------------ */
uint8_t HMB_ramRead(uint8_t *pPackage, uint8_t servoID, Herkulex_RAM_Address_t startAddress, uint8_t lenghtDataToRead){
	Herkulex_Message_Struct msg;

    msg.pSize = 0x09;
	msg.pID   = servoID;
	msg.cmd   = HRAMREAD;
	msg.data[0] = startAddress;               
	msg.data[1] = lenghtDataToRead;               
	      
	msg.lenghtData = 2;

	msg.ck1 = HMB_checksum1(&(msg));
	msg.ck2 = HMB_checksum2(msg.ck1);

	pPackage[0] = 0xFF;
	pPackage[1] = 0xFF;
	pPackage[2] = msg.pSize;
	pPackage[3] = msg.pID;
	pPackage[4] = HRAMREAD;
	pPackage[5] = msg.ck1;
	pPackage[6] = msg.ck2;
	pPackage[7] = msg.data[0];
	pPackage[8] = msg.data[1];

    return msg.pSize;
}


/* --------------------------- HSJOG --------------------------------------- */
uint8_t HMB_sJog(uint8_t *pPackage, uint8_t servoID, uint8_t pTime, uint8_t goalLSB, uint8_t goalMSB, uint8_t set){
	Herkulex_Message_Struct msg;

	msg.pSize = 0x0C;
	msg.cmd   = HSJOG;

	msg.data[0] = goalLSB;
	msg.data[1] = goalMSB;
	msg.data[2] = set;
	msg.data[3] = servoID;

	msg.playTime = (uint8_t)((float)pTime/11.2);			// Execution time (11.2ms per unit)
	msg.pID = servoID ^ msg.playTime;

	msg.lenghtData = 4;

	msg.ck1 = HMB_checksum1(&(msg));
	msg.ck2 = HMB_checksum2(msg.ck1);

	msg.pID = servoID;

	pPackage[0] = 0xFF;
	pPackage[1] = 0xFF;
	pPackage[2] = msg.pSize;
	pPackage[3] = msg.pID;
	pPackage[4] = msg.cmd;
	pPackage[5] = msg.ck1;
	pPackage[6] = msg.ck2;
	pPackage[7] = msg.playTime;
	pPackage[8] = msg.data[0];
	pPackage[9] = msg.data[1];
	pPackage[10] = msg.data[2];
	pPackage[11] = msg.data[3];

	return msg.pSize;
}

/* --------------------------- HSTAT ------------------------------------ */
uint8_t HMB_stat(uint8_t *pPackage, uint8_t servoID){
	Herkulex_Message_Struct msg;

    msg.pSize = 0x07;
	msg.pID   = servoID;
	msg.cmd   = HSTAT;             

	msg.ck1 = (msg.pSize^msg.pID^msg.cmd)&0xFE;
	msg.ck2 = (~(msg.pSize^msg.pID^msg.cmd))&0xFE;

	pPackage[0] = 0xFF;
	pPackage[1] = 0xFF;
	pPackage[2] = msg.pSize;
	pPackage[3] = msg.pID;
	pPackage[4] = HSTAT;
	pPackage[5] = msg.ck1;
	pPackage[6] = msg.ck2;

    return msg.pSize;
}
