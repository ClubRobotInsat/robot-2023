/**
 * @file can_stm32.h
 * @brief This file contains the declarations and definitions for the STM32 CAN interface.
 * @author Ronan Bonnet, Liam Chrisment, Triet Nguyen
 * @date 2024-04-09
 * @version 1.0
 */

#include "can_stm32.h"

/**
 * Variable to handling the CAN interface
 */
FDCAN_HandleTypeDef * canHandle = NULL;
FDCAN_RxHeaderTypeDef rxHeader; /* The rxHeader does not need to be constructed , it is only filled in when Rx messages are read */
FDCAN_TxHeaderTypeDef txHeader;
FDCAN_FilterTypeDef canfil;

/*
 * CAN ID of the STM32 <-- CHANGE THIS ACCORDING TO THE STM32 using CAN_Init()
 * ID for CDF2024 :
 * 0 : Urgency
 * 1 : Raspy
 * 2 : Base Roulante (Left + Right)
 * 3 : Base Roulante 2 (Front + Rear)
 * 4 : Bras + Storage
 * 5 : Sensors
 */
uint8_t CAN_ID_STM = 0;

/* CAN data buffers */
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};   
uint8_t canTX[8] = {0,0,0,0,0,0,0,0};
void (*CAN_receiveCallback)(void);

void CAN_errorHandler(void);

void CAN_initInterface(FDCAN_HandleTypeDef * hfdcan, uint8_t idSTM){
    canHandle = hfdcan;
    CAN_ID_STM = idSTM;
}

void CAN_filterConfig(void)
{
    /* Configure the global filter before configure the filter canfil, if not, the filter will reject everything */
    if (HAL_FDCAN_ConfigGlobalFilter(canHandle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        CAN_errorHandler();
    }

    canfil.IdType = FDCAN_STANDARD_ID;
    canfil.FilterIndex = 0;
    canfil.FilterType = FDCAN_FILTER_MASK;
    canfil.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    canfil.FilterID1 = CAN_ID_STM << 4;
    canfil.FilterID2 = 0xF0;						// Mask for dest_ID

    if (HAL_FDCAN_ConfigFilter(canHandle, &canfil) != HAL_OK)
    {
      /* Filter configuration Error */
        CAN_errorHandler();
    }
}

void CAN_setReceiveCallback(void (*callback)(void)){
    CAN_receiveCallback = callback;
}

void CAN_start(void)
{
    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(canHandle) != HAL_OK)
    {
        CAN_errorHandler();
    }

//    if (HAL_FDCAN_ConfigInterruptLines(canHandle, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0) != HAL_OK)
//	{
//		CAN_errorHandler();
//	}

    if (HAL_FDCAN_ActivateNotification(canHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        CAN_errorHandler();
    }
}

void CAN_makeHeader(uint8_t priority, uint8_t destID)
{
    uint32_t identifier = ((priority & 0x07) << 8) | ((destID & 0x0F) << 4) | (CAN_ID_STM & 0x0F);
	// Make tx
	txHeader.Identifier = identifier;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_FD_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0;
}

void CAN_send(uint8_t* data, uint8_t priority, uint8_t destID)
{
    CAN_makeHeader(priority, destID);
    for (int i = 0; i < 8; i++)
    {
        canTX[i] = data[i];
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(canHandle, &txHeader, canTX) != HAL_OK)
    {
        CAN_errorHandler();
    }
}

void CAN_read(void)
{
    if (HAL_FDCAN_GetRxMessage(canHandle, FDCAN_RX_FIFO0, &rxHeader, canRX) != HAL_OK)
    {   
        if (canHandle->ErrorCode == HAL_FDCAN_ERROR_FIFO_EMPTY)
        {

        }
        else
        {
            CAN_errorHandler(); /* When the RxFifo is empty, an error is returned by the HAL */
        }
        
    }
    else {
        CAN_receiveCallback();
    }
}

//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
//	if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE){
//		CAN_receiveCallback();
//	}
//}

uint8_t * CAN_getRXData(void){
    return canRX;
}

uint8_t CAN_decodeIDSrc(void)
{
    return (rxHeader.Identifier & 0x0F);
}

void CAN_sendBackPing(uint8_t destID) {
	uint8_t data[8] = {1,0,1,0,1,0,1,0};
	CAN_send(data, destID, CAN_ID_STM);
}

void CAN_errorHandler(void)
{
    /* Error handling to implement */
}
