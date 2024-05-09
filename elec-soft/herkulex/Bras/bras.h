/**
 * @file bras.h
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Header of code to control the arm of robot CDF2024
 * @version 0.1
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef BRAS_H
#define BRAS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"


typedef struct {
    TIM_HandleTypeDef * pinceTimer;
    uint32_t pinceChannel;
} Pince_StructTypeDef;

extern uint16_t position_actuel[3];

void BRAS_init(UART_HandleTypeDef * huartArm, UART_HandleTypeDef * huartChaine, TIM_HandleTypeDef * pinceTim, uint32_t pinceChannel, FDCAN_HandleTypeDef * hfdcan);

uint8_t BRAS_moveHomePosition(void);

uint8_t BRAS_moveReadyPosition(void);

uint8_t BRAS_moveFindObjetLow();

uint8_t BRAS_moveFindObjetHigh();

uint8_t BRAS_movePutInStock(void);

uint8_t BRAS_moveGetFromStock(void);

uint8_t BRAS_movePlaceObject(void);

void BRAS_grab(void);

void BRAS_release(void);

void BRAS_getPosition(void);

void BRAS_readMsgFromCAN(void);

void BRAS_execCMD(void);

#ifdef __cplusplus
extern "C" }
#endif

#endif /* BRAS_H */
