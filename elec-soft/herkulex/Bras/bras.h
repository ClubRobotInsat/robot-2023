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

void BRAS_init(UART_HandleTypeDef * huart, TIM_HandleTypeDef * pinceTim, uint32_t pinceChannel);

void BRAS_moveHomePosition(void);

void BRAS_moveReadyPosition(void);

void BRAS_movePutInStock(void);

void BRAS_moveGetFromStock(void);

void BRAS_grab(void);

void BRAS_release(void);

void BRAS_getPosition(void);

#ifdef __cplusplus
extern "C" }
#endif

#endif /* BRAS_H */
