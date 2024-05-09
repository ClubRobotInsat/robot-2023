/**
 * @file chaine.h
 * @author Triet NGUYEN (tr_nguye@insa-toulouse.fr)
 * @brief Header of code to control the chaine of robot CDF2024
 * @version 0.1
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef CHAINE_H
#define CHAINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

extern uint16_t CHAINE_pos_actuel;

/**
 * @brief Initialize the chaine, set the speed to CHAINE_speed
 * 
 * @param huart UART handler to communicate with the chaine
 */
void CHAINE_init(UART_HandleTypeDef * huart);

/**
 * @brief Make the chaine spin at speed CHAINE_speed (put the torque on)
 * 
 */
void CHAINE_spin(void);

/**
 * @brief Stop the chaine(put the torque off)
 * 
 */
void CHAINE_stop(void);

#ifdef __cplusplus
extern "C" }
#endif

#endif /* CHAINE_H */
