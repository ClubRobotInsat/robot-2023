/*
 * movement.h
 *
 *  Created on: Feb 1, 2024
 *      Author: hoang
 */

#ifndef MOUVEMENT_MOVEMENT_H_
#define MOUVEMENT_MOVEMENT_H_

#include "stm32g4xx_hal.h"
#include "Encoder/Encoder.h"
#include "MOTOR_DC/motor_dc.h"

#define MOVEMENT_STOP_LIMIT_DISTANCE			5
#define MOVEMENT_SLOWDOWN_LIMIT_DISTANCE		10
#define MOVEMENT_SLOWDOWN_VITESSE				25
#define MOVEMENT_NORMAL_VITESSE					60

#define MOVEMENT_MAX_VITESSE					80
#define MOVEMENT_MIN_VITESSE					20

#define MOVEMENT_MAX_DISTANCE					150 //in centimeter


void Movement_Init (void);
void Movement_Set_Speed(int speed);
void Movement_Regulation(int distance);
void Movement_Set_Distance(int distance);

#endif /* MOUVEMENT_MOVEMENT_H_ */
