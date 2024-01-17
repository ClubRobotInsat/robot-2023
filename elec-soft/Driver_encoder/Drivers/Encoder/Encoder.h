/*
 * Encoder.h
 *
 *  Created on: Jan 4, 2024
 *      Author: TANG Huong Cam
 */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_


#include "stm32g4xx_hal.h"



void Encoder_Init (void);


float Encoder_Left_Get_Distance (void);
float Encoder_Right_Get_Distance (void);



void Encoder_Start_Record (void);


#endif /* ENCODER_ENCODER_H_ */
