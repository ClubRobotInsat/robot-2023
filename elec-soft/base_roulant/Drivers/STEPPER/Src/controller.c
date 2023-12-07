/**
  ******************************************************************************
  * @file    controller.c
  * @author  Triet NGUYEN - Club Robot INSA Toulouse
  * 		 Cam TANG - Club Robot INSA Toulouse
  * @brief   Driver for Stepper Motor Controller DRV8811
  * 		 Datasheet of DRV8811 is available here:
  * 		 https://www.ti.com/lit/ds/symlink/drv8811.pdf?ts=1698326732361&ref_url=https%253A%252F%252Fwww.google.com%252F
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "../Inc/controller.h"

/* Private define ------------------------------------------------------------*/
/** @defgroup CONTROLLER_Private_Constants
  * @{
  */
/** @defgroup Peripheral_Configuration Peripheral Configuration
  * @brief Peripheral Configuration to Communicate with Controller from STM32G431KBT6
  * @{
  */
#define Enable_Port 	GPIOA
#define Enable_Pin		(0x00U)

#define Reset_Port		GPIOA
#define Reset_Pin		(0x01U)

#define USM0_Port		GPIOA
#define USM0_Pin		(0x02U)

#define USM1_Port		GPIOA
#define USM1_Pin		(0x03U)

#define DIR_Port		GPIOA
#define DIR_Pin			(0x06U)

#define HOME_Port		GPIOA
#define HOME_Pin		(0x04U)

#define PWM_Tim_Channel	TIM_CHANNEL_1


TIM_HandleTypeDef  PWM_Tim;

/**
  * @}
  */

/** @defgroup Controller_Init_Timer Controller init timer
  * @brief Init configuration for Timer in STM
  * @{
  */
#define  CONTROLLER_ARR_INIT				(0x64U)	    /*!< Timer registre ARR = 100 at the init phase */
#define  CONTROLLER_DUTY_CYCLE_INIT    		(0x0U)   	/*!< Timer registre CCR = 0 at the init phase */
/**
  * @}
  */

/**
  * @}
  */

void CONTROLLER_InitController(CONTROLLER_FuncConfigTypeDef * hFuncConfig, TIM_HandleTypeDef TimerController){

	PWM_Tim = TimerController;

	/* Reset controller */
	CONTROLLER_Reset();

	/* Disable output */
	CONTROLLER_Disable();

	/* Set Step Mode for controller */
	if (hFuncConfig->Step_Mode == CONTROLLER_MODE_FULL_STEP){
		HAL_GPIO_WritePin(USM0_Port, USM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(USM1_Port, USM1_Pin, GPIO_PIN_RESET);
	}
	else if (hFuncConfig->Step_Mode == CONTROLLER_MODE_HALF_STEP){
		HAL_GPIO_WritePin(USM0_Port, USM0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(USM1_Port, USM1_Pin, GPIO_PIN_RESET);
	}
	else if (hFuncConfig->Step_Mode == CONTROLLER_MODE_QUARTER_STEP){
		HAL_GPIO_WritePin(USM0_Port, USM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(USM1_Port, USM1_Pin, GPIO_PIN_SET);
	}
	else if (hFuncConfig->Step_Mode == CONTROLLER_MODE_EIGHT_STEP){
		HAL_GPIO_WritePin(USM0_Port, USM0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(USM1_Port, USM1_Pin, GPIO_PIN_SET);
	}
	else {
		CONTROLLER_Error_Handler();
	}

	/* Set Step Direction for controller */
	HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_SET);

	/* Set Max frequency by ARR and PRC*/

	__HAL_TIM_SET_PRESCALER(&PWM_Tim,(HAL_RCC_GetSysClockFreq()/hFuncConfig->Step_Freq_Max/CONTROLLER_ARR_INIT)-1);

	__HAL_TIM_SET_AUTORELOAD(&PWM_Tim, CONTROLLER_ARR_INIT-1);

	__HAL_TIM_SET_COMPARE(&PWM_Tim,PWM_Tim_Channel,CONTROLLER_DUTY_CYCLE_INIT);
}

void CONTROLLER_Reset(){
	HAL_GPIO_WritePin(Reset_Port, Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(Reset_Port, Reset_Pin, GPIO_PIN_SET);
}


void CONTROLLER_Enable(){
	HAL_TIM_PWM_Start(&PWM_Tim, PWM_Tim_Channel);
	HAL_GPIO_WritePin(Enable_Port, Enable_Pin, GPIO_PIN_RESET);
}

void CONTROLLER_Disable(){
	HAL_GPIO_WritePin(Enable_Port, Enable_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Stop(&PWM_Tim, PWM_Tim_Channel);
}


void CONTROLLER_Set_DIR(uint8_t direction){

	if (direction == CONTROLLER_DIR_CLOCK_WISE){
		HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_SET);
	}
	else if (direction == CONTROLLER_DIR_COUNTER_CLOCK_WISE){
		HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET);
	}
	else{
		CONTROLLER_Error_Handler();
	}
}

void CONTROLLER_Set_STEP_freq(uint8_t percentage_freq){
	uint8_t autoreloadWanted;
	if((1 <= percentage_freq) && (percentage_freq <= 100))
	{
		autoreloadWanted = (CONTROLLER_ARR_INIT*100/percentage_freq)-1;
		__HAL_TIM_SET_AUTORELOAD(&PWM_Tim,autoreloadWanted);
		__HAL_TIM_SET_COMPARE(&PWM_Tim, PWM_Tim_Channel, autoreloadWanted*10/100);
	}
	else if (percentage_freq == 0)
	{
		__HAL_TIM_SET_COMPARE(&PWM_Tim, PWM_Tim_Channel, CONTROLLER_DUTY_CYCLE_INIT);
	}
	else
	{
		CONTROLLER_Error_Handler();
	}
}

uint16_t CONTROLLER_Get_STEP_freq(){
	return 0;
			//HAL_RCC_GetSysClockFreq()/((__HAL_TIM_GET_PRESCALER(&PWM_Tim)+1)*(__HAL_TIM_GET_AUTORELOAD(&PWM_Tim)+1));
}

void CONTROLLER_Error_Handler(){

}
