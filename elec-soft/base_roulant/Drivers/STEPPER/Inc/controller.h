/**
  ******************************************************************************
  * @file    controller.h
  * @author  Triet NGUYEN - Club Robot INSA Toulouse
  * 		 Cam TANG - Club Robot INSA Toulouse
  * @brief   Header file of Driver for Stepper Motor Controller DRV8811
  * 		 Datasheet of DRV8811 is available here:
  * 		 https://www.ti.com/lit/ds/symlink/drv8811.pdf?ts=1698326732361&ref_url=https%253A%252F%252Fwww.google.com%252F
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROLLER_STEPPER
#define CONTROLLER_STEPPER

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/** @defgroup Controller_Exported_Types Controller Exported Types
  * @{
  */

/**
  * @brief  Definition of Structure for Functional Configuration of Stepper Motor Controller
  */
typedef struct {
	uint8_t Step_Mode;			/*!< Controller Step Mode
                                 	 This parameter can be a value of @ref Controller_step_mode*/
	uint16_t Step_Freq_Max;		/*!< Controller Max Frequency
                                 This parameter can be a value of @ref CONTROLLER_freq_max*/
} CONTROLLER_FuncConfigTypeDef;

/**
  * @brief  Different Errors Types may occurs
  */
typedef enum {
	Wrong_Step_Mode,
	Wrong_Direction_Config,
	Wrong_Percentage_Of_Freq_Max
} CONTROLLER_Error;

/* Exported constants --------------------------------------------------------*/
/** @defgroup Controller_Exported_Constants Controller Exported Constants
  * @{
  */

/**
  * @}
  */

/** @defgroup CONTROLLER_step_mode Controller step mode
  * @brief Controller output step mode
  * @{
  */
#define  CONTROLLER_MODE_FULL_STEP      (0x00U)   /*!< Full step (2-phase excitation) */
#define  CONTROLLER_MODE_HALF_STEP     	(0x01U)   /*!< 1/2 step (1-2 phase excitation) */
#define  CONTROLLER_MODE_QUARTER_STEP   (0x02U)   /*!< 1/4 step (W1-2 phase excitation) */
#define  CONTROLLER_MODE_EIGHT_STEP  	(0x03U)   /*!< 1/8 step (phase excitation) */
/**
  * @}
  */

/** @defgroup CONTROLLER_step_dir Controller step direction
  * @brief Controller output step direction
  * @{
  */
#define  CONTROLLER_DIR_COUNTER_CLOCK_WISE		(0x00U)   /*!< Counter clock wise rotation */
#define  CONTROLLER_DIR_CLOCK_WISE     			(0x01U)   /*!< Clock wise rotation */

/**
  * @}
  */

/** @defgroup CONTROLLER_freq_max Controller max frequency
  * @brief Controller max frequency of the PWM signal
  * @{
  */
#define  CONTROLLER_FREQ_MAX_LOW			(0x03E8U)   /*!< Max frequency 500Hz */
#define  CONTROLLER_FREQ_MAX_MEDIUM    		(0x1388U)   /*!< Max frequency 5kHz */
#define  CONTROLLER_FREQ_MAX_HIGH			(0x2710U)	/*!< Max frequency 10kHz */
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup Controller_Exported_Functions Controller Exported Functions
  *  @brief    Controller Exported Functions
  * @{
  */

/**
  * @brief  Initialize the controller
  *
  * @param  hFuncConfig FuncConfig Handle
  * @param  hPerifConfig PerifConfig Handle
  *
  * @retval None
  */
void CONTROLLER_InitController(CONTROLLER_FuncConfigTypeDef * hFuncConfig, TIM_HandleTypeDef TimerController);

/**
  * @brief  Enable the controller
  *
  * @param  None
  * @retval None
  */
void CONTROLLER_Enable(void);

/**
  * @brief  Reset the controller
  *
  * @param  None
  * @retval None
  */
void CONTROLLER_Reset(void);

/**
  * @brief  Disable the controller
  *
  * @param  None
  * @retval None
  */
void CONTROLLER_Disable(void);

/**
  * @brief  Set the direction of the step output
  *
  * @param  Direction of the step to be set
  *         This parameter can be one of the following values:
  *            @arg CONTROLLER_DIR_COUNTER_CLOCK_WISE: Step in counter clock wise orientation
  *            @arg CONTROLLER_DIR_CLOCK_WISE: Step in clock wise orientation
  * @retval None
  */
void CONTROLLER_Set_DIR(uint8_t direction);

/**
  * @brief  Set the frequency of the step output
  *
  * @param  Percentage of the max frequency to be set
  *         This parameter must be between 0 and 100.
  * @retval None
  */
void CONTROLLER_Set_STEP_freq(uint8_t CONTROLLER_percentage);

/**
  * @brief  Set the frequency of the step output
  *
  * @param  None
  * @retval Percentage of the max frequency.
  */
uint16_t CONTROLLER_Get_STEP_freq(void);

void CONTROLLER_Error_Handler(void);

#endif
