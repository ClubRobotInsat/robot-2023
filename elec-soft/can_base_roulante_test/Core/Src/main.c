/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MOTOR_DC/motor_dc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_1_TIM_CHANNEL TIM_CHANNEL_1
#define MOTOR_2_TIM_CHANNEL TIM_CHANNEL_2
#define MOTOR_3_TIM_CHANNEL TIM_CHANNEL_3
#define MOTOR_4_TIM_CHANNEL TIM_CHANNEL_4

/*
 * CAN ID for this module: 2 (Base roulante)
 *
 * 0 : urgency EMERGENCY
 * 1 : raspy
 * 2 : base roulante (Left + Right)
 * 3 : base roulante 2 (Front + Rear)
 * 4 : bras
 * 5 : stockage
 * 6 : etc...
 */
#define CAN_ID_SRC 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef rxHeader; /* The rxHeader does not need to be constructed , it is only filled in when Rx messages are read */
FDCAN_TxHeaderTypeDef txHeader;
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};
uint8_t canTX[8] = {1,2,3,4,5,6,7,8};

FDCAN_FilterTypeDef canfil;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*  Commands:
	| ID commande | 1st param | Opt params | Description |
	| :--: | :--: | :--: | :-- |
	| 0 | | | stop() |
	| 1 | | | ping() |
	| 2 | idMot | speed | setSpeed(idMot, speed) |
	| 3 | idMot | direction | setMotorDirection(idMot, direction) |
	| 4 | idMot | | getPosition(idMotor) |
	| 5 | idMot | position | getPositionACK(idMot, position) |
*/

/*
 * 3 bits priority
 * 4 bits id_dest
 * 4 bits id_src
 */
void CAN_make_header(uint8_t priority, uint8_t id_dest, uint8_t id_src) {
	uint32_t identifier;
	identifier = ((priority & 0x07) << 8) | ((id_dest & 0x0F) << 4) | (id_src & 0x0F);
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

void CAN_Send(uint8_t* data, uint8_t priority, uint8_t id_dest) {
	CAN_make_header(priority, id_dest, CAN_ID_SRC);

	for(int i=0; i < 8; i++)
		canTX[i] = data[i];

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, canTX) != HAL_OK)
	{
		Error_Handler();
	}

}

uint8_t CAN_decode_id_src() {
	return (rxHeader.Identifier & 0x0F);
}

void sendBackPing() {
	uint8_t data[8] = {1,0,0,0,0,0,0,0};
	CAN_Send(data, 1, CAN_decode_id_src());
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	Motor_Config motor_L;
	Motor_Config motor_R;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  /* Make function to easily create Identifier */
  /* Make function to decode Identifier */
  txHeader.Identifier = 0x11;

    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;



	motor_L = Motor_Init(dir_motor_l_GPIO_Port, dir_motor_l_Pin, &htim2, TIM_CHANNEL_1);
	motor_R = Motor_Init(dir_motor_r_GPIO_Port, dir_motor_r_Pin, &htim2, TIM_CHANNEL_2);
	Motor_Set_Direction(motor_L, MOTOR_DIRECTION_CW);
	Motor_Set_Direction(motor_R, MOTOR_DIRECTION_CW); /* This works when I change CW to CCW , the oscilloscope displays different voltages */
	Motor_Start(motor_L);
	Motor_Start(motor_R);




	HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&rxHeader,canRX) != HAL_OK)
	  {
		  if (hfdcan1.ErrorCode == HAL_FDCAN_ERROR_FIFO_EMPTY)
		  {

		  }
		  else
		  {
			  Error_Handler(); /* When the RxFifo is empty, an error is returned by the HAL */
		  }
	  } else {
		  switch (canRX[0]) {
			  case 4: // getPosition(idMotor)
				  if(canRX[1] == 0) { // LEFT
					  uint8_t speed = Motor_Get_Speed(motor_L);
					  uint8_t data4[8] = {5,0,speed,0,0,0,0,0};
					  CAN_Send(data4, 1, CAN_decode_id_src());
				  } else if(canRX[1] == 1) {
					  uint8_t speed = Motor_Get_Speed(motor_R);
					  uint8_t data4[8] = {5,1,speed,0,0,0,0,0};
					  CAN_Send(data4, 1, CAN_decode_id_src());
				  }
				  break;
			  case 3: //setMotorDirection(idMot, direction)

				  if(canRX[1] == 0) { // LEFT
					  if(canRX[2] == 0) { // CW
						  Motor_Set_Direction(motor_L, MOTOR_DIRECTION_CW);

					  } else if(canRX[2] == 1) { // CCW
						  Motor_Set_Direction(motor_L, MOTOR_DIRECTION_CCW);
					  }
				  } else if(canRX[1] == 1) { // RIGHT
					  if(canRX[2] == 0) { // CW
						  Motor_Set_Direction(motor_R, MOTOR_DIRECTION_CW);
					  } else if(canRX[2] == 1) { // CCW
						  Motor_Set_Direction(motor_R, MOTOR_DIRECTION_CCW);
					  }
				  }
				  uint8_t data3[8] = {canRX[0],canRX[1],canRX[2],1,1,1,1,1};
				  CAN_Send(data3, 1, CAN_decode_id_src());


				  break;
			  case 2: // setSpeed(idMot, speed)
				  if(canRX[1] == 0) { // LEFT
					  uint8_t speed = canRX[2];
					  Motor_Set_Speed(motor_L, speed);
				  } else if(canRX[1] == 1) { // RIGHT
					  uint8_t speed = canRX[2];
					  Motor_Set_Speed(motor_R, speed);
				  }
				  uint8_t data2[8] = {canRX[0],canRX[1],canRX[2],1,1,1,1,1};
				  CAN_Send(data2, 1, CAN_decode_id_src());

				  break;
			  case 1: // PING
				  sendBackPing();
				  break;
			  case 0: // STOP
				  Motor_Set_Speed(motor_L, 0);
				  Motor_Set_Speed(motor_R, 0);
				  uint8_t data0[8] = {canRX[0],canRX[1],canRX[2],1,1,1,1,1};
				  CAN_Send(data0, 1, CAN_decode_id_src());
			  default:
				  break;

		  }
		  HAL_Delay(10);
	  }





  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 3;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  	/* Configurer le filtre global avant de configurer le filtre canfil , sinon le filtre rejette tout */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT,FDCAN_REJECT, FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE) != HAL_OK)
      {
      	Error_Handler();
      }


    canfil.IdType = FDCAN_STANDARD_ID;
    canfil.FilterIndex = 0;
    canfil.FilterType = FDCAN_FILTER_MASK;
    canfil.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    canfil.FilterID1 = 0x13;
    canfil.FilterID2 = 0x13;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &canfil) != HAL_OK)
    {
      /* Filter configuration Error */
      Error_Handler();
    }


    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      Error_Handler();
    }

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, dir_motor_l_Pin|dir_motor_r_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : dir_motor_l_Pin dir_motor_r_Pin */
  GPIO_InitStruct.Pin = dir_motor_l_Pin|dir_motor_r_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
