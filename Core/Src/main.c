///* USER CODE BEGIN Header */
/////////////**
////////////  ******************************************************************************
////////////  * @file           : main.c
////////////  * @brief          : Main program body
////////////  ******************************************************************************
////////////  * @attention
////////////  *
////////////  * Copyright (c) 2024 STMicroelectronics.
////////////  * All rights reserved.
////////////  *
////////////  * This software is licensed under terms that can be found in the LICENSE file
////////////  * in the root directory of this software component.
////////////  * If no LICENSE file comes with this software, it is provided AS-IS.
////////////  *
////////////  ******************************************************************************
////////////  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
////////////
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
////////////
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
////////////
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
// CAN_HandleTypeDef hcan1;
//
//I2C_HandleTypeDef hi2c1;
//
//IWDG_HandleTypeDef hiwdg;
//
//SPI_HandleTypeDef hspi2;
//
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//
//UART_HandleTypeDef huart2;
//UART_HandleTypeDef huart3;
//
//WWDG_HandleTypeDef hwwdg;
//
///* USER CODE BEGIN PV */
////////////
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_CAN1_Init(void);
//static void MX_I2C1_Init(void);
//static void MX_IWDG_Init(void);
//static void MX_SPI2_Init(void);
//static void MX_USART2_UART_Init(void);
//static void MX_WWDG_Init(void);
//static void MX_USART3_UART_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_TIM2_Init(void);
///* USER CODE BEGIN PFP */
////////////
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
////////////
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
////////////
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
////////////
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
////////////
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_CAN1_Init();
//  MX_I2C1_Init();
//  MX_IWDG_Init();
//  MX_SPI2_Init();
//  MX_USART2_UART_Init();
//  MX_WWDG_Init();
//  MX_USART3_UART_Init();
//  MX_TIM1_Init();
//  MX_TIM2_Init();
//  /* USER CODE BEGIN 2 */
////////////
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
////////////  while (1)
////////////  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
////////////  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
//                              |RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
//}
//
///**
//  * @brief CAN1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_CAN1_Init(void)
//{
//
//  /* USER CODE BEGIN CAN1_Init 0 */
////////////
//  /* USER CODE END CAN1_Init 0 */
//
//  /* USER CODE BEGIN CAN1_Init 1 */
////////////
//  /* USER CODE END CAN1_Init 1 */
//  hcan1.Instance = CAN1;
//  hcan1.Init.Prescaler = 16;
//  hcan1.Init.Mode = CAN_MODE_NORMAL;
//  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
//  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
//  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
//  hcan1.Init.TimeTriggeredMode = DISABLE;
//  hcan1.Init.AutoBusOff = DISABLE;
//  hcan1.Init.AutoWakeUp = DISABLE;
//  hcan1.Init.AutoRetransmission = DISABLE;
//  hcan1.Init.ReceiveFifoLocked = DISABLE;
//  hcan1.Init.TransmitFifoPriority = DISABLE;
//  if (HAL_CAN_Init(&hcan1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN CAN1_Init 2 */
////////////
//  /* USER CODE END CAN1_Init 2 */
//
//}
//
///**
//  * @brief I2C1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_I2C1_Init(void)
//{
//
//  /* USER CODE BEGIN I2C1_Init 0 */
////////////
//  /* USER CODE END I2C1_Init 0 */
//
//  /* USER CODE BEGIN I2C1_Init 1 */
////////////
//  /* USER CODE END I2C1_Init 1 */
//  hi2c1.Instance = I2C1;
//  hi2c1.Init.ClockSpeed = 400000;
//  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C1_Init 2 */
////////////
//  /* USER CODE END I2C1_Init 2 */
//
//}
//
///**
//  * @brief IWDG Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_IWDG_Init(void)
//{
//
//  /* USER CODE BEGIN IWDG_Init 0 */
////////////
//  /* USER CODE END IWDG_Init 0 */
//
//  /* USER CODE BEGIN IWDG_Init 1 */
////////////
//  /* USER CODE END IWDG_Init 1 */
//  hiwdg.Instance = IWDG;
//  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
//  hiwdg.Init.Reload = 4095;
//  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN IWDG_Init 2 */
////////////
//  /* USER CODE END IWDG_Init 2 */
//
//}
//
///**
//  * @brief SPI2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_SPI2_Init(void)
//{
//
//  /* USER CODE BEGIN SPI2_Init 0 */
////////////
//  /* USER CODE END SPI2_Init 0 */
//
//  /* USER CODE BEGIN SPI2_Init 1 */
////////////
//  /* USER CODE END SPI2_Init 1 */
//  /* SPI2 parameter configuration*/
//  hspi2.Instance = SPI2;
//  hspi2.Init.Mode = SPI_MODE_SLAVE;
//  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
//  hspi2.Init.NSS = SPI_NSS_SOFT;
//  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
//  hspi2.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI2_Init 2 */
////////////
//  /* USER CODE END SPI2_Init 2 */
//
//}
//
///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM1_Init(void)
//{
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//////////
//  /* USER CODE END TIM1_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//////////
//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 24;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
//  htim1.Init.Period = 260;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 255;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 260;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */
//////////
//  /* USER CODE END TIM1_Init 2 */
//  HAL_TIM_MspPostInit(&htim1);
//
//}
//
///**
//  * @brief TIM2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
//////////
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
//////////
//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 24;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 4294967295;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM2_Init 2 */
//////////
//  /* USER CODE END TIM2_Init 2 */
//
//}
//
///**
//  * @brief USART2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
////////////
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
////////////
//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 115200;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
////////////
//  /* USER CODE END USART2_Init 2 */
//
//}
//
///**
//  * @brief USART3 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART3_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART3_Init 0 */
////////////
//  /* USER CODE END USART3_Init 0 */
//
//  /* USER CODE BEGIN USART3_Init 1 */
////////////
//  /* USER CODE END USART3_Init 1 */
//  huart3.Instance = USART3;
//  huart3.Init.BaudRate = 115200;
//  huart3.Init.WordLength = UART_WORDLENGTH_9B;
//  huart3.Init.StopBits = UART_STOPBITS_1;
//  huart3.Init.Parity = UART_PARITY_NONE;
//  huart3.Init.Mode = UART_MODE_TX_RX;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART3_Init 2 */
////////////
//  /* USER CODE END USART3_Init 2 */
//
//}
//
///**
//  * @brief WWDG Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_WWDG_Init(void)
//{
//
//  /* USER CODE BEGIN WWDG_Init 0 */
////////////
//  /* USER CODE END WWDG_Init 0 */
//
//  /* USER CODE BEGIN WWDG_Init 1 */
////////////
//  /* USER CODE END WWDG_Init 1 */
//  hwwdg.Instance = WWDG;
//  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
//  hwwdg.Init.Window = 87;
//  hwwdg.Init.Counter = 125;
//  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
//  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN WWDG_Init 2 */
////////////
//  /* USER CODE END WWDG_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, STEPPER_1_PUL_Pin|STEPPER_1_DIR_Pin, GPIO_PIN_SET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, STEPPER_0_DIR_Pin|STEPPER_1_ENA_Pin|STEPPER_0_PUL_Pin|STEPPER_0_POWER_RELAY_Pin
//                          |STEPPER_1_POWER_RELAY_Pin, GPIO_PIN_SET);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PC0 PC3 PC4 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PA1 PA4 PA15 */
//  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LD2_Pin */
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : STEPPER_1_PUL_Pin */
//  GPIO_InitStruct.Pin = STEPPER_1_PUL_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  HAL_GPIO_Init(STEPPER_1_PUL_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : STEPPER_1_DIR_Pin */
//  GPIO_InitStruct.Pin = STEPPER_1_DIR_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(STEPPER_1_DIR_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PC5 STEPPER_0_POWER_RELAY_Pin STEPPER_1_POWER_RELAY_Pin */
//  GPIO_InitStruct.Pin = GPIO_PIN_5|STEPPER_0_POWER_RELAY_Pin|STEPPER_1_POWER_RELAY_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PB0 PB1 PB2 PB12
//                           PB14 PB15 PB4 PB5
//                           PB9 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
//                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5
//                          |GPIO_PIN_9;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : STEPPER_0_DIR_Pin STEPPER_1_ENA_Pin */
//  GPIO_InitStruct.Pin = STEPPER_0_DIR_Pin|STEPPER_1_ENA_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : STEPPER_0_PUL_Pin */
//  GPIO_InitStruct.Pin = STEPPER_0_PUL_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  HAL_GPIO_Init(STEPPER_0_PUL_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : STEPPER_0_ALM_Pin */
//  GPIO_InitStruct.Pin = STEPPER_0_ALM_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(STEPPER_0_ALM_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PA8 */
//  GPIO_InitStruct.Pin = GPIO_PIN_8;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : STEPPER_1_ALM_Pin */
//  GPIO_InitStruct.Pin = STEPPER_1_ALM_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(STEPPER_1_ALM_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PD2 */
//  GPIO_InitStruct.Pin = GPIO_PIN_2;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//}
//
///* USER CODE BEGIN 4 */
////////////
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
////////////  /* User can add his own implementation to report the HAL error return state */
////////////  __disable_irq();
////////////  while (1)
////////////  {
////////////  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
////////////  /* User can add his own implementation to report the file name and line number,
////////////     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
