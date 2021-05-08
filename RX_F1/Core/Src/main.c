/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_NRF24L01.h"
#include "ST_MPU6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define dt 0.004f
//#define __ESC_PROGRAMMING
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t ui8RBuffer[32];
uint8_t ui8RAck[32];
uint64_t RxpipeAddrs = 0xFF223344AA;
ACK_Packagae ackPac = ACK_ERROR_SENT;
//For PWM
uint16_t ui16Pwm[4];

//For IMU
MPU6050_AcceDataScaled mpuAcScaled;
MPU6050_GyroDataScaled mpuGyScaled;
MPU6050_AcceAxis mpuAcAxis;
MPU6050_GyroAxis mpuGyAxis;
MPU6050_GyroDataScaled mpuGyOffset;
MPU6050_AcceAxis mpuAcOffset;
uint8_t ui8Buffer[40];
// Variable for user
uint16_t ui16UserBuffer;
uint16_t ui16UserThurst = 1000;
float flUSerRoll, flUserPitch, flUserYaw;
// For System
float VBat = 12;																// Battery Voltage
uint16_t ui16VBat;
float flThrust1 , flThrust2, flThrust3, flThrust4;			// Speed of Motor

// Variable for PID Roll
float errorRc, errorRp;
float KpR = 3.0  , KiR = 1.5 , KdR = 0.5;
float RollP, RollI, RollD;
float RollPID;

//Variable for PID Pitch
float errorPc, errorPp;
float KpP = 3.0  , KiP = 0.9, KdP = 0.15;
float PitchP, PitchI, PitchD;
float PitchPID;

//Variable for PID Yaw
float errorYc, errorYp;
float KpY = 3, KiY = 0.1, KdY;
float YawP, YawI, YawD;
float YawPID;
//System State
CONTROL_State ctlState = CONTROL_STOP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
bool Compare_String (uint8_t* ui8String1, uint8_t* ui8String2, uint8_t ui8Size);
void ConvertString2Int (uint16_t* ui16Int, uint8_t* ui8String);
void RollPIDCal (void);
void AngleCal (void);
void PitchPIDCal (void);
void YawPIDCal (void);
void MotorThrust (void);
void CompareThrust (float* flValue);
void ConvertPID (float* flConstant, uint8_t* ui8Data);
void SetPID (float* flP, float* flI, float* flD);
void Baterry_Calculate (uint16_t *ui16AdcVal, float * Vbat);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  htim3.Instance->CCR1 = 1000;
  htim3.Instance->CCR2 = 1000;
  htim3.Instance->CCR3 = 1000;
  htim3.Instance->CCR4 = 1000;

  //Mpu 6050 Initialise
#ifndef __ESC_PROGRAMMING
  uint8_t i ;
  MPU6050_InitTypedef mpuInit;
  mpuInit.ui8AcceFullScale = MPU6050_ACCE_FULLSCALE_2G;
  mpuInit.ui8GyroFullScale = MPU6050_GYRO_FULLSCALE_500DPS;
  mpuInit.ui8DLPF 		   = MPU6050_DLPF_4;
  MPU6050_Init(&mpuInit, &hi2c1);
  for (i = 0; i < 10; i++)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  if(HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0)
	  {
		  MPU6050_Calib(&mpuAcAxis, &mpuGyOffset, 100);
		  break;
	  }
	  HAL_Delay(300);
  }
  if(mpuGyOffset.x == 0)
  {
	  mpuGyOffset.x = 0.754504144;
	  mpuGyOffset.y = -2.73450375;
	  mpuGyOffset.z = -0.113435157;
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
  }
#endif

  //NRF24 - L01 Initialise
  NRF24_begin(CSN_GPIO_Port, CSN_Pin, CE_Pin, hspi2);
  nrf24_DebugUART_Init(huart1);
  NRF24_setAutoAck(true);
  NRF24_setChannel(11);
  NRF24_setPayloadSize(32);
  NRF24_stopListening();
  NRF24_openReadingPipe(0, RxpipeAddrs);
  NRF24_openWritingPipe(RxpipeAddrs);
  NRF24_setAutoAck(true);
  NRF24_enableAckPayload();
  NRF24_enableDynamicPayloads();
//  NRF24_setPALevel(RF24_PA_m6dB);
  NRF24_startListening();
  printRadioSettings();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(5);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ui16VBat, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 48000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|LED1_Pin|LED2_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin LED1_Pin LED2_Pin CSN_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LED1_Pin|LED2_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CE_Pin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		/* Angle Calculate */
		AngleCal();

		/* Roll PID calculate */
		RollPIDCal();

		/* Pitch PID calculate */
		PitchPIDCal();

		/* Yaw PID calculate */
		YawPIDCal();

		/* Compile 3 PID for 4 Motor Channel */
		MotorThrust();
		/* Battery voltage */
		Baterry_Calculate(&ui16VBat, &VBat);
	}
	if(htim->Instance == htim2.Instance)
	{
		static uint16_t i = 0;
		static uint16_t count = 0;
		i++;							// This value is used to detect lost connection
		//Prepare String for ACK
		if(NRF24_available())			//Read Data Payload
		{
			i = 0;
			memset(ui8RBuffer, '\0', sizeof(ui8RBuffer));
			NRF24_read(ui8RBuffer, 32);
			memset(ui8RAck, '\0', sizeof(ui8RAck));
			sprintf((char*)ui8RAck, "ACKR%05.1fP%05.1fY%05.0fB%04.1f", mpuGyAxis.roll, mpuGyAxis.pitch, mpuGyAxis.yaw, VBat);
			NRF24_writeAckPayload(0, ui8RAck, 32);
			if(Compare_String(ui8RBuffer, (uint8_t*)"CMD", 3))			// Read CMD data
			{
				if(ui8RBuffer[3] == 'T')
				{
					ConvertString2Int(&ui16UserBuffer, &ui8RBuffer[4]);
					ui16UserThurst = ui16UserBuffer;
#ifndef __ESC_PROGRAMMING
					if(ui16UserThurst > 1600)
						ui16UserThurst = 1600;
#endif
				}
				if(ui8RBuffer[8] == 'R')
				{
					ConvertString2Int(&ui16UserBuffer, &ui8RBuffer[9]);
					flUSerRoll = flUSerRoll*0.7 +  0.3*((float)ui16UserBuffer - 1500)*20/500;
					if((flUSerRoll > -0.5) && (flUSerRoll < 0.5) )
					{
						flUSerRoll = 0;
					}
					if(ui16UserThurst < 1030)
						flUSerRoll = 0;
				}
				if(ui8RBuffer[13] == 'P')
				{
					ConvertString2Int(&ui16UserBuffer, &ui8RBuffer[14]);
					flUserPitch =flUserPitch*0.7 - 0.3*((float)ui16UserBuffer - 1500)*20/500;
					if((flUserPitch > -0.5) && (flUserPitch < 0.5) )
					{
						flUserPitch = 0;
					}
					if(ui16UserThurst < 1030)
						flUserPitch = 0;
				}
				if(ui8RBuffer[18] == 'Y')
				{
					float buf;
					ConvertString2Int(&ui16UserBuffer, &ui8RBuffer[19]);
					buf = ((float)ui16UserBuffer - 1500)*2/500;
					if((buf > -0.2) && (buf < 0.2) )
					{
						buf = 0;
					}
					flUserYaw -= buf;
					if(ui16UserThurst < 1030)
						flUserYaw = 0;
				}
			}
			if(Compare_String(ui8RBuffer, (uint8_t*)"PID", 3))				// Read PID setup
			{
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				switch (ui8RBuffer[3])
				{
					case '1':
						SetPID(&KpR, &KiR, &KdR);
						break;
					case '2':
						SetPID(&KpP, &KiP, &KdP);
						break;
					case '3':
						SetPID(&KpY, &KiY, &KdY);
					break;
					default:
						break;
				}
			}
			if(Compare_String(ui8RBuffer, (uint8_t*)"CTL", 3))
			{
				switch (ui8RBuffer[3])
				{
					case 'S':
						if(Compare_String(&ui8RBuffer[4],(uint8_t*) "START", 4))
						{
							ctlState = CONTROL_READY;
						}
						if(Compare_String(&ui8RBuffer[4], (uint8_t*)"STOP", 3))
						{
							ctlState = CONTROL_STOP;
						}
						break;
					default:
						break;
				}
			}
		}
		if(ui16UserThurst < 1020)
		{
			count++;
			if(count > 40)
			{
				count = 0;
				RollI = 0;
				PitchI = 0;
				YawI = 0;
				mpuGyAxis.yaw = 0;
				mpuGyAxis.pitch = mpuAcAxis.pitch;
				mpuGyAxis.roll = mpuAcAxis.roll;
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			}
		}
		else
		{
			count = 0;
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		}
		if(i > 20)			// Lost Connection with TX
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			// Set User Thrust, Pitch, Roll and Yaw Angle to Default
			ui16UserThurst = 1000;
			flUSerRoll = 0;
			flUserPitch = 0;
			flUserYaw = 0;
		}
	}
	}

bool Compare_String (uint8_t* ui8String1, uint8_t* ui8String2, uint8_t ui8Size)
{
	uint8_t i;
	for (i = 0; i < ui8Size; i++)
	{
		if(*ui8String1 != *ui8String2)
		{
			return false;
		}
		ui8String1++;
		ui8String2++;
	}
	return true;
	}
void ConvertString2Int (uint16_t* ui16Int, uint8_t* ui8String)
{
	uint8_t ui8Buffer[4];
	uint8_t i;
	for(i = 0; i < 4; i++)
	{
		ui8Buffer[i] = *ui8String - 0x30;
		ui8String++;
	}
	*ui16Int = ui8Buffer[0]*1000 + ui8Buffer[1]*100 + ui8Buffer[2]*10 + ui8Buffer[3];

	}
void AngleCal (void)
{
	mpuAcScaled = MPU6050_AcceRead_Scaled();
	mpuGyScaled = MPU6050_GyroRead_Scaled();
    mpuGyScaled.x -= mpuGyOffset.x;
    mpuGyScaled.y -= mpuGyOffset.y;
    mpuGyScaled.z -= mpuGyOffset.z;

    mpuAcAxis.pitch = -180* atan2f(mpuAcScaled.x, sqrt(mpuAcScaled.y*mpuAcScaled.y + mpuAcScaled.z*mpuAcScaled.z))/M_PI;
    mpuAcAxis.roll = 180* atan2f(mpuAcScaled.y, sqrt(mpuAcScaled.x*mpuAcScaled.x + mpuAcScaled.z*mpuAcScaled.z))/M_PI;

    mpuAcAxis.pitch -= mpuAcOffset.pitch;
    mpuAcAxis.roll -= mpuAcOffset.roll;

    mpuGyAxis.roll += mpuGyScaled.x*dt;
    mpuGyAxis.pitch += mpuGyScaled.y*dt;
    mpuGyAxis.yaw += mpuGyScaled.z*dt;

    mpuGyAxis.pitch = mpuGyAxis.pitch*0.9996 + 0.0004*mpuAcAxis.pitch;
    mpuGyAxis.roll = mpuGyAxis.roll*0.9996 + 0.0004*mpuAcAxis.roll;
	}
void RollPIDCal (void)
{
	errorRc = flUSerRoll - mpuGyAxis.roll;
	RollP = KpR*errorRc;
	RollI += KiR*(errorRc + errorRp)/2*dt;
	RollD = KdR*(errorRc - errorRp)/dt;
	RollPID = RollP + RollI + RollD;
	if (RollPID > 400)
		RollPID = 400;
	if(RollPID < -400)
		RollPID = -400;
	errorRp = errorRc;
	}
void PitchPIDCal (void)
{
	errorPc = flUserPitch - mpuGyAxis.pitch;
	PitchP = KpP*errorPc;
	PitchI += KiP*(errorPc + errorPp)/2*dt;
	PitchD = KiP*(errorPc - errorPp)/dt;
	PitchPID = PitchP + PitchI + PitchD;
	if (PitchPID > 400)
		PitchPID = 400;
	if(PitchPID < -400)
		PitchPID = -400;
	errorPp = errorPc;
	}
void YawPIDCal (void)
{
	errorYc = flUserYaw - mpuGyAxis.yaw;
	YawP = KpY*errorYc;
	YawI += KiY*(errorYc + errorYp)/2*dt;
	YawD = KdY*(errorYc - errorYp)/dt;
	YawPID = YawP + YawI + YawD;
	if (YawPID > 300)
		YawPID = 300;
	if(YawPID <-300)
		YawPID = -300;
	errorYp = errorYc;
	}
void MotorThrust (void)
{
	flThrust1 = ui16UserThurst - RollPID + PitchPID + YawPID;
	flThrust2 = ui16UserThurst + RollPID + PitchPID - YawPID;
	flThrust3 = ui16UserThurst + RollPID - PitchPID + YawPID;
	flThrust4 = ui16UserThurst - RollPID - PitchPID - YawPID;
	if(ctlState == CONTROL_STOP)
	{
		flThrust1 = 1000;
		flThrust2 = 1000;
		flThrust3 = 1000;
		flThrust4 = 1000;
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	}
	else
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	}
	if(ui16UserThurst < 1010)
	{
		flThrust1 = 1000;
		flThrust2 = 1000;
		flThrust3 = 1000;
		flThrust4 = 1000;
	}
	else
	{
#ifndef  __ESC_PROGRAMMING
	CompareThrust(&flThrust1);
	CompareThrust(&flThrust2);
	CompareThrust(&flThrust3);
	CompareThrust(&flThrust4);
#else
	if(ui16UserThurst > 1950)
	{
		flThrust1 = 2000;
		flThrust2 = 2000;
		flThrust3 = 2000;
		flThrust4 = 2000;
	}
#endif
	}
	htim3.Instance->CCR1 = (uint16_t)flThrust1;
	htim3.Instance->CCR2 = (uint16_t)flThrust2;
	htim3.Instance->CCR3 = (uint16_t)flThrust3;
	htim3.Instance->CCR4 = (uint16_t)flThrust4;
//	htim3.Instance->CCR1 = ui16UserThurst;

	}
void CompareThrust (float* flValue)
{
	if (*flValue > 1700)
		*flValue  = 1700;
	if (*flValue  < 1100)
		*flValue = 1100;
	}
void ConvertPID (float* flConstant, uint8_t* ui8Data)
{
	*flConstant = (float)((*ui8Data) - 0x30)*10 + (float)(*(ui8Data + 1) - 0x30) + (float)(*(ui8Data + 3) - 0x30)/10 + (float)(*(ui8Data + 4) - 0x30)/100 + (float)(*(ui8Data + 5) - 0x30)/1000;
	}
void SetPID (float* flP, float* flI, float* flD)
{
	if(ui8RBuffer[4] == 'P')
	{
		ConvertPID(flP, &ui8RBuffer[5]);
	}
	if(ui8RBuffer[11] == 'I')
	{
		ConvertPID(flI, &ui8RBuffer[12]);
	}
	if(ui8RBuffer[18] == 'D')
	{
		ConvertPID(flD, &ui8RBuffer[19]);
	}
	}
void Baterry_Calculate (uint16_t *ui16AdcVal, float * Vbat)
{
	float Vbatbuf;
	Vbatbuf = (float)(*ui16AdcVal + 70)*3.268/4095*11 + 0.7;
	*Vbat = *Vbat*0.7 + Vbatbuf*0.3;
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
