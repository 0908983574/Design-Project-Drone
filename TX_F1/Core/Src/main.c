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
#include "stdio.h"
#include "stdbool.h"
#include "MY_NRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define __TUNE_MATLAB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t ui8Rbuffer[32];
uint16_t ui16AdcVal[4];
uint16_t ui16Pwm[4] ={1500, 1500, 1500, 1000};
uint8_t ui8RAck[32];

uint64_t RxpipeAddrs = 0xFF223344AA;
uint8_t buffer[32];
uint8_t myRxData[32];

//Variable for USART
uint8_t ui8UartBuff[32];

//For Tune PID
float KpR, KiR, KdR;
float KpP, KiP, KdP;
float KpY, KiY, KdY;

//State of System
NRF_State nrfState;
bool PIDtune = true;
//For Adc
VariRes_Value VrOffset;
//For Motor
Motor_State MotorState;
//For Buzzer State
Buzzer_State Buzzer = BUZZER_OFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
bool Compare_String (uint8_t* ui8String1, uint8_t* ui8String2, uint8_t size);		//Return 1 if we are equal and 0 if not
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void ConvertPID (float* flConstant, uint8_t* ui8Data);
void SetPID (uint8_t ui8char, float* flP, float* flI, float* flD);
void Varires_Calib (VariRes_Value* vrOffset, uint16_t* AdcDMA, uint16_t ui16Times);
void Calculate_PWM (uint16_t* ui16PWM,uint16_t* ui16Adc, VariRes_Value* VrOffsetValue);
void Motor_State_Set (Motor_State* motorState, uint16_t* ui16PWM);
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	NRF24_begin(GPIOA, CSN_Pin, CE_Pin, hspi2);
	nrf24_DebugUART_Init(huart1);
	NRF24_stopListening();
	NRF24_openReadingPipe(0, RxpipeAddrs);
	NRF24_openWritingPipe(RxpipeAddrs);
	NRF24_setAutoAck(true);
	NRF24_setChannel(11);
	NRF24_setPayloadSize(32);
	NRF24_setPALevel(RF24_PA_m6dB);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();
//	printRadioSettings();



	//Adc setup
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ui16AdcVal, 4);
	HAL_UART_Receive_DMA(&huart1, ui8UartBuff, 25);
	Varires_Calib(&VrOffset, ui16AdcVal, 100);

	HAL_Delay(5);
	//Timer Initialise

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);			//Buzzer Control Timer
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);

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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000-1;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 144-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, LED_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CE_Pin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSN_Pin */
  GPIO_InitStruct.Pin = CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		uint8_t ui8buf[34];
		static uint8_t i = 0;
		//Calculate Adc value
		 Calculate_PWM(ui16Pwm, ui16AdcVal, &VrOffset);

		//Change Motor State
		Motor_State_Set(&MotorState, ui16Pwm);
		//Package information
		if(nrfState == NRF_CMD)
		{
		if(MotorState == MOTOR_START)
		{
			sprintf((char*)ui8Rbuffer, "CMDT%dR%dP%dY%d", ui16Pwm[3], ui16Pwm[1], ui16Pwm[0], ui16Pwm[2]);
		}
		if(MotorState == MOTOR_STOP)
		{
			sprintf((char*)ui8Rbuffer, "CMDT%dR%dP%dY%d", 1000, 1500, 1500, 1500);
		}
		}


		if(NRF24_write(ui8Rbuffer, 32))
		{
			NRF24_read(ui8RAck, 32);
			if(Compare_String(ui8RAck,(uint8_t*) "ACK", 3))
			{
#ifndef __TUNE_MATLAB
				HAL_UART_Transmit(&huart1, ui8RAck, 32, 50);
#else
				if(PIDtune == true)
				{
				memset(ui8buf, '\0', sizeof(ui8buf));
				sprintf((char*)ui8buf, "%s\r\n", ui8RAck);
				HAL_UART_Transmit(&huart1, ui8buf, 34, 50);
				}

#endif
				if(ui8RAck[21] == 'B')
				{
					float BatBuf;
					BatBuf = (ui8RAck[22] - 0x30)*10 + (ui8RAck[23] - 0x30) + (float)(ui8RAck[25] - 0x30)/10;
					if((BatBuf >= 10.0) || (MotorState == MOTOR_STOP))
					{
						Buzzer = BUZZER_OFF;
					}
					if((BatBuf < 10.0) && (MotorState == MOTOR_START))
					{
						Buzzer = BUZZER_FLASHBEEP;
					}
				}
			}
			if(nrfState == NRF_PID)
			{
				//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					nrfState = NRF_CMD;
#ifdef __TUNE_MATLAB
					HAL_UART_Transmit(&huart1, (uint8_t*)"PIDOK\r\n", sizeof("PIDOK\r\n"), 10);
					PIDtune = true;
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif
			}
			if(nrfState == NRF_CTL)
			{
				nrfState = NRF_CMD;
			}
		}
		i++;
#ifdef __TUNE_MATLAB
		if(i > 20)
		{

		if(Compare_String(ui8UartBuff, (uint8_t*)"PID", 3))
		{
			memset(ui8Rbuffer, '\0', sizeof(ui8Rbuffer));
			sprintf((char*)ui8Rbuffer,"%s", ui8UartBuff);
			memset(ui8UartBuff, '\0', sizeof(ui8UartBuff));
			nrfState = NRF_PID;


		}
		i = 0;
		}
#endif
	}
	if(htim->Instance == htim3.Instance)
	{
		static uint8_t ui8long = 0;
		if(Buzzer == BUZZER_OFF)
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
			ui8long = 0;
		}
		else
		{
			switch (Buzzer)
			{
				case BUZZER_BEEP:
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
					Buzzer = BUZZER_OFF;
					break;
				case BUZZER_LONGBEEP:
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
					ui8long ++;
					if(ui8long == 5)
					{
						ui8long = 0;
						Buzzer = BUZZER_OFF;
					}
					break;
				case BUZZER_FLASHBEEP:
					if(ui8long == 0)
					{
						HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
					}
					if(ui8long == 2)
					{
						HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
					}
					ui8long++;
					if(ui8long > 10)
						ui8long = 0;

					break;
				default:
					break;
			}

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
void ConvertPID (float* flConstant, uint8_t* ui8Data)
{
	*flConstant = (float)((*ui8Data) - 0x30)*10 + (float)(*(ui8Data + 1) - 0x30) + (float)(*(ui8Data + 3) - 0x30)/10 + (float)(*(ui8Data + 4) - 0x30)/100;
	}

void Varires_Calib (VariRes_Value* vrOffset, uint16_t* AdcDMA, uint16_t ui16Times)
{
	uint16_t i;
	uint32_t buffer[4];
	for(i = 0; i < 4; i++)
	{
		buffer[i] = 0x00;
	}
	for(i = 0; i < ui16Times; i++)
	{
		buffer[0] += *AdcDMA;
		buffer[1] += *(AdcDMA + 1);
		buffer[2] += *(AdcDMA + 2);
		buffer[3] += *(AdcDMA + 3);
		HAL_Delay(20);
		/* USER CODE BEGIN */
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		/* USER CODE End */
	}
	vrOffset->Adc1 = buffer[0]/ui16Times;
	vrOffset->Adc2 = buffer[1]/ui16Times;
	vrOffset->Adc3 = buffer[2]/ui16Times;
	vrOffset->Adc4 = buffer[3]/ui16Times;
	}
void Calculate_PWM (uint16_t* ui16PWM,uint16_t* ui16Adc, VariRes_Value* VrOffsetValue)
{
	uint8_t i;
	//Calculate Adc value
		*ui16PWM = (*ui16Adc - VrOffsetValue->Adc1 + 2048)*1000/4095 + 1000;
		*(ui16PWM + 1)= (*(ui16Adc + 1) - VrOffsetValue->Adc2 + 2048)*1000/4095 + 1000;
		*(ui16PWM + 2)= (*(ui16Adc + 2) - VrOffsetValue->Adc3 + 2048)*1000/4095 + 1000;
		*(ui16PWM + 3)= (*(ui16Adc + 3) - VrOffsetValue->Adc4 + 0)*1000/4095 + 1000;
	for (i = 0; i < 4; i++)
	{
		if(*(ui16PWM + i) < 1000)
			*(ui16PWM + i) = 1000;
		if(*(ui16PWM + i) > 2000)
			*(ui16PWM + i) = 2000;
	}
	}
void Motor_State_Set (Motor_State* motorState, uint16_t* ui16PWM)
{
	static uint8_t i =0;
	if(*motorState == MOTOR_STOP)
	{
		if((*(ui16PWM + 3) < 1020) && (*(ui16PWM + 2) < 1070) && (*(ui16PWM + 0) < 1080) && (*(ui16PWM + 1) > 1910) )
		{
			i++;
		}
		if(i > 15)
		{
			i = 0;
			*motorState = MOTOR_START;
			sprintf((char*)ui8Rbuffer, "CTLSSTART");
			nrfState = NRF_CTL;
			Buzzer = BUZZER_LONGBEEP;
		}
	}
	if(*motorState == MOTOR_START)
	{
		if(*(ui16PWM + 3) < 1020)
		{
		if((*(ui16PWM + 3) < 1020) && (*(ui16PWM + 2) > 1930) && (*(ui16PWM + 0) < 1080) && (*(ui16PWM + 1) < 1090) )
		{
			i++;
		}
		if(i > 15)
		{
			i = 0;
			*motorState = MOTOR_STOP;
			Buzzer = BUZZER_LONGBEEP;
			sprintf((char*)ui8Rbuffer, "CTLSSTOP ");
			nrfState = NRF_CTL;
		}
		}
	}


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
