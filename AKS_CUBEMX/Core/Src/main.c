/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t BTN_States = 0;
uint8_t emergency_check_deadman = 0;
uint8_t emergency_check_can = 0;

uint16_t BTN1;
uint16_t BTN2;
uint16_t BTN3;
uint16_t Brake;
uint16_t Deadman;
uint16_t Wiper;
uint16_t Hazard;

uint32_t adcbuffer[2];
uint32_t adc_data1;
uint32_t adc_data2;

uint32_t Riso_neg;
uint32_t Riso_pos;
double isoneg,isopos;
double Vref = 2.5;
double Rs1 = 5110;
double Rs2 = 5110;
double Rps1_Rps2 = 10000;
double Rns1_Rns2 = 10000;
uint8_t SoC;//soc degeri bataryadan çekilecek


CAN_FilterTypeDef canfilterconfig;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;
uint8_t				  			RFData[64];

uint8_t UARTData[6];
uint8_t endcom[] = {0xFF,0xFF,0xFF};

int wiperFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
	if(hadc->Instance == ADC1){
		adc_data1 = adcbuffer[0];
		adc_data2 = adcbuffer[1];
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

enum Controls
{
	Headlight_On,
	Headlight_Off,
	LeftSignal_On,
	LeftSignal_Off,
	RightSignal_On,
	RightSignal_Off,
	Brake_On,
	Brake_Off,
	Deadman_On,
	Deadman_Off,
	Wiper_On,
	Wiper_Off,
	Hazard_On,
	Hazard_Off
};

void buttonListener(void);
void updateLight(enum Controls command);
void ADC_RELAY(void);
float map(float num, float oldmax, float oldmin, float conMax, float conMin);
void emergency(void);
void send_Riso(int control);
void GET_CAN_MESSAGE(void);

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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x7FF;
	
	TxHeader.TransmitGlobalTime = DISABLE;
	
	TxData[0] = 0x80;
	TxData[1] = 0x80;
	
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	
	HAL_ADC_Start_DMA(&hadc1,adcbuffer,2);
	
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.FilterActivation = ENABLE;
	
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
	HAL_CAN_Start(&hcan);
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&RxHeader,RxData);
		GET_CAN_MESSAGE();
		
	
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
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim2.Init.Prescaler = 500;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim3.Init.Prescaler = 3599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3599;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 6999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, Rel2Signal_Pin|Rel1Signal_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, P2_Pin|P6_Pin|P3_Pin|Silecek1_Pin
                          |Silecek2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, P1_Pin|Relay2_Pin|Latch_Pin|Relay1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Rel2Signal_Pin Rel1Signal_Pin EN_Pin */
  GPIO_InitStruct.Pin = Rel2Signal_Pin|Rel1Signal_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : P2_Pin P6_Pin P3_Pin Silecek1_Pin
                           Silecek2_Pin */
  GPIO_InitStruct.Pin = P2_Pin|P6_Pin|P3_Pin|Silecek1_Pin
                          |Silecek2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Deadman_Pin Brake_Pin */
  GPIO_InitStruct.Pin = Deadman_Pin|Brake_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : P1_Pin Relay2_Pin Latch_Pin Relay1_Pin */
  GPIO_InitStruct.Pin = P1_Pin|Relay2_Pin|Latch_Pin|Relay1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN2_Pin BTN3_Pin BTN4_Pin BTN5_Pin
                           BTN6_Pin BTN1_Pin */
  GPIO_InitStruct.Pin = BTN2_Pin|BTN3_Pin|BTN4_Pin|BTN5_Pin
                          |BTN6_Pin|BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	
	if(htim->Instance == TIM2){
		
		HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
		
		buttonListener();
		emergency();
	}
	
	if(htim->Instance == TIM3){
		
		if(Hazard == 1){
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);
		}
		
		if(BTN1 == 1){
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
		}
		if(BTN2 == 1){
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);
		}
		
	}
	
	if(htim->Instance == TIM4){
		
		if(Wiper == 1 && wiperFlag == 0){
		
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
			wiperFlag = 1;
		}
		
		else if(Wiper == 1 && wiperFlag == 1){
		
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
			wiperFlag = 0;
		}
		
	}
	

}

void buttonListener(void) 
{
	BTN1 = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15);
	if(BTN1==0 && Hazard==0) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	

	BTN2 = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8);
	if(BTN2==0 && Hazard==0) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
	
	//BTN 4
	Hazard = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10);
	
	

	BTN3 = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9);
	if(BTN3==0) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	if(BTN3==1) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

  //BTN5
	Brake = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);
	if(Brake==0) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	if(Brake==1) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);

	//Deadman
	Deadman = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
	if(Deadman==0) emergency_check_deadman = 0;
	if(Deadman==1) emergency_check_deadman = 1;
	
	//BTN6
	Wiper = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12);
	if(Wiper==0) {HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET); wiperFlag = 0;}
	
}

//void updateLight(enum Controls command)
//{
//	uint8_t checkBTN_States = BTN_States;
//	// Headlight - Brake - RightSignal - LeftSignal
//	switch(command){
//		
//		case LeftSignal_On:

//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//				
//				BTN_States ^= (1 << 0);
//				HAL_SPI_Transmit(&hspi1,&BTN_States,sizeof(BTN_States),HAL_MAX_DELAY);
//				
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//			
//			break;
//		
//		case LeftSignal_Off:
//			
//			if((checkBTN_States &= ~(1<<0)) != BTN_States){
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//				BTN_States &= ~(1 << 0);
//				HAL_SPI_Transmit(&hspi1,&BTN_States,sizeof(BTN_States),HAL_MAX_DELAY);
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//				
//			}
//			break;
//		
//		case RightSignal_On:

//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//	
//				BTN_States ^= (1 << 1);
//				HAL_SPI_Transmit(&hspi1,&BTN_States,sizeof(BTN_States),HAL_MAX_DELAY);
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//			
//			break;
//		
//		case RightSignal_Off:
//			
//			if((checkBTN_States &= ~(1 << 1)) != BTN_States){
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//				BTN_States &= ~(1 << 1);
//				HAL_SPI_Transmit(&hspi1,&BTN_States,sizeof(BTN_States),HAL_MAX_DELAY);
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//			}
//			break;	
//		
//		case Brake_On:
//			
//			if((checkBTN_States |= (1 << 2)) != BTN_States){
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//				BTN_States |= (1 << 2);
//				HAL_SPI_Transmit(&hspi1,&BTN_States,sizeof(BTN_States),HAL_MAX_DELAY);
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//			}
//			break;
//		
//		case Brake_Off:
//			
//			if((checkBTN_States &= ~(1 << 2)) != BTN_States){
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//				BTN_States &= ~(1 << 2);
//				HAL_SPI_Transmit(&hspi1,&BTN_States,sizeof(BTN_States),HAL_MAX_DELAY);
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//			}
//			break;
//		
//		case Headlight_On:
//			
//			if((checkBTN_States |= (1 << 3)) != BTN_States){
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//				BTN_States |= (1 << 3);
//				HAL_SPI_Transmit(&hspi1,&BTN_States,sizeof(BTN_States),HAL_MAX_DELAY);
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//			}
//			break;
//		
//		case Headlight_Off:
//			
//			if((checkBTN_States &= ~(1 << 3)) != BTN_States){
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//				BTN_States &= ~(1 << 3);
//				HAL_SPI_Transmit(&hspi1,&BTN_States,sizeof(BTN_States),HAL_MAX_DELAY);
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//			}
//			break;
//	}
//}

//void ADC_RELAY(void){
//	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
//	
//	isoneg = map(adc_data1,4096,0,3.3,0);//
//	
//	if(isoneg > 2.55){//
//		HAL_GPIO_WritePin(BuzzerSignal_GPIO_Port,BuzzerSignal_Pin,GPIO_PIN_SET);
//	}
//	else{
//		HAL_GPIO_WritePin(BuzzerSignal_GPIO_Port,BuzzerSignal_Pin,GPIO_PIN_RESET);
//	}
//	
//	Riso_neg = (uint32_t)(((((double)SoC + Vref) * Rs2) / isoneg - Vref) - Rns1_Rns2);
//	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
//	
//	isopos = map(adc_data2,4096,0,3.3,0);//
//	
//	if(isopos > 2.55){//
//		HAL_GPIO_WritePin(BuzzerSignal_GPIO_Port,BuzzerSignal_Pin,GPIO_PIN_SET);
//	}
//	else{
//		HAL_GPIO_WritePin(BuzzerSignal_GPIO_Port,BuzzerSignal_Pin,GPIO_PIN_RESET);
//	}
//	
//	Riso_pos = (uint32_t)(((Vref-((SoC-Vref)*Rs1))/isopos)-Rps1_Rps2);
//}

//void send_Riso(int control){
//	if (!control){//isopos
//		Riso = (uint32_t)(((Vref-((SoC-Vref)*Rs1))/isopos)-Rps1_Rps2);
//	}
//	if(control){//isoneg
//		Riso = (uint32_t)(((((double)SoC + Vref) * Rs2) / isoneg - Vref) - Rns1_Rns2);
//		Riso = Riso / 1000000; //MegaOhm cinsine çevirme
//	}
//}

float map(float num, float oldmax, float oldmin, float conMax, float conMin){
	return (((num - oldmin) * (conMax-conMin))/ (oldmax - oldmin)) + conMin;
}

void emergency(void){
	if(emergency_check_deadman == 1 || emergency_check_can == 1){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13|GPIO_PIN_14,GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13|GPIO_PIN_14,GPIO_PIN_RESET);
	}
}

void GET_CAN_MESSAGE(void){

  switch (RxHeader.StdId)
  {
  case 0x300://batarya
		RFData[0] = RxData[0];//Voltage
		RFData[1] = RxData[1];//Max Temperature
		RFData[2] = RxData[2];//SOC
	  break;
	
	case 0x303://motor sürücü
		RFData[3] = RxData[3];//Hiz
		RFData[4] = RxData[4]; //Akim
		break;
  }
	
	if(RFData[1] >= 70){
		emergency_check_can = 1;
	}
	
	if(RFData[1] < 70){
		emergency_check_can = 0;
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
