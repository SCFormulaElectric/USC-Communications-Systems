/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void Comm_Init();
void Read_Cell_Volts();
uint16_t crc16(const uint8_t *data, uint32_t len);
void SPI_frame(uint8_t* frame, uint8_t packet_type, int8_t device_addr, uint16_t reg_addr, uint8_t data[], uint8_t size);
void delay_us(uint32_t us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TX_Data[15] = "Hello World!";
uint8_t RX_Buffer[BUFFER_SIZE] = {0};

// LUT for message type
typedef enum {
    SINGLE_R,
	SINGLE_W,
	STACK_R,
	STACK_W,
	BROADCAST_R,
	BROADCAST_W,
	BROADCASE_W_REVERSE,
} init_id_t;

static const uint16_t packet_init_table[7] = {
    0x80, // SINGLE_R
	0x90, // SINGLE_W
	0xA0, // STACK_R
	0XB0, // STACK_W
	0XC0, // BROADCAST_R
	0XD0, // BROADCAST_W
	0XE0, // BROADCASE_W_REVERSE
};

// LUTs for device id addresses
typedef enum {
	BRIDGE,
	CELL1,
	CELL2,
	CELL3,
	CELL4,
	CELL5,
	CELL6,
	CELL7,
	CELL8,
	CELL9,
	CELL10,
	CELL11,
	CELL12,
	CELL13,
	CELL14,
	CELL15,
	CELL16,
} device_id_t;

static const uint16_t device_addr_table[17] = {
	0x00, // Bridge device
	0x01, // Cell 1
	0x02, // Cell 2
	0x03, // Cell 3
	0x04, // Cell 4
	0x05, // Cell 5
	0x06, // Cell 6
	0x07, // Cell 7
	0x08, // Cell 8
	0x09, // Cell 9
	0x0A, // Cell 10
	0x0B, // Cell 11
	0x0C, // Cell 12
	0x0D, // Cell 13
	0x0E, // Cell 14
	0x0F, // Cell 15
	0x10, // Cell 16
};


// LUTs for register addresses
typedef enum {
	DIR0_ADDR,
	DIR1_ADDR,
	CONTROL1,
	CONTROL2,
	ACTIVE_CELL,
	ADC_CTRL1,
	VCELL16_HI,

} reg_id_t;

static const uint16_t reg_addr_table[7] = {
	0X306, // DIR0_ADDR
	0x307, // DIR2_ADDR
	0x309, // CONTROL1
	0x30A, // CONTROL2
	0x0003, // ACTIVE_CELL
	0x030D, // ADC_CTRL1
	0x0568, // VCELL16_HI
};



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
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  Comm_Init();
  // enable timers


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_0);

//	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)){ // Read SPI_RDY
//		  SPI_frame(TX_Data, packet_init_table[SINGLE_R], device_addr_table[BRIDGE], 0x2100, (uint8_t[]){0x00}, 1); // Read a register
//	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	  	  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
//	  }
	  SPI_frame(TX_Data, packet_init_table[SINGLE_R], device_addr_table[BRIDGE], 0x2100, (uint8_t[]){0x00}, 1); // Read a register
//
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
//
//	  // 3. Start Frame
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	  delay_us(10); // Give the BQ a moment to wake up the SPI port
//
//	  // 4. Transmit the command
//	  HAL_SPI_Transmit(&hspi1, TX_Data, 7, 100);
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//
//
//	  HAL_SPI_Receive(&hspi1, RX_Buffer, 7, 100);
//
//	  HAL_Delay(200);
	  // 1. PHASE ONE: SEND THE COMMAND
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // nCS Low
	  HAL_SPI_Transmit(&hspi1, TX_Data, 7, 100);           // Send 7-byte Read Command
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // nCS HIGH (Crucial!)

	  // 2. THE WAIT: POLL THE HARDWARE PIN
	  // The BQ will pull SPI_RDY low here while it talks to the stack.
	  // You MUST wait for it to go High again.
	  uint32_t timeout = 10000;
	  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET && timeout > 0) {
	      timeout--; // Wait for BQ to be ready
	  }

	  // 3. PHASE TWO: GET THE DATA
	  // Response frame for a 1-byte read is 7 bytes: [Header][ID][Addr][Addr][Data][CRC][CRC]
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // nCS Low again
	  HAL_SPI_Receive(&hspi1, RX_Buffer, 7, 100);          // Clock out the 7-byte response
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // nCS High

	  // 4. MOSI IDLE HIGH HANDOVER
	  // Your handover logic is correct, but only do it here, after the whole transaction.
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	  HAL_Delay(500);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_nCS_GPIO_Port, SPI_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Blinky_LED_GPIO_Port, Blinky_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI_RDY_Pin nFAULT_Pin */
  GPIO_InitStruct.Pin = SPI_RDY_Pin|nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_nCS_Pin */
  GPIO_InitStruct.Pin = SPI_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_nCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Blinky_LED_Pin */
  GPIO_InitStruct.Pin = Blinky_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Blinky_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void delay_us(uint32_t us) {
//    uint32_t start = TIM2->CNT;
//    uint32_t duration = us * 16;
//    while (TIM2->CNT - start < duration);
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	HAL_TIM_Base_Start(&htim6);
	while (__HAL_TIM_GET_COUNTER(&htim6) < us);
	HAL_TIM_Base_Stop(&htim6);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi){
	HAL_SPI_Receive_DMA(&hspi1, RX_Buffer, BUFFER_SIZE);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if(hspi->Instance == SPI1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Pull CS high only when done
	}
}

uint16_t crc16(const uint8_t *data, uint32_t len)
{
	uint16_t crc = 0xFFFF; // Initialization
	//x16+x15+x2+x1 = 1100000000000101
	uint16_t poly = 0xA001; // Reflected form of 0x8005 since LSB first

	for (uint32_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i];
		for (uint8_t b = 0; b < 8; b++) {
			if (crc & 0x0001) {
				crc = (crc >> 1) ^ poly;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}

// create SPI frame with format:  initialization byte (1) | device id (1) | reg address (2) | data bytes (<=8) | CRC (2)
// set device address to -1 if not used
void SPI_frame(uint8_t* frame, uint8_t packet_type, int8_t device_addr, uint16_t reg_addr, uint8_t data[], uint8_t size){
	if (size == 0 || size > 8)
	    return;

	memset(frame, 0, 15);
	uint8_t total_payload_len = 0;
	frame[0] = packet_type;

	if (device_addr >= 0){
		frame[1] = (uint8_t)device_addr;
		frame[2] = (reg_addr >> 8) & 0xFF;
		frame[3] = reg_addr & 0xFF;
		memcpy(&frame[4], data, size);
		total_payload_len = 4 + size;
	}
	else{
		frame[1] = (reg_addr >> 8) & 0xFF;
		frame[2] = reg_addr & 0xFF;
		memcpy(&frame[3], data, size);
		total_payload_len = 3 + size;
	}

	uint16_t crc = crc16(frame, total_payload_len);
	frame[total_payload_len] = crc & 0xFF;        // Low Byte
	frame[total_payload_len + 1] = (crc >> 8) & 0xFF; // High Byte
}

void Comm_Init(){
  // wake ping: pull nCS line low for 2us, pull MOSI line low for 2.75ms, high for 2us, send 90 00 03 09 20 13 95
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(2);
  delay_us(750);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  delay_us(2);

  HAL_Delay(4);

  SPI_frame(TX_Data, packet_init_table[SINGLE_W], device_addr_table[BRIDGE], reg_addr_table[CONTROL1], (uint8_t[]){0x20}, 1);
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  // then wait ((~1.6 + 10ms) * n stacked devices)
  HAL_Delay(12);

  // device address self-assignment: sync delay-locked loop
  SPI_frame(TX_Data, packet_init_table[STACK_W], -1, 0x0343, (uint8_t[]){0x00}, 1); // OPT_ECC_DATAIN1 - B0 03 43 00 E7 D4
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, packet_init_table[STACK_W], -1, 0x0344, (uint8_t[]){0x00}, 1); // OPT_ECC_DATAIN2 - B0 03 44 00 E5 E4
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, packet_init_table[STACK_W], -1, 0x0345, (uint8_t[]){0x00}, 1); // OPT_ECC_DATAIN3
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, packet_init_table[STACK_W], -1, 0x0346, (uint8_t[]){0x00}, 1); // OPT_ECC_DATAIN4
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, packet_init_table[STACK_W], -1, 0x0347, (uint8_t[]){0x00}, 1); // OPT_ECC_DATAIN5
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, packet_init_table[STACK_W], -1, 0x0348, (uint8_t[]){0x00}, 1); // OPT_ECC_DATAIN6
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, packet_init_table[STACK_W], -1, 0x0349, (uint8_t[]){0x00}, 1); // OPT_ECC_DATAIN7
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, packet_init_table[STACK_W], -1, 0x034A, (uint8_t[]){0x00}, 1); // OPT_ECC_DATAIN8
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));

  SPI_frame(TX_Data, packet_init_table[BROADCAST_W], -1, reg_addr_table[CONTROL1], (uint8_t[]){0x01}, 1);  // enable auto addressing - D0 03 09 01 0F 74
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, packet_init_table[BROADCAST_W], -1, reg_addr_table[DIR0_ADDR], (uint8_t[]){0x00}, 1); // set bridge device - D0 03 06 00 CB 44
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, packet_init_table[BROADCAST_W], -1, reg_addr_table[DIR0_ADDR], (uint8_t[]){0x01}, 1); // set device 1 - D0 03 06 01 0A 84
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
}


void Start_Cell_Volts(){
  SPI_frame(TX_Data, packet_init_table[BROADCAST_W], -1, reg_addr_table[ACTIVE_CELL], (uint8_t[]){0x0A}, 1);  // set active cells - B0 00 03 0A A6 13
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  SPI_frame(TX_Data, &hspi1, -1, reg_addr_table[ADC_CTRL1], (uint8_t[]){0x06}, 1);  // start continuous run ADC
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
  delay_us(250); // 92us + (5us x TOTALBOARDS)
  SPI_frame(TX_Data, packet_init_table[STACK_R], -1, reg_addr_table[VCELL16_HI], (uint8_t[]){0x1F}, 1);  // start continuous run ADC
  HAL_SPI_Transmit_DMA(&hspi1, TX_Data, sizeof(TX_Data));
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
#ifdef USE_FULL_ASSERT
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
