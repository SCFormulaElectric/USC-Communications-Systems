/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "string.h" // Required for memset()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// --- UART Interrupt Variables ---
uint8_t FinalData[20];   // max amount of rx data before processing
uint8_t RxData[20];      // storing rx data
uint8_t temp[100];
volatile int indx = 0;            // counting bytes
volatile int message_received_flag = 0;    // marks message completion
int MAX_MESSAGE_LENGTH = 20;

// --- CAN Interrupt Variables --
int CAN_received = 0;   // marks when to send CAN via UART
CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure


void toggle_LED() {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}



void AT_Config(void) {
	 // AT configuration

	  HAL_UART_Receive_IT(&huart2, temp, 1);   // interrupt initialization

	  // +++ to AT mode
	  uint8_t cmd_enter_config[] = {0x2B, 0x2B, 0x2B, 0x0d, 0x0a};

	  // AT+MODE0\r\n
	  uint8_t cmd_mode1[] = {0x41, 0x54, 0x2B, 0x4D, 0x4F, 0x44, 0x45, 0x31, 0x0D, 0x0A};

	  // AT+LEVEL7\r\n
	  uint8_t cmd_level7[] = {0x41, 0x54, 0x2B, 0x4C, 0x45, 0x56, 0x45, 0x4C, 0x37, 0x0D, 0x0A};

	  // AT+CHANNEL03\r\n
	  uint8_t cmd_channel03[] = {0x41, 0x54, 0x2B, 0x43, 0x48, 0x41, 0x4E, 0x4E, 0x45, 0x4C, 0x30, 0x33, 0x0D, 0x0A};

	  // AT+MAC0a,02\r\n
	  uint8_t cmd_mac0a02[] = {0x41, 0x54, 0x2B, 0x4D, 0x41, 0x43, 0x30, 0x61, 0x2C, 0x30, 0x32, 0x0D, 0x0A};
	  uint8_t cmd_sf12[] = {0x41, 0x54, 0x2B, 0x53, 0x46, 0x31, 0x32, 0x0D, 0x0A};

	  // AT+POWE22\r\n
	  uint8_t cmd_powe22[] = {0x41, 0x54, 0x2B, 0x50, 0x4F, 0x57, 0x45, 0x32, 0x32, 0x0D, 0x0A};

	    // AT+RESET\r\n
	    uint8_t cmd_reset[] = {0x41, 0x54, 0x2B, 0x52, 0x45, 0x53, 0x45, 0x54, 0x0D, 0x0A};

	  send_at_command(cmd_enter_config, sizeof(cmd_enter_config));
	  // should receive "Entry AT\r\n" or something
	  // HAL_Delay(6000); // A small delay is often good practice after commands
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
		indx = 0;

	  // Set transmission mode
	  send_at_command(cmd_mode1, sizeof(cmd_mode1));
	  // HAL_Delay(6000);
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
	    indx = 0;

	  // Set air data rate level
	  send_at_command(cmd_level7, sizeof(cmd_level7));
	  //HAL_Delay(6000);
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
	    indx = 0;

	  // Set frequency channel
	  send_at_command(cmd_channel03, sizeof(cmd_channel03));
	  // HAL_Delay(6000);
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
	    indx = 0;

	  // Set device address
	  send_at_command(cmd_mac0a02, sizeof(cmd_mac0a02));
	  // HAL_Delay(6000);
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
	    indx = 0;

	  // Set TX power
	  send_at_command(cmd_powe22, sizeof(cmd_powe22));
	  // HAL_Delay(6000);
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
	    indx = 0;

	  //send SF
	  send_at_command(cmd_sf12, sizeof(cmd_sf12));
	  // HAL_Delay(6000);
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
	    indx = 0;
	  // check AT for OK

	  // Reset the module to apply settings
	  send_at_command(cmd_reset, sizeof(cmd_reset));
	  // HAL_Delay(6000); // A longer delay after reset is recommended
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
	    indx = 0;

	  send_at_command(cmd_enter_config, sizeof(cmd_enter_config));
	  // HAL_Delay(6000);
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
	    indx = 0;

	  send_at_command(cmd_enter_config, sizeof(cmd_enter_config));
	  // HAL_Delay(6000);
	  while (message_received_flag == 0) {  }
	  toggle_LED();
	    message_received_flag = 0;
	    indx = 0;


}

void CAN_Config(void) {
	// CAN Initialization

	  uint8_t bigData[20] = { /* ... */ };
	  uint8_t chunk[8];

	  CAN_TxHeaderTypeDef txHeader;
		  // uint8_t txData[20];
		  uint32_t txMailbox;

		  txHeader.StdId = 0x123;
		  txHeader.RTR = CAN_RTR_DATA;
		  txHeader.IDE = CAN_ID_STD;
		  txHeader.DLC = 2;			// sending 16
		  txHeader.TransmitGlobalTime = DISABLE;

		// https://github.com/timsonater/stm32-CAN-bus-example-HAL-API/blob/master/main.c

		pHeader.DLC=1; //give message size of 1 byte
		pHeader.IDE=CAN_ID_STD; //set identifier to standard
		pHeader.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
		pHeader.StdId=0x244; //define a standard identifier, used for message identification by filters (switch this for the other microcontroller)

		//filter one (stack light blink)
		sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
		sFilterConfig.FilterIdHigh=0x245<<5; //the ID that the filter looks for (switch this for the other microcontroller)
		sFilterConfig.FilterIdLow=0;
		sFilterConfig.FilterMaskIdHigh=0;
		sFilterConfig.FilterMaskIdLow=0;
		sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
		sFilterConfig.FilterActivation=ENABLE;

		HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

	  if (HAL_CAN_Start(&hcan) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

		uint8_t received_byte = temp[0];

		// if ends with \n, then message complete
		if (received_byte == 0x0a) {
			message_received_flag = 1;
		}
		// if not complete message or not max length, store it
		if (indx < (MAX_MESSAGE_LENGTH - 1) && message_received_flag == 0) { // Leave 1 byte for null terminator
			RxData[indx] = received_byte;
			indx++;
		}
		else if (indx >= (MAX_MESSAGE_LENGTH - 1)) {
			// Buffer overflow occurred before newline, treat as error/reset
			indx = 0;
		}

		// Restart the interrupt
		HAL_UART_Receive_IT(&huart2, temp, 1);

}


// Called automatically when a new frame lands in FIFO0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t data[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK) {
		Error_Handler(); // error handling
	}

	// Example: check ID, length, print/process
	if (rxHeader.IDE == CAN_ID_STD) {
		uint16_t id = rxHeader.StdId;     // 11-bit
		uint8_t  dlc = rxHeader.DLC;      // 0..8
		// TODO: handle 'data[0..dlc-1]'
		send_at_command(data, dlc);
			// CAN_received = 1;
	}
	else {
		uint32_t id = rxHeader.ExtId;     // 29-bit
		// TODO: handle extended ID
	}


}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void send_at_command(const uint8_t* hex_cmd, uint16_t len);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  AT_Config();

  CAN_Config();

  // --- START THE FIRST UART RECEPTION WITH INTERRUPT ---
  // This tells the UART hardware to start listening for the very first byte.
  // The interrupt callback will handle receiving all subsequent bytes.
  //HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  //uint8_t rx_buffer_main[RX_BUFFER_SIZE];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	if (message_received_flag)
	  	{
	  		// indx holds the number of bytes received (up to, but not including, the '\n')
	  		memcpy(FinalData, RxData, indx);

	  		// Toggle the LED to signal received
	  		toggle_LED();

	  		// clear the flag and reset index
	  		message_received_flag = 0;
	  		indx = 0;
	  	}


    // 1. Transmit the UART message.
	  	// address 0a01, channel 3, "selena"
	   uint8_t lora_message[] = {0x0A, 0x01, 0x03, 0x73, 0x65, 0x6C, 0x65, 0x6E, 0x61, 0x0D, 0x0A};
	   send_at_command(lora_message, sizeof(lora_message));
	   HAL_Delay(100);

	   while (message_received_flag == 0) {  }
	   toggle_LED();
		message_received_flag = 0;
		indx = 0;

	/*
	for (int i =0; i <5; i++){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(100);
	}
	*/
	  	/*
	if (CAN_received) {


		send_at_command(data, sizeof(lora_message));
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(100); // Shortened delay for more frequent checks
	}
	*/

  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Reception Complete Callback.
  * This function is automatically called by the HAL driver every time
  * a single byte is received via interrupt.
  * @param  huart: UART handle
  * @retval None
  */


void send_at_command(const uint8_t* hex_cmd, uint16_t len) {
    // Buffer to hold the command plus the required "\r\n"
	HAL_UART_Transmit(&huart2, (uint8_t*)hex_cmd, len, 100);
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
