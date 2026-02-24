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
#include <string.h>

#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
  /* ADC BUF & dma_flag must be reread always, do not optimize values away*/
static volatile uint16_t adc_buf[4];
static volatile uint16_t dma_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
#define THERM_COUNT 40
#define module_number 0x00
#define ADC_CH_COUNT 4

void adc_start_dma_4();

void set_muxOutput(int count);

// lol is this declaration correct?
int8_t volt2temp(uint16_t adc_buf, int16_t temp_adc_lut[33][2]);

void send_thermistor_CAN_msg(int8_t temp_array[THERM_COUNT]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Voltage to temperature look-up table
const int16_t temp_adc_lut[33][2] = {
	{-40, 3028}, {-35, 3004}, {-30, 2979}, {-25, 2955},
	{-20, 2916}, {-15, 2878}, {-10, 2817}, { -5, 2767},
	{  0, 2694}, {  5, 2621}, { 10, 2548}, { 15, 2475},
	{ 20, 2389}, { 25, 2316}, { 30, 2234}, { 35, 2161},
	{ 40, 2088}, { 45, 2022}, { 50, 1973}, { 55, 1924},
	{ 60, 1874}, { 65, 1837}, { 70, 1800}, { 75, 1775},
	{ 80, 1737}, { 85, 1712}, { 90, 1699}, { 95, 1674},
	{100, 1662}, {105, 1650}, {110, 1638}, {115, 1626},
	{120, 1614}
};

int8_t temp_array[THERM_COUNT];		// holds converted temperature values of 40 pins

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  dma_flag = 0;
  HAL_ADC_Start_DMA(&hadc1, (uint16_t*)adc_buf, 4);
  char message[8] = "testing";

  if(HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 100) == HAL_OK) {
  		  HAL_Delay(4000);
  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
  		  HAL_Delay(4000);
  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
  		  HAL_Delay(4000);
  	  }

  int count_muxpins = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);  // Green LED on PB5
	  //HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(dma_flag) {
//		  dma_flag = 0;
//		  HAL_Delay(250);
//		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
//		  HAL_Delay(250);
//		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
//		  HAL_Delay(250);
//
//	  }

	  // set output pins going to muxes
	  set_muxOutput(count_muxpins);

	  // when adc_buf finished collecting voltage values
	  if (dma_flag == 1) {
	  HAL_Delay(100);


		  // convert and load temperatures
		  for (int m = 0; m < 4; m++) {
			  // Debug UART transmit: adc_buf data at each count_muxpins step (0—9).
			   char buffer[30];
			   sprintf(buffer, "Mux %d, channel %d: %d \r\n", m, count_muxpins, adc_buf[m]);
			   HAL_UART_Transmit(&huart1, (uint8_t*) buffer, (uint16_t) strlen(buffer), 100);

			  temp_array[m*10 + count_muxpins] = volt2temp(adc_buf[m], temp_adc_lut);
			  char buffer2[20];
			  sprintf(buffer2, "Temp: %d C\r\n", temp_array[m*10 + count_muxpins]);
			  HAL_UART_Transmit(&huart1, (uint8_t*) buffer2, (uint16_t) strlen(buffer2), 100);
		  }
		  count_muxpins++;
		  adc_start_dma_4();
	  }

	  // read through all 10 on each - can send signal now
	  if (count_muxpins == 9) {

		  // Debug UART transmit: confirming CAN message sent after all 40 ADC readings collected.
		  char can_msg[] = "Sending CAN message\r\n";
		  //HAL_UART_Transmit(&huart1, (uint8_t*) can_msg, (uint16_t) strlen(can_msg), 100);

		  // Debug UART transmit: printing all temp_array temperature data.
		  char title[] = "Temp array temperature data for all channels:\r\n";
		 // HAL_UART_Transmit(&huart1, (uint8_t*) title, (uint16_t) strlen(title), 100);
		  for (int mux = 0; mux < 4; mux++)
		  {
			char msg[100];
			sprintf(msg,
			"Mux %d: %d %d %d %d %d %d %d %d %d %d\r\n",
			mux + 1,
			temp_array[mux*10 + 0], temp_array[mux*10 + 1], temp_array[mux*10 + 2],
			temp_array[mux*10 + 3], temp_array[mux*10 + 4], temp_array[mux*10 + 5],
			temp_array[mux*10 + 6], temp_array[mux*10 + 7], temp_array[mux*10 + 8],
			temp_array[mux*10 + 9]);
		   // HAL_UART_Transmit(&huart1, (uint8_t*) msg, (uint16_t) strlen(msg), 100);
		  }


		  send_thermistor_CAN_msg(temp_array);
		  count_muxpins = 0;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  huart3.Init.BaudRate = 115200;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//  we enter this callback after the 4th channel is read.
{
    if (hadc->Instance == ADC1) // here, we only have one adc, but its future-proofed
    {
    	dma_flag = 1;
    }
}


void adc_start_dma_4(void)
{
    dma_flag = 0;
    // Starts regular sequence conversions; DMA stores ADC_CH_COUNT samples
    HAL_ADC_Start_DMA(&hadc1, (uint16_t*)adc_buf, ADC_CH_COUNT);
}


void set_muxOutput(int count){
  /* This function inputs an integer (mux counter) and outputs it in binary to the mux select pins (PA0-3)
    test:
    count = 5 -> PA0 = 1, PA1 = 0, PA2 = 1, PA3 = 0
    which is 0101 which is 5

    Also double-checked against schematic to ensure bit-ordering is correct.

  */

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (count & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (count & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (count & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (count & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


int8_t volt2temp(uint16_t adc_val, int16_t temp_adc_lut[33][2]) {
    /* This function takes in an adc_value and compares it against a look-up table

to get a corresponding temperature value.
Will linearly interpolate.
*/

      // TODO:   maybe also add in something for if the adc read is outside range of lookup table....

    int8_t temp = 0;
    int16_t temp16 = 0;

  if (adc_val >= temp_adc_lut[0][1])
  {
    return -128;  // Too cold, return minimum int8_t value.
  }

  else if (adc_val <= temp_adc_lut[32][1])
  {
    return 127;  // Too hot, return maximum int8_t value.
  }
  else {
      for (int t = 1; t < 33; t++) {
        if (adc_val > temp_adc_lut[t][1]) {
            // linear interpolation between row t-1 and row t
            temp16 = temp_adc_lut[t-1][0] +
                     (temp_adc_lut[t][0] - temp_adc_lut[t-1][0]) * (temp_adc_lut[t-1][1]-adc_val) /
                     (temp_adc_lut[t-1][1] - temp_adc_lut[t][1]);
            break;
        }
      }
     // convert to 8 bit, as sent in CAN protocol
     temp = (int8_t)temp16;
     return temp;
  }

 }


 void send_thermistor_CAN_msg(int8_t temp_array[THERM_COUNT])
 {
	 // This function prepares thermistor CAN message in format specified in datasheet
	 // https://www.orionbms.com/downloads/misc/thermistor_module_canbus.pdf

	 // CAN message is 8 bytes long
     int8_t data[8] = {0};

     // finding min, max, and average
     int8_t min_temp = temp_array[0];
     int8_t max_temp = temp_array[0];
     uint8_t min_id = 0;
     uint8_t max_id = 0;
     int16_t sum = 0;

     for (uint8_t i = 0; i < THERM_COUNT; i++) {
         int8_t t = (int8_t)temp_array[i];
         sum += t;

         if (t < min_temp) {
             min_temp = t;
             min_id = i;
         }
         if (t > max_temp) {
             max_temp = t;
             max_id = i;
         }
     }
     int8_t avg_temp = (int8_t)(sum / THERM_COUNT);

     char buff3[30];
     sprintf(buff3, "Min: %d F, Max: %d F, Avg: %d F\r\n", min_temp, max_temp, avg_temp);
     HAL_UART_Transmit(&huart1, (uint8_t*) buff3, (uint16_t) strlen(buff3), 100);


     // --------- Fill payload ----------
     data[0] = module_number;       	// Byte 1
     data[1] = (int8_t)min_temp;   	// Byte 2
     data[2] = (int8_t)max_temp;   	// Byte 3
     data[3] = (int8_t)avg_temp;   	// Byte 4
     data[4] = (uint8_t)THERM_COUNT;	// Byte 5
     data[5] = max_id;              	// Byte 6
     data[6] = min_id;              	// Byte 7
     //data[7] = orion_checksum(data, CAN_LEN);  // Byte 8

     // --------- CAN transmit ----------
     CAN_TxHeaderTypeDef txHeader = {0};
     uint32_t txMailbox;

     txHeader.StdId = 0;                 // not used
     txHeader.ExtId = 0x1839F380 + module_number;
     txHeader.IDE   = CAN_ID_EXT;
     txHeader.RTR   = CAN_RTR_DATA;
     //txHeader.DLC   = CAN_LEN;

     HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox);
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
