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

#include <stdint.h>  // For standard integer types like uint8_t, uint16_t

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

	// Global flag variable, accessible by both the ISR and the main loop.

	// Timer values
	volatile uint32_t post_captures[2] = {0};
	volatile uint8_t post_index = 0;
	volatile float post_frequency = 0.0f;

	volatile uint32_t pre_captures[2] = {0};
	volatile uint8_t pre_index = 0;
	volatile float pre_frequency = 0.0f;

	volatile uint8_t ratio = 0;

	// volatile uint8_t post_isr_flag = 0;
	// volatile uint8_t pre_isr_flag = 0;

	// some flags / booleans
	volatile char start_flag = 0; 	 // if in pre-en state
	volatile char charged_flag = 0;   // if in charged state
	volatile char error_flag = 0;		// if in error state

	volatile char prev_sdc = 1; 	// previous sdc reading
	volatile char sdc = 1;			// current sdc reading


	// freq / voltage constraints
	uint8_t lower_charge_tresh = 100;      // don't want to reach 90% charge by this time - 0.05 sec
	uint8_t upper_charge_tresh = 2000;    // want to have reached 90% charge by this time
	uint8_t upper_thresh_count = upper_charge_thres / lower_charge_thres;	// count to interrupt
	uint8_t counter_preen = 0; 		// counting # of interrupts that have passed

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

void start_capture(void)
{
    // Start Input Capture in interrupt mode
    HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);    // not sure if TIM_CHANNEL_1 is right
    HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
}

// interrupt for the freq pins PWM
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t captured;
    uint32_t diff;
	float timer_clk;
    // post frequency
    if (htim->Instance == TIM16) {
    	// read and storing IC value
       captured = HAL_TIM_ReadCapturedValue(&htim16, TIM_CHANNEL_1);
       post_captures[post_index++] = captured;

       if (post_index >= 2) {
    	   // make sure diff is a positive value or no overflow
    	   diff = (post_captures[1] >= post_captures[0])
			   ? (post_captures[1] - post_captures[0])
			   : (0xFFFF - post_captures[0] + post_captures[1]);

		   timer_clk = (float)HAL_RCC_GetPCLK2Freq(); // TIM16 on APB2
		   post_frequency = timer_clk / diff;
		   post_isr_flag = 1;
		   post_index = 0;
	  }
    }
    // pre frequency
    else if (htim->Instance == TIM17) {
    	// read and storing IC value
	   captured = HAL_TIM_ReadCapturedValue(&htim16, TIM_CHANNEL_1);
	   pre_captures[pre_index++] = captured;

	   if (pre_index >= 2) {
		   // make sure diff is a positive value or no overflow
		   diff = (pre_captures[1] >= pre_captures[0])
			   ? (pre_captures[1] - pre_captures[0])
			   : (0xFFFF - pre_captures[0] + pre_captures[1]);

		   timer_clk = (float)HAL_RCC_GetPCLK2Freq(); // TIM16 on APB2
		   pre_frequency = timer_clk / diff;
		   pre_index = 0;
		   pre_isr_flag = 1;
	  }
    }
}


// callback for Timer 14 to keep track after pre-charge has started

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        // executes each time CNT == CCR1
    	counter_preen++;

    	if (counter_preen == 1) {
    		// make sure voltage ratio is not over 90%
    		ratio = post_frequency * 100 / pre_frequency;

    		if (ratio > 90) {
    			// BAD -> ERROR
    			error_flag = 1;
    			start_flag = 0;
    		}

    	}
    	else if (counter_preen == upper_thresh_count) {
    		// make sure voltage ratio is over 90%
    		ratio = post_frequency * 100 / pre_frequency;

			if (ratio < 90) {
				// BAD -> ERROR
				error_flag = 1;
				start_flag = 0;
			}
			else {
				charged_flag = 1;
				start_flag = 0;
			}

			// reset counter
			counter_preen = 0;
    	}
    }
}


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
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  // starting timers to read inputs
  start_capture();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 prevsdc = sdc;
	 sdc = HAL_GPIO_ReadPin(SDC_PIN);

	 // To be worked out:
	 /*
	 if(!(gpio_reg & sdc) & !charged_flag){ // sdc inverted because we forgot
													 // to step it down so we added
													 // an inverter
		 GPIO |= (1 << CHR_PIN);

		 //uart_tx(1);
		 if(!start_flag){ // if we're about to start a precharge attempt
			 TMR1L = 0;  // reset the counters
			 TMR1H = 0;
		 }
		 start_flag = 1;
		 post_pin_new = gpio_reg & (1 << F_POST_PIN);
		 pre_pin_new = gpio_reg & (1 << F_PRE_PIN);
		 //uart_tx(post_pin_new);
		 //uart_tx(pre_pin_new);
		 if(pre_pin_new != pre_pin_old){ // if the value on pre_pin has changed
			 pre_pin_count += 1;
			 pre_pin_old = pre_pin_new;
		 }
		 if(post_pin_new != post_pin_old){ // if the value on post_pin has changed
			 post_pin_count += 1;
			 post_pin_old = post_pin_new;
		 }
	 }
	 else if (gpio_reg & (1 << SDC_PIN)){   // SDC went low (inverted to high)
		 pre_pin_count = 0;
		 post_pin_count = 0;
		 start_flag = 0;
		 charged_flag = 0;
		 start_count = 0;
		 //TRISIO &= ~(1 << CHR_PIN);
		 //WPU &= ~(1 << CHR_PIN);
		 GPIO &= ~((1 << AIR_PIN)|(1 << CHR_PIN));
	 }
	 */

	 // if SDC closes and we haven't already precharged
	 if (sdc && !charged_flag) {

	 }

	 // if SDC flipped from high to low, then system is good ->  PRE-CHARGE STATE
	 if(!sdc && prev_sdc && !start_flag) {

		 // start timer
		 HAL_TIM_OC_Start_IT(&htim14, TIM_CHANNEL_1);

		 start_flag = 1;
		 charged_flag = 0;

		 // set pre-en to 1
		 HAL_GPIO_WritePin(GPIOB, PRE_EN, 1);
	 }

	 // if it goes to charge state
	 if (!sdc && charged_flag && !start_flag) {
		 HAL_GPIO_WritePin(GPIOA, AIR_EN, 1);
		 HAL_GPIO_WritePin(GPIOB, PRE_EN, 0);

	 }

	 // if it goes to error state
	 if (!sdc && error_flag) {
		HAL_GPIO_WritePin(GPIOA, AIR_EN, 0);
		HAL_GPIO_WritePin(GPIOA, ERROR_PIN, 1);
		HAL_GPIO_WritePin(GPIOB, PRE_EN, 0);
	 }

	 // if car gets turned off


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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4096;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 64;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 128;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim17, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PRE_EN_GPIO_Port, PRE_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIR_EN_Pin|ERROR_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PRE_EN_Pin */
  GPIO_InitStruct.Pin = PRE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PRE_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : F_PRE_PIN_Pin F_POST_PIN_Pin */
  GPIO_InitStruct.Pin = F_PRE_PIN_Pin|F_POST_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SDC_PIN_Pin */
  GPIO_InitStruct.Pin = SDC_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDC_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIR_EN_Pin ERROR_PIN_Pin */
  GPIO_InitStruct.Pin = AIR_EN_Pin|ERROR_PIN_Pin;
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
