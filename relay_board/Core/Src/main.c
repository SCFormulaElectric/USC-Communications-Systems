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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RED_PIC_O_PIN		 GPIO_PIN_0
#define GREEN_PIC_O_PIN      GPIO_PIN_1
#define BMS_FAULT_O_PIN      GPIO_PIN_2
#define BMS_FAULT_I_PIN	     GPIO_PIN_3
#define RESET_BUTTON_PIN	 GPIO_PIN_4
#define RESET_LATCH_PIN	 	 GPIO_PIN_5
#define IMD_GATE_I_PIN		 GPIO_PIN_6


#define FREQ_SCALAR 10000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
uint8_t start = 0;
uint8_t faulted = 0;
uint8_t fault_counter = 0x00;
uint8_t current_fault = 0x00;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM14) {

        uint8_t bms_gate_state = HAL_GPIO_ReadPin(GPIOA, BMS_FAULT_I_PIN);
        uint8_t imd_gate_state = HAL_GPIO_ReadPin(GPIOA, IMD_GATE_I_PIN);
        uint8_t reset_button_state = HAL_GPIO_ReadPin(GPIOA, RESET_BUTTON_PIN);
        if(start == 0) {
        	start = 1;
        	HAL_GPIO_WritePin(GPIOA, RESET_LATCH_PIN, GPIO_PIN_SET);
        }
        if (bms_gate_state == GPIO_PIN_SET) {
            HAL_GPIO_WritePin(GPIOA, BMS_FAULT_O_PIN, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOA, BMS_FAULT_O_PIN, GPIO_PIN_SET);
        }

        if ((reset_button_state == GPIO_PIN_SET) && (current_fault == 0)) {
            faulted = 0;
        }

        if ((current_fault == 1) || faulted) {
			fault_counter = 0;
			HAL_GPIO_TogglePin(GPIOA, RED_PIC_O_PIN);
			HAL_GPIO_WritePin(GPIOA, GREEN_PIC_O_PIN, GPIO_PIN_RESET);
			faulted = 1;

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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  uint8_t blink_counter;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint8_t bms_gate_state = HAL_GPIO_ReadPin(GPIOA, BMS_FAULT_I_PIN);
	  uint8_t imd_gate_state = HAL_GPIO_ReadPin(GPIOA, IMD_GATE_I_PIN);
	  uint8_t reset_button_state = HAL_GPIO_ReadPin(GPIOA, RESET_BUTTON_PIN);

	  //copy pasted fault logic may not be applicable
	  uint8_t potential_fault = (bms_gate_state == GPIO_PIN_SET) || (imd_gate_state == GPIO_PIN_RESET);
	  uint8_t fault_counter
	  if (potential_fault) {
		  if (fault_counter < 20) {
			  fault_counter++;
		  } else if (fault_counter == 20) {
			  current_fault = 1;
		  } else {
			  fault_counter = 0;
		  }
	  } else {
		  if (fault_counter < 20) {
			  fault_counter = 0;
		  }
		  if (fault_counter < 40) {
			  fault_counter++;
		  } else {
			  current_fault = 0;
			  fault_counter = 0;
		  }
	  }
	if(reset_button_state) {
		HAL_GPIO_WritePin(GPIOA, RESET_LATCH_PIN, GPIO_PIN_SET);
	}
	if(!faulted) {
		blink_counter++;
		HAL_GPIO_WritePin(GPIOA, GREEN_PIC_O_PIN, GPIO_PIN_RESET);
		if ((blink_counter % 9) == 0) {
			HAL_GPIO_WritePin(GPIOA, GREEN_PIC_O_PIN, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(GPIOA, RED_PIC_O_PIN, GPIO_PIN_RESET);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 684;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 9999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */
  HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM14_IRQn);

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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



