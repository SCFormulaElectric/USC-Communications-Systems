/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMS_out_Pin GPIO_PIN_7
#define BMS_out_GPIO_Port GPIOB
#define IMD_in_Pin GPIO_PIN_14
#define IMD_in_GPIO_Port GPIOC
#define LATCH_out_Pin GPIO_PIN_15
#define LATCH_out_GPIO_Port GPIOC
#define RESET_in_Pin GPIO_PIN_1
#define RESET_in_GPIO_Port GPIOA
#define BMS_FAULT_in_Pin GPIO_PIN_2
#define BMS_FAULT_in_GPIO_Port GPIOA
#define X1_out_Pin GPIO_PIN_4
#define X1_out_GPIO_Port GPIOA
#define X2_out_Pin GPIO_PIN_11
#define X2_out_GPIO_Port GPIOA
#define NF_RP_IN_Pin GPIO_PIN_12
#define NF_RP_IN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
