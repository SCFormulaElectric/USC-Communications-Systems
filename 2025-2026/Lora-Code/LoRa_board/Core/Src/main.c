/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (non-blocking LoRa AT config + CAN/LoRa loop)
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
#include <string.h>   // memcpy
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// One AT command + how long to wait AFTER the module replies before sending the next one
typedef struct {
    const uint8_t *cmd;
    uint16_t       len;
    uint32_t       settle_ms;
} at_step_t;

// Where the AT configuration currently is
typedef enum {
    CFG_SEND,        // send the current command
    CFG_WAIT_REPLY,  // waiting for the module to answer (with a timeout)
    CFG_SETTLE,      // got a reply, waiting the short settle time
    CFG_DONE,        // all commands sent successfully
    CFG_FAILED       // module never answered after retries
} cfg_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_MESSAGE_LENGTH    20     // longest line we accept from the LoRa module
#define AT_REPLY_TIMEOUT_MS   300    // give up waiting for a reply after this long...
#define AT_MAX_RETRIES        3      // ...and resend the command up to this many times
#define AT_SETTLE_MS          20     // normal wait after a reply (the 10-20 ms you mentioned)
#define AT_RESET_SETTLE_MS    1000   // module reboots after AT+RESET; give it longer
#define LORA_TX_PERIOD_MS     500    // heartbeat message interval
#define FAIL_BLINK_MS         100    // fast LED blink if config failed

// LoRa fixed-point header: target address 0x0A01, channel 0x03
#define LORA_ADDR_H           0x0A
#define LORA_ADDR_L           0x01
#define LORA_CHANNEL          0x03
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// ---------- UART receive (filled by the interrupt) ----------
static uint8_t rx_byte;                         // each interrupt drops one byte here
static uint8_t RxData[MAX_MESSAGE_LENGTH];      // the line the ISR is currently building
static uint8_t indx = 0;                        // only touched inside the ISR

// A finished line, handed from the ISR to main.
// Rule: the ISR only writes FinalData when line_ready == 0,
//       main only reads FinalData when line_ready == 1.
static uint8_t          FinalData[MAX_MESSAGE_LENGTH];
static volatile uint8_t final_len  = 0;
static volatile uint8_t line_ready = 0;

// ---------- CAN receive (filled by the interrupt) ----------
static uint8_t          CAN_rx_data[8];
static volatile uint8_t CAN_rx_len   = 0;
static volatile uint8_t CAN_received = 0;
static CAN_FilterTypeDef sFilterConfig;

// ---------- AT commands (static const so they stay valid during interrupt-driven TX) ----------
static const uint8_t cmd_enter_config[] = "+++\r\n";
static const uint8_t cmd_mode1[]        = "AT+MODE1\r\n";
static const uint8_t cmd_level7[]       = "AT+LEVEL7\r\n";
static const uint8_t cmd_channel03[]    = "AT+CHANNEL03\r\n";
static const uint8_t cmd_mac0a02[]      = "AT+MAC0a,02\r\n";
static const uint8_t cmd_powe22[]       = "AT+POWE22\r\n";
static const uint8_t cmd_sf12[]         = "AT+SF12\r\n";
static const uint8_t cmd_reset[]        = "AT+RESET\r\n";

// sizeof() of a string literal counts the hidden '\0' at the end, so subtract 1
#define AT_STEP(c, ms) { (c), (uint16_t)(sizeof(c) - 1), (ms) }

static const at_step_t at_steps[] = {
    AT_STEP(cmd_enter_config, AT_SETTLE_MS),
    AT_STEP(cmd_mode1,        AT_SETTLE_MS),
    AT_STEP(cmd_level7,       AT_SETTLE_MS),
    AT_STEP(cmd_channel03,    AT_SETTLE_MS),
    AT_STEP(cmd_mac0a02,      AT_SETTLE_MS),
    AT_STEP(cmd_powe22,       AT_SETTLE_MS),
    AT_STEP(cmd_sf12,         AT_SETTLE_MS),
    AT_STEP(cmd_reset,        AT_RESET_SETTLE_MS),
    // These two "+++" were at the end of the original sequence.
    // Kept so behavior matches; check the module datasheet to see if they're needed.
    AT_STEP(cmd_enter_config, AT_SETTLE_MS),
    AT_STEP(cmd_enter_config, AT_SETTLE_MS),
};
#define N_AT_STEPS (sizeof(at_steps) / sizeof(at_steps[0]))

// ---------- AT config state machine "bookmark" ----------
static cfg_state_t cfg_state   = CFG_SEND;
static uint8_t     cfg_idx     = 0;   // which command we're on
static uint8_t     cfg_retries = 0;
static uint32_t    cfg_t0      = 0;   // timestamp of the last state change

// ---------- LoRa transmit ----------
// Heartbeat: header + "selena" + \r\n
static const uint8_t lora_heartbeat[] = {
    LORA_ADDR_H, LORA_ADDR_L, LORA_CHANNEL,
    's', 'e', 'l', 'e', 'n', 'a',
    '\r', '\n'
};
// Buffer for forwarding CAN frames. Must NOT be changed while a transmit is in progress.
static uint8_t lora_tx_buf[3 + 1 + 8 + 2];   // header + length byte + up to 8 data + \r\n

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_Config(void);
static void AT_Config_Step(void);
static uint8_t uart_tx_ready(void);
static void handle_lora_line(const uint8_t *data, uint8_t len);
static void lora_forward_can(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// True when the UART has finished sending and can take new bytes
static uint8_t uart_tx_ready(void)
{
    return (huart2.gState == HAL_UART_STATE_READY);
}

/*
 * AT configuration, one small step per call. It NEVER waits.
 * Each call checks "is there something to do right now?" and returns immediately.
 */
static void AT_Config_Step(void)
{
    uint32_t now = HAL_GetTick();

    switch (cfg_state)
    {
    case CFG_SEND:
        if (!uart_tx_ready()) {
            break;                       // UART still busy; try again next loop pass
        }
        line_ready = 0;                  // throw away any old/stale reply
        if (HAL_UART_Transmit_IT(&huart2,
                                 (uint8_t *)at_steps[cfg_idx].cmd,
                                 at_steps[cfg_idx].len) == HAL_OK)
        {
            cfg_t0    = now;             // start the reply timeout clock
            cfg_state = CFG_WAIT_REPLY;
        }
        break;

    case CFG_WAIT_REPLY:
        if (line_ready) {
            // Optional: check FinalData for "OK" here before accepting
            line_ready = 0;
            cfg_t0     = now;            // start the settle clock
            cfg_state  = CFG_SETTLE;
        }
        else if (now - cfg_t0 > AT_REPLY_TIMEOUT_MS) {
            cfg_retries++;
            cfg_state = (cfg_retries > AT_MAX_RETRIES) ? CFG_FAILED : CFG_SEND;
        }
        break;

    case CFG_SETTLE:
        if (now - cfg_t0 >= at_steps[cfg_idx].settle_ms) {
            cfg_retries = 0;
            cfg_idx++;
            cfg_state = (cfg_idx >= N_AT_STEPS) ? CFG_DONE : CFG_SEND;
        }
        break;

    case CFG_DONE:
    case CFG_FAILED:
    default:
        break;
    }
}

/*
 * Called from main whenever a full line arrives from the LoRa module
 * after configuration is done (i.e. incoming telemetry / messages).
 */
static void handle_lora_line(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    // TODO: parse incoming LoRa data here
}

/*
 * Copy the latest CAN frame and send it over LoRa.
 * Only call when uart_tx_ready() is true, so lora_tx_buf isn't still being sent.
 */
static void lora_forward_can(void)
{
    uint8_t data[8];
    uint8_t len;

    // Briefly block interrupts so the CAN ISR can't overwrite the data mid-copy
    __disable_irq();
    len = CAN_rx_len;
    memcpy(data, CAN_rx_data, len);
    CAN_received = 0;
    __enable_irq();

    uint8_t i = 0;
    lora_tx_buf[i++] = LORA_ADDR_H;
    lora_tx_buf[i++] = LORA_ADDR_L;
    lora_tx_buf[i++] = LORA_CHANNEL;
    lora_tx_buf[i++] = len;              // tells the receiver how many data bytes follow
    memcpy(&lora_tx_buf[i], data, len);
    i += len;
    lora_tx_buf[i++] = '\r';
    lora_tx_buf[i++] = '\n';

    HAL_UART_Transmit_IT(&huart2, lora_tx_buf, i);
}

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
  MX_CAN_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start listening on the UART: the ISR will fire for each byte from now on
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  // Start CAN (this was never called in the original)
  CAN_Config();

  uint32_t last_heartbeat = 0;
  uint32_t last_fail_blink = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    // ---- Task 1: AT configuration (until done or failed) ----
    if (cfg_state != CFG_DONE && cfg_state != CFG_FAILED) {
        AT_Config_Step();
    }

    // ---- Task 2: normal operation once configured ----
    else if (cfg_state == CFG_DONE) {

        // Incoming LoRa data
        if (line_ready) {
            handle_lora_line(FinalData, final_len);
            line_ready = 0;
        }

        // Outgoing: CAN data has priority over the heartbeat
        if (uart_tx_ready()) {
            if (CAN_received) {
                lora_forward_can();
            }
            else if (now - last_heartbeat >= LORA_TX_PERIOD_MS) {
                if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)lora_heartbeat,
                                         sizeof(lora_heartbeat)) == HAL_OK) {
                    last_heartbeat = now;
                    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                }
            }
        }
    }

    // ---- Task 3: config failed -> fast blink so you can see it ----
    else {
        if (now - last_fail_blink >= FAIL_BLINK_MS) {
            last_fail_blink = now;
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }

    // Add more tasks here, written the same way: check, act if needed, never wait.
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
  * @brief  Set up the CAN filter and start CAN with RX interrupts.
  */
static void CAN_Config(void)
{
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x245 << 5;  // ID we're "looking for"
    sFilterConfig.FilterIdLow          = 0;
    sFilterConfig.FilterMaskIdHigh     = 0;           // mask 0 = don't compare any bits,
    sFilterConfig.FilterMaskIdLow      = 0;           // so this currently accepts ALL IDs
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;          // ignored on single-CAN parts

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  UART receive interrupt: runs once per received byte.
  *         Builds a line in RxData; on '\n' hands the finished line to main.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint8_t b = rx_byte;

        if (b == '\n') {
            if (!line_ready) {               // main has consumed the previous line
                memcpy(FinalData, RxData, indx);
                final_len  = indx;
                line_ready = 1;              // "mailbox flag up"
            }
            // if main hasn't consumed the last line yet, this one is dropped
            indx = 0;
        }
        else if (indx < MAX_MESSAGE_LENGTH) {
            RxData[indx++] = b;
        }
        else {
            indx = 0;                        // line too long: discard and resync
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   // re-arm for the next byte
    }
}

/**
  * @brief  If a UART error (e.g. overrun) happens, HAL stops receiving.
  *         Re-arm so we keep listening.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        indx = 0;
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/**
  * @brief  CAN receive interrupt: a new frame landed in FIFO0.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    CAN_RxHeaderTypeDef rxHeader;

    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rxHeader, CAN_rx_data) != HAL_OK) {
        Error_Handler();
    }

    if (rxHeader.IDE == CAN_ID_STD) {
        CAN_rx_len   = (uint8_t)rxHeader.DLC;
        CAN_received = 1;                    // newest frame overwrites an unsent older one
    }
    // Extended (29-bit) IDs are ignored for now
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
