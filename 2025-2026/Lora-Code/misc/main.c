/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h" // For CDC_Transmit_FS()

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART2_RX_BUFFER_SIZE 1 // Size of the buffer for single byte reception via interrupt
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- SD Card / FatFs Variables ---
FATFS SDFatFs;     // File system object for SD card logical drive
FIL MyFile;        // File object variable
extern  char SDPath[4];    // SD card logical drive path, default "0:/"


uint8_t uart2_rx_buffer[UART2_RX_BUFFER_SIZE]; // Buffer for UART2 reception via interrupt (for USB CDC forwarding)
uint8_t usb_cdc_msg_buffer[128]; // Buffer for status messages over USB CDC
uint8_t uart2_rx_buffer[UART2_RX_BUFFER_SIZE]; // Buffer for UART2 reception via interrupt (for USB CDC forwarding)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void logToSDCard(const char* message); // Prototype for our logging function
HAL_StatusTypeDef CAN_TransmitMessage(uint32_t id, uint8_t id_type, uint8_t dlc, uint8_t* data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Sends an AT command string to the LoRa module via UART.
  * @param  cmd: Pointer to the command string (without \r\n).
  * @retval None
  */
void sendATCommand(const char *cmd) {
  char full_cmd[64];
  snprintf(full_cmd, sizeof(full_cmd), "%s\r\n", cmd); // Append CR+LF

  // Transmit the command
  HAL_UART_Transmit(&huart2, (uint8_t*)full_cmd, strlen(full_cmd), HAL_MAX_DELAY);

  // Simple delay to allow module processing time.
  // A robust implementation would wait for and parse the response.
  HAL_Delay(200);

  // Optional: Add code here to receive and check the response from lora_rx_buffer
  // using HAL_UART_Receive or HAL_UART_Receive_IT/DMA.
}

/**
  * @brief  Configures the LoRa module using AT commands.
  * @param  None
  * @retval None
  */
void loraConfigure(void) {
  // Wait for the LoRa module to power up and stabilize. Adjust delay if needed.
  HAL_Delay(3000);

  // --- Enter AT Mode (If required by your module, e.g., EBYTE E32) ---
  // Uncomment or adapt if needed. Many modules are always in AT command mode initially
  // or use a dedicated pin.
  HAL_UART_Transmit(&huart2, (uint8_t*)"+++\r\n", 5, HAL_MAX_DELAY);
  HAL_Delay(200); // Wait for "OK" or similar response

  // --- Configure LoRa Parameters (Example Commands - VERIFY WITH YOUR MODULE DATASHEET!) ---
  sendATCommand("AT+BAUD4");      // Set UART baud rate to 9600 bps. Check module docs for index '4'.
  sendATCommand("AT+MODE1");      // Set transmission mode (e.g., Fixed-point). Check module docs.
  sendATCommand("AT+LEVEL0");     // Set air data rate level. Check module docs.
  sendATCommand("AT+CHANNEL03");  // Set frequency channel (e.g., 433.2 MHz). Check module docs & local regulations.
  sendATCommand("AT+MAC0a,01");   // Set device address (e.g., 0x0A01). Check format.
  sendATCommand("AT+POWE22");     // Set TX power (e.g., 22 dBm). Check allowed limits.
  sendATCommand("AT+SF12");       // Set Spreading Factor (e.g., SF12).

  // --- Reset/Save Configuration (If required by your module) ---
  sendATCommand("AT+RESET");      // Apply settings and reset module.
  HAL_Delay(1000);                // Wait for module reboot.

  // --- UART Re-initialization (IMPORTANT if baud rate was changed) ---
  // If an AT command changed the LoRa module's UART baud rate, you MUST
  // reconfigure the STM32's UART peripheral to match before further communication.
  // Example (if AT+BAUDx set it to 115200):
  // LORA_UART->Init.BaudRate = 115200;
  // if (HAL_UART_Init(LORA_UART) != HAL_OK) {
  //   Error_Handler();
  // }
  // Since we used AT+BAUD4 (9600) which matches MX_USART2_UART_Init, no re-init is needed here.
}

/**
  * @brief  UART Error Callback.
  * @param  huart: UART handle pointer.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // An error occurred on UART2 after "Power on" or during main loop
        //uint32_t error_code = HAL_UART_GetError(huart);

        // Halt execution and blink LED rapidly to indicate an error occurred
         //__disable_irq(); // Stop everything
         //while(1)
         //{
         //    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2); // Use your LED pin/port
         //    HAL_Delay(50); // Blink fast
         //}

        /* Optional: More advanced recovery (use only if halting isn't desired)
        // Clear specific error flags (example for Overrun)
        // __HAL_UART_CLEAR_OREFLAG(huart);
        // Re-arm reception AFTER clearing the error
        */
        HAL_UART_Receive_IT(&huart2, uart2_rx_buffer, UART2_RX_BUFFER_SIZE);

    }
}


/**
  * @brief  Rx Transfer completed callback for UART.
  * @param  huart: UART handle pointer.
  * @retval None         __disable_irq(); // Stop everything
         while(1)
         {
             HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2); // Use your LED pin/port
             HAL_Delay(50); // Blink fast
         }
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // Check if the interrupt is from the LoRa UART (USART2)
  if (huart->Instance == USART2)
  {
    // Forward the received byte(s) to the USB Virtual COM Port
    // Assumes USB CDC Full Speed. Use CDC_Transmit_HS for High Speed.
    CDC_Transmit_FS(uart2_rx_buffer, UART2_RX_BUFFER_SIZE);

    // Re-enable UART reception interrupt for the next byte(s)
    HAL_UART_Receive_IT(&huart2, uart2_rx_buffer, UART2_RX_BUFFER_SIZE);
  }
}

/**
  * @brief  Logs a message to a file on the SD card.
  * @param  message: The string message to write.
  * @retval None
  */
void logToSDCard(const char* message)
{
    FRESULT res;      // FatFs function common result code
    uint32_t byteswritten; // File write counts

    // --- Open file for appending ---
    // FA_OPEN_APPEND: Opens the file if it exists and moves the pointer to the end. Creates it if it doesn't exist.
    // FA_WRITE:       Required to enable writing.
    // FA_CREATE_ALWAYS: Use this instead of FA_OPEN_APPEND if you want to overwrite the file every time.
    res = f_open(&MyFile, "log.txt", FA_OPEN_APPEND | FA_WRITE);

    if (res != FR_OK)
    {
        // Handle error - e.g., send message via USB CDC
        snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "SD Card: Error opening/creating file! (%d)\r\n", res);
        CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));
        // Optionally blink an error LED rapidly
        // Error_Handler(); // Or just continue
    }
    else
    {
        // --- Write data to the file ---
        res = f_write(&MyFile, message, strlen(message), (UINT*)&byteswritten);

        if (res != FR_OK || byteswritten != strlen(message))
        {
            // Handle error - e.g., send message via USB CDC
            snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "SD Card: Error writing to file or not all bytes written! (%d)\r\n", res);
            CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));
            // Optionally blink an error LED
        }
        else
        {
             // Optional: Send confirmation via USB
             // snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "SD Card: Wrote %lu bytes.\r\n", byteswritten);
             // CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));

             // --- IMPORTANT: Close the file to save changes ---
             // f_sync(&MyFile) can be used to flush without closing, but f_close does this too.
             res = f_close(&MyFile);

             if (res != FR_OK)
             {
                // Handle error - e.g., send message via USB CDC
                snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "SD Card: Error closing file! (%d)\r\n", res);
                CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));
             }
        }
    }
}

/**
  * @brief  Transmits a CAN message using CAN1.
  * @param  id: The CAN identifier (Standard or Extended).
  * @param  id_type: Specifies if the ID is Standard (CAN_ID_STD) or Extended (CAN_ID_EXT).
  * @param  dlc: Data Length Code (0 to 8).
  * @param  data: Pointer to the data payload array (size must match DLC).
  * @retval HAL_StatusTypeDef: HAL_OK if message added successfully,
  * HAL_BUSY if no Tx mailbox available,
  * HAL_ERROR for other errors (e.g., invalid DLC, CAN peripheral error).
  * Note: HAL_OK only means the message was accepted by a Tx mailbox, not that it was successfully sent on the bus.
  */
HAL_StatusTypeDef CAN_TransmitMessage(uint32_t id, uint8_t id_type, uint8_t dlc, uint8_t* data)
{
    CAN_TxHeaderTypeDef   txHeader;      // Local variable for Tx Header
    uint32_t              txMailbox;     // Local variable for Tx Mailbox
    HAL_StatusTypeDef     status;        // To store HAL function results

    // Basic input validation
    if (dlc > 8) {
        return HAL_ERROR; // Invalid DLC
    }
    if (data == NULL && dlc > 0) {
        return HAL_ERROR; // Null data pointer with non-zero DLC
    }

    // --- Configure the Tx Header ---
    if (id_type == CAN_ID_STD) {
        txHeader.StdId = id;
        txHeader.ExtId = 0;         // Not used
        txHeader.IDE = CAN_ID_STD;
    } else if (id_type == CAN_ID_EXT) {
        txHeader.StdId = 0;         // Not used
        txHeader.ExtId = id;
        txHeader.IDE = CAN_ID_EXT;
    } else {
        return HAL_ERROR; // Invalid ID type
    }

    txHeader.RTR = CAN_RTR_DATA;        // Sending Data frame
    txHeader.DLC = dlc;                 // Set Data Length Code from parameter
    txHeader.TransmitGlobalTime = DISABLE;

    // --- Attempt to transmit ---
    // Check if a Tx Mailbox is free
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        // Add the message to the Tx Mailbox
        status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);
        // status will be HAL_OK if successfully added, or HAL_ERROR if something went wrong during addition
    }
    else
    {
        // No Tx mailbox was free
        status = HAL_BUSY; // Indicate that mailboxes were full
    }

    return status; // Return the status (HAL_OK, HAL_ERROR, or HAL_BUSY)
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	FRESULT res;
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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);

  res = f_mount(&SDFatFs, (TCHAR const*)SDPath, 1); // 1=Mount immediately
    if (res != FR_OK)
    {
        // Mounting failed, handle error
        snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "SD Card: Mount Failed! Error Code: %d\r\nPlease check connection and card format (FAT32 recommended).\r\n", res);
        CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));
        // You might want to halt or prevent further SD operations here
        // Error_Handler();
    }
    else
    {
        // Mount successful
        snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "SD Card: Mounted Successfully!\r\n");
        CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));

        // Optional: Check available space or perform other initial checks
        // DWORD free_clusters;
        // FATFS* fs;
        // f_getfree("", &free_clusters, &fs); // Get free space on default drive
        // uint64_t free_space = (uint64_t)free_clusters * fs->csize * _MAX_SS;
        // snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "SD Card: Free space: %llu bytes\r\n", free_space);
        // CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));

        // --- Write an initial message to the SD card ---
        logToSDCard("--- System Initialized ---\r\n");
    }

  // The HAL_UART_RxCpltCallback function will handle received data.
  HAL_UART_Receive_IT(&huart2, uart2_rx_buffer, UART2_RX_BUFFER_SIZE);

  loraConfigure();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // --- Define CAN message parameters ---
      uint32_t myCanID = 0x456;          // Example Standard ID
      uint8_t myDataPayload[] = { 0x11, 0x22, 0x33, 0x44, 0x55 }; // Example 5 bytes of data
      uint8_t myDLC = sizeof(myDataPayload); // DLC = 5 bytes

      // --- Call the transmit function ---
      HAL_StatusTypeDef tx_status = CAN_TransmitMessage(myCanID, CAN_ID_STD, myDLC, myDataPayload);

      // --- Handle the transmission result ---
      if (tx_status == HAL_OK)
      {
          // Message successfully added to a Tx Mailbox
          // Optional: Send confirmation via USB CDC
          snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "CAN: Tx message 0x%lX added OK.\r\n", myCanID);
          CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));
      }
      else if (tx_status == HAL_BUSY)
      {
          // No Tx Mailbox was available
          snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "CAN: Tx Mailboxes full, message 0x%lX skipped.\r\n", myCanID);
          CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));
      }
      else // HAL_ERROR or other
      {
          // An error occurred during parameter setup or HAL_CAN_AddTxMessage call
          snprintf((char*)usb_cdc_msg_buffer, sizeof(usb_cdc_msg_buffer), "CAN: Error adding Tx message 0x%lX! Status: %d\r\n", myCanID, tx_status);
          CDC_Transmit_FS(usb_cdc_msg_buffer, strlen((char*)usb_cdc_msg_buffer));
          // You might want to check HAL_CAN_GetError(&hcan1) here for more details
      }
	  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2); // Heartbeat LED

	  // Fixed-point transmission example (address 0a01, channel 03, data "ABCD")
	  //uint8_t packet[] = {0x0A, 0x01, 0x03, 'A', 'B', 'C', 'D'};
	  //HAL_UART_Transmit(&huart2, packet, sizeof(packet), HAL_MAX_DELAY);

	  HAL_Delay(1000); // Send every 1 second
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 28;
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 128;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	__disable_irq();

	// Send error message via USB CDC if possible/initialized
	const char* errMsg = "!!! SYSTEM ERROR !!!\r\n";
	CDC_Transmit_FS((uint8_t*)errMsg, strlen(errMsg));
	HAL_Delay(100); // Allow time for transmission

	while (1)
	{
	    // Blink LED very fast to indicate critical error
	    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2); // Use your LED pin/port
	    HAL_Delay(100);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
