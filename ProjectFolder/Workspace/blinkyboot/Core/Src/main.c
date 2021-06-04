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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void vishwaboot_jumptoApplication(void);
void vishwaboot_runBootloader(void);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
	  {
		  vishwaboot_jumptoApplication();
	  }
	  else
	  {
		  vishwaboot_runBootloader();
	  }

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void printmsg(char *format,...);
uint8_t rcv_len=0;
char somedata[] = "Hello from Bootloader\r\n";
#define BL_RX_LEN  200
uint8_t bl_rx_buffer[BL_RX_LEN];
void vishwaboot_jumptoApplication(void){
	void (*GotoApplResetAddress)(void);
	uint32_t msp_value;
	uint32_t resethandleradd;
	msp_value = *(volatile uint32_t*)APPL_START_ADDRESS;
	resethandleradd =  *(volatile uint32_t*) RESET_HANDLER_ADDRESS;
	GotoApplResetAddress = (void*)resethandleradd;
	HAL_RCC_DeInit();
	__set_MSP(msp_value);
	GotoApplResetAddress();
	//App_Start_Fun_Ptr();
}

void vishwaboot_runBootloader(void){

	HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
	HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
	HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
	HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
	memset(bl_rx_buffer,0,200);
	//here we will read and decode the commands coming from host
	//first read only one byte from the host , which is the "length" field of the command packet

	HAL_UART_Receive(&huart3,bl_rx_buffer,1,HAL_MAX_DELAY);
	rcv_len= bl_rx_buffer[0];
	HAL_UART_Receive(&huart3,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
	switch(bl_rx_buffer[1])
	{
        case BL_GET_VER:
            bootloader_handle_getver_cmd(bl_rx_buffer);
            break;
        case BL_GET_HELP:
            bootloader_handle_gethelp_cmd(bl_rx_buffer);
            break;
        case BL_GET_CID:
            bootloader_handle_getcid_cmd(bl_rx_buffer);
            break;
        case BL_GET_RDP_STATUS:
            bootloader_handle_getrdp_cmd(bl_rx_buffer);
            break;
        case BL_GO_TO_ADDR:
            bootloader_handle_go_cmd(bl_rx_buffer);
            break;
        case BL_FLASH_ERASE:
            bootloader_handle_flash_erase_cmd(bl_rx_buffer);
            break;
        case BL_MEM_WRITE:
            bootloader_handle_mem_write_cmd(bl_rx_buffer);
            break;
        case BL_EN_RW_PROTECT:
            bootloader_handle_en_rw_protect(bl_rx_buffer);
            break;
        case BL_MEM_READ:
            bootloader_handle_mem_read(bl_rx_buffer);
            break;
        case BL_READ_SECTOR_P_STATUS:
            bootloader_handle_read_sector_protection_status(bl_rx_buffer);
            break;
        case BL_OTP_READ:
            bootloader_handle_read_otp(bl_rx_buffer);
            break;
					case BL_DIS_R_W_PROTECT:
            bootloader_handle_dis_rw_protect(bl_rx_buffer);
            break;
         default:
            //printmsg("BL_DEBUG_MSG:Invalid command code received from host \n");
            break;
	}

}

void  bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer){}
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer){}
void bootloader_handle_getcid_cmd(uint8_t *pBuffer){}
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer){}
void bootloader_handle_go_cmd(uint8_t *pBuffer){}
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer){}
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer){}
void bootloader_handle_en_rw_protect(uint8_t *pBuffer){}
void bootloader_handle_mem_read (uint8_t *pBuffer){}
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer){}
void bootloader_handle_read_otp(uint8_t *pBuffer){}
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer){}

void bootloader_send_ack(uint8_t command_code, uint8_t follow_len){}
void bootloader_send_nack(void){}

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host){}
uint8_t get_bootloader_version(void){}
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len){}

uint16_t get_mcu_chip_id(void){}
uint8_t get_flash_rdp_level(void){}
uint8_t verify_address(uint32_t go_address){}
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector){}
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len){}

uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable){}

uint16_t read_OB_rw_protection_status(void){}
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
