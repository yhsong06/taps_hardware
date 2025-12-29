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
#include "stdio.h"
#include "stm32_SPI_IIM42652_drv_ex.h"
#include "stm32_SPI_IIM42652_drv_ex.h"
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
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rxuart;
uint16_t data_i2s[2];
volatile uint8_t full_i2s, half_i2s;
volatile uint8_t tim_flag;
volatile int32_t sample_i2s;
uint8_t ledflag = 0;
uint8_t uart_flag = 0;
uint8_t mic_data_reset = 0;
uint8_t send_flag = 0;

uint8_t str_buffer[8];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(uint8_t reg, uint8_t bufp);
static int32_t platform_read(uint8_t reg, uint8_t *bufp, uint16_t len);
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

//  HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_SET);


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOA, SPI3_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, SPI3_CS_Pin, GPIO_PIN_RESET);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* IIM42652 soft reset */
  platform_write(IIM_REG_DEVICE_CONFIG, 0x01); HAL_Delay(10); // Reset enable	//check OK_2022.12.25
  platform_write(IIM_REG_DEVICE_CONFIG, 0x00); HAL_Delay(10); // Reset disable	//check OK_2022.12.25

  /* IIM42652 Bank 0 */
  platform_write(IIM_REG_REG_BANK_SEL, 0x00); HAL_Delay(1); // Bank 0 setting	//check OK_2022.12.25
  platform_write(IIM_REG_INT_CONFIG, 0x03); HAL_Delay(1);						//check OK_2022.12.25
  platform_write(IIM_REG_FIFO_CONFIG, 0x00); HAL_Delay(1); // bypass mode		//check OK_2022.12.25
  // platform_write(IIM_REG_INT_STATUS, 0x80); HAL_Delay(1); // read INT status	//check OK_2022.12.25
  // platform_write(IIM_REG_FIFO_DATA, 0x00); HAL_Delay(1); // read FIFO data	//check OK_2022.12.25
  platform_write(IIM_REG_INTF_CONFIG0, 0x33); HAL_Delay(1);						//check OK_2022.12.25
  platform_write(IIM_REG_PWR_MGMT0, 0x33); HAL_Delay(1);						//check OK_2022.12.25
  platform_write(IIM_REG_ACCEL_CONFIG0, 0x42); HAL_Delay(1); 					//check OK_2022.12.25
  // IIM_REG_ACCEL_CONFIG0   0x43: 8 kHz --> real: 7.4 kHz,   0x42: 16 kHz --> real: 14.8 kHz
  platform_write(IIM_REG_GYRO_ACCEL_CONFIG0, 0x01); HAL_Delay(1);				//check OK_2022.12.25
  platform_write(IIM_REG_ACCEL_CONFIG1, 0x0D); HAL_Delay(1);					//check OK_2022.12.25
  platform_write(IIM_REG_FIFO_CONFIG1, 0x95); HAL_Delay(1);						//check OK_2022.12.25
  platform_write(IIM_REG_FIFO_CONFIG2, 0x00); HAL_Delay(1); //threshold			//check OK_2022.12.25
  platform_write(IIM_REG_FIFO_CONFIG3, 0x00); HAL_Delay(1); //threshold			//check OK_2022.12.25
  platform_write(IIM_REG_INT_CONFIG1, 0x70); HAL_Delay(1);						//check OK_2022.12.25
  platform_write(IIM_REG_INT_SOURCE0, 0x08); HAL_Delay(1);						//check OK_2022.12.25

  /* IIM42652 Bank 1 */
  platform_write(IIM_REG_REG_BANK_SEL, 0x01); HAL_Delay(1); // Bank 1 setting	//check OK_2022.12.25
  platform_write(IIM_REG_SENSOR_CONFIG0, 0x38); HAL_Delay(1);					//check OK_2022.12.25
  platform_write(IIM_REG_GYRO_CONFIG_STATIC2, 0x03); HAL_Delay(1);				//check OK_2022.12.25
  platform_write(IIM_REG_INTF_CONFIG5, 0x20); HAL_Delay(1);						//check OK_2022.12.25
  platform_write(IIM_REG_INTF_CONFIG6, 0x80); HAL_Delay(1);						//check OK_2022.12.25

  /* IIM42652 Bank 2 */
  platform_write(IIM_REG_REG_BANK_SEL, 0x02); HAL_Delay(1); // Bank 2 setting	//check OK_2022.12.25
  platform_write(IIM_REG_ACCEL_CONFIG_STATIC2, 0x7E); HAL_Delay(1);				//check OK_2022.12.25
  platform_write(IIM_REG_ACCEL_CONFIG_STATIC3, 0x80); HAL_Delay(1);				//check OK_2022.12.25
  platform_write(IIM_REG_ACCEL_CONFIG_STATIC4, 0x3F); HAL_Delay(1);				//check OK_2022.12.25

  /* IIM42652 Bank 3 */
  platform_write(IIM_REG_REG_BANK_SEL, 0x03); HAL_Delay(1); // Bank 3 setting	//check OK_2022.12.25
  platform_write(IIM_REG_PU_PD_CONFIG1, 0x01); HAL_Delay(1);					//check OK_2022.12.25
  platform_write(IIM_REG_PU_PD_CONFIG2, 0x00); HAL_Delay(1);					//check OK_2022.12.25

  /* IIM42652 Bank 4 */
  platform_write(IIM_REG_REG_BANK_SEL, 0x04); HAL_Delay(1); // Bank 4 setting	//check OK_2022.12.25


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* FIFO setting - threshold value - INT1*/
//  platform_write(IIM_REG_REG_BANK_SEL, 0x00); HAL_Delay(1); // Bank 0 setting			//check OK_2022.12.25
//  platform_write(IIM_REG_FIFO_CONFIG, 0x40); HAL_Delay(1); // stream-to-FIFO mode		//check OK_2022.12.25
//  platform_write(IIM_REG_SIGNAL_PATH_RESET, 0x02); HAL_Delay(1); // reset FIFO			//check OK_2022.12.25
//  platform_write(IIM_REG_INTF_CONFIG0, 0x73); HAL_Delay(1);								//check OK_2022.12.25
//  platform_write(IIM_REG_FIFO_CONFIG1, 0x21); HAL_Delay(1);								//check OK_2022.12.25
//  platform_write(IIM_REG_FIFO_CONFIG2, 0x00); HAL_Delay(1);								//check OK_2022.12.25
//  platform_write(IIM_REG_FIFO_CONFIG3, 0x00); HAL_Delay(1);								//check OK_2022.12.25
//  platform_write(IIM_REG_FIFO_CONFIG2, 0x64); HAL_Delay(1);								//check OK_2022.12.25
//  platform_write(IIM_REG_FIFO_CONFIG3, 0x00); HAL_Delay(1);								//check OK_2022.12.25
//  platform_write(IIM_REG_INT_SOURCE0, 0x04); HAL_Delay(1);								//check OK_2022.12.25

  /* related FIFO status and read data */
  /*
	  IIM_REG_INT_STATUS, 0x40	// read INT status
	  The bit clears to 0 after the register has been read.

	  IIM_REG_FIFO_COUNTH, IIM_REG_FIFO_COUNTL	// read FIFO count (number of records)
	  Reading this byte latches the data for both FIFO_COUNTH, and FIFO_COUNTL.

	  IIM_REG_FIFO_DATA		// read FIFO data
  	  1packet (8 bytes): header( 1byte) + accelerometer (6bytes) + temperature (1byte)
   */


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  platform_write(IIM_REG_REG_BANK_SEL, 0x00); HAL_Delay(1); // Bank 0 reading
  HAL_GPIO_WritePin(GPIOA, SPI3_CS_Pin, GPIO_PIN_RESET);

  int16_t tempVal_x = 0;
  int16_t tempVal_y = 0;
  int16_t tempVal_z = 0;


  int16_t tempVal_mic = 0;
//  int32_t temp_mic = 0;


//  uint8_t str_buffer[8];
  uint8_t str_rx_buffer;
  //int16_t buf16[2];

  uint16_t count = 0;





  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)data_i2s, sizeof(data_i2s)/2);
  uint16_t data;
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)data_i2s, sizeof(data_i2s)/2);




//  HAL_I2S_Receive(&hi2s2, (uint16_t *)data_i2s, sizeof(data_i2s), 100);

  HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, Red_Pin, GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart1, &rxuart, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (uart_flag == 1)
	  {
		  if(send_flag == 1)
		  {
			  HAL_UART_Transmit(&huart1,str_buffer,sizeof(str_buffer), 100);
			  send_flag = 0;
		  }
//					  HAL_UART_Receive_IT(&huart1, &rxuart, 1);
//		  HAL_UART_Transmit(&huart1,str_buffer,sizeof(str_buffer), 100);
	  }


    // Individual board
//	str_buffer[8] = 0xA5;
//	    HAL_UART_Receive_IT(&huart1, &rxuart, 1);


//	  HAL_UART_Receive_IT(&huart1, &rxuart, 1);
//		if(tim_flag == 1)
//		{
//
//			// reading acoustic microphone and accelerometer z
//			// accelerometer - 8 kHz, acoustic microphone - 16 kHz
//			// Storing by UART communication method
//
//			  tim_flag = 0;
//
//			  sample_i2s = (int32_t) ((data_i2s[0] << 16) | data_i2s[1]);
//			  temp_mic = (sample_i2s >> 8);
//
//			  if(mic_data_reset == 0)
//			  {
////				  str_buffer[0] = (uint8_t)(temp_mic >> 16);
////				  str_buffer[1] = (uint8_t)(temp_mic >> 8);
////				  str_buffer[2] = (uint8_t)(temp_mic);
//				  str_buffer[0] = (uint8_t)(0x11);
//				  str_buffer[1] = (uint8_t)(0x11);
//				  str_buffer[2] = (uint8_t)(0x11);
//
//				  platform_read(IIM_REG_ACCEL_DATA_Z1_UI, rxBuf, 2);
////				  str_buffer[6] = (uint8_t)(rxBuf[0]);
////				  str_buffer[7] = (uint8_t)(rxBuf[1]);
//				  str_buffer[6] = (uint8_t)(0x00);
//				  str_buffer[7] = (uint8_t)(0x00);
//
//				  mic_data_reset = 1;
//			  }
//			  else if(mic_data_reset == 1)
//			  {
////				  str_buffer[3] = (uint8_t)(temp_mic >> 16);
////				  str_buffer[4] = (uint8_t)(temp_mic >> 8);
////				  str_buffer[5] = (uint8_t)(temp_mic);
//				  str_buffer[3] = (uint8_t)(0x22);
//				  str_buffer[4] = (uint8_t)(0x22);
//				  str_buffer[5] = (uint8_t)(0x22);
//
//				  mic_data_reset = 0;
//
//				  if (uart_flag == 1)
//				  {
////					  HAL_UART_Receive_IT(&huart1, &rxuart, 1);
//					  HAL_UART_Transmit(&huart1,str_buffer,sizeof(str_buffer), 100);
//				  }
//
//			  }
//
//
//		}


  //	if(tim_flag == 1)
//	{
////		  HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_SET);
//
//		// reading acoustic microphone and accelerometer z
//		// accelerometer - 8 kHz, acoustic microphone - 16 kHz
//		// Storing by UART communication method
//
//		  tim_flag = 0;
//
//		  sample_i2s = (int32_t) ((data_i2s[0] << 16) | data_i2s[1]);
//		  temp_mic = (sample_i2s >> 8);
//
//		  if(mic_data_reset == 0)
//		  {
//			  str_buffer[0] = (uint8_t)(temp_mic >> 16);
//			  str_buffer[1] = (uint8_t)(temp_mic >> 8);
//			  str_buffer[2] = (uint8_t)(temp_mic);
//
//			  platform_read(IIM_REG_ACCEL_DATA_Z1_UI, rxBuf, 2);
//			  str_buffer[6] = (uint8_t)(rxBuf[0]);
//			  str_buffer[7] = (uint8_t)(rxBuf[1]);
//
//			  mic_data_reset = 1;
//		  }
//		  else if(mic_data_reset == 1)
//		  {
//			  str_buffer[3] = (uint8_t)(temp_mic >> 16);
//			  str_buffer[4] = (uint8_t)(temp_mic >> 8);
//			  str_buffer[5] = (uint8_t)(temp_mic);
//
//			  mic_data_reset = 0;
//
//			  GPIO_PinState pin_state = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
//			  if(pin_state == GPIO_PIN_RESET)
////			  if(str_rx_buffer=='0x01')
//			  {
////				  HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_RESET);
//				  HAL_UART_Transmit(&huart1,str_buffer,sizeof(str_buffer), 100);
////				  HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_SET);
//
////				  pin_state = GPIO_PIN_SET;
////				  count++;
////				  if(count == 4000)
////				  {
////					  HAL_GPIO_TogglePin(GPIOA, Red_Pin);
////					  count = 0;
////				  }
//			  }
//
//		  }

		// reading accelerometer x, y, z

//		  tim_flag = 0;
//
//		  sample_i2s = (int32_t) ((data_i2s[0] << 16) | data_i2s[1]);
//
//		  platform_read(IIM_REG_ACCEL_DATA_X1_UI, rxBuf, 6);
//
//		  str_buffer[0] = rxBuf[0];
//		  str_buffer[1] = rxBuf[1];
//		  str_buffer[2] = rxBuf[2];
//		  str_buffer[3] = rxBuf[3];
//		  str_buffer[4] = rxBuf[4];
//		  str_buffer[5] = rxBuf[5];
//
//
//		  GPIO_PinState pin_state = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
//		  if(pin_state == GPIO_PIN_RESET)
//		  {
//			  HAL_UART_Transmit(&huart1,str_buffer,sizeof(str_buffer), 100);
//			  pin_state = GPIO_PIN_SET;
//
//			  count++;
//			  if(count == 4000)
//			  {
//				  HAL_GPIO_TogglePin(GPIOA, Red_Pin);
//				  count = 0;
//			  }
//		  }
//	}
//	HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_RESET);

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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV4;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.CPOL = I2S_CPOL_HIGH;
  hi2s2.Init.ClockSource = I2S_CLOCK_SYSCLK;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|Blue_Pin|Red_Pin|SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 Blue_Pin Red_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|Blue_Pin|Red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM2)
//	{
//		if(ledflag ==1)
//		{
//			HAL_GPIO_TogglePin(GPIOA, Red_Pin);
//		}
//	}
//
//}




//void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
//{
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	tim_flag = 1;
//}
//void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
//{
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	tim_flag = 1;
//
//}
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	  tim_flag = 1;
	// reading acoustic microphone and accelerometer z
	// accelerometer - 8 kHz, acoustic microphone - 16 kHz
	// Storing by UART communication method

//	  tim_flag = 0;

	  int32_t temp_mic = 0;
//	  uint8_t str_buffer[8];
	  uint8_t rxBuf[2];

//	  str_buffer[8] = 0xa5;

	  sample_i2s = (int32_t) ((data_i2s[0] << 16) | data_i2s[1]);
	  temp_mic = (sample_i2s >> 8);

	  if(mic_data_reset == 0)
	  {
		  str_buffer[0] = (uint8_t)(temp_mic >> 16);
		  str_buffer[1] = (uint8_t)(temp_mic >> 8);
		  str_buffer[2] = (uint8_t)(temp_mic);
//		  str_buffer[0] = (uint8_t)(0x11);
//		  str_buffer[1] = (uint8_t)(0x11);
//		  str_buffer[2] = (uint8_t)(0x11);

		  platform_read(IIM_REG_ACCEL_DATA_Z1_UI, rxBuf, 2);
		  str_buffer[6] = (uint8_t)(rxBuf[0]);
		  str_buffer[7] = (uint8_t)(rxBuf[1]);
//		  str_buffer[6] = (uint8_t)(0x00);
//		  str_buffer[7] = (uint8_t)(0x00);

		  mic_data_reset = 1;
	  }
	  else if(mic_data_reset == 1)
	  {
		  str_buffer[3] = (uint8_t)(temp_mic >> 16);
		  str_buffer[4] = (uint8_t)(temp_mic >> 8);
		  str_buffer[5] = (uint8_t)(temp_mic);
//		  str_buffer[3] = (uint8_t)(0x22);
//		  str_buffer[4] = (uint8_t)(0x22);
//		  str_buffer[5] = (uint8_t)(0x22);

		  mic_data_reset = 0;

		  send_flag = 1;
	  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(uint8_t reg, uint8_t bufp)
{
  HAL_GPIO_WritePin(GPIOA, SPI3_CS_Pin, GPIO_PIN_RESET);
  uint8_t addr_data_bit[2] = {reg, bufp};
  HAL_SPI_Transmit(&hspi3, addr_data_bit, 2, 1000); // transfer address
  HAL_GPIO_WritePin(GPIOA, SPI3_CS_Pin, GPIO_PIN_SET);

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(uint8_t reg, uint8_t *bufp, uint16_t len)
{
  reg |= 0x80;
  HAL_GPIO_WritePin(GPIOA, SPI3_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &reg, 1, 1000); // transfer address
  HAL_SPI_Receive(&hspi3, bufp, len, 1000); // read data
  HAL_GPIO_WritePin(GPIOA, SPI3_CS_Pin, GPIO_PIN_SET);

  return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	HAL_UART_Receive_IT(&huart1, &rxuart, 1);
//	HAL_UART_Transmit(&huart1, &rxuart, sizeof(rxuart), 100);
	if(rxuart == 's')
	{
		if(uart_flag == 0)
		{
			uart_flag = 1;
			HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_SET);
		}

		else
		{
			uart_flag = 0;
			HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_RESET);
		}
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
