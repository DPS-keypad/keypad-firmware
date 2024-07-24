/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "u8g2.h"
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

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

static u8g2_t u8g2;
int AD_RES;
char display_values1[3];
char display_values2[3];
char display_values3[3];

uint16_t pot1;
uint16_t pot2;
uint16_t pot3;

char hoursandminutes[6];

int hours = 0;
int minutes = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
                                  U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
                                  U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    HAL_Delay(1);
    break;
  case U8X8_MSG_DELAY_MILLI:
    HAL_Delay(arg_int);
    break;
  case U8X8_MSG_GPIO_DC:
    HAL_GPIO_WritePin(SPI1_DC_GPIO_Port, SPI1_DC_Pin, arg_int);
    break;
  case U8X8_MSG_GPIO_CS:
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, arg_int);
    break;
  case U8X8_MSG_GPIO_RESET:
    HAL_GPIO_WritePin(SPI1_RESET_GPIO_Port, SPI1_RESET_Pin, arg_int);
    break;
  }
  return 1;
}

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                               void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_BYTE_SEND:
    HAL_SPI_Transmit(&hspi1, (uint8_t *)arg_ptr, arg_int, 10000);
    break;
  case U8X8_MSG_BYTE_INIT:
    break;
  case U8X8_MSG_BYTE_SET_DC:
    u8x8_gpio_SetDC(u8x8, arg_int);
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
    u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
    u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
    break;
  default:
    return 0;
  }
  return 1;
}

void clearOLED()
{
  u8g2_FirstPage(&u8g2);
  do
  {
  } while (u8g2_NextPage(&u8g2));
}

void getHour()
{
  hoursandminutes[0] = (hours / 10) + 48;
  hoursandminutes[1] = (hours % 10) + 48;
  hoursandminutes[2] = ':';
  hoursandminutes[3] = (minutes / 10) + 48;
  hoursandminutes[4] = (minutes % 10) + 48;
  hoursandminutes[5] = '\0';
}

void ADC_read(void)

{
  HAL_ADC_Start(&hadc1);                // Start ADC Conversion
  HAL_ADC_PollForConversion(&hadc1, 1); // Poll ADC1 Peripheral & TimeOut = 1mSec
  pot1 = HAL_ADC_GetValue(&hadc1);      // Read ADC Conversion Result
  HAL_ADC_Start(&hadc1);                // Start ADC Conversion
  HAL_ADC_PollForConversion(&hadc1, 1); // Poll ADC1 Peripheral & TimeOut = 1mSec
  pot2 = HAL_ADC_GetValue(&hadc1);      // Read ADC Conversion Result
  HAL_ADC_Start(&hadc1);                // Start ADC Conversion
  HAL_ADC_PollForConversion(&hadc1, 1); // Poll ADC1 Peripheral & TimeOut = 1mSec
  pot3 = HAL_ADC_GetValue(&hadc1);      // Read ADC Conversion Result

  // Convert ADC value to 0-100 range
  float converted_result1 = 99 - ((pot1 * 100) / 256);
  float converted_result2 = 99 - ((pot2 * 100) / 256);
  float converted_result3 = 99 - ((pot3 * 100) / 256);

  display_values1[0] = (int)converted_result1 / 10 + 48;
  display_values1[1] = (int)converted_result1 % 10 + 48;
  display_values1[2] = '\0';

  display_values2[0] = (int)converted_result2 / 10 + 48;
  display_values2[1] = (int)converted_result2 % 10 + 48;
  display_values2[2] = '\0';

  display_values3[0] = (int)converted_result3 / 10 + 48;
  display_values3[1] = (int)converted_result3 % 10 + 48;
  display_values3[2] = '\0';
}

uint16_t RX_DATA[20];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart1, RX_DATA, 20);
}

void constructSkeleton()
{
  u8g2_FirstPage(&u8g2);

  do
  {
    // TODO add the skeleton of the display
    // clear the string
  } while (u8g2_NextPage(&u8g2));
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
  switch (GPIO_Pin)
  {
  case BUT1_Pin:
    HAL_UART_Transmit(&huart1, (uint8_t *)"Button 1 pressed", 16, 1000);
    break;
  case BUT2_Pin:
    HAL_UART_Transmit(&huart1, (uint8_t *)"Button 2 pressed", 16, 1000);
    break;
  case BUT3_Pin:
    HAL_UART_Transmit(&huart1, (uint8_t *)"Button 3 pressed", 16, 1000);
    break;
  case BUT4_Pin:
    HAL_UART_Transmit(&huart1, (uint8_t *)"Button 4 pressed", 16, 1000);
    break;
  case BUT5_Pin:
    // Code for button 5
    HAL_UART_Transmit(&huart1, (uint8_t *)"Button 5 pressed", 16, 1000);
    break;
  case BUT6_Pin:
    // Code for button 6
    HAL_UART_Transmit(&huart1, (uint8_t *)"Button 6 pressed", 16, 1000);
    break;
  case BUT7_Pin:
    // Code for button 7
    HAL_UART_Transmit(&huart1, (uint8_t *)"Button 7 pressed", 16, 1000);
    break;
  case BUT8_Pin:
    // Code for button 8
    HAL_UART_Transmit(&huart1, (uint8_t *)"Button 8 pressed", 16, 1000);
    break;
  case BUT9_Pin:
    // Code for button 9
    HAL_UART_Transmit(&huart1, (uint8_t *)"Button 9 pressed", 16, 1000);
    break;
  default:
    // Code for other pins
    break;
  }
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */



  HAL_UART_Receive_IT(&huart1, RX_DATA, 20);

  u8g2_Setup_sh1107_pimoroni_128x128_1(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi,
                                       u8x8_stm32_gpio_and_delay);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // take the first 2 values from potentiometers and convert them to string

    u8g2_FirstPage(&u8g2);
    ADC_read();

    do
    {
      u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
      getHour();
      u8g2_DrawStr(&u8g2, 50, 18, hoursandminutes);
      u8g2_DrawLine(&u8g2, 0, 20, 128, 20);
      u8g2_DrawLine(&u8g2, 20, 0, 20, 20);
      u8g2_DrawCircle(&u8g2, 10, 10, 7, U8G2_DRAW_ALL);
      u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
      u8g2_DrawStr(&u8g2, 20, 55, "Listening to:");
      u8g2_DrawStr(&u8g2, 2, 70, "Upside Down");
      u8g2_DrawStr(&u8g2, 50, 80, "by");
      u8g2_DrawStr(&u8g2, 2, 90, "Mezzosangue");
      u8g2_DrawLine(&u8g2, 0, 108, 128, 108);
      u8g2_DrawStr(&u8g2, 2, 122, "1-");
      u8g2_DrawStr(&u8g2, 14, 122, display_values1);
      u8g2_DrawStr(&u8g2, 54, 122, "2-");
      u8g2_DrawStr(&u8g2, 66, 122, display_values2);
      u8g2_DrawStr(&u8g2, 102, 122, "3-");
      u8g2_DrawStr(&u8g2, 114, 122, display_values3);
      // clear the string
    } while (u8g2_NextPage(&u8g2));

    char packet[6];
    // Constructing the packet with display values
    packet[0] = display_values1[0];
    packet[1] = display_values1[1];
    packet[2] = display_values2[0];
    packet[3] = display_values2[1];
    packet[4] = display_values3[0];
    packet[5] = display_values3[1];

    // Sending the whole packet to the serial port
    //HAL_UART_Transmit(&huart1, (uint8_t *)packet, 6, 1000);

    // Clearing the packet
    for (int i = 0; i < 6; i++)
    {
      packet[i] = '\0';
    }

    HAL_Delay(100);
    minutes++;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_RESET_Pin|SPI1_CS_Pin|SPI1_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_RESET_Pin SPI1_CS_Pin SPI1_DC_Pin */
  GPIO_InitStruct.Pin = SPI1_RESET_Pin|SPI1_CS_Pin|SPI1_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT9_Pin BUT8_Pin */
  GPIO_InitStruct.Pin = BUT9_Pin|BUT8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT7_Pin BUT6_Pin BUT5_Pin BUT4_Pin
                           BUT3_Pin BUT2_Pin BUT1_Pin */
  GPIO_InitStruct.Pin = BUT7_Pin|BUT6_Pin|BUT5_Pin|BUT4_Pin
                          |BUT3_Pin|BUT2_Pin|BUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
