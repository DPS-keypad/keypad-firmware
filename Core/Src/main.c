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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "u8g2.h"
#include "time.h"
#include "display.h"  // Nuovo include per il modulo display
#include "serial.h"  // Nuovo include per il modulo seriale

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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Rimuovi la variabile u8g2 (ora gestita nel modulo display)
// static u8g2_t u8g2;

// Struttura per lo stato del display
static DISPLAY_State_t display_state;

// ADC result (temporary)
int AD_RES;

// Display values for potentiometers
char display_values1[3];
char display_values2[3];
char display_values3[3];

// ADC values from potentiometers
uint16_t pot1;
uint16_t pot2;
uint16_t pot3;

// Packet to send through UART
char packet[5];

// Song and artist variables
char song[SERIAL_SONG_SIZE];
char artist[SERIAL_ARTIST_SIZE];

// Default values for song and artist
const char default_song[SERIAL_SONG_SIZE] = "No song playing";
const char default_artist[SERIAL_ARTIST_SIZE] = "No artist playing";

// Buffer to store the received data from UART
uint8_t RX_DATA[44];

// Last key pressed
char last_key[2] = " ";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
// Rimuovi le funzioni spostate nel modulo display:
// - uint8_t u8x8_stm32_gpio_and_delay(...)
// - uint8_t u8x8_byte_4wire_hw_spi(...)
// - void clearOLED()
// - void constructSkeleton()
// - void updateScreen()

void ADC_read(void);

/* RIMUOVI la funzione sendThroughUART() - ora gestita dal modulo seriale */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Callback chiamato quando vengono ricevuti nuovi dati sulla canzone
 */
static void OnSongDataReceived(const char* new_song, const char* new_artist)
{
    // Copia i dati ricevuti nelle variabili locali
    strncpy(song, new_song, sizeof(song));
    strncpy(artist, new_artist, sizeof(artist));
    
    // Aggiorna lo stato del display
    strncpy(display_state.song, song, sizeof(display_state.song));
    strncpy(display_state.artist, artist, sizeof(display_state.artist));
    DISPLAY_UpdateState(&display_state);
}

/**
 * @brief Callback chiamato quando viene ricevuta l'ora iniziale
 */
static void OnInitialTimeReceived(int hours, int minutes)
{
    // Imposta l'ora utilizzando il modulo TIME
    TIME_Set(hours, minutes);
    
    // Aggiorna l'ora nel display
    char time_str[6];
    TIME_GetFormatted(time_str);
    strcpy(display_state.time, time_str);
    DISPLAY_UpdateState(&display_state);
}

/**
  * @brief Timer period elapsed callback
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Codice per TIM2 e TIM3 invariato
    if (htim->Instance == TIM2) {
        TIME_IncrementMinute();
        
        // Aggiorna l'ora nella struttura dello stato del display
        char time_str[6];
        TIME_GetFormatted(time_str);
        strcpy(display_state.time, time_str);
        
        // Aggiorna lo stato del display
        DISPLAY_UpdateState(&display_state);
    }
    else if (htim->Instance == TIM3) {
        DISPLAY_TimerHandler();
    }
}

/**
  * @brief  GPIO EXTI callback function.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_interrupt_time_1 = 0;
  uint32_t interrupt_time_1 = HAL_GetTick();

  if (interrupt_time_1 - last_interrupt_time_1 > 200) // Debounce time of 200ms
  {
    switch (GPIO_Pin)
    {
    case BUT1_Pin:
      HAL_UART_Transmit(&huart1, (uint8_t *)"key1\0", 5, 1000);
      last_key[0] = '1';
      break;
    case BUT2_Pin:
      HAL_UART_Transmit(&huart1, (uint8_t *)"key2\0", 5, 1000);
      last_key[0] = '2';
      break;
    case BUT3_Pin:
      HAL_UART_Transmit(&huart1, (uint8_t *)"key3\0", 5, 1000);
      last_key[0] = '3';
      break;
    case BUT4_Pin:
      HAL_UART_Transmit(&huart1, (uint8_t *)"key4\0", 5, 1000);
      last_key[0] = '4';
      break;
    case BUT5_Pin:
      HAL_UART_Transmit(&huart1, (uint8_t *)"key5\0", 5, 1000);
      last_key[0] = '5';
      break;
    case BUT6_Pin:
      HAL_UART_Transmit(&huart1, (uint8_t *)"key6\0", 5, 1000);
      last_key[0] = '6';
      break;
    case BUT7_Pin:
      HAL_UART_Transmit(&huart1, (uint8_t *)"key7\0", 5, 1000);
      last_key[0] = '7';
      break;
    case BUT8_Pin:
      HAL_UART_Transmit(&huart1, (uint8_t *)"key8\0", 5, 1000);
      last_key[0] = '8';
      break;
    case BUT9_Pin:
      HAL_UART_Transmit(&huart1, (uint8_t *)"key9\0", 5, 1000);
      last_key[0] = '9';
      break;
    default:
      break;
    }
    
    // Aggiorna lo stato del display
    strcpy(display_state.last_key, last_key);
    DISPLAY_UpdateState(&display_state);
  }

  last_interrupt_time_1 = interrupt_time_1;
}

/**
  * @brief  UART receive complete callback function.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Delega la gestione al modulo seriale
    SERIAL_RxCpltCallback(huart);
}

/**
 * @brief Reads the ADC values.
 */
void ADC_read(void)
{
  static uint32_t last_update_time_4 = 0;
  uint32_t current_time_4 = HAL_GetTick();

  if (current_time_4 - last_update_time_4 >= 100) 
  {
    last_update_time_4 = current_time_4;

    HAL_ADC_Start(&hadc1);                
    HAL_ADC_PollForConversion(&hadc1, 1); 
    pot1 = HAL_ADC_GetValue(&hadc1);      
    HAL_ADC_Start(&hadc1);                
    HAL_ADC_PollForConversion(&hadc1, 1); 
    pot2 = HAL_ADC_GetValue(&hadc1);      
    HAL_ADC_Start(&hadc1);                
    HAL_ADC_PollForConversion(&hadc1, 1); 
    pot3 = HAL_ADC_GetValue(&hadc1);      

    // Convert ADC value to 0-100 range
    float converted_result1 = 99 - ((pot1 * 100) / 256);
    float converted_result2 = 99 - ((pot2 * 100) / 256);
    float converted_result3 = 99 - ((pot3 * 100) / 256);

    // Convert the result to a string
    display_values1[0] = (int)converted_result1 / 10 + 48;  
    display_values1[1] = (int)converted_result1 % 10 + 48;  
    display_values1[2] = '\0';                              

    display_values2[0] = (int)converted_result2 / 10 + 48;  
    display_values2[1] = (int)converted_result2 % 10 + 48;  
    display_values2[2] = '\0';                              

    display_values3[0] = (int)converted_result3 / 10 + 48;  
    display_values3[1] = (int)converted_result3 % 10 + 48;  
    display_values3[2] = '\0';                              

    // Aggiorna lo stato del display
    strcpy(display_state.pot1_value, display_values1);
    strcpy(display_state.pot2_value, display_values2);
    strcpy(display_state.pot3_value, display_values3);
    DISPLAY_UpdateState(&display_state);

    // Invia i valori dei potenziometri tramite il modulo seriale
    SERIAL_PotValues_t pot_values = {
        .pot1 = pot1,
        .pot2 = pot2,
        .pot3 = pot3
    };
    SERIAL_SendPotValues(&pot_values);
  }
}

/* La funzione setFirstHour è ora rimossa poiché gestita dal modulo seriale */

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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();  // Timer per il display
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Inizializza il modulo di gestione del tempo
  TIME_Init();

  // Inizializza il modulo display (passa TIM3 e SPI1)
  if (!DISPLAY_Init(&htim3, &hspi1)) {
    Error_Handler();
  }
  
  // Inizializza il modulo seriale
  if (!SERIAL_Init(&huart1)) {
    Error_Handler();
  }
  
  // Registra i callback per la ricezione dei dati
  SERIAL_RegisterSongCallback(OnSongDataReceived);
  SERIAL_RegisterTimeCallback(OnInitialTimeReceived);
  
  // Mostra la schermata di attesa
  DISPLAY_ShowWaitScreen();
  
  // Attendi l'orario iniziale
  if (!SERIAL_ReceiveInitialTime()) {
    Error_Handler();
  }

  // Avvia il timer per l'orologio
  HAL_TIM_Base_Start_IT(&htim2);

  // Avvia la ricezione seriale in modalità interrupt
  if (!SERIAL_StartReceive()) {
    Error_Handler();
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // take the first 2 values from potentiometers and convert them to string
    ADC_read();

    // handle the case when the song and artist are empty
    if (song[0] == '\0')
    {
      strncpy(song, default_song, sizeof(song));
      strncpy(display_state.song, song, sizeof(display_state.song));
    }
    if (artist[0] == '\0')
    {
      strncpy(artist, default_artist, sizeof(artist));
      strncpy(display_state.artist, artist, sizeof(display_state.artist));
    }
    
    /* RIMOSSO: updateScreen(); - ora gestito dal modulo display */

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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000-1;          /* 16MHz / 16000 = 1kHz */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;              /* 1000ms = 1 secondo */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  
  /* Imposta una priorità bassa per l'interrupt */
  HAL_NVIC_SetPriority(TIM3_IRQn, 15, 0);
  
  /* USER CODE END TIM3_Init 2 */
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT7_Pin BUT6_Pin BUT5_Pin BUT4_Pin
                           BUT3_Pin BUT2_Pin BUT1_Pin */
  GPIO_InitStruct.Pin = BUT7_Pin|BUT6_Pin|BUT5_Pin|BUT4_Pin
                          |BUT3_Pin|BUT2_Pin|BUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
