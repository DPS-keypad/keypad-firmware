/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
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
#define SPI1_RESET_Pin GPIO_PIN_3
#define SPI1_RESET_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI1_DC_Pin GPIO_PIN_6
#define SPI1_DC_GPIO_Port GPIOA
#define BUT9_Pin GPIO_PIN_11
#define BUT9_GPIO_Port GPIOA
#define BUT8_Pin GPIO_PIN_12
#define BUT8_GPIO_Port GPIOA
#define BUT7_Pin GPIO_PIN_3
#define BUT7_GPIO_Port GPIOB
#define BUT6_Pin GPIO_PIN_4
#define BUT6_GPIO_Port GPIOB
#define BUT5_Pin GPIO_PIN_5
#define BUT5_GPIO_Port GPIOB
#define BUT4_Pin GPIO_PIN_6
#define BUT4_GPIO_Port GPIOB
#define BUT3_Pin GPIO_PIN_7
#define BUT3_GPIO_Port GPIOB
#define BUT2_Pin GPIO_PIN_8
#define BUT2_GPIO_Port GPIOB
#define BUT1_Pin GPIO_PIN_9
#define BUT1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
