/**
 * @file display.h
 * @brief OLED display management module
 * 
 * This module handles the OLED display in a completely isolated way,
 * updating via low-priority TIM3 timer interrupts.
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
 * @brief Display state with all information to be shown
 */
typedef struct {
    char time[6];         /*!< Time in HH:MM\0 format */
    char last_key[2];     /*!< Last pressed key */
    char song[23];        /*!< Currently playing song */
    char artist[23];      /*!< Song artist */
    char pot1_value[3];   /*!< Potentiometer 1 value */
    char pot2_value[3];   /*!< Potentiometer 2 value */
    char pot3_value[3];   /*!< Potentiometer 3 value */
} DISPLAY_State_t;

/**
 * @brief Initializes the display module
 * 
 * @param timer_handle Pointer to the already configured TIM3 handle
 * @param spi_handle Pointer to the already configured SPI1 handle
 * @return true If initialization was successful
 * @return false In case of error
 */
bool DISPLAY_Init(TIM_HandleTypeDef* timer_handle, SPI_HandleTypeDef* spi_handle);

/**
 * @brief Updates the display state with new data
 * 
 * @param state Pointer to the structure with new data
 * @return true If update was successful
 * @return false In case of error
 */
bool DISPLAY_UpdateState(const DISPLAY_State_t* state);

/**
 * @brief Shows the time setup waiting screen
 * 
 * @return true If operation was successful
 * @return false In case of error
 */
bool DISPLAY_ShowWaitScreen(void);

/**
 * @brief Timer interrupt handler for the display
 * 
 * To be called in HAL_TIM_PeriodElapsedCallback when timer is TIM3
 */
void DISPLAY_TimerHandler(void);

#endif /* DISPLAY_H_ */