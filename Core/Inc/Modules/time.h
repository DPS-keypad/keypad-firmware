#ifndef TIME_H_
#define TIME_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#include "u8g2.h"

/**
 * @brief Initializes the time management module
 * @return true if initialization was successful
 */
bool TIME_Init(void);

/**
 * @brief Sets the current time
 * @param hours Hours (0-23)
 * @param minutes Minutes (0-59)
 * @return true if time was set correctly
 */
bool TIME_Set(int hours, int minutes);

/**
 * @brief Increments time by one minute
 * @return true if operation was successful
 */
bool TIME_IncrementMinute(void);

/**
 * @brief Returns current time as formatted string "HH:MM"
 * @param buffer Buffer where the string will be written (at least 6 bytes)
 * @return true if operation was successful
 */
bool TIME_GetFormatted(char* buffer);

/**
 * @brief Gets the current hours and minutes
 * @param hours Pointer to the variable to store hours
 * @param minutes Pointer to the variable to store minutes
 */
void TIME_GetCurrent(int* hours, int* minutes);

#endif /* TIME_H_ */