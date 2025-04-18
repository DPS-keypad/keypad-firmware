#include "time.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"

/* Private definitions */
#define TIME_FORMAT_BUFFER_SIZE 6
#define TIME_HOURS_MAX          23
#define TIME_MINUTES_MAX        59

/* Static module variables (internal state) */
static int current_hours = 0;
static int current_minutes = 0;
static volatile bool time_initialized = false;

/**
 * @brief Initialize the time management module
 * @return true if initialization was successful
 */
bool TIME_Init(void) {
    /* Initialize state variables */
    current_hours = 0;
    current_minutes = 0;
    time_initialized = true;
    
    return true;
}

/**
 * @brief Manually set the current time
 * @param hours Hours value (0-23)
 * @param minutes Minutes value (0-59)
 * @return true if the time was successfully set
 */
bool TIME_Set(int hours, int minutes) {
    /* Parameter validation */
    if (hours < 0 || hours > TIME_HOURS_MAX || 
        minutes < 0 || minutes > TIME_MINUTES_MAX || 
        !time_initialized) {
        return false;
    }
    
    /* Set new values in a thread-safe manner */
    __disable_irq();
    current_hours = hours;
    current_minutes = minutes;
    __enable_irq();
    
    return true;
}

/**
 * @brief Increment the time by one minute
 * @return true if successful
 */
bool TIME_IncrementMinute(void) {
    if (!time_initialized) {
        return false;
    }
    
    /* Thread-safe minute increment */
    __disable_irq();
    current_minutes++;
    
    /* Handle minute and hour rollover */
    if (current_minutes > TIME_MINUTES_MAX) {
        current_minutes = 0;
        current_hours++;
        
        if (current_hours > TIME_HOURS_MAX) {
            current_hours = 0;
        }
    }
    __enable_irq();
    
    return true;
}

/**
 * @brief Get current time as formatted "HH:MM" string
 * @param buffer Output buffer (must be at least 6 bytes)
 * @return true if successful
 */
bool TIME_GetFormatted(char* buffer) {
    /* Parameter and state validation */
    if (buffer == NULL || !time_initialized) {
        return false;
    }
    
    /* Thread-safe state acquisition */
    __disable_irq();
    int h = current_hours;
    int m = current_minutes;
    __enable_irq();
    
    /* Format time as "HH:MM\0" */
    buffer[0] = (h / 10) + '0';
    buffer[1] = (h % 10) + '0';
    buffer[2] = ':';
    buffer[3] = (m / 10) + '0';
    buffer[4] = (m % 10) + '0';
    buffer[5] = '\0';
    
    return true;
}

/**
 * @brief Get current hours and minutes
 * @param hours Pointer to store hours (can be NULL)
 * @param minutes Pointer to store minutes (can be NULL)
 */
void TIME_GetCurrent(int* hours, int* minutes) {
    /* Get values if pointers are valid */
    if (hours != NULL) {
        __disable_irq();
        *hours = current_hours;
        __enable_irq();
    }
    
    if (minutes != NULL) {
        __disable_irq();
        *minutes = current_minutes;
        __enable_irq();
    }
}