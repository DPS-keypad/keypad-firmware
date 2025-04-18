/**
 * @file serial.h
 * @brief Module for UART serial communication management
 * 
 * This module handles UART data transmission and reception independently,
 * implementing secure communication with low coupling.
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Buffer size definitions */
#define SERIAL_RX_BUFFER_SIZE   44    /**< Reception buffer size */
#define SERIAL_SONG_SIZE        23    /**< Maximum song name size (including null terminator) */
#define SERIAL_ARTIST_SIZE      23    /**< Maximum artist name size (including null terminator) */
#define SERIAL_TIME_SIZE        6     /**< Time string format size (HH:MM\0) */

/**
 * @brief Data structure for potentiometer values
 */
typedef struct {
    uint8_t pot1;    /**< Potentiometer 1 value (0-255) */
    uint8_t pot2;    /**< Potentiometer 2 value (0-255) */
    uint8_t pot3;    /**< Potentiometer 3 value (0-255) */
} SERIAL_PotValues_t;

/**
 * @brief Callback type for song data reception
 * @param song String containing the song name
 * @param artist String containing the artist name
 */
typedef void (*SERIAL_SongDataCallback_t)(const char* song, const char* artist);

/**
 * @brief Callback type for initial time reception
 * @param hours Received hours (0-23)
 * @param minutes Received minutes (0-59)
 */
typedef void (*SERIAL_TimeCallback_t)(int hours, int minutes);

/**
 * @brief Initialize the serial communication module
 * 
 * @param uart_handle Pointer to already initialized UART handle
 * @return true If initialization was successful
 * @return false In case of error
 */
bool SERIAL_Init(UART_HandleTypeDef* uart_handle);

/**
 * @brief Register song data reception callback
 * 
 * @param callback Function to call when new data is received
 */
void SERIAL_RegisterSongCallback(SERIAL_SongDataCallback_t callback);

/**
 * @brief Register initial time reception callback
 * 
 * @param callback Function to call when initial time is received
 */
void SERIAL_RegisterTimeCallback(SERIAL_TimeCallback_t callback);

/**
 * @brief Start data reception in interrupt mode
 * 
 * @return true If operation was successful
 * @return false In case of error
 */
bool SERIAL_StartReceive(void);

/**
 * @brief Send potentiometer values via serial
 * 
 * @param values Structure containing potentiometer values
 * @return true If transmission was successful
 * @return false In case of error
 */
bool SERIAL_SendPotValues(const SERIAL_PotValues_t* values);

/**
 * @brief Waits for and receives the initial time format
 * 
 * Blocking function to be called only at startup
 * 
 * @return true If reception was successful
 * @return false In case of error
 */
bool SERIAL_ReceiveInitialTime(void);

/**
 * @brief Handler for UART reception completion
 * 
 * To be called in HAL_UART_RxCpltCallback
 * 
 * @param huart UART handle that generated the callback
 */
void SERIAL_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* SERIAL_H_ */