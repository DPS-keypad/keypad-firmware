/**
 * @file serial.h
 * @brief Modulo per la comunicazione seriale
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Dimensioni dei buffer */
#define SERIAL_RX_BUFFER_SIZE   44    /**< Reception buffer size */
#define SERIAL_SONG_SIZE        23    /**< Maximum song name size (including null terminator) */
#define SERIAL_ARTIST_SIZE      23    /**< Maximum artist name size (including null terminator) */
#define SERIAL_TIME_SIZE        6     /**< Time string format size (HH:MM\0) */

/* Struttura per i valori dei potenziometri */
typedef struct {
    uint8_t pot1;    /**< Potentiometer 1 value (0-255) */
    uint8_t pot2;    /**< Potentiometer 2 value (0-255) */
    uint8_t pot3;    /**< Potentiometer 3 value (0-255) */
} SERIAL_PotValues_t;

/* Tipi di callback */
typedef void (*SERIAL_SongDataCallback_t)(const char* song, const char* artist);
typedef void (*SERIAL_TimeCallback_t)(uint8_t hours, uint8_t minutes);

/**
 * @brief Inizializza il modulo seriale
 * 
 * @param uart_handle Puntatore all'handle UART
 * @return true Se l'inizializzazione è riuscita
 */
bool SERIAL_Init(UART_HandleTypeDef* uart_handle);

/**
 * @brief Registra un callback per i dati delle canzoni
 * 
 * @param callback Funzione da chiamare con i dati ricevuti
 */
void SERIAL_RegisterSongCallback(SERIAL_SongDataCallback_t callback);

/**
 * @brief Registra un callback per i dati dell'orario
 * 
 * @param callback Funzione da chiamare con l'orario ricevuto
 */
void SERIAL_RegisterTimeCallback(SERIAL_TimeCallback_t callback);

/**
 * @brief Avvia la ricezione e invia l'handshake
 * 
 * @return true Se l'operazione è riuscita
 */
bool SERIAL_StartReceive(void);

/**
 * @brief Invia i valori dei potenziometri
 * 
 * @param values Puntatore alla struttura con i valori
 * @return true Se l'invio è riuscito
 */
bool SERIAL_SendPotValues(const SERIAL_PotValues_t* values);

/**
 * @brief Invia un evento di pressione tasto
 * 
 * @param key Numero del tasto premuto (1-9)
 * @return true Se l'invio è riuscito
 */
bool SERIAL_SendKeyPress(uint8_t key);

/**
 * @brief Invia la maschera dei pulsanti premuti
 * 
 * @param mask Maschera di bit (1 = premuto, 0 = rilasciato)
 * @return true Se l'invio è riuscito
 */
bool SERIAL_SendButtonMask(uint16_t mask);

/**
 * @brief Richiede l'orario corrente
 * 
 * @return true Se la richiesta è stata inviata con successo
 */
bool SERIAL_RequestTime(void);

/**
 * @brief Callback da chiamare in HAL_UART_RxCpltCallback
 * 
 * @param huart Handle UART che ha generato il callback
 */
void SERIAL_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief Mostra la schermata di attesa sul display
 */
void SERIAL_ShowWaitScreen(void);

/**
 * @brief Attende la ricezione dell'orario iniziale
 * 
 * @param timeout_ms Timeout in millisecondi
 * @return true Se l'orario è stato ricevuto
 */
bool SERIAL_WaitForInitialTime(uint32_t timeout_ms);

#endif /* SERIAL_H_ */