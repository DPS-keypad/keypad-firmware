/**
 * @file serial_protocol.h
 * @brief Protocol definitions for serial communication
 */

#ifndef SERIAL_PROTOCOL_H_
#define SERIAL_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Costanti di protocollo */
#define PROTO_STX           0x02    /**< Start of transmission byte */
#define PROTO_ETX           0x03    /**< End of transmission byte */
#define PROTO_MAX_PAYLOAD   64      /**< Maximum payload size */
#define PROTO_OVERHEAD      5       /**< Protocol overhead: STX+TYPE+LEN+CRC+ETX */
#define PROTO_TIMEOUT_MS    1000    /**< Reception timeout in ms */

/* Tipi di messaggio: Dispositivo -> PC */
#define MSG_HANDSHAKE       0x10    /**< Handshake iniziale */
#define MSG_POT_VALUES      0x11    /**< Valori potenziometri */
#define MSG_KEY_PRESS       0x12    /**< Evento pressione tasto singolo */
#define MSG_BUTTON_MASK     0x13    /**< Maschera di bit tasti multipli */
#define MSG_TIME_REQUEST    0x14    /**< Richiesta orario corrente */

/* Tipi di messaggio: PC -> Dispositivo */
#define MSG_ACK             0x20    /**< Acknowledge */
#define MSG_NACK            0x21    /**< Negative acknowledge */
#define MSG_SET_TIME        0x22    /**< Imposta ora sul dispositivo */
#define MSG_SONG_INFO       0x23    /**< Informazioni canzone */
#define MSG_DISPLAY_CONFIG  0x24    /**< Configura display */

/* Codici di errore */
#define ERR_NONE            0x00    /**< Nessun errore */
#define ERR_BAD_CRC         0x01    /**< Errore CRC */
#define ERR_BAD_FORMAT      0x02    /**< Messaggio malformato */
#define ERR_TIMEOUT         0x03    /**< Timeout comunicazione */
#define ERR_UNKNOWN_MSG     0x04    /**< Tipo messaggio sconosciuto */
#define ERR_PAYLOAD_SIZE    0x05    /**< Dimensione payload non valida */

/**
 * @brief Struttura messaggio di protocollo
 */
typedef struct {
    uint8_t type;                        /**< Tipo messaggio */
    uint8_t length;                      /**< Lunghezza payload */
    uint8_t payload[PROTO_MAX_PAYLOAD];  /**< Payload messaggio */
} PROTO_Message_t;

/**
 * @brief Callback per ricezione messaggio
 */
typedef void (*PROTO_RxCallbackTypeDef)(const PROTO_Message_t* msg);

/**
 * @brief Inizializza il modulo di protocollo
 * 
 * @param uart_handle Puntatore all'handle UART
 * @return true Se l'inizializzazione è riuscita
 */
bool PROTO_Init(UART_HandleTypeDef* uart_handle);

/**
 * @brief Invia un messaggio usando il protocollo
 * 
 * @param msg Puntatore alla struttura messaggio
 * @return true Se l'invio è riuscito
 */
bool PROTO_SendMessage(const PROTO_Message_t* msg);

/**
 * @brief Processa i byte ricevuti ed estrae i messaggi
 * 
 * @param byte Byte ricevuto
 * @return int8_t -1 se nessun messaggio completo, 0 se errore, 1 se messaggio valido
 */
int8_t PROTO_ProcessByte(uint8_t byte);

/**
 * @brief Avvia la ricezione in interrupt
 * 
 * @return true Se l'operazione è riuscita
 */
bool PROTO_StartReceive(void);

/**
 * @brief Callback UART da chiamare in HAL_UART_RxCpltCallback
 * 
 * @param huart Handle UART che ha generato il callback
 */
void PROTO_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief Registra callback per ricezione messaggi
 * 
 * @param callback Funzione da chiamare alla ricezione
 */
void PROTO_RegisterCallback(PROTO_RxCallbackTypeDef callback);

/**
 * @brief Crea un messaggio per inviare i valori dei potenziometri
 * 
 * @param msg Puntatore a struttura messaggio
 * @param pot1 Valore potenziometro 1
 * @param pot2 Valore potenziometro 2
 * @param pot3 Valore potenziometro 3
 * @return true Se creazione riuscita
 */
bool PROTO_CreatePotValuesMsg(PROTO_Message_t* msg, uint8_t pot1, uint8_t pot2, uint8_t pot3);

/**
 * @brief Crea un messaggio per inviare un tasto premuto
 * 
 * @param msg Puntatore a struttura messaggio
 * @param key Numero tasto (1-9)
 * @return true Se creazione riuscita
 */
bool PROTO_CreateKeyPressMsg(PROTO_Message_t* msg, uint8_t key);

/**
 * @brief Crea un messaggio con la maschera di bit per tutti i pulsanti
 * 
 * @param msg Puntatore a struttura messaggio
 * @param button_mask Maschera di bit (1 = premuto, 0 = rilasciato)
 * @return true Se creazione riuscita
 * @return false In caso di errore
 */
bool PROTO_CreateButtonMaskMsg(PROTO_Message_t* msg, uint16_t button_mask);

/**
 * @brief Estrae informazioni canzone da un messaggio
 * 
 * @param msg Puntatore al messaggio
 * @param song Buffer per nome canzone (almeno 23 byte)
 * @param artist Buffer per nome artista (almeno 23 byte)
 * @return true Se estrazione riuscita
 */
bool PROTO_ExtractSongInfo(const PROTO_Message_t* msg, char* song, char* artist);

/**
 * @brief Estrae informazioni di tempo da un messaggio
 * 
 * @param msg Puntatore al messaggio
 * @param hours Puntatore per memorizzare le ore
 * @param minutes Puntatore per memorizzare i minuti
 * @return true Se estrazione riuscita
 */
bool PROTO_ExtractTimeInfo(const PROTO_Message_t* msg, uint8_t* hours, uint8_t* minutes);

/**
 * @brief Calcola CRC-16 per l'integrità dei dati
 * 
 * @param data Puntatore buffer dati
 * @param length Lunghezza dati
 * @return uint16_t CRC calcolato
 */
uint16_t PROTO_CalculateCRC16(const uint8_t* data, uint16_t length);

#endif /* SERIAL_PROTOCOL_H_ */