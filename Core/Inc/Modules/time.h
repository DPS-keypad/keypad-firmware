#ifndef TIME_H_
#define TIME_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#include "u8g2.h"

/**
 * @brief Inizializza il modulo di gestione del tempo
 * @return true se l'inizializzazione è avvenuta con successo
 */
bool TIME_Init(void);

/**
 * @brief Imposta l'orario corrente
 * @param hours Ore (0-23)
 * @param minutes Minuti (0-59)
 * @return true se l'orario è stato impostato correttamente
 */
bool TIME_Set(int hours, int minutes);

/**
 * @brief Incrementa l'orario di un minuto
 * @return true se l'operazione è avvenuta con successo
 */
bool TIME_IncrementMinute(void);

/**
 * @brief Restituisce l'ora corrente come stringa formattata "HH:MM"
 * @param buffer Buffer in cui scrivere la stringa (almeno 6 byte)
 * @return true se l'operazione è avvenuta con successo
 */
bool TIME_GetFormatted(char* buffer);

/**
 * @brief Ottiene l'ora e i minuti correnti
 * @param hours Puntatore alla variabile in cui memorizzare l'ora
 * @param minutes Puntatore alla variabile in cui memorizzare i minuti
 */
void TIME_GetCurrent(int* hours, int* minutes);

#endif /* TIME_H_ */