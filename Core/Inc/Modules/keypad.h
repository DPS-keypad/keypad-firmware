/**
 * @file keypad.h
 * @brief Modulo di gestione tastierino con 9 tasti
 * 
 * Questo modulo gestisce il polling dei tasti del keypad con TIM4,
 * supportando rilevamento di pressioni multiple e continue.
 */

#ifndef KEYPAD_H_
#define KEYPAD_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
 * @brief Numero massimo di tasti supportati
 */
#define KEYPAD_MAX_KEYS 9

/**
 * @brief Enumerazione degli stati possibili di un tasto
 */
typedef enum {
    KEY_RELEASED = 0,  /**< Tasto rilasciato */
    KEY_PRESSED,       /**< Tasto premuto */
    KEY_HELD           /**< Tasto mantenuto premuto */
} KEYPAD_KeyState_t;

/**
 * @brief Struttura per rappresentare lo stato completo del keypad
 */
typedef struct {
    KEYPAD_KeyState_t key_states[KEYPAD_MAX_KEYS]; /**< Stato di ogni tasto */
    uint16_t button_mask;                          /**< Maschera di bit dei tasti premuti */
    char last_key;                                 /**< Ultimo tasto premuto (1-9, ' ' se nessuno) */
    bool state_changed;                            /**< Flag per indicare cambiamento di stato */
} KEYPAD_State_t;

/**
 * @brief Tipo di callback per notificare cambiamenti di stato
 * 
 * @param state Puntatore alla struttura con lo stato completo
 */
typedef void (*KEYPAD_StateChangedCallback_t)(const KEYPAD_State_t* state);

/**
 * @brief Inizializza il modulo keypad
 * 
 * @param timer_handle Puntatore all'handle TIM4 già inizializzato
 * @return true Se l'inizializzazione è riuscita
 * @return false In caso di errore
 */
bool KEYPAD_Init(TIM_HandleTypeDef* timer_handle);

/**
 * @brief Registra un callback per cambiamenti di stato dei tasti
 * 
 * @param callback Funzione da chiamare quando lo stato cambia
 */
void KEYPAD_RegisterCallback(KEYPAD_StateChangedCallback_t callback);

/**
 * @brief Ottiene lo stato corrente del keypad
 * 
 * @param state Puntatore alla struttura da riempire con lo stato
 * @return true Se l'operazione è riuscita
 * @return false In caso di errore
 */
bool KEYPAD_GetState(KEYPAD_State_t* state);

/**
 * @brief Timer handler da chiamare in HAL_TIM_PeriodElapsedCallback
 * 
 * @param htim Timer handle che ha generato l'interrupt
 */
void KEYPAD_TimerHandler(TIM_HandleTypeDef* htim);

/**
 * @brief Controlla se un tasto specifico è premuto
 * 
 * @param key_num Numero del tasto (1-9)
 * @return true Se il tasto è premuto
 * @return false Se il tasto è rilasciato o parametro non valido
 */
bool KEYPAD_IsKeyPressed(uint8_t key_num);

/**
 * @brief Ottiene il numero dell'ultimo tasto premuto
 * 
 * @return uint8_t Numero dell'ultimo tasto (1-9), 0 se nessuno
 */
uint8_t KEYPAD_GetLastKey(void);

#endif /* KEYPAD_H_ */