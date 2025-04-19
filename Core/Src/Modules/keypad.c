/**
 * @file keypad.c
 * @brief Implementazione del modulo di gestione tastierino
 */

#include "keypad.h"
#include "main.h"
#include <string.h>

#include "stm32f4xx_hal.h"

/* Definizioni private */
#define KEYPAD_SCAN_INTERVAL_MS     1      /* Intervallo di scansione in ms */
#define KEYPAD_DEBOUNCE_COUNT       5      /* Conteggio di debounce (5ms) */
#define KEYPAD_HOLD_COUNT          50      /* Conteggio prima di considerare "mantenuto" (50ms) */
#define KEYPAD_REPEAT_COUNT       100      /* Intervallo di ripetizione in hold mode (100ms) */

/* Pin GPIO per ciascun tasto */
static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} key_pins[KEYPAD_MAX_KEYS] = {
    {BUT1_GPIO_Port, BUT1_Pin},
    {BUT2_GPIO_Port, BUT2_Pin},
    {BUT3_GPIO_Port, BUT3_Pin},
    {BUT4_GPIO_Port, BUT4_Pin},
    {BUT5_GPIO_Port, BUT5_Pin},
    {BUT6_GPIO_Port, BUT6_Pin},
    {BUT7_GPIO_Port, BUT7_Pin},
    {BUT8_GPIO_Port, BUT8_Pin},
    {BUT9_GPIO_Port, BUT9_Pin}
};

/* Variabili private */
static TIM_HandleTypeDef* keypad_timer = NULL;
static volatile bool keypad_initialized = false;
static KEYPAD_State_t current_state = {0};
static KEYPAD_StateChangedCallback_t state_callback = NULL;

/* Array per conteggio debounce (per ogni tasto) */
static uint8_t debounce_counters[KEYPAD_MAX_KEYS] = {0};
/* Array per conteggio hold time (per ogni tasto) */
static uint16_t hold_counters[KEYPAD_MAX_KEYS] = {0};
/* Array per lo stato raw dei pin (lettura diretta) */
static bool raw_pin_states[KEYPAD_MAX_KEYS] = {0};
/* Array per lo stato precedente dei tasti (per rilevare transizioni) */
static KEYPAD_KeyState_t prev_key_states[KEYPAD_MAX_KEYS] = {0};

/**
 * @brief Legge lo stato fisico di un pin
 * 
 * @param key_index Indice del tasto (0-8)
 * @return true Se il tasto è premuto fisicamente
 * @return false Se il tasto è rilasciato
 */
static bool ReadPinState(uint8_t key_index) {
    /* Tasti sono configurati con pull-up, quindi attivi bassi */
    if (key_index < KEYPAD_MAX_KEYS) {
        return (HAL_GPIO_ReadPin(key_pins[key_index].port, key_pins[key_index].pin) == GPIO_PIN_RESET);
    }
    return false;
}

/**
 * @brief Aggiorna lo stato del keypad con debouncing
 * 
 * Questa funzione implementa una macchina a stati per ogni tasto che gestisce
 * debouncing, rilevamento stato mantenuto e ripetizioni
 */
static void UpdateKeypadState(void) {
    bool state_updated = false;
    uint16_t new_button_mask = 0;
    
    /* Scansiona tutti i tasti */
    for (uint8_t i = 0; i < KEYPAD_MAX_KEYS; i++) {
        /* Leggi lo stato fisico del pin */
        raw_pin_states[i] = ReadPinState(i);
        
        /* Gestisci il debounce e la macchina a stati */
        switch (current_state.key_states[i]) {
            case KEY_RELEASED:
                if (raw_pin_states[i]) {
                    /* Potenziale pressione rilevata */
                    debounce_counters[i]++;
                    if (debounce_counters[i] >= KEYPAD_DEBOUNCE_COUNT) {
                        /* Debounce completato, passa a stato premuto */
                        current_state.key_states[i] = KEY_PRESSED;
                        debounce_counters[i] = 0;
                        hold_counters[i] = 0;
                        current_state.last_key = '1' + i;
                        state_updated = true;
                    }
                } else {
                    /* Resetta contatore debounce se il tasto è rilasciato */
                    debounce_counters[i] = 0;
                }
                break;
                
            case KEY_PRESSED:
                if (raw_pin_states[i]) {
                    /* Tasto ancora premuto, incrementa contatore hold */
                    hold_counters[i]++;
                    if (hold_counters[i] >= KEYPAD_HOLD_COUNT) {
                        /* Tasto mantenuto premuto a lungo */
                        current_state.key_states[i] = KEY_HELD;
                        state_updated = true;
                    }
                } else {
                    /* Potenziale rilascio rilevato */
                    debounce_counters[i]++;
                    if (debounce_counters[i] >= KEYPAD_DEBOUNCE_COUNT) {
                        /* Debounce completato, passa a stato rilasciato */
                        current_state.key_states[i] = KEY_RELEASED;
                        debounce_counters[i] = 0;
                        hold_counters[i] = 0;
                        state_updated = true;
                    }
                }
                break;
                
            case KEY_HELD:
                if (raw_pin_states[i]) {
                    /* Tasto ancora mantenuto premuto */
                    hold_counters[i]++;
                    if (hold_counters[i] >= KEYPAD_REPEAT_COUNT) {
                        /* Genera evento di ripetizione */
                        hold_counters[i] = 0;
                        state_updated = true;  /* Forza aggiornamento periodico */
                    }
                } else {
                    /* Potenziale rilascio rilevato */
                    debounce_counters[i]++;
                    if (debounce_counters[i] >= KEYPAD_DEBOUNCE_COUNT) {
                        /* Debounce completato, passa a stato rilasciato */
                        current_state.key_states[i] = KEY_RELEASED;
                        debounce_counters[i] = 0;
                        hold_counters[i] = 0;
                        state_updated = true;
                    }
                }
                break;
        }
        
        /* Aggiorna la maschera di bit se il tasto è premuto o mantenuto */
        if (current_state.key_states[i] != KEY_RELEASED) {
            new_button_mask |= (1 << i);
        }
        
        /* Rileva le transizioni di stato */
        if (prev_key_states[i] != current_state.key_states[i]) {
            state_updated = true;
            prev_key_states[i] = current_state.key_states[i];
        }
    }
    
    /* Aggiorna la maschera di bit globale */
    if (current_state.button_mask != new_button_mask) {
        current_state.button_mask = new_button_mask;
        state_updated = true;
    }
    
    /* Imposta il flag se c'è stato un cambiamento */
    current_state.state_changed = state_updated;
    
    /* Chiama il callback se registrato e c'è stato un cambiamento */
    if (state_updated && state_callback != NULL) {
        state_callback(&current_state);
    }
}

/* Implementazioni funzioni pubbliche */

bool KEYPAD_Init(TIM_HandleTypeDef* timer_handle) {
    /* Verifica parametri */
    if (timer_handle == NULL) {
        return false;
    }
    
    /* Salva handle timer */
    keypad_timer = timer_handle;
    
    /* Inizializza lo stato */
    memset(&current_state, 0, sizeof(KEYPAD_State_t));
    current_state.last_key = ' ';
    
    /* Azzera i contatori */
    memset(debounce_counters, 0, sizeof(debounce_counters));
    memset(hold_counters, 0, sizeof(hold_counters));
    memset(prev_key_states, KEY_RELEASED, sizeof(prev_key_states));
    
    /* Configura il timer per la scansione della tastiera */
    timer_handle->Instance = TIM4;
    timer_handle->Init.Prescaler = 16000-1;       /* 16MHz / 16000 = 1kHz */
    timer_handle->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer_handle->Init.Period = KEYPAD_SCAN_INTERVAL_MS-1; /* 1ms */
    timer_handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer_handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    /* Configura la priorità dell'interrupt del timer (media) */
    HAL_NVIC_SetPriority(TIM4_IRQn, 10, 0);
    
    /* Avvia il timer in modalità interrupt */
    if (HAL_TIM_Base_Start_IT(timer_handle) != HAL_OK) {
        return false;
    }
    
    keypad_initialized = true;
    return true;
}

void KEYPAD_RegisterCallback(KEYPAD_StateChangedCallback_t callback) {
    state_callback = callback;
}

bool KEYPAD_GetState(KEYPAD_State_t* state) {
    /* Verifica parametri e stato di inizializzazione */
    if (!keypad_initialized || state == NULL) {
        return false;
    }
    
    /* Sezione critica per evitare race condition durante l'aggiornamento */
    __disable_irq();
    memcpy(state, &current_state, sizeof(KEYPAD_State_t));
    __enable_irq();
    
    return true;
}

void KEYPAD_TimerHandler(TIM_HandleTypeDef* htim) {
    /* Verifica che sia il timer corretto */
    if (htim != keypad_timer) {
        return;
    }
    
    /* Aggiorna lo stato del keypad */
    UpdateKeypadState();
}

bool KEYPAD_IsKeyPressed(uint8_t key_num) {
    /* Verifica parametri */
    if (!keypad_initialized || key_num < 1 || key_num > KEYPAD_MAX_KEYS) {
        return false;
    }
    
    /* Indice in array è zero-based */
    uint8_t index = key_num - 1;
    
    /* Un tasto è considerato premuto in entrambi gli stati KEY_PRESSED e KEY_HELD */
    return (current_state.key_states[index] != KEY_RELEASED);
}

uint8_t KEYPAD_GetLastKey(void) {
    if (!keypad_initialized) {
        return 0;
    }
    
    if (current_state.last_key >= '1' && current_state.last_key <= '9') {
        return current_state.last_key - '0';
    }
    
    return 0;
}