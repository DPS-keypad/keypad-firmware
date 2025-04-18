#include "time.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"

/* Definizioni private */
#define TIME_FORMAT_BUFFER_SIZE 6
#define TIME_HOURS_MAX          23
#define TIME_MINUTES_MAX        59

/* Variabili statiche del modulo (stato interno) */
static int current_hours = 0;
static int current_minutes = 0;
static volatile bool time_initialized = false;

/**
 * @brief Inizializza il modulo di gestione del tempo
 */
bool TIME_Init(void) {
    /* Inizializza le variabili di stato */
    current_hours = 0;
    current_minutes = 0;
    time_initialized = true;
    
    return true;
}

/**
 * @brief Imposta manualmente l'ora corrente
 */
bool TIME_Set(int hours, int minutes) {
    /* Verifica parametri */
    if (hours < 0 || hours > TIME_HOURS_MAX || 
        minutes < 0 || minutes > TIME_MINUTES_MAX || 
        !time_initialized) {
        return false;
    }
    
    /* Imposta i nuovi valori in modo thread-safe */
    __disable_irq();
    current_hours = hours;
    current_minutes = minutes;
    __enable_irq();
    
    return true;
}

/**
 * @brief Incrementa l'orario di un minuto
 */
bool TIME_IncrementMinute(void) {
    if (!time_initialized) {
        return false;
    }
    
    /* Thread-safe incremento dei minuti */
    __disable_irq();
    current_minutes++;
    
    /* Gestisci il rollover dei minuti e delle ore */
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
 * @brief Restituisce l'ora corrente come stringa formattata "HH:MM"
 */
bool TIME_GetFormatted(char* buffer) {
    /* Verifica parametri e stato */
    if (buffer == NULL || !time_initialized) {
        return false;
    }
    
    /* Acquisizione sicura dello stato corrente */
    __disable_irq();
    int h = current_hours;
    int m = current_minutes;
    __enable_irq();
    
    /* Formatta l'ora come "HH:MM\0" */
    buffer[0] = (h / 10) + '0';
    buffer[1] = (h % 10) + '0';
    buffer[2] = ':';
    buffer[3] = (m / 10) + '0';
    buffer[4] = (m % 10) + '0';
    buffer[5] = '\0';
    
    return true;
}

/**
 * @brief Ottiene l'ora corrente
 */
void TIME_GetCurrent(int* hours, int* minutes) {
    /* Verifica parametri */
    if (hours != NULL) {
        /* Acquisizione sicura dello stato corrente */
        __disable_irq();
        *hours = current_hours;
        __enable_irq();
    }
    
    if (minutes != NULL) {
        /* Acquisizione sicura dello stato corrente */
        __disable_irq();
        *minutes = current_minutes;
        __enable_irq();
    }
}