/**
 * @file serial.c
 * @brief Implementazione del modulo per la comunicazione seriale
 */

#include "serial.h"
#include <string.h>

/* Variabili statiche del modulo */
static UART_HandleTypeDef* serial_uart = NULL;
static volatile bool serial_initialized = false;
static uint8_t rx_buffer[SERIAL_RX_BUFFER_SIZE];
static char song_buffer[SERIAL_SONG_SIZE];
static char artist_buffer[SERIAL_ARTIST_SIZE];

/* Callback registrati */
static SERIAL_SongDataCallback_t song_callback = NULL;
static SERIAL_TimeCallback_t time_callback = NULL;

/* Funzioni di implementazione */

bool SERIAL_Init(UART_HandleTypeDef* uart_handle)
{
    /* Verifica parametri */
    if (uart_handle == NULL) {
        return false;
    }
    
    /* Memorizza l'handle UART */
    serial_uart = uart_handle;
    
    /* Inizializza i buffer */
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(song_buffer, 0, sizeof(song_buffer));
    memset(artist_buffer, 0, sizeof(artist_buffer));
    
    serial_initialized = true;
    return true;
}

void SERIAL_RegisterSongCallback(SERIAL_SongDataCallback_t callback)
{
    song_callback = callback;
}

void SERIAL_RegisterTimeCallback(SERIAL_TimeCallback_t callback)
{
    time_callback = callback;
}

bool SERIAL_StartReceive(void)
{
    /* Verifica lo stato di inizializzazione */
    if (!serial_initialized || serial_uart == NULL) {
        return false;
    }
    
    /* Invia pacchetto iniziale di handshake */
    uint8_t handshake[] = "p\0";
    if (HAL_UART_Transmit(serial_uart, handshake, 2, 1000) != HAL_OK) {
        return false;
    }
    
    /* Avvia la ricezione in modalità interrupt */
    if (HAL_UART_Receive_IT(serial_uart, rx_buffer, SERIAL_RX_BUFFER_SIZE) != HAL_OK) {
        return false;
    }
    
    return true;
}

bool SERIAL_SendPotValues(const SERIAL_PotValues_t* values)
{
    /* Verifica parametri e stato di inizializzazione */
    if (!serial_initialized || serial_uart == NULL || values == NULL) {
        return false;
    }
    
    /* Prepara il pacchetto da inviare */
    uint8_t packet[5];
    packet[0] = 'p';        /* Identificatore pacchetto */
    packet[1] = values->pot1;
    packet[2] = values->pot2;
    packet[3] = values->pot3;
    packet[4] = '\0';       /* Terminatore */
    
    /* Invia il pacchetto */
    if (HAL_UART_Transmit(serial_uart, packet, 5, 1000) != HAL_OK) {
        return false;
    }
    
    return true;
}

bool SERIAL_ReceiveInitialTime(void)
{
    /* Verifica lo stato di inizializzazione */
    if (!serial_initialized || serial_uart == NULL) {
        return false;
    }
    
    /* Buffer per ricevere l'orario (formato "HH:MM\0") */
    char time_buffer[SERIAL_TIME_SIZE];
    
    /* Ricezione bloccante dell'orario iniziale */
    if (HAL_UART_Receive(serial_uart, (uint8_t*)time_buffer, SERIAL_TIME_SIZE, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }
    
    /* Estrai ore e minuti */
    int hours = (time_buffer[0] - '0') * 10 + (time_buffer[1] - '0');
    int minutes = (time_buffer[3] - '0') * 10 + (time_buffer[4] - '0');
    
    /* Chiama il callback se registrato */
    if (time_callback != NULL) {
        time_callback(hours, minutes);
    }
    
    return true;
}

void SERIAL_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Verifica che sia l'UART corretta */
    if (huart != serial_uart) {
        return;
    }
    
    /* Estrai canzone e artista dai dati ricevuti */
    strncpy(song_buffer, (char *)rx_buffer, SERIAL_SONG_SIZE - 1);
    strncpy(artist_buffer, (char *)rx_buffer + SERIAL_SONG_SIZE - 1, SERIAL_ARTIST_SIZE - 1);
    
    /* Assicura terminazione corretta delle stringhe */
    song_buffer[SERIAL_SONG_SIZE - 1] = '\0';
    artist_buffer[SERIAL_ARTIST_SIZE - 1] = '\0';
    
    /* Chiama il callback se registrato */
    if (song_callback != NULL) {
        song_callback(song_buffer, artist_buffer);
    }
    
    /* Cancella il buffer di ricezione */
    memset(rx_buffer, 0, sizeof(rx_buffer));
    
    /* Riattiva la ricezione in modalità interrupt */
    HAL_UART_Receive_IT(serial_uart, rx_buffer, SERIAL_RX_BUFFER_SIZE);
}