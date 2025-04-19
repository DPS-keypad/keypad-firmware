/**
 * @file serial.c
 * @brief Implementazione del modulo per la comunicazione seriale
 */

#include "serial.h"
#include "serial_protocol.h"
#include "display.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h" /* per HAL_GetTick e HAL_Delay */

/* Variabili statiche del modulo */
static UART_HandleTypeDef* serial_uart = NULL;
static volatile bool serial_initialized = false;
static char song_buffer[SERIAL_SONG_SIZE];
static char artist_buffer[SERIAL_ARTIST_SIZE];

/* Callback registrati */
static SERIAL_SongDataCallback_t song_callback = NULL;
static SERIAL_TimeCallback_t time_callback = NULL;

/* Callback per la ricezione dei messaggi dal protocollo */
static void OnProtocolMessageReceived(const PROTO_Message_t* msg)
{
    uint8_t hours, minutes;
    
    switch(msg->type)
    {
        case MSG_SONG_INFO:
            /* Estrai informazioni canzone */
            if (PROTO_ExtractSongInfo(msg, song_buffer, artist_buffer)) {
                /* Chiama il callback se registrato */
                if (song_callback != NULL) {
                    song_callback(song_buffer, artist_buffer);
                }
            }
            break;
            
        case MSG_SET_TIME:
            /* Estrai informazioni tempo */
            if (PROTO_ExtractTimeInfo(msg, &hours, &minutes)) {
                /* Chiama il callback se registrato */
                if (time_callback != NULL) {
                    time_callback(hours, minutes);
                }
            }
            break;
            
        case MSG_ACK:
            /* Conferma di ricezione, non richiede azioni particolari */
            break;
            
        default:
            /* Messaggio non gestito */
            break;
    }
}

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
    memset(song_buffer, 0, sizeof(song_buffer));
    memset(artist_buffer, 0, sizeof(artist_buffer));
    
    /* Inizializza il protocollo seriale */
    if (!PROTO_Init(uart_handle)) {
        return false;
    }
    
    /* Registra il callback per i messaggi ricevuti */
    PROTO_RegisterCallback(OnProtocolMessageReceived);
    
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
    PROTO_Message_t handshake_msg;
    handshake_msg.type = MSG_HANDSHAKE;
    handshake_msg.length = 1;
    handshake_msg.payload[0] = 0x01; /* Versione firmware */
    
    if (!PROTO_SendMessage(&handshake_msg)) {
        return false;
    }
    
    /* Avvia la ricezione in modalità interrupt */
    if (!PROTO_StartReceive()) {
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
    
    /* Crea un messaggio per inviare i valori dei potenziometri */
    PROTO_Message_t pot_msg;
    if (!PROTO_CreatePotValuesMsg(&pot_msg, values->pot1, values->pot2, values->pot3)) {
        return false;
    }
    
    /* Invia il messaggio */
    if (!PROTO_SendMessage(&pot_msg)) {
        return false;
    }
    
    return true;
}

bool SERIAL_SendKeyPress(uint8_t key)
{
    /* Verifica parametri e stato di inizializzazione */
    if (!serial_initialized || serial_uart == NULL || key < 1 || key > 9) {
        return false;
    }
    
    /* Crea un messaggio per inviare il tasto premuto */
    PROTO_Message_t key_msg;
    if (!PROTO_CreateKeyPressMsg(&key_msg, key)) {
        return false;
    }
    
    /* Invia il messaggio */
    if (!PROTO_SendMessage(&key_msg)) {
        return false;
    }
    
    return true;
}

bool SERIAL_SendButtonMask(uint16_t mask)
{
    /* Verifica stato di inizializzazione */
    if (!serial_initialized || serial_uart == NULL) {
        return false;
    }
    
    /* Crea un messaggio per inviare la maschera dei pulsanti */
    PROTO_Message_t mask_msg;
    if (!PROTO_CreateButtonMaskMsg(&mask_msg, mask)) {
        return false;
    }
    
    /* Invia il messaggio */
    if (!PROTO_SendMessage(&mask_msg)) {
        return false;
    }
    
    return true;
}

bool SERIAL_RequestTime(void)
{
    /* Verifica stato di inizializzazione */
    if (!serial_initialized || serial_uart == NULL) {
        return false;
    }
    
    /* Crea un messaggio di richiesta orario */
    PROTO_Message_t time_req_msg;
    time_req_msg.type = MSG_TIME_REQUEST;
    time_req_msg.length = 0;
    
    /* Invia il messaggio */
    if (!PROTO_SendMessage(&time_req_msg)) {
        return false;
    }
    
    return true;
}

void SERIAL_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Delega la gestione al modulo protocollo */
    PROTO_RxCpltCallback(huart);
}

/**
 * @brief Mostra la schermata di attesa sul display
 * 
 * Questa funzione visualizza una schermata di attesa che indica
 * all'utente che il dispositivo è in attesa dei dati iniziali
 */
void SERIAL_ShowWaitScreen(void)
{
    /* Questa funzione deve essere implementata nel modulo display */
    DISPLAY_ShowWaitScreen();
}

/**
 * @brief Attende la ricezione dell'orario iniziale
 * 
 * Questa funzione mostra la schermata di attesa e invia
 * richieste di orario finché non riceve una risposta o
 * scade il timeout
 * 
 * @param timeout_ms Timeout in millisecondi
 * @return true Se l'orario è stato ricevuto
/* Variabili globali per il callback temporaneo */
static volatile bool g_time_received_flag = false;
static SERIAL_TimeCallback_t g_original_callback = NULL;

/* Funzione di callback temporanea */
static void temp_time_callback(uint8_t h, uint8_t m)
{
    /* Chiamiamo il callback originale se esistente */
    if (g_original_callback != NULL) {
        g_original_callback(h, m);
    }
    g_time_received_flag = true;
}

bool SERIAL_WaitForInitialTime(uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();
    
    /* Mostra la schermata di attesa */
    SERIAL_ShowWaitScreen();
    
    /* Flag che sarà impostato dal callback quando l'orario è ricevuto */
    g_time_received_flag = false;
    
    /* Funzione di callback temporanea per rilevare la ricezione dell'orario */
    g_original_callback = time_callback;
    time_callback = temp_time_callback;
    
    /* Ciclo di attesa con timeout */
    while (!g_time_received_flag && (HAL_GetTick() - start_time) < timeout_ms)
    {
        /* Richiedi l'orario */
        SERIAL_RequestTime();
        
        /* Attendi un po' prima di riprovare */
        HAL_Delay(500);
    }
    
    /* Ripristina il callback originale */
    time_callback = g_original_callback;
    
    return g_time_received_flag;
}