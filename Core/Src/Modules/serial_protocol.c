/**
 * @file serial_protocol.c
 * @brief Implementazione del protocollo di comunicazione seriale
 */

#include "serial_protocol.h"
#include <string.h>

/* Variabili private */
static UART_HandleTypeDef* proto_uart = NULL;
static volatile bool proto_initialized = false;
static volatile bool rx_in_progress = false;
static uint8_t rx_byte;
static uint8_t rx_buffer[PROTO_MAX_PAYLOAD + PROTO_OVERHEAD];
static uint8_t rx_index = 0;
static bool in_message = false;
static PROTO_Message_t current_message;
static PROTO_RxCallbackTypeDef message_callback = NULL;

/* Tabella CRC-16 CCITT */
static const uint16_t crc16_tab[] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78
};

/* Implementazione funzioni */
bool PROTO_Init(UART_HandleTypeDef* uart_handle) {
    /* Verifica parametri */
    if (uart_handle == NULL) {
        return false;
    }
    
    proto_uart = uart_handle;
    rx_index = 0;
    in_message = false;
    rx_in_progress = false;
    proto_initialized = true;
    
    return true;
}

uint16_t PROTO_CalculateCRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    
    while (length--) {
        crc = (crc << 8) ^ crc16_tab[((crc >> 8) ^ *data++) & 0xFF];
    }
    
    return crc;
}

bool PROTO_SendMessage(const PROTO_Message_t* msg) {
    if (!proto_initialized || msg == NULL || msg->length > PROTO_MAX_PAYLOAD) {
        return false;
    }
    
    uint8_t packet[PROTO_MAX_PAYLOAD + PROTO_OVERHEAD];
    uint16_t index = 0;
    
    /* Costruzione pacchetto */
    packet[index++] = PROTO_STX;
    packet[index++] = msg->type;
    packet[index++] = msg->length;
    
    /* Copia payload */
    memcpy(&packet[index], msg->payload, msg->length);
    index += msg->length;
    
    /* Calcolo e aggiunta CRC */
    uint16_t crc = PROTO_CalculateCRC16(&packet[1], index - 1);
    packet[index++] = (uint8_t)(crc >> 8);
    packet[index++] = (uint8_t)(crc & 0xFF);
    
    /* Fine pacchetto */
    packet[index++] = PROTO_ETX;
    
    /* Invio pacchetto con timeout */
    if (HAL_UART_Transmit(proto_uart, packet, index, PROTO_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    
    return true;
}

int8_t PROTO_ProcessByte(uint8_t byte) {
    static enum {
        WAIT_STX,
        WAIT_TYPE,
        WAIT_LEN,
        WAIT_PAYLOAD,
        WAIT_CRC1,
        WAIT_CRC2,
        WAIT_ETX
    } state = WAIT_STX;
    
    static uint8_t payload_index = 0;
    static uint8_t payload_length = 0;
    static uint8_t msg_type = 0;
    static uint16_t received_crc = 0;
    
    /* Processa il byte ricevuto in base alla macchina a stati */
    switch (state) {
        case WAIT_STX:
            if (byte == PROTO_STX) {
                rx_index = 0;
                rx_buffer[rx_index++] = byte;
                state = WAIT_TYPE;
            }
            break;
            
        case WAIT_TYPE:
            rx_buffer[rx_index++] = byte;
            msg_type = byte;
            state = WAIT_LEN;
            break;
            
        case WAIT_LEN:
            rx_buffer[rx_index++] = byte;
            payload_length = byte;
            
            if (payload_length > PROTO_MAX_PAYLOAD) {
                state = WAIT_STX;  /* Lunghezza non valida, reset */
            } else if (payload_length == 0) {
                state = WAIT_CRC1;  /* Nessun payload */
            } else {
                state = WAIT_PAYLOAD;
                payload_index = 0;
            }
            break;
            
        case WAIT_PAYLOAD:
            rx_buffer[rx_index++] = byte;
            payload_index++;
            
            if (payload_index >= payload_length) {
                state = WAIT_CRC1;
            }
            break;
            
        case WAIT_CRC1:
            rx_buffer[rx_index++] = byte;
            received_crc = (uint16_t)byte << 8;
            state = WAIT_CRC2;
            break;
            
        case WAIT_CRC2:
            rx_buffer[rx_index++] = byte;
            received_crc |= byte;
            state = WAIT_ETX;
            break;
            
        case WAIT_ETX:
            rx_buffer[rx_index++] = byte;
            state = WAIT_STX;  /* Reset per il prossimo messaggio */
            
            if (byte == PROTO_ETX) {
                /* Verifica CRC */
                uint16_t calculated_crc = PROTO_CalculateCRC16(&rx_buffer[1], rx_index - 4);
                
                if (calculated_crc == received_crc) {
                    /* Messaggio valido ricevuto */
                    current_message.type = msg_type;
                    current_message.length = payload_length;
                    memcpy(current_message.payload, &rx_buffer[3], payload_length);
                    
                    /* Chiama callback se registrato */
                    if (message_callback != NULL) {
                        message_callback(&current_message);
                    }
                    
                    return 1;  /* Messaggio valido */
                } else {
                    return 0;  /* Errore CRC */
                }
            } else {
                return 0;  /* ETX non valido */
            }
    }
    
    return -1;  /* Nessun messaggio completo */
}

bool PROTO_StartReceive(void) {
    /* Verifica lo stato di inizializzazione */
    if (!proto_initialized || proto_uart == NULL) {
        return false;
    }
    
    /* Avvia la ricezione interrupt di un byte */
    if (!rx_in_progress) {
        rx_in_progress = true;
        if (HAL_UART_Receive_IT(proto_uart, &rx_byte, 1) != HAL_OK) {
            rx_in_progress = false;
            return false;
        }
    }
    
    return true;
}

void PROTO_RxCpltCallback(UART_HandleTypeDef *huart) {
    /* Verifica che sia l'UART corretta */
    if (huart != proto_uart) {
        return;
    }
    
    /* Processa il byte ricevuto */
    PROTO_ProcessByte(rx_byte);
    
    /* Riavvia la ricezione */
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

void PROTO_RegisterCallback(PROTO_RxCallbackTypeDef callback) {
    message_callback = callback;
}

bool PROTO_CreatePotValuesMsg(PROTO_Message_t* msg, uint8_t pot1, uint8_t pot2, uint8_t pot3) {
    if (msg == NULL) {
        return false;
    }
    
    msg->type = MSG_POT_VALUES;
    msg->length = 3;
    msg->payload[0] = pot1;
    msg->payload[1] = pot2;
    msg->payload[2] = pot3;
    
    return true;
}

bool PROTO_CreateKeyPressMsg(PROTO_Message_t* msg, uint8_t key) {
    if (msg == NULL || key < 1 || key > 9) {
        return false;
    }
    
    msg->type = MSG_KEY_PRESS;
    msg->length = 1;
    msg->payload[0] = key;
    
    return true;
}

bool PROTO_CreateButtonMaskMsg(PROTO_Message_t* msg, uint16_t button_mask)
{
    /* Verifica parametri */
    if (msg == NULL) {
        return false;
    }
    
    /* Costruisci il messaggio */
    msg->type = MSG_BUTTON_MASK;
    msg->length = 2;
    msg->payload[0] = (uint8_t)(button_mask & 0xFF);        /* Byte basso */
    msg->payload[1] = (uint8_t)((button_mask >> 8) & 0xFF); /* Byte alto */
    
    return true;
}

bool PROTO_ExtractSongInfo(const PROTO_Message_t* msg, char* song, char* artist) {
    if (msg == NULL || song == NULL || artist == NULL || msg->type != MSG_SONG_INFO) {
        return false;
    }
    
    uint8_t song_len = msg->payload[0];
    
    /* Validazione lunghezze */
    if (song_len > 22 || (song_len + 1) >= msg->length) {
        return false;
    }
    
    /* Estrazione nome canzone */
    memcpy(song, &msg->payload[1], song_len);
    song[song_len] = '\0';
    
    /* Estrazione nome artista */
    uint8_t artist_len = msg->payload[song_len + 1];
    if (artist_len > 22 || (song_len + 2 + artist_len) > msg->length) {
        return false;
    }
    
    memcpy(artist, &msg->payload[song_len + 2], artist_len);
    artist[artist_len] = '\0';
    
    return true;
}

bool PROTO_ExtractTimeInfo(const PROTO_Message_t* msg, uint8_t* hours, uint8_t* minutes) {
    if (msg == NULL || hours == NULL || minutes == NULL || 
        msg->type != MSG_SET_TIME || msg->length != 2) {
        return false;
    }
    
    uint8_t h = msg->payload[0];
    uint8_t m = msg->payload[1];
    
    /* Validazione valori orario */
    if (h > 23 || m > 59) {
        return false;
    }
    
    *hours = h;
    *minutes = m;
    
    return true;
}