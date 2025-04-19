/**
 * @file display.c
 * @brief Display module implementation
 */

#include "display.h"
#include "main.h"
#include "u8g2.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"

/* Private definitions */
#define DISPLAY_REFRESH_INTERVAL_MS 1000  /* Update every second */

/* Static variables */
static u8g2_t u8g2;                      /* Display object */
static TIM_HandleTypeDef* display_timer;  /* Timer for display refresh */
static SPI_HandleTypeDef* display_spi;    /* SPI handle for display communication */
static volatile bool display_initialized = false;
static volatile bool update_pending = false;
static DISPLAY_State_t current_state;     /* Current display state */

/* Private functions for u8g2 driver */
static uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, 
                                        U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
                                        U8X8_UNUSED void *arg_ptr)
{
    // Handle GPIO pins (DC, CS, RESET) and timing requirements for the display
    switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        HAL_Delay(1);
        break;
    case U8X8_MSG_DELAY_MILLI:
        HAL_Delay(arg_int);
        break;
    case U8X8_MSG_GPIO_DC:
        HAL_GPIO_WritePin(SPI1_DC_GPIO_Port, SPI1_DC_Pin, arg_int);
        break;
    case U8X8_MSG_GPIO_CS:
        HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, arg_int);
        break;
    case U8X8_MSG_GPIO_RESET:
        HAL_GPIO_WritePin(SPI1_RESET_GPIO_Port, SPI1_RESET_Pin, arg_int);
        break;
    }
    return 1;
}

static uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                                     void *arg_ptr)
{
    // Hardware SPI communication handler for the display
    if (!display_initialized || display_spi == NULL) {
        return 0;
    }
    
    switch (msg)
    {
    case U8X8_MSG_BYTE_SEND:
        HAL_SPI_Transmit(display_spi, (uint8_t *)arg_ptr, arg_int, 1000);
        break;
    case U8X8_MSG_BYTE_INIT:
        break;
    case U8X8_MSG_BYTE_SET_DC:
        u8x8_gpio_SetDC(u8x8, arg_int);
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
        break;
    default:
        return 0;
    }
    return 1;
}

/**
 * @brief Clear display content
 */
static void DISPLAY_Clear(void)
{
    u8g2_FirstPage(&u8g2);
    do {
        /* Empty page */
    } while (u8g2_NextPage(&u8g2));
}

/**
 * @brief Update display content based on current state
 */
static void DISPLAY_Refresh(void)
{
    /* Check initialization status */
    if (!display_initialized) {
        return;
    }
    
    /* Copy current state in a thread-safe manner */
    DISPLAY_State_t state;
    __disable_irq();
    memcpy(&state, &current_state, sizeof(DISPLAY_State_t));
    update_pending = false;
    __enable_irq();
    
    /* Update the screen with current state data */
    u8g2_FirstPage(&u8g2);
    do
    {
        /* Time (larger font) */
        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
        u8g2_DrawStr(&u8g2, 50, 18, state.time);
        
        /* Divider lines */
        u8g2_DrawLine(&u8g2, 0, 20, 128, 20);
        u8g2_DrawLine(&u8g2, 20, 0, 20, 20);
        
        /* Last pressed key */
        u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
        u8g2_DrawCircle(&u8g2, 10, 10, 7, U8G2_DRAW_ALL);
        u8g2_DrawStr(&u8g2, 8, 13, state.last_key);
        
        /* Song information */
        u8g2_DrawStr(&u8g2, 20, 55, "Listening to:");
        u8g2_DrawStr(&u8g2, 2, 70, state.song);
        u8g2_DrawStr(&u8g2, 50, 80, "by");
        u8g2_DrawStr(&u8g2, 2, 90, state.artist);
        
        /* Potentiometer values */
        u8g2_DrawLine(&u8g2, 0, 108, 128, 108);
        u8g2_DrawStr(&u8g2, 2, 122, "1-");
        u8g2_DrawStr(&u8g2, 14, 122, state.pot1_value);
        u8g2_DrawStr(&u8g2, 54, 122, "2-");
        u8g2_DrawStr(&u8g2, 66, 122, state.pot2_value);
        u8g2_DrawStr(&u8g2, 102, 122, "3-");
        u8g2_DrawStr(&u8g2, 114, 122, state.pot3_value);
    } while (u8g2_NextPage(&u8g2));
}

/* Public function implementation */

bool DISPLAY_Init(TIM_HandleTypeDef* timer_handle, SPI_HandleTypeDef* spi_handle)
{
    /* Validate parameters */
    if (timer_handle == NULL || spi_handle == NULL) {
        return false;
    }
    
    /* Store handles */
    display_timer = timer_handle;
    display_spi = spi_handle;
    
    /* Initialize OLED display with Pimoroni 128x128 SH1107 driver */
    u8g2_Setup_sh1107_pimoroni_128x128_1(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi,
                                       u8x8_stm32_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);
    
    /* Turn on display */
    u8g2_SetPowerSave(&u8g2, 0);
    
    /* Configure TIM3 timer for display refresh (1 second interval) */
    timer_handle->Instance = TIM3;
    timer_handle->Init.Prescaler = 16000-1;      /* 16MHz / 16000 = 1kHz */
    timer_handle->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer_handle->Init.Period = 1000-1;         /* 1000ms = 1 second */
    timer_handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer_handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    /* Set timer interrupt to lowest priority */
    HAL_NVIC_SetPriority(TIM3_IRQn, 15, 0);
    
    /* Start timer in interrupt mode */
    if (HAL_TIM_Base_Start_IT(timer_handle) != HAL_OK) {
        return false;
    }
    
    /* Initialize state with default values */
    memset(&current_state, 0, sizeof(DISPLAY_State_t));
    strcpy(current_state.time, "00:00");
    strcpy(current_state.last_key, " ");
    strcpy(current_state.song, "No song playing");
    strcpy(current_state.artist, "No artist playing");
    strcpy(current_state.pot1_value, "00");
    strcpy(current_state.pot2_value, "00");
    strcpy(current_state.pot3_value, "00");
    
    /* Successfully completed */
    display_initialized = true;
    update_pending = true;
    
    return true;
}

bool DISPLAY_UpdateState(const DISPLAY_State_t* state)
{
    /* Check parameters and initialization */
    if (!display_initialized || state == NULL) {
        return false;
    }
    
    /* Update state in a thread-safe manner */
    __disable_irq();
    memcpy(&current_state, state, sizeof(DISPLAY_State_t));
    update_pending = true;  /* Request display update */
    __enable_irq();
    
    return true;
}

/**
 * @brief Mostra una schermata di attesa
 * 
 * Questa funzione visualizza una schermata che indica all'utente
 * che il dispositivo è in attesa di ricevere dati iniziali.
 */
void DISPLAY_ShowWaitScreen(void)
{
    /* Check initialization status */
    if (!display_initialized) {
        return;
    }
    
    /* Update the screen with wait message */
    static bool dots_visible = true;
    
    u8g2_FirstPage(&u8g2);
    do {
        /* Title (larger font) */
        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
        u8g2_DrawStr(&u8g2, 10, 20, "Keypad FW");
        
        /* Divider line */
        u8g2_DrawLine(&u8g2, 0, 30, 128, 30);
        
        /* Wait message */
        u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
        u8g2_DrawStr(&u8g2, 5, 55, "In attesa");
        u8g2_DrawStr(&u8g2, 5, 70, "dell'orario");
        
        /* Animated dots */
        if (dots_visible) {
            u8g2_DrawStr(&u8g2, 100, 70, "...");
        }
    } while (u8g2_NextPage(&u8g2));
    
    /* Toggle dots for animation effect */
    dots_visible = !dots_visible;
}

/* Called from timer ISR to refresh display when needed */
void DISPLAY_TimerHandler(void)
{
    if (display_initialized && update_pending) {
        DISPLAY_Refresh();
    }
}