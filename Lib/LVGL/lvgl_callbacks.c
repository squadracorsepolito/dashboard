/**
 * @file    lvgl_callbacks.c
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com || glnmatteo0@gmail.com]
 * @date    2024-12-20
 * @version v0.0.1
 * @prefix  LVGL_CLB
 *
 * @brief   Implementation of the fuctions required (and also auxiliary) for the LVGL Lib
 */

/*---------- SHARED ##########################################################*/
/*---------- Includes --------------------------------------------------------*/
#include "lvgl_callbacks.h"
/*---------- LVGL DRAW CALLBACK ##############################################*/
/*---------- Includes --------------------------------------------------------*/
/*---------- Private define --------------------------------------------------*/
/*---------- Private macro ---------------------------------------------------*/
/*---------- Private variables -----------------------------------------------*/
/*---------- Private function prototypes -------------------------------------*/
/*---------- Exported Variables ----------------------------------------------*/
uint8_t buf1[BUFFER_SIZE];
static volatile uint8_t flush_in_progress = 0; 
/*---------- Exported Functions ----------------------------------------------*/
void LVGL_CLB_flush_clb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map) {
    if (flush_in_progress) return;

    flush_in_progress = 1;
    uint32_t width       = area->x2 - area->x1 + 1;
    uint32_t height      = area->y2 - area->y1 + 1;
    uint32_t pixel_count = width * height;

    ILI9488_set_draw_window(area->x1, area->y1, area->x2, area->y2);
    ILI9488_SPI_DMA_send(px_map, pixel_count * 3);
}

/**
 * @brief Callback function for SPI transmission completion.
 *
 * This function is a weak definition in the SPI library and is overridden here
 * to perform custom actions when the SPI transmission is completed for SPIx.
 * Specifically, it deactivates the chip select pin for the TFT display and signals
 * that the flush operation is complete.
 *
 * @note This function is called automatically by the HAL SPI interrupt handler 
 * and is defined in the STM32 HAL library (see stm32xx_hal_spi.h).
 *
 * @param hspi Pointer to the SPI handle structure.
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == LCD_TFT_SPI_Handle.Instance) {
        flush_in_progress = 0;
        ILI9488_CS_set_state(GPIO_PIN_SET);
        lv_display_flush_ready(lv_disp_get_default());
    }
}
/*---------- Private Functions -----------------------------------------------*/

/*---------- LCGL LOG CALLBACK ###############################################*/

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/
void LVGL_CLB_log_clb(lv_log_level_t level, const char *buf) {
    char log_msg[256];

    const char *level_str;
    switch (level) {
        case LV_LOG_LEVEL_TRACE:
            level_str = "TRACE: ";
            break;
        case LV_LOG_LEVEL_INFO:
            level_str = "INFO: ";
            break;
        case LV_LOG_LEVEL_WARN:
            level_str = "WARN: ";
            break;
        case LV_LOG_LEVEL_ERROR:
            level_str = "ERROR: ";
            break;
        default:
            level_str = "LOG: ";
            break;
    }

    snprintf(log_msg, sizeof(log_msg), "%s%s\r\n", level_str, buf);

    HAL_UART_Transmit(&LCD_TFT_USART_Handle, (uint8_t *)log_msg, strlen(log_msg), HAL_MAX_DELAY);
}

void LVGL_CLB_mem_usage() {
    lv_mem_monitor_t mem_mon;
    lv_mem_monitor(&mem_mon);

    char buffer[256];

    snprintf(buffer,
             sizeof(buffer),
             "Total memory: %lu bytes\n"
             "Used memory: %lu bytes\n"
             "Free memory: %lu bytes\n"
             "percentage of usage: %d%%\n"
             "Biggest memory block available: %lu bytes\n",
             (unsigned long)mem_mon.total_size,
             (unsigned long)(mem_mon.total_size - mem_mon.free_size),
             (unsigned long)mem_mon.free_size,
             mem_mon.used_pct,
             (unsigned long)mem_mon.free_biggest_size);

    HAL_UART_Transmit(&LCD_TFT_USART_Handle, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
/*---------- Private Functions -----------------------------------------------*/