/**
 * @file    lvgl_utils.c
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com || glnmatteo0@gmail.com]
 * @date    2024-12-20
 * @update  2025-04-02
 * @version v1.0.1
 * @prefix  LVGL
 *
 * @brief   Implementation of the fuctions required (and also auxiliary) for the LVGL Lib
 */

#include "lvgl_utils.h"
/*---------- Includes --------------------------------------------------------*/

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/
static volatile uint8_t flush_in_progress = 0;

/*---------- Exported Functions ----------------------------------------------*/
void LVGL_init(void) {
    lv_init();
    lv_tick_set_cb(HAL_GetTick);
    lv_display_t *display = lv_display_create(ILI9488_HORIZONTAL_RES, ILI9488_VERTICAL_RES);
    lv_display_set_buffers(display, DISP_buffer, NULL, DISP_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, LVGL_flush_clbk);

    lv_theme_t *theme = lv_theme_default_init(
        display, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(display, theme);
}
void LVGL_flush_clbk(lv_display_t *display, const lv_area_t *area, uint8_t *px_map) {
    if (flush_in_progress)
        return;

    flush_in_progress    = 1;
    uint32_t width       = area->x2 - area->x1 + 1;
    uint32_t height      = area->y2 - area->y1 + 1;
    uint32_t pixel_count = width * height;

    ILI9488_set_draw_window(&ili9488_handle, area->x1, area->y1, area->x2, area->y2);
    ILI9488_SPI_Send_DMA(&ili9488_handle, px_map, pixel_count * 3);
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
        ili9488_handle.CS_SetState(PinState_Set);
        lv_display_flush_ready(lv_disp_get_default());
    }
}
/*---------- Private Functions -----------------------------------------------*/