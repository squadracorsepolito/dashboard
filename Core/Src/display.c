/**
 * @file    display.c
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com | glnmatteo0@gmail.com]
 * @date    2025-04-01 
 * @version v1.0.0
 * @prefix  DISPLAY
 *
 * @brief   Implementation code for the display
 * @details This code implements bla bla
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/*---------- Includes --------------------------------------------------------*/

#include "display.h"

#include "lvgl.h"
#include "lvgl_callbacks.h"
#include "screen_loader.h"
#include "screens.h"

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/
 struct ILI9488_GPIO_Map ili9488_gpio_map = {
    .CS  = {.GPIO_Port = LCD_TFT_CS_GPIO_OUT_GPIO_Port, .GPIO_Pin = LCD_TFT_CS_GPIO_OUT_Pin},
    .DC  = {.GPIO_Port = LCD_TFT_DC_GPIO_OUT_GPIO_Port, .GPIO_Pin = LCD_TFT_DC_GPIO_OUT_Pin},
    .RST = {.GPIO_Port = LCD_TFT_RST_GPIO_OUT_GPIO_Port, .GPIO_Pin = LCD_TFT_RST_GPIO_OUT_Pin},
};

struct ILI9488_Handle ili9488_handle = {
    .CS_SetState      = ILI9488_CS_Pin_SetState,
    .DC_RS_SetState   = ILI9488_DC_Pin_SetState,
    .RST_SetState     = ILI9488_RST_Pin_SetState,
    .SPI_Transmit_DMA = ILI9488_SPI_Transmit_DMA,
    .Delay            = ILI9488_Delay,
};

/*---------- Exported Functions ----------------------------------------------*/
int j = 0;

void Display_Setup(void) {
    if (ILI9488_init(&ili9488_handle, &ili9488_gpio_map) == Status_OK) {
        lv_init();

        lv_tick_set_cb(HAL_GetTick);
        lv_display_t *display1 = lv_display_create(HORIZONTAL_RES, VERTICAL_RES);
        lv_display_set_buffers(display1, buf1, NULL, LVGL_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
        lv_display_set_flush_cb(display1, LVGL_CLB_flush_clb);
        custom_ui_init();

        create_screen_main();
        lv_scr_load(objects.main);

        j = -1;
    } else {
        j = 1;
    }
}

void LCD_DisplayUpdateRoutine(void) {
    lv_obj_t *scr_act = lv_scr_act();
    if (scr_act != NULL) {
        lv_obj_del(lv_scr_act());
    }

    create_screen_main();
    lv_scr_load(objects.main);
}

/*---------- Private Functions -----------------------------------------------*/
