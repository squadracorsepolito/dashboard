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
#include "screen_loader.h"
#include "screens.h"
#include "lvgl_callbacks.h"
#include "lvgl.h"
#include "ili9488.h"



/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/
void Display_Setup(void) {
    ILI9488_init();
    lv_init();

    lv_tick_set_cb(HAL_GetTick);
    lv_display_t *display1 = lv_display_create(HORIZONTAL_RES, VERTICAL_RES);
    lv_display_set_buffers(display1, buf1, NULL, LVGL_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display1, LVGL_CLB_flush_clb);

#if LV_USE_LOG
    lv_log_register_print_cb(LVGL_CLB_log_clb);
#endif

    custom_ui_init();

    create_screen_main();
    lv_scr_load(objects.main);
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
