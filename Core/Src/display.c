/**
 * @file    display.c
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com | glnmatteo0@gmail.com]
 * @date    2025-04-01 
 * @version v1.0.0
 * @prefix  DISP
 *
 * @brief   Implementation code for the display
 * @details This code implements bla bla
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/*---------- Includes --------------------------------------------------------*/

#include "display.h"


/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/
uint8_t DISP_buffer[DISP_BUFFER_SIZE];
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
void DISP_init(void) {
    if (ILI9488_init(&ili9488_handle, &ili9488_gpio_map) == Status_OK) {
        LVGL_init();
        EEZ_create_screen(EEZ_UTI_Tires_Page);
        EEZ_ACT_cmn_set_lbl_lv_bat_v(11.2);
        EEZ_ACT_cmn_set_lbl_hv_soc(90);
        EEZ_ACT_cmn_set_lbl_sx_rot_sw_map(2);
        EEZ_ACT_cmn_set_lbl_dx_rot_sw_map(5);
        EEZ_ACT_cmn_set_pnl_status_bar_color(EEZ_STATUS_COLOR_RED);
        EEZ_ACT_tires_set_lbl_fl_tmp(40.9);
        EEZ_ACT_tires_set_lbl_fr_tmp(2.5);
        EEZ_ACT_tires_set_lbl_rr_tmp(1.4);
        EEZ_ACT_tires_set_lbl_rl_tmp(2.0);
        EEZ_ACT_tires_set_lbl_fl_bar(22.0);
        EEZ_ACT_tires_set_lbl_fr_bar(11.0);
        EEZ_ACT_tires_set_lbl_rr_bar(21.0);
        EEZ_ACT_tires_set_lbl_rl_bar(22.0);
    }//TODO else statement
}
int cont = 0;
void DISP_update_routine(void) {
    if (cont>100)cont=0;
    
    
}

/*---------- Private Functions -----------------------------------------------*/
