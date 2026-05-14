/**
 * @file    display.h
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com | glnmatteo0@gmail.com]
 * @date    2025-04-01
 * @version v1.0.0
 * @prefix  DISP
 *
 * @brief   Library for the display
 * @details This code implements bla bla
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DISPLAY_H_
#define _DISPLAY_H_

/* ---------- Includes -------------------------------------------------------*/

#include "spi.h"
#include "usart.h"
#include "lvgl_utils.h"
#include "ili9488.h"
#include "lvgl.h"
#include "eez_utils.h"
#include "eez_actions.h"
#include "bsp.h"
#include "screens.h"
#include "mission.h"

/* ---------- Exported types -------------------------------------------------*/
/* ---------- Exported constants ---------------------------------------------*/
struct ILI9488_GPIO_Map {
    struct GPIO_Tuple CS;
    struct GPIO_Tuple DC;
    struct GPIO_Tuple RST;
};

#define LCD_TFT_SPI_Handle hspi1

#define DISP_BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB888))
#define DISP_BUFFER_SIZE     (ILI9488_VERTICAL_RES * ILI9488_HORIZONTAL_RES * DISP_BYTES_PER_PIXEL / 7)

/* ---------- Exported variables ---------------------------------------------*/
extern uint8_t DISP_buffer[DISP_BUFFER_SIZE];
/* ---------- Exported macros ------------------------------------------------*/


/* ---------- Exported functions ---------------------------------------------*/
void DISP_init(void);
void DISP_update_routine(void);

/* ---------- Private types --------------------------------------------------*/


/* ---------- Private variables ----------------------------------------------*/


/* ---------- Private constants ----------------------------------------------*/


/* ---------- Private Macros -------------------------------------------------*/


#endif