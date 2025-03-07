/**
 * @file    screen_loader.h
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com]
 * @date    2024-11-22 (date of creation)
 * @updated 202x-xx-xx (date of last update)
 * @version v0.0.1
 * @prefix  TMP
 *
 * @brief   Header file of some sceen loader
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SCREEN_LOADER_H_
#define _SCREEN_LOADER_H_

/* ---------- Includes -------------------------------------------------------*/
#include "lvgl.h"
#include "ui.h"

/* ---------- Exported types -------------------------------------------------*/


/* ---------- Exported constants ---------------------------------------------*/


/* ---------- Exported variables ---------------------------------------------*/


/* ---------- Exported macros ------------------------------------------------*/


/* ---------- Exported functions ---------------------------------------------*/


/* ---------- Private types --------------------------------------------------*/


/* ---------- Private variables ----------------------------------------------*/


/* ---------- Private constants ----------------------------------------------*/


/* ---------- Private Macros -------------------------------------------------*/
static lv_obj_t *getLvglObjectFromIndex(int32_t index);
void custom_ui_init();

#endif