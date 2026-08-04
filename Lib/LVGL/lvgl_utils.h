/**
 * @file    lvgl_utils.h
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com || glnmatteo0@gmail.com]
 * @date    2024-12-20
 * @version v1.0.1
 * @prefix  LVGL
 *
 * @brief   Implementation of the fuctions required (and also auxiliary) for the LVGL Lib
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LVGL_UTILS_H_
#define _LVGL_UTILS_H_

/* ---------- Includes -------------------------------------------------------*/
#include "ili9488.h"
#include "lvgl.h"
#include "display.h"
/* ---------- Exported types -------------------------------------------------*/
/* ---------- Exported constants ---------------------------------------------*/
/* ---------- Exported variables ---------------------------------------------*/
extern struct ILI9488_Handle ili9488_handle; 
/* ---------- Exported macros ------------------------------------------------*/
/* ---------- Exported functions ---------------------------------------------*/
void LVGL_flush_clbk(lv_display_t *display, const lv_area_t *area, uint8_t *px_map);
void LVGL_init(void);
/* ---------- Private types --------------------------------------------------*/
/* ---------- Private variables ----------------------------------------------*/
/* ---------- Private constants ----------------------------------------------*/
/* ---------- Private Macros -------------------------------------------------*/
#endif