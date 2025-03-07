/**
 * @file    lvgl_callbacks.h
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com || glnmatteo0@gmail.com]
 * @date    2024-12-20
 * @version v0.0.1
 * @prefix  LVGL_CLB
 *
 * @brief   Implementation of the fuctions required (and also auxiliary) for the LVGL Lib
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LVGL_CALLBACKS_H_
#define _LVGL_CALLBACKS_H_

/*---------- SHARED ##########################################################*/
/* ---------- Includes -------------------------------------------------------*/
#include "ili9488.h"
#include "lvgl.h"
#include "bsp.h"

/*---------- LVGL DRAW CALLBACK ##############################################*/
/* ---------- Includes -------------------------------------------------------*/
/* ---------- Exported types -------------------------------------------------*/
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB888))
#define BUFFER_SIZE     (VERTICAL_RES * HORIZONTAL_RES * BYTES_PER_PIXEL / 7)
/* ---------- Exported constants ---------------------------------------------*/
/* ---------- Exported variables ---------------------------------------------*/
extern uint8_t buf1[];
/* ---------- Exported macros ------------------------------------------------*/
/* ---------- Exported functions ---------------------------------------------*/
void LVGL_CLB_flush_clb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map);
/* ---------- Private types --------------------------------------------------*/
/* ---------- Private variables ----------------------------------------------*/
/* ---------- Private constants ----------------------------------------------*/
/* ---------- Private Macros -------------------------------------------------*/

/*---------- LVGL LOG CALLBACK ###############################################*/
/* ---------- Includes -------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
/* ---------- Exported types -------------------------------------------------*/
/* ---------- Exported constants ---------------------------------------------*/
/* ---------- Exported variables ---------------------------------------------*/
/* ---------- Exported macros ------------------------------------------------*/
/* ---------- Exported functions ---------------------------------------------*/
void LVGL_CLB_log_clb(lv_log_level_t level, const char *buf);
void LVGL_CLB_mem_usage();
/* ---------- Private types --------------------------------------------------*/
/* ---------- Private variables ----------------------------------------------*/
/* ---------- Private constants ----------------------------------------------*/
/* ---------- Private Macros -------------------------------------------------*/

#endif