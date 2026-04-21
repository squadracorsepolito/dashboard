/**
 * @file    eez_actions.h
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com]
 * @date    2024-11-22 (date of creation)
 * @updated 2025-04-02 (date of last update)
 * @version v1.0.1
 * @prefix  EEZ_ACT
 *
 * @brief   Header of the functions that defines actions for the eez ui 
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _EEZ_ACTIONS_H_
#define _EEZ_ACTIONS_H_

/* ---------- Includes -------------------------------------------------------*/
#include "screens.h"

#include <stdio.h>
/* ---------- Exported types -------------------------------------------------*/
enum EEZ_Colors {
    EEZ_COLOR_WHITE = 0,
    EEZ_COLOR_BLACK,
    EEZ_COLOR_GREEN,
    EEZ_COLOR_RED,
    EEZ_COLOR_BLUE,
    EEZ_COLOR_PURPLE,
    EEZ_COLOR_CYAN,
    EEZ_COLOR_COUNT
};

/* ---------- Exported constants ---------------------------------------------*/

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/
void EEZ_ACT_set_lbl_float(float new_value, lv_obj_t *screen, lv_obj_t *obj);
void EEZ_ACT_set_lbl_uint8(uint8_t new_value, lv_obj_t *screen, lv_obj_t *obj);
void EEZ_ACT_set_lbl_color(enum EEZ_Colors new_color, lv_obj_t *screen, lv_obj_t *obj);
void EEZ_ACT_set_panel_color(enum EEZ_Colors new_color, lv_obj_t *screen, lv_obj_t *obj_sx, lv_obj_t *obj_dx);
void EEZ_ACT_set_lbl_str(char* new_value, lv_obj_t *screen, lv_obj_t *obj);
/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* ---------- Private Macros -------------------------------------------------*/
#endif