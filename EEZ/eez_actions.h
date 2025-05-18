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

/* SHARED ####################################################################*/
/* ---------- Includes -------------------------------------------------------*/
#include "screens.h"
#include <stdio.h>
/* ---------- Exported types -------------------------------------------------*/
enum EEZ_Colors{
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
void EEZ_ACT_cmn_set_lbl_lv_bat_v(float new_value);
void EEZ_ACT_cmn_set_lbl_hv_soc(uint8_t new_value);
void EEZ_ACT_cmn_set_lbl_sx_rot_sw_map(uint8_t new_value);
void EEZ_ACT_cmn_set_lbl_dx_rot_sw_map(uint8_t new_value);
void EEZ_ACT_cmn_set_pnl_status_bar_color(enum EEZ_Colors new_color);
void EEZ_ACT_cmn_set_lbl_LC_color(enum EEZ_Colors new_color);
void EEZ_ACT_cmn_set_lbl_TC_color(enum EEZ_Colors new_color);
void EEZ_ACT_cmn_set_lbl_TV_color(enum EEZ_Colors new_color);
/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* ---------- Private Macros -------------------------------------------------*/

/* TIRES #####################################################################*/
/* ---------- Includes -------------------------------------------------------*/

/* ---------- Exported types -------------------------------------------------*/

/* ---------- Exported constants ---------------------------------------------*/

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/
void EEZ_ACT_tires_set_lbl_fl_tmp(float new_value);
void EEZ_ACT_tires_set_lbl_fr_tmp(float new_value);
void EEZ_ACT_tires_set_lbl_rr_tmp(float new_value);
void EEZ_ACT_tires_set_lbl_rl_tmp(float new_value);
void EEZ_ACT_tires_set_lbl_fl_bar(float new_value);
void EEZ_ACT_tires_set_lbl_fr_bar(float new_value);
void EEZ_ACT_tires_set_lbl_rr_bar(float new_value);
void EEZ_ACT_tires_set_lbl_rl_bar(float new_value);
/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* ---------- Private Macros -------------------------------------------------*/
#endif