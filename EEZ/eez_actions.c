/**
 * @file    eez_actions.c
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com]
 * @date    2024-11-22 (date of creation)
 * @updated 2025-04-02 (date of last update)
 * @version v1.0.1
 * @prefix  EEZ_ACT
 *
 * @brief   Implementation of the functions that defines actions for the eez ui
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/* SHARED ####################################################################*/
/*---------- Includes --------------------------------------------------------*/

#include "eez_actions.h"

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/
static const lv_color_t LVGL_Colors[EEZ_STATUS_BAR_COLOR_COUNT] = {
    LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),  // White
    LV_COLOR_MAKE(0x00, 0x00, 0x00),  // Black
    LV_COLOR_MAKE(0x00, 0xFF, 0x00),  // Green
    LV_COLOR_MAKE(0x00, 0x00, 0xFF),  // Red
    LV_COLOR_MAKE(0xFF, 0x00, 0x00),  // Blue
    LV_COLOR_MAKE(0x80, 0x00, 0x80),  // Purple
    LV_COLOR_MAKE(0xFF, 0xFF, 0x00),  // Yellow
};
/*---------- Private function prototypes -------------------------------------*/
static lv_obj_t *EEZ_ACT_get_active_lbl_lv_bat_v_object(void);
static lv_obj_t *EEZ_ACT_get_active_lbl_hv_soc_object(void);
static lv_obj_t *EEZ_ACT_get_active_lbl_sx_rot_sw_map_object(void);
static lv_obj_t *EEZ_ACT_get_active_lbl_dx_rot_sw_map_object(void);
static void EEZ_ACT_get_active_pnl_status_bar_object(lv_obj_t **left_bar, lv_obj_t **right_bar);

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/
void EEZ_ACT_cmn_set_lbl_lv_bat_v(float new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    lv_obj_t *active_label = EEZ_ACT_get_active_lbl_lv_bat_v_object();
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_cmn_set_lbl_hv_soc(uint8_t new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1i", new_value);

    lv_obj_t *active_label = EEZ_ACT_get_active_lbl_hv_soc_object();
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_cmn_set_lbl_sx_rot_sw_map(uint8_t new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1i", new_value);

    lv_obj_t *active_label = EEZ_ACT_get_active_lbl_sx_rot_sw_map_object();
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_cmn_set_lbl_dx_rot_sw_map(uint8_t new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1i", new_value);

    lv_obj_t *active_label = EEZ_ACT_get_active_lbl_dx_rot_sw_map_object();
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_cmn_set_pnl_status_bar_color(enum EEZ_Status_Bar_Color new_color) {
    if (new_color >= EEZ_STATUS_BAR_COLOR_COUNT) {
        return;
    }

    lv_obj_t *left_bar  = NULL;
    lv_obj_t *right_bar = NULL;
    EEZ_ACT_get_active_pnl_status_bar_object(&left_bar, &right_bar);

    if (left_bar != NULL) {
        lv_obj_set_style_bg_color(left_bar, LVGL_Colors[new_color], LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    if (right_bar != NULL) {
        lv_obj_set_style_bg_color(right_bar, LVGL_Colors[new_color], LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

/*---------- Private Functions -----------------------------------------------*/

/**
 * @brief Gets the correct battery voltage label for the currently active screen
 * @return Pointer to the active screen's battery voltage label object
 * 
 * This function checks which screen is currently active and returns
 * the corresponding battery voltage display object.
 * Returns NULL if no matching screen is found.
 */
static lv_obj_t *EEZ_ACT_get_active_lbl_lv_bat_v_object(void) {
    lv_obj_t *current_screen = lv_scr_act();

    if (current_screen == objects.tires) {
        return objects.pg_tires_lv_bat_v;
    } 
    return NULL;
}

/**
 * @brief Gets the correct high voltage SoC label for the currently active screen
 * @return Pointer to the active screen's label object
 * 
 * This function checks which screen is currently active and returns
 * the corresponding display object.
 * Returns NULL if no matching screen is found.
 */
static lv_obj_t *EEZ_ACT_get_active_lbl_hv_soc_object(void) {
    lv_obj_t *current_screen = lv_scr_act();

    if (current_screen == objects.tires) {
        return objects.pg_tires_hv_soc_est;
    } 

    return NULL;
}

/**
 * @brief Gets the correct left rotary switch label for the currently active screen
 * @return Pointer to the active screen's label object
 * 
 * This function checks which screen is currently active and returns
 * the corresponding display object.
 * Returns NULL if no matching screen is found.
 */
static lv_obj_t *EEZ_ACT_get_active_lbl_sx_rot_sw_map_object(void) {
    lv_obj_t *current_screen = lv_scr_act();

    if (current_screen == objects.tires) {
        return objects.pg_tires_sx_map;
    }

    return NULL;
}

/**
 * @brief Gets the correct right rotary switch label for the currently active screen
 * @return Pointer to the active screen's label object
 * 
 * This function checks which screen is currently active and returns
 * the corresponding display object.
 * Returns NULL if no matching screen is found.
 */
static lv_obj_t *EEZ_ACT_get_active_lbl_dx_rot_sw_map_object(void) {
    lv_obj_t *current_screen = lv_scr_act();

    if (current_screen == objects.tires) {
        return objects.pg_tires_dx_map;
    } 

    return NULL;
}

/**
 * @brief Gets the active status bar objects for the current screen
 * @param[out] left_bar Pointer to store the left status bar object
 * @param[out] right_bar Pointer to store the right status bar object
 * 
 * This function retrieves both status bar objects for the currently active screen.
 * If no matching screen is found, both pointers will be set to NULL.
 */
static void EEZ_ACT_get_active_pnl_status_bar_object(lv_obj_t **left_bar, lv_obj_t **right_bar) {
    lv_obj_t *current_screen = lv_scr_act();

    *left_bar  = NULL;
    *right_bar = NULL;

    if (current_screen == objects.tires) {
        *left_bar  = objects.pg_tires_sx_status_bar;
        *right_bar = objects.pg_tires_dx_status_bar;
    } 
}

/* TIRES #####################################################################*/
/*---------- Includes --------------------------------------------------------*/

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/
void EEZ_ACT_tires_set_lbl_fl_tmp(float new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    lv_obj_t *active_label = objects.pg_tires_fl_temp;
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_tires_set_lbl_fr_tmp(float new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    lv_obj_t *active_label = objects.pg_tires_fr_temp;
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_tires_set_lbl_rr_tmp(float new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    lv_obj_t *active_label = objects.pg_tires_rr_temp;
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_tires_set_lbl_rl_tmp(float new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    lv_obj_t *active_label = objects.pg_tires_rl_temp;
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_tires_set_lbl_fl_bar(float new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    lv_obj_t *active_label = objects.pg_tires_fl_bar;
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_tires_set_lbl_fr_bar(float new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    lv_obj_t *active_label = objects.pg_tires_fr_bar;
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_tires_set_lbl_rr_bar(float new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    lv_obj_t *active_label = objects.pg_tires_rr_bar;
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

void EEZ_ACT_tires_set_lbl_rl_bar(float new_value) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    lv_obj_t *active_label = objects.pg_tires_rl_bar;
    if (active_label != NULL) {
        lv_label_set_text(active_label, new_value_str);
    }
}

/*---------- Private Functions -----------------------------------------------*/
