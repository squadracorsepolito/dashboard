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

/*---------- Includes --------------------------------------------------------*/

#include "eez_actions.h"

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/
static const lv_color_t LVGL_Colors[EEZ_COLOR_COUNT] = {
    LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),  // White
    LV_COLOR_MAKE(0x00, 0x00, 0x00),  // Black
    LV_COLOR_MAKE(0x00, 0xFF, 0x00),  // Green
    LV_COLOR_MAKE(0x00, 0x00, 0xFF),  // Red
    LV_COLOR_MAKE(0xFF, 0x00, 0x00),  // Blue
    LV_COLOR_MAKE(0x80, 0x00, 0x80),  // Purple
    LV_COLOR_MAKE(0xFF, 0xFF, 0x00),  // Yellow
};
/*---------- Private function prototypes -------------------------------------*/
static lv_obj_t *EEZ_ACT_get_curr_screen(void);
/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/

/**
 * @brief Sets the text of a label object to a formatted float value.
 *
 * @param new_value The float value to display.
 * @param screen The screen object where the label resides.
 * @param obj The label object to update.
 *
 * The label is updated only if the provided screen is the current screen and the object is not NULL.
 */
void EEZ_ACT_set_lbl_float(float new_value, lv_obj_t *screen, lv_obj_t *obj) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%.1f", new_value);

    if (screen == EEZ_ACT_get_curr_screen()) {
        if (obj != NULL) {
            lv_label_set_text(obj, new_value_str);
        }
    }
}

/**
 * @brief Sets the text of a label object to a formatted uint8_t value.
 *
 * @param new_value The uint8_t value to display.
 * @param screen The screen object where the label resides.
 * @param obj The label object to update.
 *
 * The label is updated only if the provided screen is the current screen and the object is not NULL.
 */
void EEZ_ACT_set_lbl_uint8(uint8_t new_value, lv_obj_t *screen, lv_obj_t *obj) {
    char new_value_str[16];
    snprintf(new_value_str, sizeof(new_value_str), "%u", new_value);

    if (screen == EEZ_ACT_get_curr_screen()) {
        if (obj != NULL) {
            lv_label_set_text(obj, new_value_str);
        }
    }
}

/**
 * @brief Sets the text color of a label object.
 *
 * @param new_color The color to set, as an EEZ_Colors enum value.
 * @param screen The screen object where the label resides.
 * @param obj The label object to update.
 *
 * The color is updated only if the provided screen is the current screen, the object is not NULL,
 * and the color is within the valid range.
 */
void EEZ_ACT_set_lbl_color(enum EEZ_Colors new_color, lv_obj_t *screen, lv_obj_t *obj) {
    if (new_color >= EEZ_COLOR_COUNT) {
        return;
    }

    if (screen == EEZ_ACT_get_curr_screen()) {
        if (obj != NULL) {
            lv_obj_set_style_text_color(obj, LVGL_Colors[new_color], LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

/**
 * @brief Sets the background color of a panel object.
 *
 * @param new_color The color to set, as an EEZ_Colors enum value.
 * @param screen The screen object where the panel resides.
 * @param obj The panel object to update.
 *
 * The color is updated only if the provided screen is the current screen, the object is not NULL,
 * and the color is within the valid range.
 */
void EEZ_ACT_set_panel_color(enum EEZ_Colors new_color, lv_obj_t *screen, lv_obj_t *obj_sx, lv_obj_t *obj_dx) {
    if (new_color >= EEZ_COLOR_COUNT) {
        return;
    }

    if (screen == EEZ_ACT_get_curr_screen()) {
        if (obj_sx != NULL && obj_dx != NULL) {
            lv_obj_set_style_bg_color(obj_sx, LVGL_Colors[new_color], LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj_dx, LVGL_Colors[new_color], LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

/*---------- Private Functions -----------------------------------------------*/
static lv_obj_t *EEZ_ACT_get_curr_screen(void) {
    return lv_scr_act();
}
