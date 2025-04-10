/**
 * @file    eez_utilis.c
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com]
 * @date    2024-11-22 (date of creation)
 * @updated 2025-04-02 (date of last update)
 * @version v1.0.1
 * @prefix  EEZ
 *
 * @brief   Implementation of the function utilities for the eez ui 
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/*---------- Includes --------------------------------------------------------*/

#include "eez_utils.h"

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

/*---------- Private function prototypes -------------------------------------*/
/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/
/**
 * @brief Creates or switches to the requested screen
 * @param page The screen page to create/display
 * 
 * Handles screen management efficiently by:
 * - Checking if we're already displaying the requested content
 * - Properly cleaning up existing screens
 * - Preventing unnecessary recreations
 */
void EEZ_create_screen(enum EEZ_UTI_Page page) {
    lv_obj_t* current_screen = lv_scr_act();
    lv_obj_t** target_screen_ptr = NULL;
    
    switch (page) {
        case EEZ_UTI_Tires_Page:
            target_screen_ptr = &objects.tires;
            break;
        default:
            return;
    }

    // Check if we're already showing this screen
    if (*target_screen_ptr != NULL && current_screen == *target_screen_ptr) {
        return;
    }

    // Clean up existing screen if it exists
    if (*target_screen_ptr != NULL) {
        lv_obj_del(*target_screen_ptr);
        *target_screen_ptr = NULL;
    }

    switch (page) {
        case EEZ_UTI_Tires_Page:
            create_screen_tires();
            lv_scr_load(objects.tires);
            break;
            
        default:
            break;
    }
}

/*---------- Private Functions -----------------------------------------------*/
