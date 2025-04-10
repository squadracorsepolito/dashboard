/**
 * @file    eez_utils.h
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com]
 * @date    2024-11-22 (date of creation)
 * @updated 2025-04-02 (date of last update)
 * @version v1.0.1
 * @prefix  EEZ_UTI
 *
 * @brief   Header of the function utilities for the eez ui 
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _EEZ_UTILS_H_
#define _EEZ_UTILS_H_

/* ---------- Includes -------------------------------------------------------*/
#include "screens.h"
#include "styles.h"
#include "ui.h"
/* ---------- Exported types -------------------------------------------------*/

/* ---------- Exported constants ---------------------------------------------*/
enum EEZ_UTI_Page {
    EEZ_UTI_Tires_Page = SCREEN_ID_TIRES,
    EEZ_UTI_NUM_Page,
};

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/
void EEZ_create_screen(enum EEZ_UTI_Page page);

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* ---------- Private Macros -------------------------------------------------*/

#endif