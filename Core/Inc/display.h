/**
 * @file    display.h
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com | glnmatteo0@gmail.com]
 * @date    2025-04-01
 * @version v1.0.0
 * @prefix  DISPLAY
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
#include "ili9488.h"

/* ---------- Exported types -------------------------------------------------*/


/* ---------- Exported constants ---------------------------------------------*/
#define LCD_TFT_SPI_Handle hspi1
#define LCD_TFT_USART_Handle huart1

/* ---------- Exported variables ---------------------------------------------*/
extern struct ILI9488_Handle ili9488_handle; 
extern struct ILI9488_GPIO_Map ili9488_gpio_map;
/* ---------- Exported macros ------------------------------------------------*/


/* ---------- Exported functions ---------------------------------------------*/
void Display_Setup(void);
void LCD_DisplayUpdateRoutine(void);

/* ---------- Private types --------------------------------------------------*/


/* ---------- Private variables ----------------------------------------------*/


/* ---------- Private constants ----------------------------------------------*/


/* ---------- Private Macros -------------------------------------------------*/


#endif