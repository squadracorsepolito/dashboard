/**
 * @file    display.c
 * @author  Matteo Giuliani [matteo.giuliani.sc@gmail.com | glnmatteo0@gmail.com]
 * @date    2025-04-01 
 * @version v1.0.0
 * @prefix  DISP
 *
 * @brief   Implementation code for the display
 * @details This code implements bla bla
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/*---------- Includes --------------------------------------------------------*/

#include "display.h"

#include "bsp.h"
#include "dashboard.h"

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/
uint8_t DISP_buffer[DISP_BUFFER_SIZE];
/*---------- Private function prototypes -------------------------------------*/
void ILI9488_CS_Pin_SetState(enum ILI9488_PinState state);
void ILI9488_DC_Pin_SetState(enum ILI9488_PinState state);
void ILI9488_RST_Pin_SetState(enum ILI9488_PinState state);
enum ILI9488_Status ILI9488_SPI_Transmit_DMA(uint8_t *data, uint16_t size);
void ILI9488_Delay(uint32_t delay_ms);
/*---------- Exported Variables ----------------------------------------------*/
struct ILI9488_GPIO_Map ili9488_gpio_map = {
    .CS  = {.GPIO_Port = LCD_TFT_CS_GPIO_OUT_GPIO_Port, .GPIO_Pin = LCD_TFT_CS_GPIO_OUT_Pin},
    .DC  = {.GPIO_Port = LCD_TFT_DC_GPIO_OUT_GPIO_Port, .GPIO_Pin = LCD_TFT_DC_GPIO_OUT_Pin},
    .RST = {.GPIO_Port = LCD_TFT_RST_GPIO_OUT_GPIO_Port, .GPIO_Pin = LCD_TFT_RST_GPIO_OUT_Pin},
};

struct ILI9488_Handle ili9488_handle = {
    .CS_SetState      = ILI9488_CS_Pin_SetState,
    .DC_RS_SetState   = ILI9488_DC_Pin_SetState,
    .RST_SetState     = ILI9488_RST_Pin_SetState,
    .SPI_Transmit_DMA = ILI9488_SPI_Transmit_DMA,
    .Delay            = ILI9488_Delay,
};

/*---------- Exported Functions ----------------------------------------------*/
void DISP_init(void) {
    if (ILI9488_init(&ili9488_handle) == Status_OK) {
        LVGL_init();
        EEZ_create_screen(EEZ_UTI_Tires_Page);
    }  //TODO else statement
}

void DISP_update_routine(void) {
    EEZ_ACT_cmn_set_lbl_lv_bat_v(dashboard_data.LV_BAT_mV/1000.0);
    EEZ_ACT_cmn_set_lbl_hv_soc(dashboard_data.HV_BAT_SOC);
    EEZ_ACT_cmn_set_lbl_sx_rot_sw_map(ROT_SW_getState(ROT_SW_Device1));
    EEZ_ACT_cmn_set_lbl_dx_rot_sw_map(ROT_SW_getState(ROT_SW_Device2));

    if (!BTN_getStatus(BTN_LC)) {
        EEZ_ACT_cmn_set_lbl_LC_color(EEZ_COLOR_BLACK);
    }else{
        EEZ_ACT_cmn_set_lbl_LC_color(EEZ_COLOR_CYAN);
    }

    if (!BTN_getStatus(BTN_TC)) {
        EEZ_ACT_cmn_set_lbl_TC_color(EEZ_COLOR_BLACK);
    }else{
        EEZ_ACT_cmn_set_lbl_TC_color(EEZ_COLOR_CYAN);
    }

    if (!BTN_getStatus(BTN_TV)) {
        EEZ_ACT_cmn_set_lbl_TV_color(EEZ_COLOR_BLACK);
    }else{
        EEZ_ACT_cmn_set_lbl_TV_color(EEZ_COLOR_CYAN);
    }

    switch (dashboard_data.RTD_FSM_State) {
        case STATE_IDLE:
            EEZ_ACT_cmn_set_pnl_status_bar_color(EEZ_COLOR_BLUE);
            break;
        case STATE_TSON:
            EEZ_ACT_cmn_set_pnl_status_bar_color(EEZ_COLOR_PURPLE);
            break;
        case STATE_RTD_SOUND:
            break;
        case STATE_RTD:
            EEZ_ACT_cmn_set_pnl_status_bar_color(EEZ_COLOR_GREEN);
            break;
        case STATE_DISCHARGE:
            EEZ_ACT_cmn_set_pnl_status_bar_color(EEZ_COLOR_CYAN);
            break;
        default:
            break;
    }

    EEZ_ACT_tires_set_lbl_fl_tmp(dashboard_data.TIRE_FL_TEMP);
    EEZ_ACT_tires_set_lbl_fr_tmp(dashboard_data.TIRE_FR_TEMP);
    EEZ_ACT_tires_set_lbl_rl_tmp(dashboard_data.TIRE_RL_TEMP);
    EEZ_ACT_tires_set_lbl_rr_tmp(dashboard_data.TIRE_RR_TEMP);
    EEZ_ACT_tires_set_lbl_fl_bar(dashboard_data.TIRE_FL_PRESSURE);
    EEZ_ACT_tires_set_lbl_fr_bar(dashboard_data.TIRE_FR_PRESSURE);
    EEZ_ACT_tires_set_lbl_rl_bar(dashboard_data.TIRE_RL_PRESSURE);
    EEZ_ACT_tires_set_lbl_rr_bar(dashboard_data.TIRE_RR_PRESSURE);
}

void ILI9488_CS_Pin_SetState(enum ILI9488_PinState state) {
    HAL_GPIO_WritePin(ili9488_gpio_map.CS.GPIO_Port,
                      ili9488_gpio_map.CS.GPIO_Pin,
                      (state == PinState_Set) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ILI9488_DC_Pin_SetState(enum ILI9488_PinState state) {
    HAL_GPIO_WritePin(ili9488_gpio_map.DC.GPIO_Port,
                      ili9488_gpio_map.DC.GPIO_Pin,
                      (state == PinState_Set) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ILI9488_RST_Pin_SetState(enum ILI9488_PinState state) {
    HAL_GPIO_WritePin(ili9488_gpio_map.RST.GPIO_Port,
                      ili9488_gpio_map.RST.GPIO_Pin,
                      (state == PinState_Set) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

enum ILI9488_Status ILI9488_SPI_Transmit_DMA(uint8_t *data, uint16_t size) {
    if (HAL_SPI_Transmit_DMA(&LCD_TFT_SPI_Handle, data, size) == HAL_OK) {
        // OCIOOOOOO
        //TODO to remove
        uint32_t timeout = 1000;
        while (HAL_SPI_GetState(&LCD_TFT_SPI_Handle) != HAL_SPI_STATE_READY && timeout > 0) {
            timeout--;
            ILI9488_Delay(1);
        }
        return (timeout > 0) ? Status_OK : Status_Timeout;
    }
    return Status_ERR;
}

void ILI9488_Delay(uint32_t delay_ms) {
    HAL_Delay(delay_ms);
}

/*---------- Private Functions -----------------------------------------------*/
