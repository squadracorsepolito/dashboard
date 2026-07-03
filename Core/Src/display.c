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
        EEZ_create_screen(EEZ_UTI_Main_Page);
    }  //TODO else statement
}

void DISP_update_routine(void) {
    EEZ_ACT_set_lbl_float(dashboard_data.LV_BAT_mV / 1000.0, objects.main, objects.pg_main_lv_bat_v);
    EEZ_ACT_set_lbl_uint8(dashboard_data.HV_BAT_SOC, objects.main, objects.pg_main_hv_soc_est);
    EEZ_ACT_set_lbl_uint8(dashboard_data.ROT_SW_1_STATE, objects.main, objects.pg_main_sx_map);
    EEZ_ACT_set_lbl_uint8(dashboard_data.ROT_SW_2_STATE, objects.main, objects.pg_main_dx_map);
    EEZ_ACT_set_lbl_str(mission_convert(dashboard_data.AS_MISSION), objects.main, objects.mission_name);

    if (dashboard_data.LC_BTN_STATE == 0) {
        EEZ_ACT_set_lbl_color(EEZ_COLOR_WHITE, objects.main, objects.pg_main_lc);
    } else {
        EEZ_ACT_set_lbl_color(EEZ_COLOR_GREEN, objects.main, objects.pg_main_lc);
    }

    if (dashboard_data.TC_BTN_STATE == 0) {
        EEZ_ACT_set_lbl_color(EEZ_COLOR_WHITE, objects.main, objects.pg_main_tc);
    } else {
        EEZ_ACT_set_lbl_color(EEZ_COLOR_GREEN, objects.main, objects.pg_main_tc);
    }

    if (dashboard_data.TV_BTN_STATE == 0) {
        EEZ_ACT_set_lbl_color(EEZ_COLOR_WHITE, objects.main, objects.pg_main_tv);
    } else {
        EEZ_ACT_set_lbl_color(EEZ_COLOR_GREEN, objects.main, objects.pg_main_tv);
    }

    switch (dashboard_data.RTD_FSM_State) {
        case STATE_IDLE:
            EEZ_ACT_set_panel_color(
                EEZ_COLOR_BLUE, objects.main, objects.pg_main_sx_status_bar, objects.pg_main_dx_status_bar);
            break;
        case STATE_TSON:
            EEZ_ACT_set_panel_color(
                EEZ_COLOR_PURPLE, objects.main, objects.pg_main_sx_status_bar, objects.pg_main_dx_status_bar );

            break;
        case STATE_RTD_SOUND:
            break;
        case STATE_RTD:
            EEZ_ACT_set_panel_color(EEZ_COLOR_GREEN, objects.main, objects.pg_main_sx_status_bar, objects.pg_main_dx_status_bar);
            break;
        case STATE_DISCHARGE:
            EEZ_ACT_set_panel_color(EEZ_COLOR_CYAN, objects.main, objects.pg_main_sx_status_bar, objects.pg_main_dx_status_bar);
            break;
        default:
            break;
    }
    
    EEZ_ACT_set_lbl_float(dashboard_data.COOL_PRESS_LEFT_mV, objects.main, objects.pg_main_cool_press_l);
    EEZ_ACT_set_lbl_float(dashboard_data.COOL_PRESS_RIGHT_mV, objects.main, objects.pg_main_cool_press_r);
    EEZ_ACT_set_lbl_float(dashboard_data.INVERTER_FL_TEMP, objects.main, objects.pg_main_inv_temp_fl);
    EEZ_ACT_set_lbl_float(dashboard_data.INVERTER_FR_TEMP, objects.main, objects.pg_main_inv_temp_fr);
    EEZ_ACT_set_lbl_float(dashboard_data.INVERTER_RL_TEMP, objects.main, objects.pg_main_inv_temp_rl);
    EEZ_ACT_set_lbl_float(dashboard_data.INVERTER_RR_TEMP, objects.main, objects.pg_main_inv_temp_rr);
    EEZ_ACT_set_lbl_float(dashboard_data.MOTOR_FL_TEMP, objects.main, objects.pg_main_motor_temp_fl);
    EEZ_ACT_set_lbl_float(dashboard_data.MOTOR_FR_TEMP, objects.main, objects.pg_main_motor_temp_fr);
    EEZ_ACT_set_lbl_float(dashboard_data.MOTOR_RL_TEMP, objects.main, objects.pg_main_motor_temp_rl);
    EEZ_ACT_set_lbl_float(dashboard_data.MOTOR_RR_TEMP, objects.main, objects.pg_main_motor_temp_rr);
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
