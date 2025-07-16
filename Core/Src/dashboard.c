/*INCLUDE*/

#include "dashboard.h"

#include "bsp.h"
#include "button.h"
#include "can.h"
#include "dac.h"
#include "display.h"
#include "hvcb.h"
#include "main.h"
#include "mcb.h"
#include "tim.h"
#include "usart.h"
#include "utils.h"
#include "wdg.h"

#include <stdio.h>

/* Initialize the dashboard data structure */
DashboardData_t dashboard_data = {
    .HV_BAT_SOC            = 0,
    .LV_BAT_mV              = 0.0,
    .TIRE_FL_TEMP          = 0.0,
    .TIRE_FR_TEMP          = 0.0,
    .TIRE_RL_TEMP          = 0.0,
    .TIRE_RR_TEMP          = 0.0,
    .TIRE_FL_PRESSURE      = 0.0,
    .TIRE_FR_PRESSURE      = 0.0,
    .TIRE_RL_PRESSURE      = 0.0,
    .TIRE_RR_PRESSURE      = 0.0,
    .RTD_FSM_State         = STATE_IDLE,
    .SD_CLOSED             = GPIO_PIN_RESET,
    .BMS_ERR               = GPIO_PIN_RESET,
    .TS_OFF                = GPIO_PIN_RESET,
    .IMD_ERR               = GPIO_PIN_SET,
    .AMS_ERR               = GPIO_PIN_SET,
    .btn_press_at_start    = 0,
    .hvb_diag_bat_vlt_sna  = 0,
    .hvb_diag_inv_vlt_sna  = 0,
    .hvb_diag_bat_curr_sna = 0,
    .hvb_diag_vcu_can_sna  = 0,
    .hvb_diag_cell_sna     = 0,
    .hvb_diag_bat_uv       = 0,
    .hvb_diag_cell_ov      = 0,
    .hvb_diag_cell_uv      = 0,
    .hvb_diag_cell_ot      = 0,
    .hvb_diag_cell_ut      = 0,
    .hvb_diag_inv_vlt_ov   = 0,
    .hvb_diag_bat_curr_oc  = 0,
    .PWM_RADIATOR_FAN      = 0,
    .DAC_PUMPS_PERCENTAGE  = 0,
    .RTD_BUTTON            = false,
};

/* State change triggers */
typedef enum {
    TRIG_NONE = 0,  // No trigger/triggered by CAN bus
    TRIG_COCK = 1,  // Cockpit button
    TRIG_EXT  = 2   // External button
} state_trig;
state_trig STATE_CHANGE_TRIG = TRIG_NONE;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
uint8_t TxData[8] = {0};
uint8_t RxData[8] = {0};

CAN_RxHeaderTypeDef RxHeader1;
uint8_t RxData1[8] = {0};

enum error_t error = ERROR_NONE;

/*CUSTOM FUNCTIONS*/

//CAN 1 - J4
/*Rx Message interrupt from CAN*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    union {
        struct mcb_dspace_fsm_states_t rtd_ack;
        struct mcb_dspace_peripherals_ctrl_t per_ctrl;
        struct mcb_dspace_signals_t dspace_signals;
        struct mcb_tlb_bat_signals_status_t tsal_status;
        struct mcb_tlb_bat_sd_csensing_status_t shut_status;
        struct mcb_dspace_dash_leds_color_rgb_t rgb_status;
        struct mcb_bms_lv_lv_bat_general_t lv_bat_general;
        struct mcb_tpms_front_wheels_pressure_t front_wheels_status;
        struct mcb_tpms_rear_wheels_pressure_t rear_wheels_status;
    } msgs = {};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
        /* Transmission request Error */
        HAL_CAN_ResetError(hcan);
        Error_Handler();
    }

    // Reset watchdog
    uint32_t now = ReturnTime_100us();
    switch (RxHeader.StdId) {
        case MCB_DSPACE_FSM_STATES_FRAME_ID:
        case MCB_DSPACE_PERIPHERALS_CTRL_FRAME_ID:
        case MCB_DSPACE_SIGNALS_FRAME_ID:
            // Reset dSpace timeout after boot
            wdg_timeouts_100us[WDG_BOARD_DSPACE] = 4800;  //480ms
            wdg_reset(WDG_BOARD_DSPACE, now);
            break;
        case MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME_ID:
        case MCB_TLB_BAT_SIGNALS_STATUS_FRAME_ID:
            wdg_reset(WDG_BOARD_TLB, now);
            break;
        case MCB_BMS_LV_LV_BAT_GENERAL_FRAME_ID:
            wdg_reset(WDG_BOARD_BMS_LV, now);
            break;
    }

    /*Reboot Board - Received command byte from CAN*/
    if ((RxHeader.StdId == MCB_DIAG_TOOL_XCP_TX_DASH_FRAME_ID) && (RxHeader.DLC == 2) && (RxData[0] == 0xFF) &&
        (RxData[1] == 0x00)) {
        NVIC_SystemReset();
    }
    /*
     *
     * dSpace
     *
     */
    else if ((RxHeader.StdId == MCB_DSPACE_FSM_STATES_FRAME_ID) && (RxHeader.DLC == MCB_DSPACE_FSM_STATES_LENGTH)) {
        mcb_dspace_fsm_states_unpack(&msgs.rtd_ack, RxData, MCB_DSPACE_FSM_STATES_LENGTH);
        dashboard_data.Dspace_RTD_State = msgs.rtd_ack.dspace_main_fsm_state;
    } else if ((RxHeader.StdId == MCB_DSPACE_PERIPHERALS_CTRL_FRAME_ID) &&
               (RxHeader.DLC == MCB_DSPACE_PERIPHERALS_CTRL_LENGTH)) {
        mcb_dspace_peripherals_ctrl_unpack(&msgs.per_ctrl, RxData, MCB_DSPACE_PERIPHERALS_CTRL_LENGTH);

        dashboard_data.PWM_RADIATOR_FAN =
            ((msgs.per_ctrl.rad_fan_pwm_duty_cicle_ctrl > 100 ? 100 : msgs.per_ctrl.rad_fan_pwm_duty_cicle_ctrl) /
             100. * __HAL_TIM_GetAutoreload(&RADIATOR_FANS_PWM_TIM));
        __HAL_TIM_SET_COMPARE(&RADIATOR_FANS_PWM_TIM, RADIATOR_FANS_PWM_CH, dashboard_data.PWM_RADIATOR_FAN);
        //PWM_BAT_FAN = ((msgs.per_ctrl.batt_hv_fan_ctrl > 100 ? 100: msgs.per_ctrl.batt_hv_fan_ctrl) / 100. * __HAL_TIM_GetAutoreload(&BAT_FAN_PWM_TIM));
        // __HAL_TIM_SET_COMPARE(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH, PWM_BAT_FAN);
        dashboard_data.DAC_PUMPS_PERCENTAGE =
            ((msgs.per_ctrl.cool_pumps_speed_ctrl > 100 ? 100 : msgs.per_ctrl.cool_pumps_speed_ctrl) / 100.0 * 256U);
        //HAL_DAC_SetValue(&PUMPS_DAC,PUMPS_DAC_CHANNEL,DAC_ALIGN_8B_R,(uint8_t)DAC_PUMPS_PERCENTAGE);
    }

    /*
     *
     * TLB
     *
     */
    else if ((RxHeader.StdId == MCB_TLB_BAT_SIGNALS_STATUS_FRAME_ID) &&
             (RxHeader.DLC == MCB_TLB_BAT_SIGNALS_STATUS_LENGTH)) {
        // TS_OFF when tsal green is enabled
        mcb_tlb_bat_signals_status_unpack(&msgs.tsal_status, RxData, MCB_TLB_BAT_SIGNALS_STATUS_LENGTH);
        dashboard_data.TS_OFF  = msgs.tsal_status.tsal_green_is_active ? GPIO_PIN_SET : GPIO_PIN_RESET;
        dashboard_data.AMS_ERR = msgs.tsal_status.ams_err_is_active;
        dashboard_data.IMD_ERR = msgs.tsal_status.imd_err_is_active ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    // shut status
    else if ((RxHeader.StdId == MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME_ID) &&
             (RxHeader.DLC == MCB_TLB_BAT_SD_CSENSING_STATUS_LENGTH)) {
        mcb_tlb_bat_sd_csensing_status_unpack(&msgs.shut_status, RxData, MCB_TLB_BAT_SD_CSENSING_STATUS_LENGTH);
        dashboard_data.SD_CLOSED = msgs.shut_status.sdc_tsac_final_in_is_active
                                       ? GPIO_PIN_SET
                                       : GPIO_PIN_RESET;  // isShutdownClosed_preTLBBattFinal
    } else if ((RxHeader.StdId == MCB_DSPACE_DASH_LEDS_COLOR_RGB_FRAME_ID) &&
               (RxHeader.DLC == MCB_DSPACE_DASH_LEDS_COLOR_RGB_LENGTH)) {
        mcb_dspace_dash_leds_color_rgb_unpack(&msgs.rgb_status, RxData, MCB_DSPACE_DASH_LEDS_COLOR_RGB_LENGTH);
        dashboard_data.LED1.R = mcb_dspace_dash_leds_color_rgb_led_1_red_decode(msgs.rgb_status.led_1_red);
        dashboard_data.LED1.G = mcb_dspace_dash_leds_color_rgb_led_1_green_decode(msgs.rgb_status.led_1_green);
        dashboard_data.LED1.B = mcb_dspace_dash_leds_color_rgb_led_1_blue_decode(msgs.rgb_status.led_1_blue);

        dashboard_data.LED2.R = mcb_dspace_dash_leds_color_rgb_led_2_red_decode(msgs.rgb_status.led_2_red);
        dashboard_data.LED2.G = mcb_dspace_dash_leds_color_rgb_led_2_green_decode(msgs.rgb_status.led_2_green);
        dashboard_data.LED2.B = mcb_dspace_dash_leds_color_rgb_led_2_blue_decode(msgs.rgb_status.led_2_blue);

        dashboard_data.LED3.R = mcb_dspace_dash_leds_color_rgb_led_3_red_decode(msgs.rgb_status.led_3_red);
        dashboard_data.LED3.G = mcb_dspace_dash_leds_color_rgb_led_3_green_decode(msgs.rgb_status.led_3_green);
        dashboard_data.LED3.B = mcb_dspace_dash_leds_color_rgb_led_3_blue_decode(msgs.rgb_status.led_3_blue);

        dashboard_data.LED4.R = mcb_dspace_dash_leds_color_rgb_led_4_red_decode(msgs.rgb_status.led_4_red);
        dashboard_data.LED4.G = mcb_dspace_dash_leds_color_rgb_led_4_green_decode(msgs.rgb_status.led_4_green);
        dashboard_data.LED4.B = mcb_dspace_dash_leds_color_rgb_led_4_blue_decode(msgs.rgb_status.led_4_blue);
    } else if ((RxHeader.StdId == MCB_DSPACE_SIGNALS_FRAME_ID) && (RxHeader.DLC == MCB_DSPACE_SIGNALS_LENGTH)) {
        mcb_dspace_signals_unpack(&msgs.dspace_signals, RxData, MCB_DSPACE_SIGNALS_LENGTH);
        dashboard_data.HV_BAT_SOC = msgs.dspace_signals.hvbat_soc;
    } else if ((RxHeader.StdId == MCB_BMS_LV_LV_BAT_GENERAL_FRAME_ID) &&
               (RxHeader.DLC == MCB_BMS_LV_LV_BAT_GENERAL_LENGTH)) {
        mcb_bms_lv_lv_bat_general_unpack(&msgs.lv_bat_general, RxData, MCB_BMS_LV_LV_BAT_GENERAL_LENGTH);
        dashboard_data.LV_BAT_mV =
            mcb_bms_lv_lv_bat_general_lv_bat_summed_voltage_decode(msgs.lv_bat_general.lv_bat_summed_voltage);
    }

    /*
     *
     * TMPS Front Wheels And Rear Wheels
     *
     */
    else if ((RxHeader.StdId == MCB_TPMS_FRONT_WHEELS_PRESSURE_FRAME_ID) &&
             (RxHeader.DLC == MCB_TPMS_FRONT_WHEELS_PRESSURE_LENGTH)) {
        mcb_tpms_front_wheels_pressure_unpack(&msgs.front_wheels_status, RxData, MCB_TPMS_FRONT_WHEELS_PRESSURE_LENGTH);
        dashboard_data.TIRE_FL_TEMP =
            mcb_tpms_front_wheels_pressure_tire_fl_temperature_decode(msgs.front_wheels_status.tire_fl_temperature);
        dashboard_data.TIRE_FR_TEMP =
            mcb_tpms_front_wheels_pressure_tire_fr_temperature_decode(msgs.front_wheels_status.tire_fr_temperature);
        dashboard_data.TIRE_FL_PRESSURE =
            mcb_tpms_front_wheels_pressure_tire_fl_pressure_decode(msgs.front_wheels_status.tire_fl_pressure);
        dashboard_data.TIRE_FR_PRESSURE =
            mcb_tpms_front_wheels_pressure_tire_fr_pressure_decode(msgs.front_wheels_status.tire_fr_pressure);
    } else if ((RxHeader.StdId == MCB_TPMS_REAR_WHEELS_PRESSURE_FRAME_ID) &&
               (RxHeader.DLC == MCB_TPMS_REAR_WHEELS_PRESSURE_LENGTH)) {
        mcb_tpms_rear_wheels_pressure_unpack(&msgs.rear_wheels_status, RxData, MCB_TPMS_REAR_WHEELS_PRESSURE_LENGTH);
        dashboard_data.TIRE_RL_TEMP =
            mcb_tpms_rear_wheels_pressure_tire_rl_temperature_decode(msgs.rear_wheels_status.tire_rl_temperature);
        dashboard_data.TIRE_RR_TEMP =
            mcb_tpms_rear_wheels_pressure_tire_rr_temperature_decode(msgs.rear_wheels_status.tire_rr_temperature);
        dashboard_data.TIRE_RL_PRESSURE =
            mcb_tpms_rear_wheels_pressure_tire_rl_pressure_decode(msgs.rear_wheels_status.tire_rl_pressure);
        dashboard_data.TIRE_RR_PRESSURE =
            mcb_tpms_rear_wheels_pressure_tire_rr_pressure_decode(msgs.rear_wheels_status.tire_rr_pressure);
    }
}

//CAN 2 - J5
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    union {
        struct hvcb_hvb_rx_diagnosis_t hvb_rx_diagnosis;
    } msgs = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader1, RxData1) != HAL_OK) {
        /* Transmission request Error */
        HAL_CAN_ResetError(hcan);
        Error_Handler();
    }
    if ((RxHeader1.StdId == HVCB_HVB_RX_DIAGNOSIS_FRAME_ID) && (RxHeader1.DLC == HVCB_HVB_RX_DIAGNOSIS_LENGTH)) {
        hvcb_hvb_rx_diagnosis_unpack(&msgs.hvb_rx_diagnosis, RxData1, HVCB_HVB_RX_DIAGNOSIS_LENGTH);
        dashboard_data.hvb_diag_bat_vlt_sna =
            hvcb_hvb_rx_diagnosis_hvb_diag_bat_vlt_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_bat_vlt_sna);
        dashboard_data.hvb_diag_inv_vlt_sna =
            hvcb_hvb_rx_diagnosis_hvb_diag_inv_vlt_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_inv_vlt_sna);
        dashboard_data.hvb_diag_bat_curr_sna =
            hvcb_hvb_rx_diagnosis_hvb_diag_bat_curr_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_bat_curr_sna);
        dashboard_data.hvb_diag_vcu_can_sna =
            hvcb_hvb_rx_diagnosis_hvb_diag_vcu_can_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_vcu_can_sna);
        dashboard_data.hvb_diag_cell_sna =
            hvcb_hvb_rx_diagnosis_hvb_diag_cell_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_sna);
        dashboard_data.hvb_diag_bat_uv =
            hvcb_hvb_rx_diagnosis_hvb_diag_bat_uv_decode(msgs.hvb_rx_diagnosis.hvb_diag_bat_uv);
        dashboard_data.hvb_diag_cell_ov =
            hvcb_hvb_rx_diagnosis_hvb_diag_cell_ov_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_ov);
        dashboard_data.hvb_diag_cell_uv =
            hvcb_hvb_rx_diagnosis_hvb_diag_cell_uv_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_uv);
        dashboard_data.hvb_diag_cell_ot =
            hvcb_hvb_rx_diagnosis_hvb_diag_cell_ot_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_ot);
        dashboard_data.hvb_diag_cell_ut =
            hvcb_hvb_rx_diagnosis_hvb_diag_cell_ut_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_ut);
        dashboard_data.hvb_diag_inv_vlt_ov =
            hvcb_hvb_rx_diagnosis_hvb_diag_inv_vlt_ov_decode(msgs.hvb_rx_diagnosis.hvb_diag_inv_vlt_ov);
        dashboard_data.hvb_diag_bat_curr_oc =
            hvcb_hvb_rx_diagnosis_hvb_diag_bat_curr_oc_decode(msgs.hvb_rx_diagnosis.hvb_diag_bat_curr_oc);
    }
}

void cock_callback() {
    dashboard_data.RTD_BUTTON = true;
}

char uart_buf[54] = {0};
extern struct PCA9555_Handle pca9555Handle;
/*Update Cockpit's LEDs*/
void UpdateCockpitLed(uint32_t delay_100us) {
    static uint32_t delay_100us_last = 0;

    static uint32_t cnt200ms              = 0;
    static uint8_t toggle_led_value_200ms = 255U;

    if (HAL_GetTick() > cnt200ms) {
        cnt200ms               = (HAL_GetTick() + 200);
        toggle_led_value_200ms = !toggle_led_value_200ms;
    }

    if (delay_fun(&delay_100us_last, delay_100us)) {
        //HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, SD_CLOSED);

        if (dashboard_data.boards_timeouts & (1 << WDG_BOARD_TLB)) {
            // CAN timeout. everything is bad
            HAL_GPIO_WritePin(AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, ON);
            HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, OFF);
            HAL_GPIO_WritePin(IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, ON);

        } else {
            HAL_GPIO_WritePin(
                AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, dashboard_data.AMS_ERR);
            HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, dashboard_data.TS_OFF);
            HAL_GPIO_WritePin(
                IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, dashboard_data.IMD_ERR);
        }

#if 0
        // Control of Dashboard reserved led
        if ((boards_timeouts & (1 << WDG_BOARD_DSPACE)) || (boards_timeouts & (1 << WDG_BOARD_TLB))) {
            // tlb message or dspace message timeout
            LED_RGB_setColor(LED_RGB_DASH, 0U, 0U, 0U);
        } else if (Dspace_RTD_State == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_IDLE_CHOICE) {
            if (SD_CLOSED) {
                // DSPACE fsm in IDLE and SDC closed: BLUE led -> we could can go in RTD
                LED_RGB_setColor(LED_RGB_DASH, 0U, 0U, 255U);
            } else {
                // DSPACE fsm in IDLE: green led but SDC open -> can't co in RTD
                LED_RGB_setColor(LED_RGB_DASH, 0U, 255U, 0U);
            }

        } else if (Dspace_RTD_State == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_PRECHARGE_CHOICE) {
            LED_RGB_setColor(LED_RGB_DASH, 0U, 0U, toggle_led_value_200ms);  //DSPACE precharge: BLUE
        } else if (Dspace_RTD_State == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_TS_ON_CHOICE) {
            // Blink RED when in TSON (5Hz, 50% duty)
            LED_RGB_setColor(LED_RGB_DASH, toggle_led_value_200ms, 0U, 0U);
        } else if ((Dspace_RTD_State == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_RTD_REQUEST_CHOICE) ||
                   (Dspace_RTD_State == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_RTD_CHOICE)) {
            // DSAPCE in RTD: purple led
            LED_RGB_setColor(LED_RGB_DASH, 255U, 0U, 255U);
        } else if (Dspace_RTD_State == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_TS_OFF_CHOICE) {
            // DSAPCE in RTD: purple led
            LED_RGB_setColor(LED_RGB_DASH, 0U, 255U, 255U);
        } else if (Dspace_RTD_State == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_DISCHARGE_CHOICE) {
            // DSAPCE in DISCHARGE: yellow led
            LED_RGB_setColor(LED_RGB_DASH, 255U, 255U, 0U);
        } else {
            // DSAPCE in other states : WHITE state for unkown states
            LED_RGB_setColor(LED_RGB_DASH, 255U, 255U, 255U);
        }

        //Control of other leds
        LED_RGB_setColor(LED_RGB1, LED1.R, LED1.G, LED1.B);
        LED_RGB_setColor(LED_RGB2, LED2.R, LED2.G, LED2.B);
        LED_RGB_setColor(LED_RGB3, LED3.R, LED3.G, LED3.B);
#endif
    }
}

/*Setup TIMER, CAN*/
void Dashboard_Setup(void) {
    HAL_TIM_Base_Start_IT(&COUNTER_TIM);

    // start pwm at 0%
    __HAL_TIM_SET_COMPARE(&RADIATOR_FANS_PWM_TIM, RADIATOR_FANS_PWM_CH, 0);
    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&RADIATOR_FANS_PWM_TIM, RADIATOR_FANS_PWM_CH) != HAL_OK) {
        /* PWM generation Error */
        Error_Handler();
    }

    // start pwm at 0%
    __HAL_TIM_SET_COMPARE(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH, 0);
    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH) != HAL_OK) {
        /* PWM generation Error */
        Error_Handler();
    }
    // button_set_shortpress_callback(BUTTON_RTD, cock_callback);

    if (HAL_DAC_Start(&PUMPS_DAC, PUMPS_DAC_CHANNEL) != HAL_OK) {
        /* DAC Start error*/
        Error_Handler();
    }
    HAL_DAC_SetValue(&PUMPS_DAC, PUMPS_DAC_CHANNEL, DAC_ALIGN_8B_R, 0);

    //Send hello message
    MCB_send_msg(MCB_DASH_HELLO_FRAME_ID);

    LED_MONO_setState(LED_TS_Off, LED_Off);
    LED_MONO_setState(LED_AMS_Error, LED_Off);
    LED_MONO_setState(LED_IMD_Error, LED_Off);

// Disable The SDC relay and wait later for closing it
#if 0
       HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
#endif
    BUZZER_setState(BUZZER, BUZZER_On);
    HAL_Delay(50);
    BUZZER_setState(BUZZER, BUZZER_Off);

#if 0
       HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
#endif

    dashboard_data.btn_press_at_start = BTN_sampleStatus(BTN_GENERAL);
}

void Display_Setup(void) {
    DISP_init();
}
void Display_Loop(void) {
    DISP_update_routine();
}

/*Send status data to CAN BUS*/
void can_send_state(uint32_t delay_100us) {
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us)) {
        MCB_send_msg(MCB_DASH_HMI_DEVICES_STATE_FRAME_ID);
    }
}

void RTD_fsm(uint32_t delay_100us) {
    static uint32_t time;
    static uint32_t delay_100us_last = 0;
    static uint32_t blink_delay_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us)) {
        switch (dashboard_data.RTD_FSM_State) {
            case STATE_IDLE:
                //LED_RGB_setColor(LED_RGB_DASH,0U,0U,255U); // BLUE
                HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, dashboard_data.SD_CLOSED);
#if 0
                HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
#endif
                if (dashboard_data.Dspace_RTD_State == 2)
                    dashboard_data.RTD_FSM_State = STATE_TSON;
                else if (dashboard_data.Dspace_RTD_State == 5 || dashboard_data.Dspace_RTD_State == -1)
                    dashboard_data.RTD_FSM_State = STATE_DISCHARGE;
                break;
            case STATE_TSON:
                //LED_RGB_setColor(LED_RGB_DASH,255U,0U,255U); // PURPLE
                LedBlinking(RTD_LED_GPIO_Port, RTD_LED_Pin, &blink_delay_last, 2000);
                if (dashboard_data.Dspace_RTD_State == 3 || dashboard_data.Dspace_RTD_State == 4) {
                    dashboard_data.RTD_FSM_State = STATE_RTD_SOUND;
                    time                         = HAL_GetTick();
                } else if (dashboard_data.Dspace_RTD_State == 5 || dashboard_data.Dspace_RTD_State == -1)
                    dashboard_data.RTD_FSM_State = STATE_DISCHARGE;
                break;
            case STATE_RTD_SOUND:
                HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
                if (HAL_GetTick() - time > 2000) {
                    HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
                    dashboard_data.RTD_FSM_State = STATE_RTD;
                }
                break;
            case STATE_RTD:
                // LED_RGB_setColor(LED_RGB_DASH,0U,255U,0U); // GREEN
                HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, GPIO_PIN_SET);
                if (dashboard_data.Dspace_RTD_State <= 0)
                    dashboard_data.RTD_FSM_State = STATE_DISCHARGE;
                break;
            case STATE_DISCHARGE:
                //LED_RGB_setColor(LED_RGB_DASH,255U,255U,0U); // YELLOW
                HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, GPIO_PIN_RESET);
#if 0
                HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
#endif
                if (dashboard_data.Dspace_RTD_State == 0)
                    dashboard_data.RTD_FSM_State = STATE_IDLE;
                break;
            default:
                dashboard_data.RTD_FSM_State = STATE_IDLE;
                break;
        }
    }
}

uint8_t AMS_detection(uint8_t AMS_ERR,
                      uint8_t hvb_diag_bat_vlt_sna,
                      uint8_t hvb_diag_inv_vlt_sna,
                      uint8_t hvb_diag_bat_curr_sna,
                      uint8_t hvb_diag_vcu_can_sna,
                      uint8_t hvb_diag_cell_sna,
                      uint8_t hvb_diag_bat_uv,
                      uint8_t hvb_diag_cell_ov,
                      uint8_t hvb_diag_cell_uv,
                      uint8_t hvb_diag_cell_ot,
                      uint8_t hvb_diag_cell_ut,
                      uint8_t hvb_diag_inv_vlt_ov,
                      uint8_t hvb_diag_bat_curr_oc) {
    static uint8_t ams_err_prev = 0;

    if (ams_err_prev && AMS_ERR) {
        return ams_err_prev;
    }

    if (!AMS_ERR) {  // ams_err_prev &
        ams_err_prev = 0;
        return ams_err_prev;
    }

    ams_err_prev = (hvb_diag_bat_vlt_sna || hvb_diag_inv_vlt_sna || hvb_diag_bat_curr_sna || hvb_diag_vcu_can_sna ||
                    hvb_diag_cell_sna || hvb_diag_bat_uv || hvb_diag_cell_ov || hvb_diag_cell_uv || hvb_diag_cell_ot ||
                    hvb_diag_cell_ut || hvb_diag_inv_vlt_ov || hvb_diag_bat_curr_oc) &
                   AMS_ERR;

    return ams_err_prev;
}

#define WAIT_FOR(X)                               \
    do {                                          \
        if (HAL_GetTick() - last_timestamp > X) { \
            counter++;                            \
            last_timestamp = HAL_GetTick();       \
        }                                         \
    } while (0)

/**
    * @brief Dash main loop
 */
void Dashboard_Loop(void) {
    // Blink green led to signal activity
    static uint32_t led_blink = 0;
    static uint32_t cnt10ms   = 0;
    //static uint32_t imd_err_blink = 0;

    LedBlinking(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin, &led_blink, 2000);

    if (dashboard_data.IMD_ERR) {
        // LedBlinking(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, &imd_err_blink, 2500);
    } else if (dashboard_data.RTD_FSM_State != STATE_RTD_SOUND) {
        HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
    }

    // ams_err_check
    if (HAL_GetTick() >= cnt10ms + 10U) {
        dashboard_data.BMS_ERR = AMS_detection(dashboard_data.AMS_ERR,
                                               dashboard_data.hvb_diag_bat_vlt_sna,
                                               dashboard_data.hvb_diag_inv_vlt_sna,
                                               dashboard_data.hvb_diag_bat_curr_sna,
                                               dashboard_data.hvb_diag_vcu_can_sna,
                                               dashboard_data.hvb_diag_cell_sna,
                                               dashboard_data.hvb_diag_bat_uv,
                                               dashboard_data.hvb_diag_cell_ov,
                                               dashboard_data.hvb_diag_cell_uv,
                                               dashboard_data.hvb_diag_cell_ot,
                                               dashboard_data.hvb_diag_cell_ut,
                                               dashboard_data.hvb_diag_inv_vlt_ov,
                                               dashboard_data.hvb_diag_bat_curr_oc);
        cnt10ms += 10U;
    }
    // Update state Cockpit's LEDs
    UpdateCockpitLed(1000);

    // Update buttons state
    button_sample();

    BTN_Routine();

    ROT_SW_Routine();
    

    // RUN the ready to drive FSM
    RTD_fsm(500);

    // Run the AS FSM
    // mission_run();
    // as_run();

    // TODO mentre fai i test dei led...
    uint8_t timeouts = wdg_check();
    if (timeouts != 0) {
        error                          = ERROR_CAN_WDG;
        dashboard_data.boards_timeouts = timeouts;
    } else {
        error                          = 0;
        dashboard_data.boards_timeouts = 0;
    }

    // Send current state via CAN
    can_send_state(500);
}
