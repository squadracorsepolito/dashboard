/*INCLUDE*/

#include "dashboard.h"

#include "button.h"
#include "can.h"
#include "dac.h"
#include "hvcb.h"
#include "main.h"
#include "mcb.h"
#include "pca9555.h"
#include "tim.h"
#include "usart.h"
#include "utils.h"
#include "wdg.h"
#include "bsp.h"
#include "STM32_ST7032.h"

#include <stdio.h>

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

/* Error Variables */
error_t error = ERROR_NONE;
volatile uint8_t boards_timeouts;

/* Cock LEDs flags */
struct RGB_Led_t {
    uint8_t R;
    uint8_t G;
    uint8_t B;
};

volatile GPIO_PinState SD_CLOSED;
volatile GPIO_PinState BMS_ERR;
volatile GPIO_PinState TSOFF;
volatile GPIO_PinState IMD_ERR;

volatile uint8_t HVBAT_SOC = 0;
volatile double LVBAT_V = 0.0;

volatile uint8_t ams_err_tlb;

volatile uint8_t hvb_diag_bat_vlt_sna;
volatile uint8_t hvb_diag_inv_vlt_sna;
volatile uint8_t hvb_diag_bat_curr_sna;
volatile uint8_t hvb_diag_vcu_can_sna;
volatile uint8_t hvb_diag_cell_sna;
volatile uint8_t hvb_diag_bat_uv;
volatile uint8_t hvb_diag_cell_ov;
volatile uint8_t hvb_diag_cell_uv;
volatile uint8_t hvb_diag_cell_ot;
volatile uint8_t hvb_diag_cell_ut;
volatile uint8_t hvb_diag_inv_vlt_ov;
volatile uint8_t hvb_diag_bat_curr_oc;

volatile struct RGB_Led_t LED1;
volatile struct RGB_Led_t LED2;
volatile struct RGB_Led_t LED3;
volatile struct RGB_Led_t LED4;


ST7032_InitTypeDef LCD_DisplayHandle = {0};

/* dSpace ACK flags */
volatile int8_t dspace_rtd_state;

enum { STATE_IDLE, STATE_TSON, STATE_RTD_SOUND, STATE_RTD, STATE_DISCHARGE } rtd_fsm_state = STATE_IDLE;

/* PWM Variables */
volatile uint32_t PWM_BAT_FAN;
volatile uint8_t PWM_ASB_MOTOR;
#if PCBVER == 2
volatile uint32_t PWM_RADIATOR_FAN     = 0;
volatile uint32_t DAC_PUMPS_PERCENTAGE = 0;
#elif PCBVER == 1
volatile uint8_t PWM_POWERTRAIN;
#endif

/* Button short press flags */
bool RTD_BUTTON = false;

/*CUSTOM FUNCTIONS*/

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
        dspace_rtd_state = msgs.rtd_ack.dspace_main_fsm_state;
    } else if ((RxHeader.StdId == MCB_DSPACE_PERIPHERALS_CTRL_FRAME_ID) &&
               (RxHeader.DLC == MCB_DSPACE_PERIPHERALS_CTRL_LENGTH)) {
        mcb_dspace_peripherals_ctrl_unpack(&msgs.per_ctrl, RxData, MCB_DSPACE_PERIPHERALS_CTRL_LENGTH);

        PWM_RADIATOR_FAN =
            ((msgs.per_ctrl.rad_fan_pwm_duty_cicle_ctrl > 100 ? 100 : msgs.per_ctrl.rad_fan_pwm_duty_cicle_ctrl) /
             100. * __HAL_TIM_GetAutoreload(&RADIATOR_FANS_PWM_TIM));
        __HAL_TIM_SET_COMPARE(&RADIATOR_FANS_PWM_TIM, RADIATOR_FANS_PWM_CH, PWM_RADIATOR_FAN);
        //PWM_BAT_FAN = ((msgs.per_ctrl.batt_hv_fan_ctrl > 100 ? 100: msgs.per_ctrl.batt_hv_fan_ctrl) / 100. * __HAL_TIM_GetAutoreload(&BAT_FAN_PWM_TIM));
        // __HAL_TIM_SET_COMPARE(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH, PWM_BAT_FAN);
        DAC_PUMPS_PERCENTAGE =
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
        // TSOFF when tsal green is enabled
        mcb_tlb_bat_signals_status_unpack(&msgs.tsal_status, RxData, MCB_TLB_BAT_SIGNALS_STATUS_LENGTH);
        TSOFF       = msgs.tsal_status.tsal_green_is_active ? GPIO_PIN_SET : GPIO_PIN_RESET;
        ams_err_tlb = msgs.tsal_status.ams_err_is_active;
        IMD_ERR     = msgs.tsal_status.imd_err_is_active ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    // shut status
    else if ((RxHeader.StdId == MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME_ID) &&
             (RxHeader.DLC == MCB_TLB_BAT_SD_CSENSING_STATUS_LENGTH)) {
        mcb_tlb_bat_sd_csensing_status_unpack(&msgs.shut_status, RxData, MCB_TLB_BAT_SD_CSENSING_STATUS_LENGTH);
        SD_CLOSED = msgs.shut_status.sdc_tsac_final_in_is_active ? GPIO_PIN_SET
                                                                 : GPIO_PIN_RESET;  // isShutdownClosed_preTLBBattFinal
    } else if ((RxHeader.StdId == MCB_DSPACE_DASH_LEDS_COLOR_RGB_FRAME_ID) &&
               (RxHeader.DLC == MCB_DSPACE_DASH_LEDS_COLOR_RGB_LENGTH)) {
        mcb_dspace_dash_leds_color_rgb_unpack(&msgs.rgb_status, RxData, MCB_DSPACE_DASH_LEDS_COLOR_RGB_LENGTH);
        LED1.R = mcb_dspace_dash_leds_color_rgb_led_1_red_decode(msgs.rgb_status.led_1_red);
        LED1.G = mcb_dspace_dash_leds_color_rgb_led_1_green_decode(msgs.rgb_status.led_1_green);
        LED1.B = mcb_dspace_dash_leds_color_rgb_led_1_blue_decode(msgs.rgb_status.led_1_blue);

        LED2.R = mcb_dspace_dash_leds_color_rgb_led_2_red_decode(msgs.rgb_status.led_2_red);
        LED2.G = mcb_dspace_dash_leds_color_rgb_led_2_green_decode(msgs.rgb_status.led_2_green);
        LED2.B = mcb_dspace_dash_leds_color_rgb_led_2_blue_decode(msgs.rgb_status.led_2_blue);

        LED3.R = mcb_dspace_dash_leds_color_rgb_led_3_red_decode(msgs.rgb_status.led_3_red);
        LED3.G = mcb_dspace_dash_leds_color_rgb_led_3_green_decode(msgs.rgb_status.led_3_green);
        LED3.B = mcb_dspace_dash_leds_color_rgb_led_3_blue_decode(msgs.rgb_status.led_3_blue);

        LED4.R = mcb_dspace_dash_leds_color_rgb_led_4_red_decode(msgs.rgb_status.led_4_red);
        LED4.G = mcb_dspace_dash_leds_color_rgb_led_4_green_decode(msgs.rgb_status.led_4_green);
        LED4.B = mcb_dspace_dash_leds_color_rgb_led_4_blue_decode(msgs.rgb_status.led_4_blue);
    } else if ((RxHeader.StdId == MCB_DSPACE_SIGNALS_FRAME_ID) &&
               (RxHeader.DLC == MCB_DSPACE_SIGNALS_LENGTH)) {
        mcb_dspace_signals_unpack(&msgs.dspace_signals, RxData, MCB_DSPACE_SIGNALS_LENGTH);
        HVBAT_SOC = msgs.dspace_signals.hvbat_soc;
    } else if ((RxHeader.StdId == MCB_BMS_LV_LV_BAT_GENERAL_FRAME_ID) &&
               (RxHeader.DLC == MCB_BMS_LV_LV_BAT_GENERAL_LENGTH)) {
        mcb_bms_lv_lv_bat_general_unpack(&msgs.lv_bat_general, RxData, MCB_BMS_LV_LV_BAT_GENERAL_LENGTH);
        LVBAT_V = mcb_bms_lv_lv_bat_general_lv_bat_summed_voltage_decode(msgs.lv_bat_general.lv_bat_summed_voltage);
    }
}

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
        hvb_diag_bat_vlt_sna =
            hvcb_hvb_rx_diagnosis_hvb_diag_bat_vlt_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_bat_vlt_sna);
        hvb_diag_inv_vlt_sna =
            hvcb_hvb_rx_diagnosis_hvb_diag_inv_vlt_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_inv_vlt_sna);
        hvb_diag_bat_curr_sna =
            hvcb_hvb_rx_diagnosis_hvb_diag_bat_curr_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_bat_curr_sna);
        hvb_diag_vcu_can_sna =
            hvcb_hvb_rx_diagnosis_hvb_diag_vcu_can_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_vcu_can_sna);
        hvb_diag_cell_sna = hvcb_hvb_rx_diagnosis_hvb_diag_cell_sna_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_sna);
        hvb_diag_bat_uv   = hvcb_hvb_rx_diagnosis_hvb_diag_bat_uv_decode(msgs.hvb_rx_diagnosis.hvb_diag_bat_uv);
        hvb_diag_cell_ov  = hvcb_hvb_rx_diagnosis_hvb_diag_cell_ov_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_ov);
        hvb_diag_cell_uv  = hvcb_hvb_rx_diagnosis_hvb_diag_cell_uv_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_uv);
        hvb_diag_cell_ot  = hvcb_hvb_rx_diagnosis_hvb_diag_cell_ot_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_ot);
        hvb_diag_cell_ut  = hvcb_hvb_rx_diagnosis_hvb_diag_cell_ut_decode(msgs.hvb_rx_diagnosis.hvb_diag_cell_ut);
        hvb_diag_inv_vlt_ov =
            hvcb_hvb_rx_diagnosis_hvb_diag_inv_vlt_ov_decode(msgs.hvb_rx_diagnosis.hvb_diag_inv_vlt_ov);
        hvb_diag_bat_curr_oc =
            hvcb_hvb_rx_diagnosis_hvb_diag_bat_curr_oc_decode(msgs.hvb_rx_diagnosis.hvb_diag_bat_curr_oc);
    }
}

void InitDashBoard() {
    //Send hello message
    MCB_send_msg(MCB_DASH_HELLO_FRAME_ID);

    // Initialize leds (turn all off)
    HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, GPIO_PIN_SET);

    // Turn on all LEDs
    HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(1500);

    // Disable The SDC relay and wait later for closing it
    HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);

    //HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);

    // Test inputs
    // if (button_get(BUTTON_RTD))
    // {
    //     error = ERROR_INIT_BTN;
    //     rtd_fsm = STATE_ERROR;
    // }

    char buffer [20] ={};
    sprintf(buffer, "SQUADRA CORSE POLITO");
    LCD_write(buffer);
    HAL_Delay(800);
    LCD_setCursor(1, 0);
    sprintf(buffer, "     ANDROMEDA");
    LCD_write(buffer);
    HAL_Delay(800);
}

void cock_callback() {
    RTD_BUTTON = true;
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

        if (boards_timeouts & (1 << WDG_BOARD_TLB)) {
            // CAN timeout. everything is bad
            HAL_GPIO_WritePin(AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, !ON);
            HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, OFF);
            HAL_GPIO_WritePin(IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, !ON);

        } else {
            HAL_GPIO_WritePin(AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, !BMS_ERR);
            HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, TSOFF);
            HAL_GPIO_WritePin(IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, !IMD_ERR);
        }

        // Control of Dashboard reserved led
        if ((boards_timeouts & (1 << WDG_BOARD_DSPACE)) || (boards_timeouts & (1 << WDG_BOARD_TLB))) {
            // tlb message or dspace message timeout
            LED_RGB_setColor(LED_RGB_DASH, 0U, 0U, 0U);
        } else if (dspace_rtd_state == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_IDLE_CHOICE){
            if (SD_CLOSED) {
                // DSPACE fsm in IDLE and SDC closed: BLUE led -> we could can go in RTD
                LED_RGB_setColor(LED_RGB_DASH, 0U, 0U, 255U);
            } else {
                // DSPACE fsm in IDLE: green led but SDC open -> can't co in RTD
                LED_RGB_setColor(LED_RGB_DASH, 0U, 255U, 0U);
            }

        } else if (dspace_rtd_state == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_PRECHARGE_CHOICE) {
                LED_RGB_setColor(LED_RGB_DASH, 0U, 0U, toggle_led_value_200ms); //DSPACE precharge: BLUE
        } else if (dspace_rtd_state == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_TS_ON_CHOICE) {
            // Blink RED when in TSON (5Hz, 50% duty)
            LED_RGB_setColor(LED_RGB_DASH, toggle_led_value_200ms, 0U, 0U);
        } else if ((dspace_rtd_state == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_RTD_REQUEST_CHOICE) ||
                   (dspace_rtd_state == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_RTD_CHOICE)) {
            // DSAPCE in RTD: purple led
            LED_RGB_setColor(LED_RGB_DASH, 255U, 0U, 255U);
        } else if (dspace_rtd_state == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_TS_OFF_CHOICE) {
            // DSAPCE in RTD: purple led
            LED_RGB_setColor(LED_RGB_DASH, 0U, 255U, 255U);
        } else if (dspace_rtd_state == (int8_t)MCB_DSPACE_FSM_STATES_DSPACE_MAIN_FSM_STATE_DISCHARGE_CHOICE) {
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
    }
}

/*Setup TIMER, CAN*/
void SetupDashBoard(void) {

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

    if (PCA9555_init(&pca9555Handle, &hi2c1, PCA9555_ADDR) != HAL_OK) {
        HAL_GPIO_WritePin(WARN_LED_GPIO_OUT_GPIO_Port, WARN_LED_GPIO_OUT_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin, GPIO_PIN_SET);
    }

    PCA9555_pinMode(&pca9555Handle, 0, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 1, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 2, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 3, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 4, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 5, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 6, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 7, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 8, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 9, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 10, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 11, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 12, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 13, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 14, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    PCA9555_pinMode(&pca9555Handle, 15, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);

    PCA9555_digitalWrite(&pca9555Handle, 0, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 1, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 2, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 3, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 4, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 5, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 6, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 7, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 8, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 9, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 10, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 11, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 12, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 13, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 14, PCA9555_BIT_RESET);
    PCA9555_digitalWrite(&pca9555Handle, 15, PCA9555_BIT_RESET);

    HAL_GPIO_WritePin(NHD_C0220BIZx_nRST_GPIO_OUT_GPIO_Port,NHD_C0220BIZx_nRST_GPIO_OUT_Pin,GPIO_PIN_SET);

    LCD_DisplayHandle.LCD_hi2c              = &hi2c1;
    LCD_DisplayHandle.LCD_htim_backlight    = NULL;
    LCD_DisplayHandle.TIM_channel_backlight = 0;
    LCD_DisplayHandle.i2cAddr               = 0x78;
    LCD_DisplayHandle.num_col               = 20;
    LCD_DisplayHandle.num_lines             = 2;


    LCD_ST7032_Init(&LCD_DisplayHandle);  // Init LCD
    LCD_clear();                          // clear LCD
    LCD_home();                           // Home The display
    LCD_contrast(15);                     // set contrast to level 15 - MAXIMUM (15 level available)

    char msg[54] = {0};
    sprintf(msg, "Dashboard 2022 Boot - build %s @ %s\r\n", __DATE__, __TIME__);
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 20);
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
        switch (rtd_fsm_state) {
            case STATE_IDLE:
                //LED_RGB_setColor(LED_RGB_DASH,0U,0U,255U); // BLUE
                HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, SD_CLOSED);
                HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
                if (dspace_rtd_state == 2)
                    rtd_fsm_state = STATE_TSON;
                else if (dspace_rtd_state == 5 || dspace_rtd_state == -1)
                    rtd_fsm_state = STATE_DISCHARGE;
                break;
            case STATE_TSON:
                //LED_RGB_setColor(LED_RGB_DASH,255U,0U,255U); // PURPLE
                LedBlinking(RTD_LED_GPIO_Port, RTD_LED_Pin, &blink_delay_last, 2000);
                if (dspace_rtd_state == 3 || dspace_rtd_state == 4) {
                    rtd_fsm_state = STATE_RTD_SOUND;
                    time          = HAL_GetTick();
                } else if (dspace_rtd_state == 5 || dspace_rtd_state == -1)
                    rtd_fsm_state = STATE_DISCHARGE;
                break;
            case STATE_RTD_SOUND:
                HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
                if (HAL_GetTick() - time > 2000) {
                    HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
                    rtd_fsm_state = STATE_RTD;
                }
                break;
            case STATE_RTD:
                // LED_RGB_setColor(LED_RGB_DASH,0U,255U,0U); // GREEN
                HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, GPIO_PIN_SET);
                if (dspace_rtd_state <= 0)
                    rtd_fsm_state = STATE_DISCHARGE;
                break;
            case STATE_DISCHARGE:
                //LED_RGB_setColor(LED_RGB_DASH,255U,255U,0U); // YELLOW
                HAL_GPIO_WritePin(RTD_LED_GPIO_Port, RTD_LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
                if (dspace_rtd_state == 0)
                    rtd_fsm_state = STATE_IDLE;
                break;
            default:
                rtd_fsm_state = STATE_IDLE;
                break;
        }
    }
}

uint8_t AMS_detection(uint8_t ams_err_tlb,
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

    if (ams_err_prev && ams_err_tlb) {
        return ams_err_prev;
    }

    if (!ams_err_tlb) {  // ams_err_prev &
        ams_err_prev = 0;
        return ams_err_prev;
    }

    ams_err_prev = (hvb_diag_bat_vlt_sna || hvb_diag_inv_vlt_sna || hvb_diag_bat_curr_sna || hvb_diag_vcu_can_sna ||
                    hvb_diag_cell_sna || hvb_diag_bat_uv || hvb_diag_cell_ov || hvb_diag_cell_uv || hvb_diag_cell_ot ||
                    hvb_diag_cell_ut || hvb_diag_inv_vlt_ov || hvb_diag_bat_curr_oc) &
                   ams_err_tlb;

    return ams_err_prev;
}

void LCD_DisplayUpdateRoutine(void) {
    static uint8_t cnt100ms = 0;
    char hv_bat_soc_str [10] = {};
    char lv_bat_v_str [10] = {};

    if(HAL_GetTick() < cnt100ms)
        return;
    cnt100ms = HAL_GetTick() + 100U;

    char buffer[20] = {};
    //LCD_clear();
    LCD_home();

    if(boards_timeouts & (1<< WDG_BOARD_DSPACE)){
        sprintf(hv_bat_soc_str,"Na");
    }else{
        sprintf(hv_bat_soc_str,"%3u",HVBAT_SOC);
    }

    if(boards_timeouts & (1<< WDG_BOARD_BMS_LV)){
        sprintf(lv_bat_v_str,"Na");
    }else{
        sprintf(lv_bat_v_str,"%04.1f",LVBAT_V/1000);
    }
    sprintf(buffer, " HV %3s%%   LV %4sV", hv_bat_soc_str,lv_bat_v_str);
    LCD_write(buffer);
    LCD_setCursor(1, 0);
    sprintf(buffer, "     ANDROMEDA");
    LCD_write(buffer);
    //sprintf(buffer, "INV %2u", (uint8_t)INV_TEMP_VAL);
    //LCD_write(buffer);
    //LCD_write_byte(0b11011111);
    //LCD_shift(ST7032_CR, 5);
    //sprintf(buffer, "TSAC %2u", (uint8_t)TSAC_TEMP_VAL);
    //LCD_write(buffer);
    //LCD_write_byte(0b11011111);
    //LCD_home();
}

/**
    * @brief Dash main loop
 */
void CoreDashBoard(void) {
    // Blink green led to signal activity
    static uint32_t led_blink = 0;
    static uint32_t cnt10ms   = 0;
    //static uint32_t imd_err_blink = 0;

    LedBlinking(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin, &led_blink, 2000);

    if (IMD_ERR) {
        //LedBlinking(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, &imd_err_blink, 2500);
    } else if (rtd_fsm_state != STATE_RTD_SOUND) {
        HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
    }

    // ams_err_check
    if (HAL_GetTick() >= cnt10ms + 10U) {
        BMS_ERR = AMS_detection(ams_err_tlb,
                                hvb_diag_bat_vlt_sna,
                                hvb_diag_inv_vlt_sna,
                                hvb_diag_bat_curr_sna,
                                hvb_diag_vcu_can_sna,
                                hvb_diag_cell_sna,
                                hvb_diag_bat_uv,
                                hvb_diag_cell_ov,
                                hvb_diag_cell_uv,
                                hvb_diag_cell_ot,
                                hvb_diag_cell_ut,
                                hvb_diag_inv_vlt_ov,
                                hvb_diag_bat_curr_oc);
        cnt10ms += 10U;
    }
    // Update state Cockpit's LEDs
    UpdateCockpitLed(1000);

    // Update buttons state
    button_sample();

    BTN_Routine();

    // RUN the ready to drive FSM
    RTD_fsm(500);

    LCD_DisplayUpdateRoutine();

    // Run the AS FSM
    // mission_run();
    // as_run();

    uint8_t timeouts = wdg_check();
    if (timeouts != 0) {
        error           = ERROR_CAN_WDG;
        boards_timeouts = timeouts;
    } else {
        error           = 0;
        boards_timeouts = 0;
    }

    // Send current state via CAN
    can_send_state(500);
}
