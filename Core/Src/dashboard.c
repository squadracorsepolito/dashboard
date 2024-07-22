/*INCLUDE*/

#include "dashboard.h"
#include "button.h"
#include "can.h"
#include "mcb.h"
#include "tim.h"
#include "usart.h"
#include "utils.h"
#include "wdg.h"
#include "main.h"
#include "dac.h"
#include <stdio.h>

/* State change triggers */
typedef enum
{
    TRIG_NONE = 0, // No trigger/triggered by CAN bus
    TRIG_COCK = 1, // Cockpit button
    TRIG_EXT = 2   // External button
} state_trig;
state_trig STATE_CHANGE_TRIG = TRIG_NONE;

CAN_TxHeaderTypeDef CanTxHeader;
CAN_RxHeaderTypeDef CanRxHeader;
uint32_t TxMailbox;
uint8_t TxData[8] = {0};
uint8_t RxData[8] = {0};

/* Error Variables */
error_t error = ERROR_NONE;
volatile uint8_t boards_timeouts;

/* Cock LEDs flags */
volatile GPIO_PinState SD_CLOSED;
volatile GPIO_PinState BMS_ERR;
volatile GPIO_PinState TSOFF;
volatile GPIO_PinState IMD_ERR;

/* dSpace ACK flags */
volatile int8_t dspace_rtd_state;

enum {
    STATE_IDLE,
    STATE_TSON,
    STATE_RTD_SOUND,
    STATE_RTD,
    STATE_DISCHARGE
} rtd_fsm_state = STATE_IDLE;

/* PWM Variables */
volatile uint32_t PWM_BAT_FAN;
volatile uint8_t PWM_ASB_MOTOR;
#if PCBVER == 2
volatile uint32_t PWM_RADIATOR_FAN = 0;
volatile uint32_t DAC_PUMPS_PERCENTAGE = 0;
#elif PCBVER == 1
volatile uint8_t PWM_POWERTRAIN;
#endif
volatile float BRAKE_EXT;

/* Button short press flags */
bool RTD_BUTTON = false;

/*CUSTOM FUNCTIONS*/

/*Rx Message interrupt from CAN*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    union
    {
        struct mcb_d_space_rtd_ack_t rtd_ack;
        struct mcb_d_space_peripherals_ctrl_t per_ctrl;
        struct mcb_sens_front_1_t sens_front_1;
        struct mcb_tlb_battery_tsal_status_t tsal_status;
        struct mcb_tlb_battery_shut_status_t shut_status;
    } msgs;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxHeader, RxData) != HAL_OK)
    {
        /* Transmission request Error */
        HAL_CAN_ResetError(hcan);
        Error_Handler();
    }

    // Reset watchdog
    uint32_t now = ReturnTime_100us();
    switch (CanRxHeader.StdId)
    {
    case MCB_D_SPACE_RTD_ACK_FRAME_ID:
    case MCB_D_SPACE_PERIPHERALS_CTRL_FRAME_ID:
        // Reset dSpace timeout after boot
        wdg_timeouts_100us[WDG_BOARD_DSPACE] = 4800;
        wdg_reset(WDG_BOARD_DSPACE, now);
        break;
    case MCB_TLB_BATTERY_SHUT_STATUS_FRAME_ID:
    case MCB_TLB_BATTERY_TSAL_STATUS_FRAME_ID:
        wdg_reset(WDG_BOARD_TLB, now);
        break;
    case MCB_SENS_FRONT_1_FRAME_ID:
        wdg_reset(WDG_BOARD_SENS_FRONT, now);
        break;
    }

    /*Reboot Board - Received command byte from CAN*/
    if ((CanRxHeader.StdId == 0x9) && (CanRxHeader.DLC == 2) && (RxData[0] == 0xFF) && (RxData[1] == 0x00))
    {
        NVIC_SystemReset();
    }

    /*
     *
     * sensFront
     *
     */
    else if ((CanRxHeader.StdId == MCB_SENS_FRONT_1_FRAME_ID) && (CanRxHeader.DLC == MCB_SENS_FRONT_1_LENGTH))
    {
        mcb_sens_front_1_unpack(&msgs.sens_front_1, RxData, MCB_SENS_FRONT_1_LENGTH);

        BRAKE_EXT = mcb_sens_front_1_brake_straingauge_voltage_m_v_decode(msgs.sens_front_1.brake_straingauge_voltage_m_v);
    }
    /*
     *
     * dSpace
     *
     */
    else if ((CanRxHeader.StdId == MCB_D_SPACE_RTD_ACK_FRAME_ID) && (CanRxHeader.DLC == MCB_D_SPACE_RTD_ACK_LENGTH))
    {
        mcb_d_space_rtd_ack_unpack(&msgs.rtd_ack, RxData, MCB_D_SPACE_RTD_ACK_LENGTH);
        dspace_rtd_state = msgs.rtd_ack.rtd_fsm_state;
    }
    else if ((CanRxHeader.StdId == MCB_D_SPACE_PERIPHERALS_CTRL_FRAME_ID) && (CanRxHeader.DLC == MCB_D_SPACE_PERIPHERALS_CTRL_LENGTH))
    {
        mcb_d_space_peripherals_ctrl_unpack(&msgs.per_ctrl, RxData, MCB_D_SPACE_PERIPHERALS_CTRL_LENGTH);

        PWM_RADIATOR_FAN = ((msgs.per_ctrl.rad_fan_pwm_ctrl > 100 ? 100 : msgs.per_ctrl.rad_fan_pwm_ctrl) / 100. * __HAL_TIM_GetAutoreload(&RADIATOR_FANS_PWM_TIM));
        __HAL_TIM_SET_COMPARE(&RADIATOR_FANS_PWM_TIM, RADIATOR_FANS_PWM_CH, PWM_RADIATOR_FAN);
        PWM_BAT_FAN = ((msgs.per_ctrl.batt_hv_fan_ctrl > 100 ? 100: msgs.per_ctrl.batt_hv_fan_ctrl) / 100. * __HAL_TIM_GetAutoreload(&BAT_FAN_PWM_TIM));
        __HAL_TIM_SET_COMPARE(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH, PWM_BAT_FAN);
        DAC_PUMPS_PERCENTAGE = ((msgs.per_ctrl.batt_hv_fan_ctrl > 100 ? 100: msgs.per_ctrl.batt_hv_fan_ctrl)/100.0 * 256U);
        //HAL_DAC_SetValue(&PUMPS_DAC,PUMPS_DAC_CHANNEL,DAC_ALIGN_8B_R,(uint8_t)DAC_PUMPS_PERCENTAGE);
    }

    /*
     *
     * TLB
     *
     */
    /* Received TLB error byte in order to turn LEDs on or off */
    // else if ((CanRxHeader.StdId == MCB_TLB_BATTERY_TSAL_STATUS_FRAME_ID) && (CanRxHeader.DLC == MCB_TLB_BATTERY_TSAL_STATUS_LENGTH))
    //{
    //     mcb_tlb_battery_tsal_status_unpack(&msgs.tsal_status, RxData, MCB_TLB_BATTERY_TSAL_STATUS_LENGTH);
    //
    //     TSOFF = (bool)msgs.tsal_status.tsal_is_green_on;
    // }
    // else if ((CanRxHeader.StdId == MCB_TLB_BATTERY_SHUT_STATUS_FRAME_ID) && (CanRxHeader.DLC == MCB_TLB_BATTERY_SHUT_STATUS_LENGTH)) {
    //     mcb_tlb_battery_shut_status_unpack(&msgs.shut_status, RxData, MCB_TLB_BATTERY_SHUT_STATUS_LENGTH);
    //     BMS_ERR = (bool)msgs.shut_status.is_ams_error_latched;
    //     IMD_ERR = (bool)msgs.shut_status.is_imd_error_latched;
    //     SD_CLOSED = (bool)msgs.shut_status.is_shutdown_closed_pre_tlb_batt_final;
    // }
    //  tsal status
    else if ((CanRxHeader.StdId == MCB_TLB_BATTERY_TSAL_STATUS_FRAME_ID) && (CanRxHeader.DLC == MCB_TLB_BATTERY_TSAL_STATUS_LENGTH))
    {
        // TSOFF when tsal green is enabled
        mcb_tlb_battery_tsal_status_unpack(&msgs.tsal_status, RxData, MCB_TLB_BATTERY_TSAL_STATUS_LENGTH);
        TSOFF = msgs.tsal_status.tsal_is_green_on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    // shut status
    else if ((CanRxHeader.StdId == MCB_TLB_BATTERY_SHUT_STATUS_FRAME_ID) && (CanRxHeader.DLC == MCB_TLB_BATTERY_SHUT_STATUS_LENGTH))
    {
        mcb_tlb_battery_shut_status_unpack(&msgs.shut_status, RxData, MCB_TLB_BATTERY_SHUT_STATUS_LENGTH);
        SD_CLOSED = msgs.shut_status.is_shutdown_closed_pre_tlb_batt_final ? GPIO_PIN_SET : GPIO_PIN_RESET; // isShutdownClosed_preTLBBattFinal
        BMS_ERR = msgs.shut_status.is_ams_error_latched ? GPIO_PIN_SET : GPIO_PIN_RESET;
        IMD_ERR = msgs.shut_status.is_imd_error_latched ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
}

void InitDashBoard()
{
    // Initialize leds (turn all off)
    HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, GPIO_PIN_SET);

    // Turn on all LEDs
    HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(RTD_LED_CMD_GPIO_Port, RTD_LED_CMD_Pin, GPIO_PIN_SET);

    // Disable The SDC relay and wait later for closing it
    HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);

    HAL_Delay(900);
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
}

void cock_callback()
{
    RTD_BUTTON = true;
}

/*Update Cockpit's LEDs*/
void UpdateCockpitLed(uint32_t delay_100us)
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, SD_CLOSED);

        if (boards_timeouts & (1 << WDG_BOARD_TLB))
        {
            // CAN timeout. everything is bad
            HAL_GPIO_WritePin(AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, !ON);
            HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, OFF);
            HAL_GPIO_WritePin(IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, !ON);
            
        }
        else
        {
            HAL_GPIO_WritePin(AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, AMS_ERR_LED_nCMD_GPIO_OUT_Pin, !BMS_ERR);
            HAL_GPIO_WritePin(TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, TS_OFF_LED_CMD_GPIO_OUT_Pin, TSOFF);
            HAL_GPIO_WritePin(IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, IMD_ERR_LED_nCMD_GPIO_OUT_Pin, !IMD_ERR);
        }
    }
}

/*Setup TIMER, CAN*/
void SetupDashBoard(void)
{
    // start pwm at 0%
    __HAL_TIM_SET_COMPARE(&RADIATOR_FANS_PWM_TIM, RADIATOR_FANS_PWM_CH,0);
    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&RADIATOR_FANS_PWM_TIM, RADIATOR_FANS_PWM_CH) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }


    // start pwm at 0%
    __HAL_TIM_SET_COMPARE(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH, 0);
    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }
    // button_set_shortpress_callback(BUTTON_RTD, cock_callback);

    if(HAL_DAC_Start(&PUMPS_DAC,PUMPS_DAC_CHANNEL) != HAL_OK){

        /* DAC Start error*/
        Error_Handler();

    }
    HAL_DAC_SetValue(&PUMPS_DAC,PUMPS_DAC_CHANNEL,DAC_ALIGN_8B_R,0);

    char msg[54] = {0};
    sprintf(msg, "Dashboard 2022 Boot - build %s @ %s\r\n", __DATE__, __TIME__);
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 20);
}

/*Send status data to CAN BUS*/
void can_send_state(uint32_t delay_100us)
{
    static uint32_t delay_100us_last = 0;

    union
    {
        struct mcb_steering_rtd_t rtd;
        // struct mcb_dash_motor_control_debug_t motor_ctrl_dbg;
    } msgs;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        msgs.rtd.rtd_cmd = button_get(BUTTON_RTD);
        mcb_steering_rtd_pack(TxData, &msgs.rtd, MCB_STEERING_RTD_LENGTH);

        CanTxHeader.StdId = MCB_STEERING_RTD_FRAME_ID;
        CanTxHeader.RTR = CAN_RTR_DATA;
        CanTxHeader.IDE = CAN_ID_STD;
        CanTxHeader.DLC = MCB_STEERING_RTD_LENGTH;

        CAN_Msg_Send(&hcan1, &CanTxHeader, TxData, &TxMailbox, 300);
    }
}

void RTD_fsm(uint32_t delay_100us) {
    static uint32_t time;
    static uint32_t delay_100us_last = 0;
    static uint32_t blink_delay_last = 0;

    if(delay_fun(&delay_100us_last, delay_100us)) {
        switch(rtd_fsm_state) {
            case STATE_IDLE:
                //HAL_GPIO_WritePin(RTD_LED_CMD_GPIO_Port, RTD_LED_CMD_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
                if(dspace_rtd_state == 2)
                    rtd_fsm_state = STATE_TSON;
                else if(dspace_rtd_state == 5 || dspace_rtd_state == -1)
                    rtd_fsm_state = STATE_DISCHARGE;
                break;
            case STATE_TSON:
                //LedBlinking(RTD_LED_CMD_GPIO_Port, RTD_LED_CMD_Pin, &blink_delay_last, 2000);
                if(dspace_rtd_state == 3 || dspace_rtd_state == 4) {
                    rtd_fsm_state = STATE_RTD_SOUND;
                    time = HAL_GetTick();
                } else if(dspace_rtd_state == 5 || dspace_rtd_state == -1)
                    rtd_fsm_state = STATE_DISCHARGE;
                break;
            case STATE_RTD_SOUND:
                //HAL_GPIO_WritePin(RTD_LED_CMD_GPIO_Port, RTD_LED_CMD_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
                if(HAL_GetTick() - time > 1000) {
                    HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);
                    rtd_fsm_state = STATE_RTD;
                }
                break;
            case STATE_RTD:
                //HAL_GPIO_WritePin(RTD_LED_CMD_GPIO_Port, RTD_LED_CMD_Pin, GPIO_PIN_SET);
                if(dspace_rtd_state <= 0)
                    rtd_fsm_state = STATE_DISCHARGE;
                break;
            case STATE_DISCHARGE:
                //HAL_GPIO_WritePin(RTD_LED_CMD_GPIO_Port, RTD_LED_CMD_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(SDC_RLY_CMD_GPIO_OUT_GPIO_Port, SDC_RLY_CMD_GPIO_OUT_Pin, GPIO_PIN_SET);
                if(dspace_rtd_state == 0)
                    rtd_fsm_state = STATE_IDLE;
                break;
            default:
                rtd_fsm_state = STATE_IDLE;
                break;
        }
    }
}

/**
 * @brief Dash main loop
 */
void CoreDashBoard(void)
{
    // Blink green led to signal activity
    static uint32_t led_blink = 0;
    static uint32_t imd_err_blink = 0;

    LedBlinking(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin, &led_blink, 2000);

    if(IMD_ERR)
        LedBlinking(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, &imd_err_blink, 2500);
    else if(rtd_fsm_state != STATE_RTD_SOUND)
        HAL_GPIO_WritePin(BUZZER_CMD_GPIO_OUT_GPIO_Port, BUZZER_CMD_GPIO_OUT_Pin, GPIO_PIN_RESET);

    // Update state Cockpit's LEDs
    UpdateCockpitLed(1000);

    // Update buttons state
    button_sample();

    // RUN the ready to drive FSM
    RTD_fsm(500);

    // Run the AS FSM
    // mission_run();
    // as_run();

    uint8_t timeouts = wdg_check();
    if (timeouts != 0)
    {
        error = ERROR_CAN_WDG;
        boards_timeouts = timeouts;
    } else {
        error = 0;
        boards_timeouts = 0;
    }

    // Send current state via CAN
    can_send_state(500);
}
