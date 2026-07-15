/*INCLUDES*/

#include "bsp.h"
#include "main.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

/*CUSTOM DEFINE*/
#define ON  1
#define OFF 0

extern enum error_t { ERROR_NONE = 0, ERROR_CAN_WDG, ERROR_INIT_BTN } error_t;
enum RTD_FSM_State_t { STATE_IDLE, STATE_TSON, STATE_RTD_SOUND, STATE_RTD, STATE_DISCHARGE };

struct RGB_Led_t {
    uint8_t R;
    uint8_t G;
    uint8_t B;
};

/* Data structure to group all dashboard-related variables */
typedef struct {
    volatile uint8_t HV_BAT_SOC;                  // High voltage battery state of charge
    
    volatile uint8_t AS_MISSION;                  //Autonomous Mission Code
    volatile uint8_t ASSI_CODE;                   //Autonomous System Status Indicator
    volatile int AS_RELAY;                        //AS Relay command (Inverse Logic!)
    volatile uint8_t ASB_EBS_RELAYS;              //ASB EBS Relays command (known as Stanlio and Ollio)
    volatile uint8_t ROLLING;                     //Rolling counter for EBS WD
    volatile uint8_t PREV_ROLL;                   //Previous counter value to compute the difference
    volatile uint8_t EBS_TEST_STATE;              //If set to 1 the changes in the rolling counter could not occur
    
    volatile uint8_t TV_BTN_STATE;                //Three of the steering buttons state
    volatile uint8_t TC_BTN_STATE;
    volatile uint8_t LC_BTN_STATE;
    
    volatile uint8_t ROT_SW_1_STATE;              //Steering rotary switches state
    volatile uint8_t ROT_SW_2_STATE;
    volatile uint8_t ROT_SW_3_STATE;

    volatile double INVERTER_FL_TEMP;
    volatile double INVERTER_FR_TEMP;
    volatile double INVERTER_RL_TEMP;
    volatile double INVERTER_RR_TEMP;
    volatile double MOTOR_FL_TEMP;
    volatile double MOTOR_FR_TEMP;
    volatile double MOTOR_RL_TEMP;
    volatile double MOTOR_RR_TEMP;
    volatile double LV_BAT_mV;                    // Low voltage batter voltage
    volatile double TIRE_FL_TEMP;                 // Front left tire temperature
    volatile double TIRE_FR_TEMP;                 // Front right tire temperature
    volatile double TIRE_RL_TEMP;                 // Rear left tire temperature
    volatile double TIRE_RR_TEMP;                 // Rear right tire temperature
    volatile double TIRE_FL_PRESSURE;             // Front left tire pressure
    volatile double TIRE_FR_PRESSURE;             // Front right tire pressure
    volatile double TIRE_RL_PRESSURE;             // Rear left tire pressure
    volatile double TIRE_RR_PRESSURE;             // Rear right tire pressure
    volatile double COOL_PRESS_LEFT_mV;           // Cooling pressure left
    volatile double COOL_PRESS_RIGHT_mV;          // Cooling pressure right
    volatile enum RTD_FSM_State_t RTD_FSM_State;  // RTD finite state machine state
    volatile int8_t Dspace_RTD_State;             // dSPACE RTD state
    volatile GPIO_PinState SD_CLOSED;             // Shutdown circuit closed state
    volatile GPIO_PinState BMS_ERR;               // Battery Management System error state
    volatile GPIO_PinState TS_OFF;                // TS (Tractive System) off state
    volatile GPIO_PinState IMD_ERR;               // Insulation Monitoring Device error state
    volatile uint8_t ams_err_tlb;                 // AMS (Accumulator Management System) error table
    volatile uint8_t btn_press_at_start;          // Button press at start
    volatile uint8_t hvb_diag_bat_vlt_sna;        // HV battery diagnostic: battery voltage signal not available
    volatile uint8_t hvb_diag_inv_vlt_sna;        // HV battery diagnostic: inverter voltage signal not available
    volatile uint8_t hvb_diag_bat_curr_sna;       // HV battery diagnostic: battery current signal not available
    volatile uint8_t hvb_diag_vcu_can_sna;        // HV battery diagnostic: VCU CAN signal not available
    volatile uint8_t hvb_diag_cell_sna;           // HV battery diagnostic: cell signal not available
    volatile uint8_t hvb_diag_bat_uv;             // HV battery diagnostic: battery undervoltage
    volatile uint8_t hvb_diag_cell_ov;            // HV battery diagnostic: cell overvoltage
    volatile uint8_t hvb_diag_cell_uv;            // HV battery diagnostic: cell undervoltage
    volatile uint8_t hvb_diag_cell_ot;            // HV battery diagnostic: cell overtemperature
    volatile uint8_t hvb_diag_cell_ut;            // HV battery diagnostic: cell undertemperature
    volatile uint8_t hvb_diag_inv_vlt_ov;         // HV battery diagnostic: inverter voltage overvoltage
    volatile uint8_t hvb_diag_bat_curr_oc;        // HV battery diagnostic: battery current overcurrent
    volatile struct RGB_Led_t LED1;               // RGB LED 1 state
    volatile struct RGB_Led_t LED2;               // RGB LED 2 state
    volatile struct RGB_Led_t LED3;               // RGB LED 3 state
    volatile struct RGB_Led_t LED4;               // RGB LED 4 state
    volatile uint8_t boards_timeouts;
    /* PWM Variables */
    volatile uint32_t PWM_BAT_FAN;
    volatile uint8_t PWM_ASB_MOTOR;
#if PCBVER == 2
    volatile uint32_t PWM_RADIATOR_FAN;
    volatile uint32_t DAC_PUMPS_PERCENTAGE;
#elif PCBVER == 1
    volatile uint8_t PWM_POWERTRAIN;
#endif
    bool RTD_BUTTON;
} DashboardData_t;

/* Declare the dashboard data structure */
extern volatile DashboardData_t dashboard_data;

/*CUSTOM FUNCTIONS PROTOTYPES*/

void Dashboard_Setup(void);
void Display_Setup(void);
void Dashboard_Loop(uint32_t *EM_time, int *flag, uint32_t *WD_time);
void Display_Loop(void);
void can_send_state(uint32_t delay_100us);
void UpdateCockpitLed(uint32_t delay_100us);
void RTD_fsm(uint32_t delay_100us);
void AS_SDC_check(void);
void ASSI_state(uint32_t delay_100us, uint32_t *time, int *flag);
void ASB_EBS_state_check(uint32_t *WD_time);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

extern void Error_Handler(void);
