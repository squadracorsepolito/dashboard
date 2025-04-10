/*INCLUDES*/

#include "bsp.h"
#include "main.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

/*CUSTOM DEFINE*/

#define ON  1
#define OFF 0

// Commenta se non si sta usando lo schermo LCD ILI9488
#define USE_ILI9488

typedef enum { ERROR_NONE = 0, ERROR_CAN_WDG, ERROR_INIT_BTN } error_t;

extern volatile uint8_t HVBAT_SOC;
extern volatile double LVBAT_V;
extern volatile double TIRE_FL_TEMP;
extern volatile double TIRE_FR_TEMP;
extern volatile double TIRE_RL_TEMP;
extern volatile double TIRE_RR_TEMP;
extern volatile double TIRE_FL_PRESSURE;
extern volatile double TIRE_FR_PRESSURE;
extern volatile double TIRE_RL_PRESSURE;
extern volatile double TIRE_RR_PRESSURE;
typedef enum { STATE_IDLE, STATE_TSON, STATE_RTD_SOUND, STATE_RTD, STATE_DISCHARGE } rtd_fsm_state_t;
extern volatile rtd_fsm_state_t rtd_fsm_state;
/*CUSTOM FUNCTIONS PROTOTYPES*/

void Dashboard_Setup(void);
void Dashboard_Loop(void);
void can_send_state(uint32_t delay_100us);
void UpdateCockpitLed(uint32_t delay_100us);
void RTD_fsm(uint32_t delay_100us);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

extern void Error_Handler(void);
