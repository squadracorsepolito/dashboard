/*INCLUDES*/

#include "main.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "bsp.h"

/*CUSTOM DEFINE*/

#define ON 1
#define OFF 0

typedef enum
{
    ERROR_NONE = 0,
    ERROR_CAN_WDG,
    ERROR_INIT_BTN
} error_t;

/*CUSTOM FUNCTIONS PROTOTYPES*/

void InitDashBoard();
void SetupDashBoard(void);
void CoreDashBoard(void);
void can_send_state(uint32_t delay_100us);
void UpdateCockpitLed(uint32_t delay_100us);
void ReadyToDriveFSM(uint32_t delay_100us);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

extern void Error_Handler(void);
