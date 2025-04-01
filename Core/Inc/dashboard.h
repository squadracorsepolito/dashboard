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

/*CUSTOM FUNCTIONS PROTOTYPES*/

void Dashboard_Setup(void);
void Dashboard_Loop(void);
void can_send_state(uint32_t delay_100us);
void UpdateCockpitLed(uint32_t delay_100us);
void ReadyToDriveFSM(uint32_t delay_100us);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

extern void Error_Handler(void);
