/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp.h"
#include "can.h"
#include "logger.h"
#include "main.h"
#include "mcb.h"
#include "stmlibs_status.h"
#include "usart.h"
#include "STM32_ST7032.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TASK_CYCLIC_1MS_CYCLE_TIME_MS   (1U)
#define TASK_CYCLIC_1MS_PHASE_MS        (0U)
#define TASK_CYCLIC_10MS_CYCLE_TIME_MS  (10U)
#define TASK_CYCLIC_100MS_CYCLE_TIME_MS (100U)
#define TASK_CYCLIC_10MS_PHASE_MS       (2U)
#define TASK_CYCLIC_100MS_PHASE_MS      (50U)

#define LOG__LOGGER_BUF_LEN     4096
#define LOG__LOGGER_UART_HANDLE huart1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile uint8_t SDC_CLOSED =0U;
volatile uint8_t AMS_ERR=0U;
volatile uint8_t IMD_ERR=0U;
volatile uint8_t TS_OFF=0U;
volatile uint8_t SOC_VAL=99U;
volatile float INV_TEMP_VAL=35.0f;
volatile float TSAC_TEMP_VAL=25.2f;

volatile uint8_t LOG_InitOk = 0;

char LOG__LOGGER_buffer[LOG__LOGGER_BUF_LEN] = {0};
LOGGER_HandleTypeDef LOG__LOGGER_handle      = {0};

#define LOG_write(mode, template, ...) LOGGER_log(&LOG__LOGGER_handle, mode, template __VA_OPT__(, ) __VA_ARGS__)

/* USER CODE END Variables */
/* Definitions for TaskSysEngine */
osThreadId_t TaskSysEngineHandle;
uint32_t TaskSysEngineBuffer[ 128 ];
osStaticThreadDef_t TaskSysEngineControlBlock;
const osThreadAttr_t TaskSysEngine_attributes = {
  .name = "TaskSysEngine",
  .cb_mem = &TaskSysEngineControlBlock,
  .cb_size = sizeof(TaskSysEngineControlBlock),
  .stack_mem = &TaskSysEngineBuffer[0],
  .stack_size = sizeof(TaskSysEngineBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for TaskCyclic1ms */
osThreadId_t TaskCyclic1msHandle;
uint32_t TaskCyclic1msBuffer[ 128 ];
osStaticThreadDef_t TaskCyclic1msControlBlock;
const osThreadAttr_t TaskCyclic1ms_attributes = {
  .name = "TaskCyclic1ms",
  .cb_mem = &TaskCyclic1msControlBlock,
  .cb_size = sizeof(TaskCyclic1msControlBlock),
  .stack_mem = &TaskCyclic1msBuffer[0],
  .stack_size = sizeof(TaskCyclic1msBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskCyclic10ms */
osThreadId_t TaskCyclic10msHandle;
uint32_t TaskCyclic10msBuffer[ 128 ];
osStaticThreadDef_t TaskCyclic10msControlBlock;
const osThreadAttr_t TaskCyclic10ms_attributes = {
  .name = "TaskCyclic10ms",
  .cb_mem = &TaskCyclic10msControlBlock,
  .cb_size = sizeof(TaskCyclic10msControlBlock),
  .stack_mem = &TaskCyclic10msBuffer[0],
  .stack_size = sizeof(TaskCyclic10msBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskCyclic100ms */
osThreadId_t TaskCyclic100msHandle;
uint32_t TaskCyclic100msBuffer[ 128 ];
osStaticThreadDef_t TaskCyclic100msControlBlock;
const osThreadAttr_t TaskCyclic100ms_attributes = {
  .name = "TaskCyclic100ms",
  .cb_mem = &TaskCyclic100msControlBlock,
  .cb_size = sizeof(TaskCyclic100msControlBlock),
  .stack_mem = &TaskCyclic100msBuffer[0],
  .stack_size = sizeof(TaskCyclic100msBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canRxMsgQueue */
osMessageQueueId_t canRxMsgQueueHandle;
uint8_t canRxMsgItem[ 50 * sizeof( struct CAN_MsgItem ) ];
osStaticMessageQDef_t canRxMsgQueueControlBlock;
const osMessageQueueAttr_t canRxMsgQueue_attributes = {
  .name = "canRxMsgQueue",
  .cb_mem = &canRxMsgQueueControlBlock,
  .cb_size = sizeof(canRxMsgQueueControlBlock),
  .mq_mem = &canRxMsgItem,
  .mq_size = sizeof(canRxMsgItem)
};
/* Definitions for canTxMsgQueue */
osMessageQueueId_t canTxMsgQueueHandle;
uint8_t canTxMsgItem[ 10 * sizeof( struct CAN_MsgItem ) ];
osStaticMessageQDef_t canTxMsgQueueControlBlock;
const osMessageQueueAttr_t canTxMsgQueue_attributes = {
  .name = "canTxMsgQueue",
  .cb_mem = &canTxMsgQueueControlBlock,
  .cb_size = sizeof(canTxMsgQueueControlBlock),
  .mq_mem = &canTxMsgItem,
  .mq_size = sizeof(canTxMsgItem)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void CAN_ReadRxMsgs(void);
void LOG_Init(void);
void LOG_Routine(void);
STMLIBS_StatusTypeDef LOG_flush(char *buffer, uint32_t size);
void LED_Routine(void);
void LCD_DisplayUpdateRoutine();

/* USER CODE END FunctionPrototypes */

void StartTaskSysEngine(void *argument);
void StartTaskCyclic1ms(void *argument);
void StartTaskCyclic10ms(void *argument);
void StartTaskCyclic100ms(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxMsgQueue */
  canRxMsgQueueHandle = osMessageQueueNew (50, sizeof(struct CAN_MsgItem), &canRxMsgQueue_attributes);

  /* creation of canTxMsgQueue */
  canTxMsgQueueHandle = osMessageQueueNew (10, sizeof(struct CAN_MsgItem), &canTxMsgQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskSysEngine */
  TaskSysEngineHandle = osThreadNew(StartTaskSysEngine, NULL, &TaskSysEngine_attributes);

  /* creation of TaskCyclic1ms */
  TaskCyclic1msHandle = osThreadNew(StartTaskCyclic1ms, NULL, &TaskCyclic1ms_attributes);

  /* creation of TaskCyclic10ms */
  TaskCyclic10msHandle = osThreadNew(StartTaskCyclic10ms, NULL, &TaskCyclic10ms_attributes);

  /* creation of TaskCyclic100ms */
  TaskCyclic100msHandle = osThreadNew(StartTaskCyclic100ms, NULL, &TaskCyclic100ms_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskSysEngine */
/**
  * @brief  Function implementing the TaskSysEngine thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskSysEngine */
void StartTaskSysEngine(void *argument)
{
  /* USER CODE BEGIN StartTaskSysEngine */
    LOG_Init();
    LED_MONO_toggleState(LED_AMS_Error);
    LED_MONO_toggleState(LED_IMD_Error);
    LED_MONO_toggleState(LED_TS_Off);
    osDelay(500);
    LED_MONO_toggleState(LED_AMS_Error);
    LED_MONO_toggleState(LED_IMD_Error);
    LED_MONO_toggleState(LED_TS_Off);
    LOG_InitOk = 1U;
    /* Infinite loop */
    for (;;) {
        //Free running task, executes every possibile tick
        LOG_Routine();
        osDelay(1U);
    }
  /* USER CODE END StartTaskSysEngine */
}

/* USER CODE BEGIN Header_StartTaskCyclic1ms */
/**
* @brief Function implementing the TaskCyclic1ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCyclic1ms */
void StartTaskCyclic1ms(void *argument)
{
  /* USER CODE BEGIN StartTaskCyclic1ms */
    while (LOG_InitOk != 1) {
    }
    /* Infinite loop */
    for (;;) {
        uint32_t currentTime = osKernelGetTickCount();

        CAN_ReadRxMsgs();

#ifdef DEBUG_TASK_STATISTICS
        uint32_t timeEntryIntoWait = osKernelGetTickCount();
#endif
        // You need to make sure that timeEntryIntoWait-currentTime >TASK_CYCLIC_1MS_CYCLE_TIME_MS
        osDelayUntil(currentTime + ((TASK_CYCLIC_1MS_CYCLE_TIME_MS * 1000U) / osKernelGetTickFreq()));
    }
  /* USER CODE END StartTaskCyclic1ms */
}

/* USER CODE BEGIN Header_StartTaskCyclic10ms */
/**
* @brief Function implementing the TaskCyclic10ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCyclic10ms */
void StartTaskCyclic10ms(void *argument)
{
  /* USER CODE BEGIN StartTaskCyclic10ms */
    uint32_t lcd_refresh_cnt = 0U;
    /* Infinite loop */
    while (LOG_InitOk != 1) {
    }
    for (;;) {
        uint32_t currentTime = osKernelGetTickCount();

        LED_Routine();
        // Write Display
        if(osKernelGetTickCount() >= lcd_refresh_cnt){
          lcd_refresh_cnt = osKernelGetTickCount() + 33U;
        }
#ifdef DEBUG_TASK_STATISTICS
        uint32_t timeEntryIntoWait = osKernelGetTickCount();
#endif
        // You need to make sure that timeEntryIntoWait-currentTime >TASK_CYCLIC_10MS_CYCLE_TIME_MS
        osDelayUntil(currentTime + ((TASK_CYCLIC_10MS_CYCLE_TIME_MS * 1000U) / osKernelGetTickFreq()));
    }
  /* USER CODE END StartTaskCyclic10ms */
}

/* USER CODE BEGIN Header_StartTaskCyclic100ms */
/**
* @brief Function implementing the TaskCyclic100ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCyclic100ms */
void StartTaskCyclic100ms(void *argument)
{
  /* USER CODE BEGIN StartTaskCyclic100ms */

    while (LOG_InitOk != 1) {
    }
    /* Infinite loop */
    for (;;) {
        uint32_t currentTime = osKernelGetTickCount();

        LCD_DisplayUpdateRoutine();
        //HAL_IWDG_Refresh(&hiwdg);  // refresh watchdog ~100ms timeout
#ifdef DEBUG_TASK_STATISTICS
        uint32_t timeEntryIntoWait = osKernelGetTickCount();
#endif
        // You need to make sure that timeEntryIntoWait-currentTime >TASK_CYCLIC_100MS_CYCLE_TIME_MS
        osDelayUntil(currentTime + ((TASK_CYCLIC_100MS_CYCLE_TIME_MS * 1000U) / osKernelGetTickFreq()));
    }
  /* USER CODE END StartTaskCyclic100ms */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void CAN_ReadRxMsgs(void) {
    union {
        struct mcb_d_space_rtd_ack_t rtd_ack;
        struct mcb_d_space_peripherals_ctrl_t per_ctrl;
        struct mcb_sens_front_1_t sens_front_1;
        struct mcb_tlb_battery_tsal_status_t tsal_status;
        struct mcb_tlb_battery_shut_status_t shut_status;
    } msgs;

    struct CAN_MsgItem canRxMsgItem = {0};
    if(osMessageQueueGet(canRxMsgQueueHandle, (void *)&canRxMsgItem, 0U, 0U) == osOK ){
        if ((canRxMsgItem.xHeader.rx.StdId == MCB_TLB_BATTERY_SHUT_STATUS_FRAME_ID) &&
            (canRxMsgItem.xHeader.rx.DLC == MCB_TLB_BATTERY_SHUT_STATUS_LENGTH)) {
            mcb_tlb_battery_shut_status_unpack(
                &msgs.shut_status, canRxMsgItem.payload, MCB_TLB_BATTERY_SHUT_STATUS_LENGTH);
            SDC_CLOSED = msgs.shut_status.is_shutdown_closed_pre_tlb_batt_final ? 1U : 0U;
            AMS_ERR    = msgs.shut_status.is_ams_error_latched ? 1U : 0U;
            IMD_ERR    = msgs.shut_status.is_imd_error_latched ? 1U : 0U;
        } else if ((canRxMsgItem.xHeader.rx.StdId == MCB_TLB_BATTERY_TSAL_STATUS_FRAME_ID) &&
                   (canRxMsgItem.xHeader.rx.DLC == MCB_TLB_BATTERY_TSAL_STATUS_LENGTH)) {
            // TSOFF when tsal green is enabled
            mcb_tlb_battery_tsal_status_unpack(
                &msgs.tsal_status, canRxMsgItem.payload, MCB_TLB_BATTERY_TSAL_STATUS_LENGTH);
            TS_OFF = msgs.tsal_status.tsal_is_green_on ? 1U : 0U;
        } // TODO: remove below line just for demo
        else if(canRxMsgItem.xHeader.rx.StdId == MCB_D_SPACE_RTD_ACK_FRAME_ID) {
            SOC_VAL = (canRxMsgItem.payload[0]%101);
            TSAC_TEMP_VAL = (float)(canRxMsgItem.payload[1]%101);
            INV_TEMP_VAL = (float)(canRxMsgItem.payload[2]%101);
        }
    }

}

void LOG_Init(void) {
    LOGGER_init(&LOG__LOGGER_handle, LOG__LOGGER_buffer, LOG__LOGGER_BUF_LEN, LOG_flush);
}

void LOG_Routine(void) {
    LOGGER_flush(&LOG__LOGGER_handle);
}

STMLIBS_StatusTypeDef LOG_flush(char *buffer, uint32_t size) {
    if (LOG__LOGGER_UART_HANDLE.gState != HAL_UART_STATE_READY)
        return STMLIBS_BUSY;

    HAL_UART_Transmit_IT(&LOG__LOGGER_UART_HANDLE, (uint8_t *)buffer, size);
    return STMLIBS_OK;
}

void LED_Routine(void) {
    static uint32_t keep_alive_cnt = 0U;
    // TODO: remove used just for testing on nucleo board
    #define LED_On 0
    #define LED_Off 1
    if (AMS_ERR) {
        LED_MONO_setState(LED_AMS_Error, LED_On);
    } else {
        LED_MONO_setState(LED_AMS_Error, LED_Off);
    }
    if (IMD_ERR) {
        LED_MONO_setState(LED_IMD_Error, LED_On);
    } else {
        LED_MONO_setState(LED_IMD_Error, LED_Off);
    }
    #undef LED_On
    #undef LED_Off
    if (TS_OFF) {
        LED_MONO_setState(LED_TS_Off, LED_On);
    } else {
        LED_MONO_setState(LED_TS_Off, LED_Off);
    }
    if (SDC_CLOSED) {
        LED_MONO_setState(LED_Warn, LED_On);
    } else {
        LED_MONO_setState(LED_Warn, LED_Off);
    }
    if( osKernelGetTickCount() >= keep_alive_cnt){
        keep_alive_cnt = osKernelGetTickCount() + 500U;
        LED_MONO_toggleState(LED_KeepAlive);
    }
}
void LCD_DisplayUpdateRoutine(void){
  char buffer[20]={};
  //LCD_clear();
  LCD_home();
  sprintf(buffer,"      SOC %3u%%",SOC_VAL);
  LCD_write(buffer);
  LCD_setCursor(1,0);
  sprintf(buffer,"INV %2u",(uint8_t)INV_TEMP_VAL);
  LCD_write(buffer);
  LCD_write_byte(0b11011111);
  LCD_shift(ST7032_CR,5);
  sprintf(buffer,"TSAC %2u",(uint8_t)TSAC_TEMP_VAL);
  LCD_write(buffer);
  LCD_write_byte(0b11011111);
  LCD_home();
}
/* USER CODE END Application */

