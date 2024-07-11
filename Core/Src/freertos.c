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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

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
  .priority = (osPriority_t) osPriorityHigh5,
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
  .priority = (osPriority_t) osPriorityHigh,
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
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for InitTask */
osThreadId_t InitTaskHandle;
uint32_t InitTaskBuffer[ 128 ];
osStaticThreadDef_t InitTaskControlBlock;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .cb_mem = &InitTaskControlBlock,
  .cb_size = sizeof(InitTaskControlBlock),
  .stack_mem = &InitTaskBuffer[0],
  .stack_size = sizeof(InitTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
uint8_t canRxQueueBuffer[ 50 * sizeof( struct CAN_BufferElement ) ];
osStaticMessageQDef_t canRxQueueControlBlock;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue",
  .cb_mem = &canRxQueueControlBlock,
  .cb_size = sizeof(canRxQueueControlBlock),
  .mq_mem = &canRxQueueBuffer,
  .mq_size = sizeof(canRxQueueBuffer)
};
/* Definitions for canTxQueue */
osMessageQueueId_t canTxQueueHandle;
uint8_t canTxQueueBuffer[ 10 * sizeof( struct CAN_BufferElement ) ];
osStaticMessageQDef_t canTxQueueControlBlock;
const osMessageQueueAttr_t canTxQueue_attributes = {
  .name = "canTxQueue",
  .cb_mem = &canTxQueueControlBlock,
  .cb_size = sizeof(canTxQueueControlBlock),
  .mq_mem = &canTxQueueBuffer,
  .mq_size = sizeof(canTxQueueBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTaskSysEngine(void *argument);
void StartTaskCyclic1ms(void *argument);
void StartTaskCyclic10ms(void *argument);
void StartTaskCyclic100ms(void *argument);
void StartInitTask(void *argument);

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
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (50, sizeof(struct CAN_BufferElement), &canRxQueue_attributes);

  /* creation of canTxQueue */
  canTxQueueHandle = osMessageQueueNew (10, sizeof(struct CAN_BufferElement), &canTxQueue_attributes);

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

  /* creation of InitTask */
  InitTaskHandle = osThreadNew(StartInitTask, NULL, &InitTask_attributes);

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
    /* Infinite loop */
    for (;;) {
        osDelay(1);
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
    /* Infinite loop */
    for (;;) {
        osDelay(1);
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
    /* Infinite loop */
    for (;;) {
        osDelay(1);
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
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END StartTaskCyclic100ms */
}

/* USER CODE BEGIN Header_StartInitTask */
/**
* @brief Function implementing the InitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInitTask */
void StartInitTask(void *argument)
{
  /* USER CODE BEGIN StartInitTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartInitTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

