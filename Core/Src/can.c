/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "bsp.h"
#include "cmsis_os.h"
#include "mcb.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterIdHigh         = (MCB_D_SPACE_RTD_ACK_FRAME_ID << 5);
    sFilterConfig.FilterIdLow          = (MCB_D_SPACE_RTD_ACK_FRAME_ID << 5);
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank           = 1;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterIdHigh         = (MCB_D_SPACE_PERIPHERALS_CTRL_FRAME_ID << 5);
    sFilterConfig.FilterIdLow          = (MCB_D_SPACE_PERIPHERALS_CTRL_FRAME_ID << 5);
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank           = 2;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterIdHigh         = (0x9 << 5);
    sFilterConfig.FilterIdLow          = (0x9 << 5);
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank           = 3;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterIdHigh         = (MCB_TLB_BATTERY_SHUT_STATUS_FRAME_ID << 5);
    sFilterConfig.FilterIdLow          = (MCB_TLB_BATTERY_SHUT_STATUS_FRAME_ID << 5);
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank           = 4;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterIdHigh         = (MCB_TLB_BATTERY_TSAL_STATUS_FRAME_ID << 5);
    sFilterConfig.FilterIdLow          = (MCB_TLB_BATTERY_TSAL_STATUS_FRAME_ID << 5);
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    sFilterConfig.FilterBank           = 5;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterIdHigh         = (MCB_SENS_FRONT_1_FRAME_ID << 5);
    sFilterConfig.FilterIdLow          = (MCB_SENS_FRONT_1_FRAME_ID << 5);
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1,
                                     CAN_IT_BUSOFF | CAN_IT_ERROR | CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR_WARNING |
                                         CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        /* Start Error */
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        /* Start Error */
        Error_Handler();
    }

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        /* Start Error */
        Error_Handler();
    }

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = SN65HVD23x_R_CAN1_RX_Pin|SN65HVD23x_D_CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = SN65HVD23x_R_CAN2_RX_Pin|SN65HVD23x_D_CAN2_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, SN65HVD23x_R_CAN1_RX_Pin|SN65HVD23x_D_CAN1_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, SN65HVD23x_R_CAN2_RX_Pin|SN65HVD23x_D_CAN2_TX_Pin);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

extern osMessageQueueId_t canTxMsgQueueHandle;
extern osMessageQueueId_t canRxMsgQueueHandle;

static HAL_StatusTypeDef CAN_TxMsg_wait(CAN_HandleTypeDef *hcan, uint8_t timeout) {
    uint32_t tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        if (HAL_GetTick() - tick > timeout)
            return HAL_TIMEOUT;
    }
    return HAL_OK;
}

HAL_StatusTypeDef CAN_TxMsg(CAN_HandleTypeDef *hcan, uint8_t *buffer, CAN_TxHeaderTypeDef *header) {
    if (CAN_TxMsg_wait(hcan, 1) != HAL_OK)
        return HAL_TIMEOUT;
    uint32_t mailbox;

    volatile HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, header, buffer, &mailbox);

    return status;
}

static void CAN_RxMsg_fromFifo_ISR(CAN_HandleTypeDef *hcan, uint8_t fifo_number) {
    assert_param(hcan != NULL_PTR);
    struct CAN_MsgItem rxMsg = {.xType = RxHeader, .xHeader = {}, .hcan = hcan, .payload = {}};

    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(rxMsg.hcan, fifo_number, &(rxMsg.xHeader.rx), rxMsg.payload);
    if (status != HAL_OK) {
        //TODO: deal with error
    }

    osStatus_t os_status = osMessageQueuePut(
        canRxMsgQueueHandle, (const void *)&rxMsg, 0, 0U);  // We are in an interrupt Timeout is not important
    if(os_status != osOK){
        //TODO: deal with error, for now ErrorHandler
        Error_Handler();
    }
}

/* ***************************************
 *  Interrupt handling
 ****************************************/

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
#if 0
    CAN_Error_s* errorStruct = NULL_PTR;

    if (hcan->Instance == CAN1) {
        errorStruct = &CAN_CAN1_errorStruct;
    } else {
        errorStruct = &CAN_CAN0_errorStruct;
    }

    uint32_t error = HAL_CAN_GetError(hcan);

        /* Check Error Warning Flag */
        if ((error & HAL_CAN_ERROR_EWG) != 0) {
            /* This bit is set by hardware when the warning limit has been
             * reached (Receive Error Counter or Transmit Error Counter>=96
             * until error counter 127 write error frames dominant on can bus
             * increment error occurrence of error warning state */
            errorStruct->canErrorCounter[0]++;
        }

        /* Check Error Passive Flag */
        if ((error & HAL_CAN_ERROR_EPV) != 0) {
            /* This bit is set by hardware when the Error Passive limit has
             * been reached (Receive Error Counter or Transmit Error Counter
             * > 127) write error frames recessive on can bus increment error
             * occurrence of error passive state */
            errorStruct->canErrorCounter[1]++;
        }
        /* Check Bus-Off Flag */
        if ((error & HAL_CAN_ERROR_BOF) != 0) {
            /* This bit is set by hardware when it enters the bus-off state.
             * The bus-off state is entered on TEC overflow, greater than 255
             * increment error occurrence of bus-off state */
            errorStruct->canErrorCounter[2]++;
        }
        /* Check stuff error flag */
        if ((error & HAL_CAN_ERROR_STF) != 0) {
            /* When five consecutive bits of the same level have been
             * transmitted by a node, it will add a sixth bit of the opposite
             * level to the outgoing bit stream. The receivers will remove this
             * extra bit.This is done to avoid excessive DC components on the
             * bus, but it also gives the receivers an extra opportunity to
             * detect errors: if more than five consecutive bits of the same
             * level occurs on the bus, a Stuff Error is signaled. */
            errorStruct->canErrorCounter[3]++;
        }
        if ((error & HAL_CAN_ERROR_FOR) != 0) {
            /* FORM ERROR --- Some parts of the CAN message have a fixed format,
             * i.e. the standard defines exactly what levels must occur and
             * when. (Those parts are the CRC Delimiter, ACK Delimiter, End of
             * Frame, and also the Intermission, but there are some extra
             * special error checking rules for that.) If a CAN controller
             * detects an invalid value in one of these fixed fields, a Form
             * Error is signaled. */
            errorStruct->canErrorCounter[4]++;
        }
        if ((error & HAL_CAN_ERROR_ACK) != 0) {
            /* ACKNOWLEDGMENT ERROR --- All nodes on the bus that correctly
             * receives a message (regardless of their being interested of its
             * contents or not) are expected to send a dominant level in the
             * so-called Acknowledgement Slot in the message. The transmitter
             * will transmit a recessive level here. If the transmitter can
             * detect a dominant level in the ACK slot, an Acknowledgement
             * Error is signaled. */
            errorStruct->canErrorCounter[5]++;
        }
        if ((error & HAL_CAN_ERROR_BR) != 0) {
            /* BIT RECESSIVE ERROR --- Each transmitter on the CAN bus monitors
             * (i.e. reads back) the transmitted signal level. If the bit level
             * actually read differs from the one transmitted, a Bit Error (No
             * bit error is raised during the arbitration process.) */
            errorStruct->canErrorCounter[6]++;
        }
        if ((error & HAL_CAN_ERROR_BD) != 0) {
            /* BIT DOMINANT ERROR --- Each transmitter on the CAN bus monitors
             * (i.e. reads back) the transmitted signal level. If the bit level
             * actually read differs from the one transmitted, a Bit Error (No
             *  bit error is raised during the arbitration process.) */
            errorStruct->canErrorCounter[7]++;
        }
        if ((error & HAL_CAN_ERROR_CRC) != 0) {
            /* CRC ERROR --- Each message features a 15-bit Cyclic Redundancy
             * Checksum (CRC), and any node that detects a different CRC in the
             * message than what it has calculated itself will signal an CRC
             * Error. */
            errorStruct->canErrorCounter[8]++;
        }
        if ((error & HAL_CAN_ERROR_RX_FOV0) != 0) {
            /* Rx FIFO0 overrun error */
            errorStruct->canErrorCounter[9]++;
        }
        if ((error & HAL_CAN_ERROR_RX_FOV1) != 0) {
            /* Rx FIFO1 overrun error */
            errorStruct->canErrorCounter[10]++;
        }
        if ((error & HAL_CAN_ERROR_TX_ALST0) != 0) {
            /* TxMailbox 0 transmit failure due to arbitration lost */
            errorStruct->canErrorCounter[11]++;
        }
        if ((error & HAL_CAN_ERROR_TX_TERR0) != 0) {
            /* TxMailbox 1 transmit failure due to transmit error */
            errorStruct->canErrorCounter[12]++;
        }
        if ((error & HAL_CAN_ERROR_TX_ALST1) != 0) {
            /* TxMailbox 0 transmit failure due to arbitration lost */
            errorStruct->canErrorCounter[13]++;
        }
        if ((error & HAL_CAN_ERROR_TX_TERR1) != 0) {
            /* TxMailbox 1 transmit failure due to transmit error */
            errorStruct->canErrorCounter[14]++;
        }
        if ((error & HAL_CAN_ERROR_TX_ALST2) != 0) {
            /* TxMailbox 0 transmit failure due to arbitration lost */
            errorStruct->canErrorCounter[15]++;
        }
        if ((error & HAL_CAN_ERROR_TX_TERR2) != 0) {
            /* TxMailbox 1 transmit failure due to transmit error */
            errorStruct->canErrorCounter[16]++;
        }
        if ((error & HAL_CAN_ERROR_TIMEOUT) != 0) {
            /* Timeout error */
            errorStruct->canErrorCounter[17]++;
        }
        if ((error & HAL_CAN_ERROR_NOT_INITIALIZED) != 0) {
            /* Peripheral not initialized */
            errorStruct->canErrorCounter[18]++;
        }
        if ((error & HAL_CAN_ERROR_NOT_READY) != 0) {
            /* Peripheral not ready */
            errorStruct->canErrorCounter[19]++;
        }
        if ((error & HAL_CAN_ERROR_NOT_STARTED) != 0) {
            /* Peripheral not started */
            errorStruct->canErrorCounter[20]++;
        }
        if ((error & HAL_CAN_ERROR_PARAM) != 0) {
            /* Parameter error */
            errorStruct->canErrorCounter[21]++;
        }

    uint16_t rec = (uint16_t)((hcan->Instance->ESR && CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
    uint16_t tec = (uint16_t)((hcan->Instance->ESR && CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);

    HAL_CAN_ResetError(hcan);
#endif
}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    /* Call callback function to interpret or save RX message in buffer */
    CAN_RxMsg_fromFifo_ISR(hcan, CAN_RX_FIFO0);
}

/**
  * @brief  Rx FIFO 1 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    /* Call callback function to interpret or save RX message in buffer */
    CAN_RxMsg_fromFifo_ISR(hcan, CAN_RX_FIFO1);
}
/* USER CODE END 1 */
