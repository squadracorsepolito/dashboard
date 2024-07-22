/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

struct CAN_error_cntrs {
    uint16_t can_error_ewg;
    uint16_t can_error_epv;
    uint16_t can_error_bof;
    uint16_t can_error_stf;
    uint16_t can_error_for;
    uint16_t can_error_ack;
    uint16_t can_error_br;
    uint16_t can_error_bd;
    uint16_t can_error_crc;
    uint16_t can_error_rx_fov0;
    uint16_t can_error_rx_fov1;
    uint16_t can_error_tx_alst0;
    uint16_t can_error_tx_terr0;
    uint16_t can_error_tx_alst1;
    uint16_t can_error_tx_terr1;
    uint16_t can_error_tx_alst2;
    uint16_t can_error_tx_terr2;
    uint16_t can_error_timeout;
    uint16_t can_error_not_initialized;
    uint16_t can_error_not_ready;
    uint16_t can_error_not_started;
    uint16_t can_error_param;
    uint16_t can_error_internal;
    uint16_t rec_val;
    uint16_t tec_val;
};

enum CAN_xHeaderType { RxHeader, TxHeader };

union CAN_xHeaderTypeDef_u {
    CAN_TxHeaderTypeDef tx;
    CAN_RxHeaderTypeDef rx;
};

struct CAN_MsgItem {
    enum CAN_xHeaderType xType; /*!< Define if xHeader below is either Rx or Tx type */
    union CAN_xHeaderTypeDef_u xHeader;
    CAN_HandleTypeDef *hcan;
    uint8_t payload[8U];
};

HAL_StatusTypeDef CAN_TxMsg(CAN_HandleTypeDef *hcan, uint8_t *buffer, CAN_TxHeaderTypeDef *header);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

