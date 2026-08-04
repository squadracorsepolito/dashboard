/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUZZER_CMD_GPIO_OUT_Pin|AMS_ERR_LED_nCMD_GPIO_OUT_Pin|IMD_ERR_LED_nCMD_GPIO_OUT_Pin|TS_OFF_LED_CMD_GPIO_OUT_Pin
                          |RGB3_RED_CMD_Pin|RGB3_BLUE_CMD_Pin|RGB3_GREEN_CMD_Pin|AS_BUZZER_Pin
                          |AS_RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_TFT_RST_GPIO_OUT_GPIO_Port, LCD_TFT_RST_GPIO_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_TFT_DC_GPIO_OUT_GPIO_Port, LCD_TFT_DC_GPIO_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_TFT_CS_GPIO_OUT_Pin|ERR_LED_GPIO_OUT_Pin|EBS_VALVE_1_OUT__Pin|EBS_VALVE_2_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SST25VF080B_nCE_GPIO_OUT_Pin|SST25VF080B_nWP_GPIO_OUT_Pin|SST25VF080B_nHOLD_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, EBS_VALVE_1_OUT_Pin|RGB1_BLUE_CMD_Pin|RGB1_RED_CMD_Pin|RGB1_GREEN_CMD_Pin
                          |RGB2_GREEN_CMD_Pin|RGB2_RED_CMD_Pin|ASSI_BLUE_OUT_Pin|ASSI_YELLOW_OUT_Pin
                          |STAT1_LED_GPIO_OUT_Pin|STAT2_LED_GPIO_OUT_Pin|WARN_LED_GPIO_OUT_Pin|STAT3_LED_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_OUT_GPIO_Port, SD_CS_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                           PEPin */
  GPIO_InitStruct.Pin = nBTN_TS_ON_IN_GPIO_IN_Pin|nBTN_MIS_SEL_IN_GPIO_IN_Pin|nBTN_LC_IN_GPIO_IN_Pin|nRTD_BTN_IN_GPIO_IN_Pin
                          |nBTN_GENERAL_IN_GPIO_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin PCPin PCPin PCPin
                           PCPin */
  GPIO_InitStruct.Pin = BUZZER_CMD_GPIO_OUT_Pin|AMS_ERR_LED_nCMD_GPIO_OUT_Pin|IMD_ERR_LED_nCMD_GPIO_OUT_Pin|TS_OFF_LED_CMD_GPIO_OUT_Pin
                          |RGB3_RED_CMD_Pin|RGB3_BLUE_CMD_Pin|RGB3_GREEN_CMD_Pin|AS_BUZZER_Pin
                          |AS_RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = LCD_TFT_RST_GPIO_OUT_Pin|SD_CS_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LCD_TFT_DC_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_TFT_DC_GPIO_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LCD_TFT_CS_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_TFT_CS_GPIO_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin PEPin */
  GPIO_InitStruct.Pin = SST25VF080B_nCE_GPIO_OUT_Pin|SST25VF080B_nWP_GPIO_OUT_Pin|SST25VF080B_nHOLD_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = EBS_VALVE_1_OUT_Pin|RGB1_BLUE_CMD_Pin|RGB1_RED_CMD_Pin|RGB1_GREEN_CMD_Pin
                          |RGB2_GREEN_CMD_Pin|RGB2_RED_CMD_Pin|ASSI_BLUE_OUT_Pin|ASSI_YELLOW_OUT_Pin
                          |STAT1_LED_GPIO_OUT_Pin|STAT2_LED_GPIO_OUT_Pin|WARN_LED_GPIO_OUT_Pin|STAT3_LED_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = ERR_LED_GPIO_OUT_Pin|EBS_VALVE_1_OUT__Pin|EBS_VALVE_2_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
