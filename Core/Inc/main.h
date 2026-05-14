/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define nBTN_TV_IN_GPIO_IN_Pin GPIO_PIN_2
#define nBTN_TV_IN_GPIO_IN_GPIO_Port GPIOE
#define nBTN_TC_IN_GPIO_IN_Pin GPIO_PIN_3
#define nBTN_TC_IN_GPIO_IN_GPIO_Port GPIOE
#define nBTN_LC_IN_GPIO_IN_Pin GPIO_PIN_4
#define nBTN_LC_IN_GPIO_IN_GPIO_Port GPIOE
#define nRTD_BTN_IN_GPIO_IN_Pin GPIO_PIN_5
#define nRTD_BTN_IN_GPIO_IN_GPIO_Port GPIOE
#define nBTN_GENERAL_IN_GPIO_IN_Pin GPIO_PIN_6
#define nBTN_GENERAL_IN_GPIO_IN_GPIO_Port GPIOE
#define BUZZER_CMD_GPIO_OUT_Pin GPIO_PIN_13
#define BUZZER_CMD_GPIO_OUT_GPIO_Port GPIOC
#define AMS_ERR_LED_nCMD_GPIO_OUT_Pin GPIO_PIN_0
#define AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port GPIOC
#define IMD_ERR_LED_nCMD_GPIO_OUT_Pin GPIO_PIN_1
#define IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port GPIOC
#define SD_SPI2_MISO_Pin GPIO_PIN_2
#define SD_SPI2_MISO_GPIO_Port GPIOC
#define TS_OFF_LED_CMD_GPIO_OUT_Pin GPIO_PIN_3
#define TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port GPIOC
#define ROT_SW1_AIN_ADC_IN_Pin GPIO_PIN_0
#define ROT_SW1_AIN_ADC_IN_GPIO_Port GPIOA
#define ROT_SW2_AIN_ADC_IN_Pin GPIO_PIN_1
#define ROT_SW2_AIN_ADC_IN_GPIO_Port GPIOA
#define SDC_IN_3V3_ADC1_IN2_Pin GPIO_PIN_2
#define SDC_IN_3V3_ADC1_IN2_GPIO_Port GPIOA
#define LCD_TFT_RST_GPIO_OUT_Pin GPIO_PIN_3
#define LCD_TFT_RST_GPIO_OUT_GPIO_Port GPIOA
#define DAC_OUT1_Pin GPIO_PIN_4
#define DAC_OUT1_GPIO_Port GPIOA
#define LCD_TFT_SPI1_SCK_Pin GPIO_PIN_5
#define LCD_TFT_SPI1_SCK_GPIO_Port GPIOA
#define LCD_TFT_SPI1_MISO_Pin GPIO_PIN_6
#define LCD_TFT_SPI1_MISO_GPIO_Port GPIOA
#define LCD_TFT_SPI1_MOSI_Pin GPIO_PIN_7
#define LCD_TFT_SPI1_MOSI_GPIO_Port GPIOA
#define LCD_TFT_DC_GPIO_OUT_Pin GPIO_PIN_5
#define LCD_TFT_DC_GPIO_OUT_GPIO_Port GPIOC
#define LCD_TFT_CS_GPIO_OUT_Pin GPIO_PIN_1
#define LCD_TFT_CS_GPIO_OUT_GPIO_Port GPIOB
#define SST25VF080B_nCE_GPIO_OUT_Pin GPIO_PIN_13
#define SST25VF080B_nCE_GPIO_OUT_GPIO_Port GPIOE
#define SST25VF080B_nWP_GPIO_OUT_Pin GPIO_PIN_14
#define SST25VF080B_nWP_GPIO_OUT_GPIO_Port GPIOE
#define SST25VF080B_nHOLD_GPIO_OUT_Pin GPIO_PIN_15
#define SST25VF080B_nHOLD_GPIO_OUT_GPIO_Port GPIOE
#define SD_SPI2_SCK_Pin GPIO_PIN_10
#define SD_SPI2_SCK_GPIO_Port GPIOB
#define SD_SPI2_MOSI_Pin GPIO_PIN_15
#define SD_SPI2_MOSI_GPIO_Port GPIOB
#define RGB2_BLUE_CMD_Pin GPIO_PIN_10
#define RGB2_BLUE_CMD_GPIO_Port GPIOD
#define RGB1_BLUE_CMD_Pin GPIO_PIN_11
#define RGB1_BLUE_CMD_GPIO_Port GPIOD
#define RGB1_RED_CMD_Pin GPIO_PIN_12
#define RGB1_RED_CMD_GPIO_Port GPIOD
#define RGB1_GREEN_CMD_Pin GPIO_PIN_13
#define RGB1_GREEN_CMD_GPIO_Port GPIOD
#define RGB2_GREEN_CMD_Pin GPIO_PIN_14
#define RGB2_GREEN_CMD_GPIO_Port GPIOD
#define RGB2_RED_CMD_Pin GPIO_PIN_15
#define RGB2_RED_CMD_GPIO_Port GPIOD
#define RGB3_RED_CMD_Pin GPIO_PIN_7
#define RGB3_RED_CMD_GPIO_Port GPIOC
#define RGB3_BLUE_CMD_Pin GPIO_PIN_8
#define RGB3_BLUE_CMD_GPIO_Port GPIOC
#define RGB3_GREEN_CMD_Pin GPIO_PIN_9
#define RGB3_GREEN_CMD_GPIO_Port GPIOC
#define SD_CS_GPIO_OUT_Pin GPIO_PIN_8
#define SD_CS_GPIO_OUT_GPIO_Port GPIOA
#define T_VCP_TX_USART1_TX_Pin GPIO_PIN_9
#define T_VCP_TX_USART1_TX_GPIO_Port GPIOA
#define T_VCP_RX_USART1_RX_Pin GPIO_PIN_10
#define T_VCP_RX_USART1_RX_GPIO_Port GPIOA
#define AS_BUZZER_Pin GPIO_PIN_11
#define AS_BUZZER_GPIO_Port GPIOC
#define AS_RELAY_Pin GPIO_PIN_12
#define AS_RELAY_GPIO_Port GPIOC
#define ASSI_BLUE_OUT_Pin GPIO_PIN_0
#define ASSI_BLUE_OUT_GPIO_Port GPIOD
#define ASSI_YELLOW_OUT_Pin GPIO_PIN_1
#define ASSI_YELLOW_OUT_GPIO_Port GPIOD
#define STAT1_LED_GPIO_OUT_Pin GPIO_PIN_2
#define STAT1_LED_GPIO_OUT_GPIO_Port GPIOD
#define STAT2_LED_GPIO_OUT_Pin GPIO_PIN_3
#define STAT2_LED_GPIO_OUT_GPIO_Port GPIOD
#define WARN_LED_GPIO_OUT_Pin GPIO_PIN_4
#define WARN_LED_GPIO_OUT_GPIO_Port GPIOD
#define STAT3_LED_GPIO_OUT_Pin GPIO_PIN_6
#define STAT3_LED_GPIO_OUT_GPIO_Port GPIOD
#define ERR_LED_GPIO_OUT_Pin GPIO_PIN_5
#define ERR_LED_GPIO_OUT_GPIO_Port GPIOB
#define EBS_VALVE_1_OUT_Pin GPIO_PIN_8
#define EBS_VALVE_1_OUT_GPIO_Port GPIOB
#define EBS_VALVE_2_OUT_Pin GPIO_PIN_9
#define EBS_VALVE_2_OUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PCBVER 2

// PCB ver 1 (Francesco Minichelli's original design)
#if PCBVER == 1

#define BAT_FAN_PWM_TIM htim2
#define BAT_FAN_PWM_CH TIM_CHANNEL_0

#define POWERTRAIN_COOLING_PWM_TIM htim2
#define POWERTRAIN_COOLING_PWM_CH TIM_CHANNEL_1

#define ASB_MOTOR_PWM_TIM htim8
#define ASB_MOTOR_PWM_CH TIM_CHANNEL_1

#define COUNTER_TIM htim3

// PCB ver 2 (the black one)
#elif PCBVER == 2

#define BAT_FAN_PWM_TIM htim4
#define BAT_FAN_PWM_CH TIM_CHANNEL_2

#define RADIATOR_FANS_PWM_TIM htim4
#define RADIATOR_FANS_PWM_CH TIM_CHANNEL_1

#define COUNTER_TIM htim7

#define PUMPS_DAC hdac
#define PUMPS_DAC_CHANNEL DAC_CHANNEL_1
#define RTD_LED_GPIO_Port ERR_LED_GPIO_OUT_GPIO_Port
#define RTD_LED_Pin ERR_LED_GPIO_OUT_Pin
#define FSM_DSPACE_LED_RED_PIN 0
#define FSM_DSPACE_LED_GREEN_PIN 1
#endif


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
