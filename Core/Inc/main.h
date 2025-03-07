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
#define BUZZER_CMD_GPIO_OUT_Pin GPIO_PIN_13
#define BUZZER_CMD_GPIO_OUT_GPIO_Port GPIOC
#define LCD_TFT_RST_GPIO_OUT_Pin GPIO_PIN_0
#define LCD_TFT_RST_GPIO_OUT_GPIO_Port GPIOC
#define M95256_D_SPI_MOSI_Pin GPIO_PIN_1
#define M95256_D_SPI_MOSI_GPIO_Port GPIOC
#define TS_OFF_LED_CMD_GPIO_OUT_Pin GPIO_PIN_2
#define TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port GPIOC
#define IMD_ERR_LED_nCMD_GPIO_OUT_Pin GPIO_PIN_3
#define IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port GPIOC
#define ROT_SW1_AIN_ADC_IN_Pin GPIO_PIN_0
#define ROT_SW1_AIN_ADC_IN_GPIO_Port GPIOA
#define ROT_SW2_AIN_ADC_IN_Pin GPIO_PIN_1
#define ROT_SW2_AIN_ADC_IN_GPIO_Port GPIOA
#define AMS_ERR_LED_nCMD_GPIO_OUT_Pin GPIO_PIN_2
#define AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port GPIOA
#define COOLING_PUMPS_V_DAC_OUT_Pin GPIO_PIN_4
#define COOLING_PUMPS_V_DAC_OUT_GPIO_Port GPIOA
#define RAD_FANS_PWM_OUT_TIM_PWM_Pin GPIO_PIN_6
#define RAD_FANS_PWM_OUT_TIM_PWM_GPIO_Port GPIOA
#define TSAC_FANS_PWM_OUT_TIM_PWM_Pin GPIO_PIN_7
#define TSAC_FANS_PWM_OUT_TIM_PWM_GPIO_Port GPIOA
#define nPUSH_BTN1_IN_GPIO_IN_Pin GPIO_PIN_4
#define nPUSH_BTN1_IN_GPIO_IN_GPIO_Port GPIOC
#define nPUSH_BTN2_IN_GPIO_IN_Pin GPIO_PIN_5
#define nPUSH_BTN2_IN_GPIO_IN_GPIO_Port GPIOC
#define nPUSH_BTN3_IN_GPIO_IN_Pin GPIO_PIN_0
#define nPUSH_BTN3_IN_GPIO_IN_GPIO_Port GPIOB
#define nPUSH_BTN4_IN_GPIO_IN_Pin GPIO_PIN_1
#define nPUSH_BTN4_IN_GPIO_IN_GPIO_Port GPIOB
#define nRTD_BTN_IN_GPIO_IN_Pin GPIO_PIN_2
#define nRTD_BTN_IN_GPIO_IN_GPIO_Port GPIOB
#define LCD_TFT_DC_GPIO_OUT_Pin GPIO_PIN_10
#define LCD_TFT_DC_GPIO_OUT_GPIO_Port GPIOB
#define SN65HVD23x_R_CAN2_RX_Pin GPIO_PIN_12
#define SN65HVD23x_R_CAN2_RX_GPIO_Port GPIOB
#define SN65HVD23x_D_CAN2_TX_Pin GPIO_PIN_13
#define SN65HVD23x_D_CAN2_TX_GPIO_Port GPIOB
#define STAT1_LED_GPIO_OUT_Pin GPIO_PIN_14
#define STAT1_LED_GPIO_OUT_GPIO_Port GPIOB
#define STAT2_LED_GPIO_OUT_Pin GPIO_PIN_15
#define STAT2_LED_GPIO_OUT_GPIO_Port GPIOB
#define STAT3_LED_GPIO_OUT_Pin GPIO_PIN_6
#define STAT3_LED_GPIO_OUT_GPIO_Port GPIOC
#define M95256_nW_GPIO_OUT_Pin GPIO_PIN_7
#define M95256_nW_GPIO_OUT_GPIO_Port GPIOC
#define M95256_nS_GPIO_OUT_Pin GPIO_PIN_8
#define M95256_nS_GPIO_OUT_GPIO_Port GPIOC
#define WARN_LED_GPIO_OUT_Pin GPIO_PIN_9
#define WARN_LED_GPIO_OUT_GPIO_Port GPIOC
#define ERR_LED_GPIO_OUT_Pin GPIO_PIN_8
#define ERR_LED_GPIO_OUT_GPIO_Port GPIOA
#define T_VCP_TX_USART_TX_Pin GPIO_PIN_9
#define T_VCP_TX_USART_TX_GPIO_Port GPIOA
#define T_VCP_RX_USART_RX_Pin GPIO_PIN_10
#define T_VCP_RX_USART_RX_GPIO_Port GPIOA
#define SN65HVD23x_R_CAN1_RX_Pin GPIO_PIN_11
#define SN65HVD23x_R_CAN1_RX_GPIO_Port GPIOA
#define SN65HVD23x_D_CAN1_TX_Pin GPIO_PIN_12
#define SN65HVD23x_D_CAN1_TX_GPIO_Port GPIOA
#define M95256_C_SPI_SCK_Pin GPIO_PIN_10
#define M95256_C_SPI_SCK_GPIO_Port GPIOC
#define M95256_Q_SPI_MISO_Pin GPIO_PIN_11
#define M95256_Q_SPI_MISO_GPIO_Port GPIOC
#define SDC_RLY_CMD_GPIO_OUT_Pin GPIO_PIN_12
#define SDC_RLY_CMD_GPIO_OUT_GPIO_Port GPIOC
#define NHD_C0220BIZx_nRST_GPIO_OUT_Pin GPIO_PIN_2
#define NHD_C0220BIZx_nRST_GPIO_OUT_GPIO_Port GPIOD
#define LCD_TFT_CS_GPIO_OUT_Pin GPIO_PIN_4
#define LCD_TFT_CS_GPIO_OUT_GPIO_Port GPIOB
#define PCA9555_nINT_GPIO_IN_Pin GPIO_PIN_5
#define PCA9555_nINT_GPIO_IN_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
#define SDC_IN_3V3_GPIO_IN_Pin GPIO_PIN_8
#define SDC_IN_3V3_GPIO_IN_GPIO_Port GPIOB
#define SDC_OUT_3V3_GPIO_IN_Pin GPIO_PIN_9
#define SDC_OUT_3V3_GPIO_IN_GPIO_Port GPIOB

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

#define BAT_FAN_PWM_TIM htim3
#define BAT_FAN_PWM_CH TIM_CHANNEL_2

#define RADIATOR_FANS_PWM_TIM htim3
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
