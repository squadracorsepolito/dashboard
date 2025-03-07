/**
 * @file    BSP.h
 * @author  Simone Ruffini [simone.ruffini@squadracorsepolito.com | simone.ruffini.work@gmail.com]
 * @date    Wed May 10 11:59:46 PM CEST 2024
 * @updated Thu Jul 11 12:00:15 AM CEST 2024
 * @version v0.0.1
 * @prefix  x
 *
 * @brief   Implementation of board support package header file
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BSP_H_
#define _BSP_H_

/* SHARED ####################################################################*/

/* ---------- Includes -------------------------------------------------------*/
#include "gpio.h"

/* ---------- Exported types -------------------------------------------------*/

//  GPIO (General Purpuse Input Output)
struct GPIO_Tuple {
    GPIO_TypeDef *GPIO_Port;
    uint16_t GPIO_Pin;
};

/* ---------- Exported constants ---------------------------------------------*/

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* SDC_RLY (ShutDown Circuit Relay) ##########################################*/

/* ---------- Includes -------------------------------------------------------*/
#include "gpio.h"

/* ---------- Exported types -------------------------------------------------*/
enum SDC_RLY_State { SDC_RLY_Open = 0, SDC_RLY_Closed = 1, SDC_RLY_State_NUM };

/* ---------- Exported constants ---------------------------------------------*/

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/
void SDC_RLY_setState(enum SDC_RLY_State state);
void SDC_RLY_toggleState(void);
enum SDC_RLY_State SDC_RLY_getState(void);
/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* ROT_SW (Rotary Switch) ####################################################*/

/* ---------- Includes -------------------------------------------------------*/
#include "adc.h"

/* ---------- Exported types -------------------------------------------------*/

/**
 * @brief Rotary Switch devices
 */
enum ROT_SW_Device {
    ROT_SW_Device1,   /*!< Rotary Switch Device 1*/
    ROT_SW_Device2,   /*!< Rotary Switch Device 2 */
    ROT_SW_Device_NUM /*!< Number of Rotary Switch devices */
};

/**
 * @brief Rotary Switch state/position
 */
enum ROT_SW_State {
    ROT_SW_State0 = 0, /*!< Rotary Switch at state/position 0*/
    ROT_SW_State1,     /*!< Rotary Switch at state/position 1*/
    ROT_SW_State2,     /*!< Rotary Switch at state/position 2*/
    ROT_SW_State3,     /*!< Rotary Switch at state/position 3*/
    ROT_SW_State4,     /*!< Rotary Switch at state/position 4*/
    ROT_SW_State5,     /*!< Rotary Switch at state/position 5*/
    ROT_SW_State6,     /*!< Rotary Switch at state/position 6*/
    ROT_SW_State7,     /*!< Rotary Switch at state/position 7*/
    ROT_SW_State8,     /*!< Rotary Switch at state/position 8*/
    ROT_SW_State9,     /*!< Rotary Switch at state/position 9*/
    ROT_SW_State_NUM   /*!< Number of Rotary Switch states/positions */
};

/* ---------- Exported constants ---------------------------------------------*/

// clang-format off
#define ROT_SW_SIGNAL_CONDITIONING_GAIN (0.6597353497) /*!< Voltage divder constant from input to output */

// (Physical input * volt_divider_const) +- offset -> ADC input

#define ROT_SW_AIN_GAIN (1/ROT_SW_SIGNAL_CONDITIONING_GAIN) /*!< voltage value in ADC Dynamic Range to Physical Dynamic Range conversion constant */
#define ROT_SW_AIN_OFFSET_V (0.0) /*!< voltage value in ADC Dynamic Range to Physical Dynamic Range conversion offset */

#define ROT_SW_STATE_STEP_V (0.54) /*!< Rotary Switch State step voltage (Physical Value)
                                        state1 - state2 = state_step_v */
#define ROT_SW_STATE_ERR_MARGIN_V (ROT_SW_STATE_STEP_V/4.0) /*!< Rotary Switch error margin between states (Physical Value). 
                                                                 State1 = [State1_v - ErrMargin, State1_v+ErrMargin] */
// clang-format on

#define ROT_SW_SAMPLING_PERIOD_MS (50U)

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/
float ROT_SW_getAnalog_V(enum ROT_SW_Device device);
enum ROT_SW_State ROT_SW_sampleState(enum ROT_SW_Device device);
enum ROT_SW_State ROT_SW_getState(enum ROT_SW_Device device);
void ROT_SW_Routine(void);

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* BTN (Buttons) #############################################################*/
/* ---------- Includes -------------------------------------------------------*/

#include "mcb.h"

/* ---------- Exported types -------------------------------------------------*/

/**
 * @brief SDC Sensing Probes
 * @details responds to isOpen / isClosed
 */
enum BTN_Device {
    BTN_RTD,       /*!< Ready To Drive Button */
    BTN_Steering1, /*!< Steering Wheel button 1 */
    BTN_Steering2, /*!< Steering Wheel button 2 */
    BTN_Steering3, /*!< Steering Wheel button 3 */
    BTN_Steering4, /*!< Steering Wheel button 4 */
    BTN_Device_NUM /*!< Number of Button Devices */
};

/* ---------- Exported constants ---------------------------------------------*/
#define BTN_DEVICE_SAMPLING_NYQ_COEFF (10U)
#define BTN_DEVICE_SAMPLING_PERIOD_MS (MCB_DASH_HMI_DEVICES_STATE_CYCLE_TIME_MS / BTN_DEVICE_SAMPLING_NYQ_COEFF)

#define BTN_DEVICE_FILTER_WINDOW_SAMPLES (5U)

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/
uint8_t BTN_sampleStatus(enum BTN_Device device);
uint8_t BTN_getStatus(enum BTN_Device device);
void BTN_Routine(void);

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* LED MONO (Monochrome Leds) ################################################*/

/* ---------- Includes -------------------------------------------------------*/

/* ---------- Exported types -------------------------------------------------*/
enum LED_MONO_State { LED_Off = 0, LED_On = 1, LED_MONO_State_NUM };

/**
 * @brief LED Monocolor/Monocromatic Device
 */
enum LED_MONO_Device {
    LED_AMS_Error,      /*!< Accumulator Management System Error led*/
    LED_IMD_Error,      /*!< Insulation Monitoring Device Error led*/
    LED_TS_Off,         /*!< Tractive System Off led */
    LED_KeepAlive,      /*!< Keep Alive Led */
    LED_User1,          /*!< Status Led User 1 */
    LED_User2,          /*!< Status Led User 2 */
    LED_Warn,           /*!< Warning Led */
    LED_Err,            /*!< Error Led */
    LED_MONO_Device_NUM /*!< Number of Monocolor/Monocromatic Led Devices */
};

/* ---------- Exported constants ---------------------------------------------*/

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/

void LED_MONO_setState(enum LED_MONO_Device device, enum LED_MONO_State state);
void LED_MONO_toggleState(enum LED_MONO_Device device, enum LED_MONO_State state);
enum LED_MONO_State LED_MONO_getState(enum LED_MONO_Device device);

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* LED RGB (RGB Leds) ########################################################*/

/* ---------- Includes -------------------------------------------------------*/

/* ---------- Exported types -------------------------------------------------*/

/**
 * @brief LED RGB Device
 */
enum LED_RGB_Device {
    LED_RGB1,      /*!< LED RGB number 1*/
    LED_RGB2,      /*!< LED RGB number 2 */
    LED_RGB3,      /*!< LED RGB number 3 */
    LED_RGB_DASH,  /*!< LED RGB used by Dashboard device */
    LED_Device_NUM /*!< Number of RGB Leds Devices */
};

/* ---------- Exported constants ---------------------------------------------*/

#define PCA9555_ADDR_A0 (0U)
#define PCA9555_ADDR_A1 (0U)
#define PCA9555_ADDR_A2 (0U)
#define PCA9555_ADDR    (PCA9555_ADDR_FIXED_PART | (PCA9555_ADDR_A0 << 2U) | (PCA9555_ADDR_A0 << 1U) | PCA9555_ADDR_A0)

/* ---------- Exported variables ---------------------------------------------*/

extern struct PCA9555_Handle pca9555Handle;

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/

void LED_RGB_setColor(enum LED_RGB_Device device, uint8_t red, uint8_t green, uint8_t blue);

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* PUMPS ####################################################################*/
/* ---------- Includes -------------------------------------------------------*/

/* ---------- Exported types -------------------------------------------------*/

/* ---------- Exported constants ---------------------------------------------*/

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* FAN (Fans) ################################################################*/
/* ---------- Includes -------------------------------------------------------*/

#include "tim.h"

/* ---------- Exported types -------------------------------------------------*/

/* ---------- Exported constants ---------------------------------------------*/

#define FAN_BAT_TIM_HANDLE htim3
#define FAN_BAT_PWM_CH     TIM_CHANNEL_2

#define RADIATOR_FANS_PWM_TIM htim3
#define RADIATOR_FANS_PWM_CH  TIM_CHANNEL_1

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* Main CAN Bus Communication ################################################*/

/* ---------- Exported types -------------------------------------------------*/

/* ---------- Exported constants ---------------------------------------------*/

#define MCB_Handle                           hcan1
#define MCB_TxAttemptsMailboxesFull          (3U)
#define MCB_AttemptRetryDelayMsMailboxesFull (1U)

/* ---------- Exported variables- --------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions- --------------------------------------------*/

void MCB_send_msg(uint32_t id);

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* ---------- Private Macros -------------------------------------------------*/

/* LCD TFT SCREEN ILI9488 ####################################################*/
/* ---------- Includes -------------------------------------------------------*/
#include "spi.h"
#include "usart.h"
/* ---------- Exported types -------------------------------------------------*/

/* ---------- Exported constants ---------------------------------------------*/
#define LCD_TFT_SPI_Handle hspi3
#define LCD_TFT_USART_Handle huart1
/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/
#endif
