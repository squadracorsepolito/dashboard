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
#include "main.h"

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
    ROT_SW_Device1, /*!< Rotary Switch Device 1*/
    ROT_SW_Device2, /*!< Rotary Switch Device 2 */
    ROT_SW_Device_NUM  /*!< Number of Rotary Switch devices */
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
    ROT_SW_State_NUM      /*!< Number of Rotary Switch states/positions */
};

/* ---------- Exported constants ---------------------------------------------*/

// clang-format off
#define ROT_SW_AIN_GAIN (1.0)
#define ROT_SW_AIN_OFFSET_V (0.0)
#define ROT_SW_AIN_DYN_RANGE_V (ADC_ADC1_VREF_V)   /*!< Full dynamic range at the ADC/analog input (Volts) of the rotary switch where states are divide from */
#define ROT_SW_AIN_STATE_STEP_V  (ROT_SW_AIN_DYN_RANGE_V/(double)(ROT_SW_State_NUM -1U))  /*!< Rotary switch step size in volts at the ADC/analog input (-1 because first state is 0V)*/
#define ROT_SW_AIN_STATE_ERR_MARGIN_V (ROT_SW_AIN_STATE_STEP_V / 2.0) /*!< ADC/analog input error margin (Volts) on a state of the rotary switch to account for noise */
// clang-format on

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/
float ROT_SW_getAanalog_V(enum ROT_SW_Device device);
enum ROT_SW_State ROT_SW_getState(enum ROT_SW_Device device);

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/* BTN (Buttons) #############################################################*/
/* ---------- Includes -------------------------------------------------------*/

/* ---------- Exported types -------------------------------------------------*/

enum BTN_State { BTN_NotPressed = 0, BTN_Pressed = 1, BTN_State_NUM };
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

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/
enum BTN_State BTN_getState(enum BTN_Device device);

/* ---------- Private types --------------------------------------------------*/

/* ---------- Private variables ----------------------------------------------*/

/* ---------- Private constants ----------------------------------------------*/

/*  LED ######################################################################*/

/* ---------- Includes -------------------------------------------------------*/

/* ---------- Exported types -------------------------------------------------*/
enum LED_State { LED_Off = 0, LED_On = 1, LED_State_NUM };

/**
 * @brief LED Monocolor/Monocromatic Device
 */
enum LED_MONO_Device {
    LED_AMS_Error,      /*!< Accumulator Management System Error led*/
    LED_IMD_Error,      /*!< Insulation Monitoring Device Error led*/
    LED_TS_Off,          /*!< Tractive System Off led */
    LED_KeepAlive,      /*!< Keep Alive Led */
    LED_User1,          /*!< Status Led User 1 */
    LED_User2,          /*!< Status Led User 2 */
    LED_Warn,           /*!< Warning Led */
    LED_Err,            /*!< Error Led */
    LED_MONO_Device_NUM /*!< Number of Monocolor/Monocromatic Led Devices */
};

/**
 * @brief LED RGB Device
 */
enum LED_RGB_Device {
    LED_RGB1, /*!< LED RGB number 1*/
    LED_RGB2, /*!< LED RGB number 2 */
    LED_RGB3, /*!< LED RGB number 3 */
    LED_RGB4, /*!< LED RGB number 4 */
    LED_Device_NUM /*!< Number of RGB Leds Devices */
};

/* ---------- Exported constants ---------------------------------------------*/

/* ---------- Exported variables ---------------------------------------------*/

/* ---------- Exported macros ------------------------------------------------*/

/* ---------- Exported functions ---------------------------------------------*/

void LED_MONO_setState(enum LED_MONO_Device device,enum LED_State state);
void LED_MONO_toggleState(enum LED_MONO_Device device);
enum LED_State LED_MONO_getState(enum LED_MONO_Device device);

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

#endif
