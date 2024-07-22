/**
 * @file    TEMPLATE.c
 * @author  AuthorName [email@email.com]
 * @date    202x-xx-xx (date of creation)
 * @updated 202x-xx-xx (date of last update)
 * @version vX.X.X
 * @prefix  TMP
 *
 * @brief   Implementation of some software
 * @details This code implements bla bla
 *
 * @license Licensed under "THE BEER-WARE LICENSE", Revision 69 
 *          see LICENSE file in the root directory of this software component
 */

/*---------- Includes -------------------------------------------------------*/

#include "bsp.h"

#include "iso646.h"  // enables writing binary opeartors as names

/* SDC_RLY (ShutDown Circuit Relay) ##########################################*/

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

static struct GPIO_Tuple SDC_RLY_Device_GPIO_Tuple_map = {.GPIO_Port = SDC_RLY_CMD_GPIO_OUT_GPIO_Port,
                                                          .GPIO_Pin  = SDC_RLY_CMD_GPIO_OUT_Pin};

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/

void SDC_RLY_setState(enum SDC_RLY_State state) {
    assert_param(state != SDC_RLY_State_NUM);
    GPIO_PinState pinState = (state == SDC_RLY_Open) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(SDC_RLY_Device_GPIO_Tuple_map.GPIO_Port, SDC_RLY_Device_GPIO_Tuple_map.GPIO_Pin, pinState);
}
void SDC_RLY_toggleState(void) {
    HAL_GPIO_TogglePin(SDC_RLY_Device_GPIO_Tuple_map.GPIO_Port, SDC_RLY_Device_GPIO_Tuple_map.GPIO_Pin);
}
enum SDC_RLY_State SDC_RLY_getState(void) {
    GPIO_PinState state;
    state = HAL_GPIO_ReadPin(SDC_RLY_Device_GPIO_Tuple_map.GPIO_Port, SDC_RLY_Device_GPIO_Tuple_map.GPIO_Pin);
    return state == GPIO_PIN_RESET ? SDC_RLY_Open : SDC_RLY_Closed;
}

/*---------- Private Functions -----------------------------------------------*/

/* ROT_SW (Rotary Switch) ####################################################*/
/*---------- Private define --------------------------------------------------*/

#define ROT_SW_ADC_Handle hadc1
/*---------- Private macro -0-------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

static const enum ADC_Channel ROT_SW_Device_to_ADC_Channel_map[ROT_SW_Device_NUM] =
    {[ROT_SW_Device1] = ADC_Channel0, [ROT_SW_Device2] = ADC_Channel1};

// state to -> Vmin,Vmax of state
static float ROT_SW_ain_V_to_state_map[ROT_SW_State_NUM][2U] = {
    [ROT_SW_State0] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State0),
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State0) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
    [ROT_SW_State1] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State1) - ROT_SW_AIN_STATE_ERR_MARGIN_V,
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State1) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
    [ROT_SW_State2] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State2) - ROT_SW_AIN_STATE_ERR_MARGIN_V,
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State2) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
    [ROT_SW_State3] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State3) - ROT_SW_AIN_STATE_ERR_MARGIN_V,
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State3) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
    [ROT_SW_State4] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State4) - ROT_SW_AIN_STATE_ERR_MARGIN_V,
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State4) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
    [ROT_SW_State5] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State5) - ROT_SW_AIN_STATE_ERR_MARGIN_V,
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State5) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
    [ROT_SW_State6] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State6) - ROT_SW_AIN_STATE_ERR_MARGIN_V,
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State6) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
    [ROT_SW_State7] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State7) - ROT_SW_AIN_STATE_ERR_MARGIN_V,
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State7) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
    [ROT_SW_State8] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State8) - ROT_SW_AIN_STATE_ERR_MARGIN_V,
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State8) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
    [ROT_SW_State9] = {(ROT_SW_AIN_STATE_STEP_V * ROT_SW_State9) - ROT_SW_AIN_STATE_ERR_MARGIN_V,
                       (ROT_SW_AIN_STATE_STEP_V * ROT_SW_State9) + ROT_SW_AIN_STATE_ERR_MARGIN_V},
};
/*---------- Private Functions -----------------------------------------------*/
enum ROT_SW_State __ROT_SW_analogV_to_state(float analogV) {
    if (analogV < 0) {
        return ROT_SW_State_NUM;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State0][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State0][1]) {
        return ROT_SW_State0;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State1][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State1][1]) {
        return ROT_SW_State1;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State2][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State2][1]) {
        return ROT_SW_State2;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State3][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State3][1]) {
        return ROT_SW_State3;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State4][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State4][1]) {
        return ROT_SW_State4;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State5][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State5][1]) {
        return ROT_SW_State5;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State6][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State6][1]) {
        return ROT_SW_State6;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State7][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State7][1]) {
        return ROT_SW_State7;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State8][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State8][1]) {
        return ROT_SW_State8;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State9][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State9][1]) {
        return ROT_SW_State9;
    } else {
        return ROT_SW_State_NUM;
    }
}

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/
float ROT_SW_getAanalog_V(enum ROT_SW_Device device) {
    assert_param(device != ROT_SW_State_NUM);
    enum ADC_Channel chnl = ROT_SW_Device_to_ADC_Channel_map[device];
    float adcVal          = ADC_ADC1_getChannelRawFiltered(chnl);
    if (adcVal < 0)
        return (-1.0f);
    float physicalVal = ADC_CONV_RAW_TO_V(adcVal, ADC_GET_RESOLUTION_BITS(&ROT_SW_ADC_Handle), ADC_ADC1_VREF_V);
    return (physicalVal * ROT_SW_AIN_GAIN + ROT_SW_AIN_OFFSET_V);
}
enum ROT_SW_State ROT_SW_getState(enum ROT_SW_Device device) {
    assert_param(device != ROT_SW_State_NUM);
    return __ROT_SW_analogV_to_state(ROT_SW_getAanalog_V(device));
}

/* BTN (Buttons) ##############################################################*/

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

static const struct GPIO_Tuple BTN_Device_to_GPIO_Tuple_map[BTN_Device_NUM] = {
    [BTN_RTD]       = {.GPIO_Port = nRTD_BTN_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nRTD_BTN_IN_GPIO_IN_Pin},
    [BTN_Steering1] = {.GPIO_Port = nPUSH_BTN1_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nPUSH_BTN1_IN_GPIO_IN_Pin},
    [BTN_Steering2] = {.GPIO_Port = nPUSH_BTN2_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nPUSH_BTN2_IN_GPIO_IN_Pin},
    [BTN_Steering3] = {.GPIO_Port = nPUSH_BTN3_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nPUSH_BTN3_IN_GPIO_IN_Pin},
    [BTN_Steering4] = {.GPIO_Port = nPUSH_BTN4_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nPUSH_BTN4_IN_GPIO_IN_Pin},
};

static uint8_t BTN_GPIO_invert_vector[BTN_Device_NUM] = {
    [BTN_RTD]       = 1U,
    [BTN_Steering1] = 1U,
    [BTN_Steering2] = 1U,
    [BTN_Steering3] = 1U,
    [BTN_Steering4] = 1U,
};

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/

enum BTN_State BTN_getState(enum BTN_Device device) {
    assert_param(device != BTN_Device_NUM);
    GPIO_TypeDef *port = BTN_Device_to_GPIO_Tuple_map[device].GPIO_Port;
    uint16_t pin       = BTN_Device_to_GPIO_Tuple_map[device].GPIO_Pin;

    GPIO_PinState compareValue = BTN_GPIO_invert_vector[device] ? GPIO_PIN_RESET : GPIO_PIN_SET;
    return HAL_GPIO_ReadPin(port, pin) == compareValue ? BTN_Pressed : BTN_NotPressed;
}

/*---------- Private Functions ----------------------------------------------*/
/*  LED ######################################################################*/
/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

static const struct GPIO_Tuple LED_MONO_Device_to_GPIO_Tuple_map[LED_MONO_Device_NUM] = {
    [LED_AMS_Error] = {.GPIO_Port = nAMS_ERR_LED_CMD_GPIO_OUT_GPIO_Port, .GPIO_Pin = nAMS_ERR_LED_CMD_GPIO_OUT_Pin},
    [LED_IMD_Error] = {.GPIO_Port = nIMD_ERR_LED_CMD_GPIO_OUT_GPIO_Port, .GPIO_Pin = nIMD_ERR_LED_CMD_GPIO_OUT_Pin},
    [LED_TS_Off]    = {.GPIO_Port = TS_OFF_LED_CMD_GPIO_OUT_GPIO_Port, .GPIO_Pin = TS_OFF_LED_CMD_GPIO_OUT_Pin},
    [LED_KeepAlive] = {.GPIO_Port = STAT1_LED_GPIO_OUT_GPIO_Port, .GPIO_Pin = STAT1_LED_GPIO_OUT_Pin},
    [LED_User1]     = {.GPIO_Port = STAT2_LED_GPIO_OUT_GPIO_Port, .GPIO_Pin = STAT2_LED_GPIO_OUT_Pin},
    [LED_User2]     = {.GPIO_Port = STAT3_LED_GPIO_OUT_GPIO_Port, .GPIO_Pin = STAT3_LED_GPIO_OUT_Pin},
    [LED_Warn]      = {.GPIO_Port = WARN_LED_GPIO_OUT_GPIO_Port, .GPIO_Pin = WARN_LED_GPIO_OUT_Pin},
    [LED_Err]       = {.GPIO_Port = ERR_LED_GPIO_OUT_GPIO_Port, .GPIO_Pin = ERR_LED_GPIO_OUT_Pin}};

static uint8_t LED_MONO_GPIO_invert_vector[LED_MONO_Device_NUM] = {
    [LED_AMS_Error] = 1U,
    [LED_IMD_Error] = 1U,
    [LED_TS_Off]    = 0U,
    [LED_KeepAlive] = 0U,
    [LED_User1]     = 0U,
    [LED_User2]     = 0U,
    [LED_Warn]      = 0U,
    [LED_Err]       = 0U,
};

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/

void LED_MONO_setState(enum LED_MONO_Device device, enum LED_State state) {
    assert_param(device != LED_MONO_Device_NUM);
    assert_param(state != LED_State_NUM);
    GPIO_TypeDef *port = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Port;
    uint16_t pin       = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Pin;

    // Invert reading if values on GPIO are inverted
    // take state and xor it with the invert matrix (the xor operator is a gate inverter)
    GPIO_PinState setValue = ((state == LED_On) xor (LED_MONO_GPIO_invert_vector[device])) ? GPIO_PIN_SET
                                                                                           : GPIO_PIN_RESET;
    return HAL_GPIO_WritePin(port, pin, setValue);
}

void LED_MONO_toggleState(enum LED_MONO_Device device) {
    assert_param(device != LED_MONO_Device_NUM);

    GPIO_TypeDef *port = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Port;
    uint16_t pin       = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Pin;

    HAL_GPIO_TogglePin(port, pin);
}

enum LED_State LED_MONO_getState(enum LED_MONO_Device device) {
    assert_param(device != LED_MONO_Device_NUM);
    GPIO_TypeDef *port = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Port;
    uint16_t pin       = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Pin;

    // Invert reading if values on GPIO are inverted
    GPIO_PinState compareValue = LED_MONO_GPIO_invert_vector[device] ? GPIO_PIN_RESET : GPIO_PIN_SET;
    return HAL_GPIO_ReadPin(port, pin) == compareValue ? LED_On : LED_Off;
}
/*---------- Private Functions -----------------------------------------------*/
