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

/*---------- Includes -----------------------.--------------------------------*/

#include "bsp.h"

/* SDC_RLY (ShutDown Circuit Relay) ##########################################*/

/*---------- Private define -----------------.--------------------------------*/

/*---------- Private macro ------------------.--------------------------------*/

/*---------- Private variables --------------.--------------------------------*/

static struct GPIO_Tuple SDC_RLY_Device_to_GPIO_Tuple_map = {.GPIO_Port = SDC_RLY_CMD_GPIO_OUT_GPIO_Port,
                                                             .GPIO_Pin  = SDC_RLY_CMD_GPIO_OUT_Pin};

/*---------- Private function prototypes ----.--------------------------------*/

/*---------- Exported Variables -------------.--------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/

void SDC_RLY_setState(enum SDC_RLY_State state) {
    assert_param(state != SDC_RLY_State_NUM);
    GPIO_PinState pinState = state == SDC_RLY_Open ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(SDC_RLY_Device_to_GPIO_Tuple_map.GPIO_Port, SDC_RLY_Device_to_GPIO_Tuple_map.GPIO_Pin, pinState);
}
void SDC_RLY_toggleState(void) {
    HAL_GPIO_TogglePin(SDC_RLY_Device_to_GPIO_Tuple_map.GPIO_Port, SDC_RLY_Device_to_GPIO_Tuple_map.GPIO_Pin);
}
enum SDC_RLY_State SDC_RLY_getState(void) {
    GPIO_PinState state;
    state = HAL_GPIO_ReadPin(SDC_RLY_Device_to_GPIO_Tuple_map.GPIO_Port, SDC_RLY_Device_to_GPIO_Tuple_map.GPIO_Pin);
    return state == GPIO_PIN_RESET ? SDC_RLY_Open : SDC_RLY_Closed;
}

/*---------- Private Functions -----------------------------------------------*/

/* BTN (Buttons) #############################################################*/

/*---------- Private define --------------------------------------------------*/
#define BTN_VALUES_ARR_LEN (BTN_DEVICE_FILTER_WINDOW_SAMPLES)

#define XSTR(x) STR(x)
#define STR(x)  #x

#if BTN_VALUES_ARR_LEN > 10U
#pragma message("BTN_values = " XSTR(BTN_VALUES_ARR_LEN))
#error "BTN_values array length > 10. Decrase BTN_DEVICE_FILTER_WINDOW_SAMPLES."
#endif

#if BTN_VALUES_ARR_LEN < 1
#pragma message("BTN_values = " XSTR(BTN_VALUES_ARR_LEN))
#error "BTN_values is < 1. Increase BTN_DEVICE_FILTER_WINDOW_SAMPLES."
#endif

#define BTN_VALUES_ARR_LEN_U            ((uint8_t)BTN_VALUES_ARR_LEN)
#define BTN_DEVICE_SAMPLING_PERIOD_MS_U ((uint32_t)BTN_DEVICE_SAMPLING_PERIOD_MS)

#if BTN_Device_NUM <= 8
#define BTN_Value_t uint8_t
#elif BTN_Device_NUM <= 16
#define BTN_Value_t uint16_t
#else
#define BTN_Value_t uint32_t
#endif

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

static struct GPIO_Tuple BTN_Device_to_GPIO_Tuple_map[BTN_Device_NUM] = {
    [BTN_RTD]       = {.GPIO_Port = nRTD_BTN_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nRTD_BTN_IN_GPIO_IN_Pin},
    [BTN_Steering1] = {.GPIO_Port = nPUSH_BTN1_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nPUSH_BTN1_IN_GPIO_IN_Pin},
    [BTN_Steering2] = {.GPIO_Port = nPUSH_BTN2_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nPUSH_BTN2_IN_GPIO_IN_Pin},
    [BTN_Steering3] = {.GPIO_Port = nPUSH_BTN3_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nPUSH_BTN3_IN_GPIO_IN_Pin},
    [BTN_Steering4] = {.GPIO_Port = nPUSH_BTN4_IN_GPIO_IN_GPIO_Port, .GPIO_Pin = nPUSH_BTN4_IN_GPIO_IN_Pin}};

static uint8_t BTN_GPIO_invert_vector[BTN_Device_NUM] =
    {[BTN_RTD] = 1U, [BTN_Steering1] = 1U, [BTN_Steering2] = 1U, [BTN_Steering3] = 1U, [BTN_Steering4] = 1U};

static volatile BTN_Value_t BTN_Device_values[BTN_VALUES_ARR_LEN_U] = {};

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/

uint8_t BTN_sampleStatus(enum BTN_Device device) {
    assert_param(device != BTN_Device_NUM);
    GPIO_TypeDef *port = BTN_Device_to_GPIO_Tuple_map[device].GPIO_Port;
    uint16_t pin       = BTN_Device_to_GPIO_Tuple_map[device].GPIO_Pin;

    // Invert reading if values on GPIO are inverted
    GPIO_PinState compareValue = BTN_GPIO_invert_vector[device] ? GPIO_PIN_RESET : GPIO_PIN_SET;
    return (HAL_GPIO_ReadPin(port, pin) == compareValue) ? 1 : 0;
}

uint8_t BTN_getStatus(enum BTN_Device device) {
    assert_param(probe != BTN_Device_NUM);
    uint32_t value_cnt = 0;

    // Get value by applying debouncing we are intrested in ones
    for (uint8_t i = 0; i < BTN_VALUES_ARR_LEN_U; i++) {
        if ((BTN_Device_values[i] & ((1U) << device))) {
            value_cnt++;
        }
    }
    if (((double)value_cnt / BTN_VALUES_ARR_LEN_U) >= 0.5)
        return 1U;
    else
        return 0U;
}

void BTN_Routine(void) {
    static uint32_t routine_tim = BTN_DEVICE_SAMPLING_PERIOD_MS_U;

    if (HAL_GetTick() < routine_tim) {
        return;
    }
    routine_tim = HAL_GetTick() + BTN_DEVICE_SAMPLING_PERIOD_MS_U;

    // Move old samples ahead
    for (uint8_t i = 0; i < BTN_VALUES_ARR_LEN_U - 1; i++) {
        BTN_Device_values[i] = BTN_Device_values[i + 1];
    }
    // Push New value back
    for (uint8_t i = 0; i < BTN_Device_NUM; i++) {
        uint8_t curr_sample = BTN_sampleStatus(i);
        BTN_Device_values[BTN_VALUES_ARR_LEN_U - 1] &= ~((BTN_Value_t)((1U) << i));  // reset the value
        BTN_Device_values[BTN_VALUES_ARR_LEN_U - 1] |= (curr_sample << i);           // set the value
    }
}

/*---------- Private Functions -----------------------------------------------*/

/* ROT_SW (Rotary Switch) ####################################################*/

/*---------- Private define --------------------------------------------------*/

#define ROT_SW_ADC_Handle hadc1

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

static const enum ADC_Channel ROT_SW_Device_to_ADC_Channel_map[ROT_SW_Device_NUM] =
    {[ROT_SW_Device1] = ADC_Channel0, [ROT_SW_Device2] = ADC_Channel1};

static enum ROT_SW_State ROT_SW_Device_LastState[ROT_SW_Device_NUM] =
    {[ROT_SW_Device1] = ROT_SW_State_NUM, [ROT_SW_Device2] = ROT_SW_State_NUM};
static enum ROT_SW_State ROT_SW_Device_State[ROT_SW_Device_NUM] =
    {[ROT_SW_Device1] = ROT_SW_State_NUM, [ROT_SW_Device2] = ROT_SW_State_NUM};

static float ROT_SW_ain_V_to_state_map[ROT_SW_State_NUM][2U] = {
    [ROT_SW_State0] = {(0.0), (0.0) + ROT_SW_STATE_ERR_MARGIN_V},
    [ROT_SW_State1] = {(2.71) - ROT_SW_STATE_ERR_MARGIN_V, (2.71) + ROT_SW_STATE_ERR_MARGIN_V},
    [ROT_SW_State2] = {(0.54) - ROT_SW_STATE_ERR_MARGIN_V, (0.54) + ROT_SW_STATE_ERR_MARGIN_V},
    [ROT_SW_State3] = {(3.26) - ROT_SW_STATE_ERR_MARGIN_V, (3.26) + ROT_SW_STATE_ERR_MARGIN_V},
    [ROT_SW_State4] = {(1.10) - ROT_SW_STATE_ERR_MARGIN_V, (1.10) + ROT_SW_STATE_ERR_MARGIN_V},
    [ROT_SW_State5] = {(3.81) - ROT_SW_STATE_ERR_MARGIN_V, (3.81) + ROT_SW_STATE_ERR_MARGIN_V},
    [ROT_SW_State6] = {(1.63) - ROT_SW_STATE_ERR_MARGIN_V, (1.63) + ROT_SW_STATE_ERR_MARGIN_V},
    [ROT_SW_State7] = {(4.35) - ROT_SW_STATE_ERR_MARGIN_V, (4.35) + ROT_SW_STATE_ERR_MARGIN_V},
    [ROT_SW_State8] = {(2.17) - ROT_SW_STATE_ERR_MARGIN_V, (2.17) + ROT_SW_STATE_ERR_MARGIN_V},
    [ROT_SW_State9] = {(4.89) - ROT_SW_STATE_ERR_MARGIN_V, (4.89) + ROT_SW_STATE_ERR_MARGIN_V},
};
/*---------- Private Functions -----------------------------------------------*/
enum ROT_SW_State __ROT_SW_analogV_to_state(float analogV) {
    enum ROT_SW_State state = ROT_SW_State_NUM;

    if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State0][0] &&
        analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State0][1]) {
        state = ROT_SW_State0;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State1][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State1][1]) {
        state = ROT_SW_State1;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State2][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State2][1]) {
        state = ROT_SW_State2;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State3][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State3][1]) {
        state = ROT_SW_State3;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State4][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State4][1]) {
        state = ROT_SW_State4;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State5][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State5][1]) {
        state = ROT_SW_State5;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State6][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State6][1]) {
        state = ROT_SW_State6;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State7][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State7][1]) {
        state = ROT_SW_State7;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State8][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State8][1]) {
        state = ROT_SW_State8;
    } else if (analogV >= ROT_SW_ain_V_to_state_map[ROT_SW_State9][0] &&
               analogV < ROT_SW_ain_V_to_state_map[ROT_SW_State9][1]) {
        state = ROT_SW_State9;
    }

    return state;
}

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/
float ROT_SW_getAnalog_V(enum ROT_SW_Device device) {
    assert_param(device != ROT_SW_State_NUM);
    enum ADC_Channel chnl = ROT_SW_Device_to_ADC_Channel_map[device];
    float adcVal          = ADC_ADC1_getChannelRawFiltered(chnl);
    if (adcVal < 0)
        return (-1.0f);
    float physicalVal = ADC_CONV_RAW_TO_V(adcVal, ADC_GET_RESOLUTION_BITS(&ROT_SW_ADC_Handle), ADC_ADC1_VREF_V);
    return (physicalVal * ROT_SW_AIN_GAIN + ROT_SW_AIN_OFFSET_V);
}
enum ROT_SW_State __ROT_SW_getState(enum ROT_SW_Device device) {
    assert_param(device != ROT_SW_Device_NUM);

    enum ROT_SW_State *const last_state = &ROT_SW_Device_LastState[device];

    enum ROT_SW_State state = __ROT_SW_analogV_to_state(ROT_SW_getAnalog_V(device));

    if (*last_state == ROT_SW_State_NUM) {
        *last_state = state;
    }

    if (state != ROT_SW_State_NUM && state >= (*last_state) - 1 && state <= (*last_state) + 1) {
        *last_state = state;
    } else {
        state = *last_state;
    }

    return state;
}

enum ROT_SW_State ROT_SW_sampleState(enum ROT_SW_Device device) {
    assert_param(device != ROT_SW_Device_NUM);

    return __ROT_SW_analogV_to_state(ROT_SW_getAnalog_V(device));
}

enum ROT_SW_State ROT_SW_getState(enum ROT_SW_Device device) {
    assert_param(device != ROT_SW_Device_NUM);

    return ROT_SW_Device_State[device];
}

void ROT_SW_Routine(void) {
    static uint32_t routine_tim = ROT_SW_SAMPLING_PERIOD_MS;

    if (HAL_GetTick() < routine_tim) {
        return;
    }
    routine_tim = HAL_GetTick() + ROT_SW_SAMPLING_PERIOD_MS;

    for (enum ROT_SW_Device i = 0; i < ROT_SW_Device_NUM; i++) {
        enum ROT_SW_State last_state = ROT_SW_Device_State[i];

        enum ROT_SW_State curr_state = ROT_SW_sampleState(i);

        // If this is the first conversion, init the last state for next computation
        if (last_state == ROT_SW_State_NUM) {
            last_state = curr_state;
        }

        if (curr_state != ROT_SW_State_NUM && curr_state >= (last_state)-1 && curr_state <= (last_state) + 1) {
            // if no error and the state progressed correctly, update current state
            ROT_SW_Device_State[i] = curr_state;
        } else {
            // if either in error or the step was not in range keep the same state
            ROT_SW_Device_State[i] = last_state;
        }
    }
}

/* LED MONO (Monochrome Leds) ################################################*/
/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

static const struct GPIO_Tuple LED_MONO_Device_to_GPIO_Tuple_map[LED_MONO_Device_NUM] = {
    [LED_AMS_Error] = {.GPIO_Port = AMS_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, .GPIO_Pin = AMS_ERR_LED_nCMD_GPIO_OUT_Pin},
    [LED_IMD_Error] = {.GPIO_Port = IMD_ERR_LED_nCMD_GPIO_OUT_GPIO_Port, .GPIO_Pin = IMD_ERR_LED_nCMD_GPIO_OUT_Pin},
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

void LED_MONO_setState(enum LED_MONO_Device device, enum LED_MONO_State state) {
    assert_param(device != LED_MONO_Device_NUM);
    assert_param(state != LED_MONO_State_NUM);
    GPIO_TypeDef *port = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Port;
    uint16_t pin       = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Pin;

    // Invert reading if values on GPIO are inverted
    GPIO_PinState setValue = LED_MONO_GPIO_invert_vector[device] ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(port, pin, setValue);
}

void LED_MONO_toggleState(enum LED_MONO_Device device, enum LED_MONO_State state) {
    assert_param(device != LED_MONO_Device_NUM);
    assert_param(state != LED_MONO_State_NUM);
    GPIO_TypeDef *port = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Port;
    uint16_t pin       = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Pin;

    HAL_GPIO_TogglePin(port, pin);
}

enum LED_MONO_State LED_MONO_getState(enum LED_MONO_Device device) {
    assert_param(device != LED_MONO_Device_NUM);
    GPIO_TypeDef *port = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Port;
    uint16_t pin       = LED_MONO_Device_to_GPIO_Tuple_map[device].GPIO_Pin;

    // Invert reading if values on GPIO are inverted
    GPIO_PinState compareValue = LED_MONO_GPIO_invert_vector[device] ? GPIO_PIN_RESET : GPIO_PIN_SET;
    return HAL_GPIO_ReadPin(port, pin) == compareValue ? LED_On : LED_Off;
}
/*---------- Private Functions -----------------------------------------------*/

/* LED RGB (RGB Leds) ########################################################*/

#include "i2c.h"
#include "pca9555.h"

/*---------- Private define --------------------------------------------------*/

enum RGB_DEVICE_TO_PCA955_CHNL_Index { RED_Index = 0, GREEN_Index, BLUE_Index, RGB_DEVICE_TO_PCA955_CHNL_Index_NUM };

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

#define PCA9555_ADDR_A0 (0U)
#define PCA9555_ADDR_A1 (0U)
#define PCA9555_ADDR_A2 (0U)
#define PCA9555_ADDR    (PCA9555_ADDR_FIXED_PART | (PCA9555_ADDR_A0 << 2U) | (PCA9555_ADDR_A0 << 1U) | PCA9555_ADDR_A0)

struct PCA9555_Handle pca9555Handle = {.hi2c = &hi2c1, .addr = PCA9555_ADDR};

/*
 * @brief This data structure maps RGB devices color channels to the PCA955 channel
 */
static const uint8_t LED_RGB_Device_to_PCA9555_Chnl[LED_MONO_Device_NUM][RGB_DEVICE_TO_PCA955_CHNL_Index_NUM] = {
    [LED_RGB1]     = {[RED_Index] = 7, [GREEN_Index] = 6, [BLUE_Index] = 8},
    [LED_RGB2]     = {[RED_Index] = 10, [GREEN_Index] = 9, [BLUE_Index] = 11},
    [LED_RGB3]     = {[RED_Index] = 4, [GREEN_Index] = 3, [BLUE_Index] = 5},
    [LED_RGB_DASH] = {[RED_Index] = 1, [GREEN_Index] = 0, [BLUE_Index] = 2},

};

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/

void LED_RGB_setColor(enum LED_RGB_Device device, uint8_t red, uint8_t green, uint8_t blue) {
    assert_param(device != LED_RGB_Device_NUM);

    uint8_t PCA9555_chnls[RGB_DEVICE_TO_PCA955_CHNL_Index_NUM] = {};
    PCA9555_chnls[RED_Index]                                   = LED_RGB_Device_to_PCA9555_Chnl[device][RED_Index];
    PCA9555_chnls[GREEN_Index]                                 = LED_RGB_Device_to_PCA9555_Chnl[device][GREEN_Index];
    PCA9555_chnls[BLUE_Index]                                  = LED_RGB_Device_to_PCA9555_Chnl[device][BLUE_Index];

    uint8_t PCA9555_chnls_val[RGB_DEVICE_TO_PCA955_CHNL_Index_NUM] = {};
    PCA9555_chnls_val[RED_Index]                                   = red > 0 ? 1U : 0U;
    PCA9555_chnls_val[GREEN_Index]                                 = green > 0 ? 1U : 0U;
    PCA9555_chnls_val[BLUE_Index]                                  = blue > 0 ? 1U : 0U;

    // Invert reading if values on GPIO are inverted
    PCA9555_digitalWrites(&pca9555Handle, RGB_DEVICE_TO_PCA955_CHNL_Index_NUM, PCA9555_chnls, PCA9555_chnls_val);
}

/*---------- Private Functions -----------------------------------------------*/

/* Main CAN Bus Comunication #################################################*/

#include "button.h"  //TODO REMOVE
#include "can.h"
#include "conf.h"
#include "mcb.h"

/*---------- Private define --------------------------------------------------*/

/*---------- Private macro ---------------------------------------------------*/

/*---------- Private variables -----------------------------------------------*/

/*---------- Private function prototypes -------------------------------------*/

/*---------- Exported Variables ----------------------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/
void MCB_send_msg(uint32_t id) {
    uint8_t buffer[8] = {0};

    union {
        struct mcb_dash_hello_t hello;
        struct mcb_dash_hmi_devices_state_t hmi_devices_state;
    } msg = {};

    CAN_TxHeaderTypeDef tx_header = {.RTR = CAN_RTR_DATA, .IDE = CAN_ID_STD};

    tx_header.StdId = id;

    switch (id) {
        case MCB_DASH_HELLO_FRAME_ID:

            // clang-format off
            msg.hello.fw_major_version = mcb_dash_hello_fw_major_version_encode(VERSION_MAJOR);
            msg.hello.fw_minor_version = mcb_dash_hello_fw_minor_version_encode(VERSION_MINOR);
            msg.hello.fw_patch_version = mcb_dash_hello_fw_patch_version_encode(VERSION_PATCH);
            // clang-format on

            tx_header.DLC = mcb_dash_hello_pack(buffer, &msg.hello, 8);

            break;
        case MCB_DASH_HMI_DEVICES_STATE_FRAME_ID:

            // clang-format off
            msg.hmi_devices_state.btn_rtd_is_pressed = mcb_dash_hmi_devices_state_btn_rtd_is_pressed_encode(button_get(BUTTON_RTD)); // TODO change this button get
            msg.hmi_devices_state.btn_1_is_pressed = mcb_dash_hmi_devices_state_btn_1_is_pressed_encode(BTN_getStatus(BTN_Steering1));
            msg.hmi_devices_state.btn_2_is_pressed = mcb_dash_hmi_devices_state_btn_2_is_pressed_encode(BTN_getStatus(BTN_Steering2));
            msg.hmi_devices_state.btn_3_is_pressed = mcb_dash_hmi_devices_state_btn_3_is_pressed_encode(BTN_getStatus(BTN_Steering3));
            msg.hmi_devices_state.btn_4_is_pressed = mcb_dash_hmi_devices_state_btn_4_is_pressed_encode(BTN_getStatus(BTN_Steering4));
            msg.hmi_devices_state.rot_sw_1_state   = mcb_dash_hmi_devices_state_rot_sw_1_state_encode(ROT_SW_getState(ROT_SW_Device1));
            msg.hmi_devices_state.rot_sw_2_state   = mcb_dash_hmi_devices_state_rot_sw_2_state_encode(ROT_SW_getState(ROT_SW_Device2));
            // clang-format on

            tx_header.DLC =
                mcb_dash_hmi_devices_state_pack(buffer, &msg.hmi_devices_state, MCB_DASH_HMI_DEVICES_STATE_LENGTH);

            break;
        default:
            return;
    };
    // TODO: check for HAL_TIMEOUT
    CAN_send(&MCB_Handle, buffer, &tx_header);
}

/*---------- Private Functions -----------------------------------------------*/

/* LCD TFT SCREEN ILI9488 ####################################################*/

/*---------- Private define -----------------.--------------------------------*/

/*---------- Private macro ------------------.--------------------------------*/

/*---------- Private variables --------------.--------------------------------*/
enum LCD_TFT_Device{CS, RST, DC, LCD_TFT_Device_NUM};
static const struct GPIO_Tuple LCD_TFT_Device_to_GPIO_Tuple_map[LCD_TFT_Device_NUM] = {
    [CS]  = {.GPIO_Port = LCD_TFT_CS_GPIO_OUT_GPIO_Port, .GPIO_Pin = LCD_TFT_CS_GPIO_OUT_Pin},
    [DC]  = {.GPIO_Port = LCD_TFT_DC_GPIO_OUT_GPIO_Port, .GPIO_Pin = LCD_TFT_DC_GPIO_OUT_Pin},
    [RST] = {.GPIO_Port = LCD_TFT_RST_GPIO_OUT_GPIO_Port, .GPIO_Pin = LCD_TFT_RST_GPIO_OUT_Pin},
};

/*---------- Private function prototypes ----.--------------------------------*/

/*---------- Exported Variables -------------.--------------------------------*/

/*---------- Exported Functions ----------------------------------------------*/

/*---------- Private Functions -----------------------------------------------*/
