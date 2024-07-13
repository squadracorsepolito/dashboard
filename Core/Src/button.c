#include "button.h"
#include "main.h"
#include "utils.h"

// Map GPIOs to button indexes
const GPIO_TypeDef *button_gpio[BUTTON_COUNT] = {
    [BUTTON_RTD] = nRTD_BTN_IN_GPIO_IN_GPIO_Port};
const uint16_t button_pin[BUTTON_COUNT] = {
    [BUTTON_RTD] = nRTD_BTN_IN_GPIO_IN_Pin};

// State keeping for each button
button_state state[BUTTON_COUNT] = {BUTTON_RELEASED};

// The last time each button changed state
uint32_t down_time[BUTTON_COUNT] = {0};
uint32_t up_time[BUTTON_COUNT] = {0};

// Callback functions to call on short and long-press events
func_type shortpress_callback[BUTTON_COUNT] = {NULL};
func_type longpress_callback[BUTTON_COUNT] = {NULL};

void button_sample()
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, BUTTON_SAMPLE_TIME_100us))
    {
        for (uint8_t i = 0; i < BUTTON_COUNT; i++)
        {
            uint8_t btn_val = (uint8_t)HAL_GPIO_ReadPin((GPIO_TypeDef *)button_gpio[i], button_pin[i]);
            switch (state[i])
            {
            case BUTTON_RELEASED:
                if (btn_val == BUTTON_DOWN)
                {
                    // If button was up and now is down
                    state[i] = BUTTON_PRESSED;
                    down_time[i] = ReturnTime_100us();
                }
                break;
            case BUTTON_PRESSED:
                if (btn_val == BUTTON_UP)
                {
                    // If button was down and now is up
                    if (ReturnTime_100us() - down_time[i] >= BUTTON_SHORT_PRESS_TIME_100us)
                    {
                        if (shortpress_callback[i])
                            shortpress_callback[i]();
                    }
                    state[i] = BUTTON_RELEASED;
                }
                else if (ReturnTime_100us() - down_time[i] > BUTTON_LONG_PRESS_TIME_100us)
                {
                    // If button has been down for a while
                    if (longpress_callback[i])
                    {
                        longpress_callback[i]();
                    }
                    state[i] = BUTTON_LONGPRESS;
                }
                break;
            case BUTTON_LONGPRESS:
                if (btn_val == BUTTON_UP)
                {
                    // If button has been down for a while but now is up
                    state[i] = BUTTON_RELEASED;
                }
                break;
            }
        }
    }
}

bool button_get(button btn)
{
    return (state[btn] != BUTTON_RELEASED) && (ReturnTime_100us() - down_time[btn] >= BUTTON_SAMPLE_TIME_100us);
}

void button_set_shortpress_callback(button btn, func_type function)
{
    shortpress_callback[btn] = function;
}

void button_set_longpress_callback(button btn, func_type function)
{
    longpress_callback[btn] = function;
}

