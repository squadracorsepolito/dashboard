/**
 * @file button.h
 * @author Matteo Bonora (matteo.bonora@studenti.polito.it)
 * @brief Sample and debounce the input buttons
 * @date 2022-04-30
 */

#pragma once

#include <inttypes.h>
#include <stdbool.h>

#define BUTTON_SAMPLE_TIME_100us 300
#define BUTTON_SHORT_PRESS_TIME_100us 1000
#define BUTTON_LONG_PRESS_TIME_100us 8000

typedef enum
{
    BUTTON_RTD=0,
    BUTTON_COUNT
} button;

// Phisical condition of the button
typedef enum
{
    BUTTON_DOWN = 0,
    BUTTON_UP = 1,
} button_condition;

// Logical state of the button
typedef enum
{
    BUTTON_PRESSED = 0,
    BUTTON_RELEASED = 1,
    BUTTON_LONGPRESS = 2
} button_state;

// Callback def
typedef void (*func_type)(void);

/**
 * @brief Sample all the buttons
 */
void button_sample();

/**
 * @brief Returns the de-bounced logical state of the button
 *
 * @param btn The button to check
 * @return true if button is PRESSED; false if button is RELEASED
 */
bool button_get(button btn);

/**
 * @brief Sets the short press callback for a button
 *
 * @param btn The button to set the callback to
 * @param function The callback function
 */
void button_set_shortpress_callback(button btn, func_type function);

/**
 * @brief Sets the long press callback for a button
 *
 * @param btn The button to set the callback to
 * @param function The callback function
 */
void button_set_longpress_callback(button btn, func_type function);
