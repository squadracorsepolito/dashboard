/**
 * @file wdg.h
 * @author Matteo Bonora (matteo.bonora@studenti.polito.it)
 * @brief Functions to check the communications of other boards on the CAN bus
 * @date 2022-06-15
 */

#pragma once
#include <inttypes.h>

/**
 * @brief Watchdog board indexes
 */
typedef enum
{
    WDG_BOARD_DSPACE = 0,
    WDG_BOARD_TLB,
    WDG_BOARD_SENS_FRONT,
    WDG_NUM_BOARDS
} wdg_boards;

/**
 * @brief Timeout values for each board
 */
static uint32_t wdg_timeouts_100us[WDG_NUM_BOARDS] = {
    [WDG_BOARD_DSPACE] = 100000, // Wait for dSpace to boot
    [WDG_BOARD_TLB] = 4800,
    [WDG_BOARD_SENS_FRONT] = 4800};

/**
 * @brief Resets the timer of a board to the given timestamp
 *
 * @param board The board to reset
 * @param timestamp_100us The timestamp to set
 */
void wdg_reset(wdg_boards board, uint32_t timestamp_100us);

/**
 * @brief Checks for timeouts
 *
 * @return uint8_t bitset that holds the timeout state for each board
 */
uint8_t wdg_check();
