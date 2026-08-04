/**
 * @file mission.h
 * @author Ethan Paonessa (ethan.paonessa@squadracorsepolito.com)
 * @brief Handles the autonomous mission selection and the ASSI lights.
 * @date 2026-04-28
 */

#pragma once

#include "main.h"
#include <inttypes.h>
#include <stdbool.h>

#include "dashboard.h"

#define NUM_MISSIONS 8

typedef enum
{
    MISSION_NO,
    ACCEL,
    SKIDPAD,
    TRACKDRIVE,
    EBSTEST,
    INSPECT,
    AUTOX,
    MANUAL,
} mission_t;

typedef enum
{
    AS_ERR,
    AS_OFF,
    AS_READY,
    AS_DRIVING,
    AS_EMERGENCY,
    AS_FINISHED,
} assi_t;

typedef enum
{
    ERR,
    OLLIO,
    STANLIO,
    BOTH,
    NONE,
} asb_ebs_t;

/**
 * @brief Setups callback functions for the mission select button
 */
void mission_setup();

/**
 * @return true mission has been confirmed
 * @return false mission is not confirmed
 */
bool mission_is_confirmed();

/**
 * @return mission_t the currently selected mission
 */
mission_t mission_get();

void mission_confirm();
void mission_select_next();

/**
 * @brief If the mission is not confirmed, set the given mission
 *
 * @param mission New mission
 */
void mission_set(mission_t mission);

/**
 * @brief Sets the AMI according to current mission and confirmation
 */
void mission_run();

char* mission_convert(uint8_t mission);