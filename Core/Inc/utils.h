/**
 * @file utils.h
 * @author Matteo Bonora (matteo.bonora@studenti.polito.it)
 * @brief Small collection of useful functions
 * @date 2022-05-26
 */

#pragma once

#include "main.h"
#include <stdbool.h>

void LedBlinking(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t *last, uint32_t period);
bool delay_fun(uint32_t *delay_100us_last, uint32_t delay_100us);
uint32_t ReturnTime_100us(void);
