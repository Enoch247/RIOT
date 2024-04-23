/*
 * Copyright (c) 2024 Prime Controls, Inc.(R)
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup         cpu_stm32
 * @{
 *
 * @file
 * @brief           Shared SYSCLK clock definition for STM32H7 family
 *
 * @author          Joshua DeWeese <jdeweese@primecontrols.com>
*/

#ifndef CLK_H7_SYSCLK_H
#define CLK_H7_SYSCLK_H

#include "hse.h"
#include "hsi.h"
#include "pll.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief The total number of clock sources selected. */
#define _USE_CLOCK_COUNT \
      IS_ACTIVE(CONFIG_USE_CLOCK_HSI) \
    + IS_ACTIVE(CONFIG_USE_CLOCK_HSE) \
    + IS_ACTIVE(CONFIG_USE_CLOCK_CSI) \
    + IS_ACTIVE(CONFIG_USE_CLOCK_PLL)

/* defaults to PLL as clock source, if none is selected, but only if
   CONFIG_USE_CLOCK_PLL wasn't explicitly set to 0 */
#if !defined(CONFIG_USE_CLOCK_PLL) && (_USE_CLOCK_COUNT == 0)
    #define CONFIG_USE_CLOCK_PLL                    1
#endif

/* ensure multiple clock sources aren't selected */
static_assert(_USE_CLOCK_COUNT == 1);

/* ensure board has a HSE if requested to use it as source */
#if IS_ACTIVE(CONFIG_USE_CLOCK_HSE)
    static_assert(IS_ACTIVE(CONFIG_BOARD_HAS_HSE))
#endif

#if IS_ACTIVE(CONFIG_USE_CLOCK_HSI)
    #define CLOCK_SYSCLK                        (CLOCK_HSI)
#elif IS_ACTIVE(CONFIG_USE_CLOCK_HSE)
    #define CLOCK_SYSCLK                        (CLOCK_HSE)
#elif IS_ACTIVE(CONFIG_USE_CLOCK_CSI)
    #define CLOCK_SYSCLK                        (CLOCK_CSI)
#elif IS_ACTIVE(CONFIG_USE_CLOCK_PLL)
    #define CLOCK_SYSCLK                        (CLOCK_PLL_P)
#endif

#ifdef __cplusplus
}
#endif

#endif /* CLK_H7_SYSCLK_H */
/** @} */
