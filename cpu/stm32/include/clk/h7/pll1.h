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
 * @brief           Shared PLL1 definition for STM32H7 family
 *
 * @author          Joshua DeWeese <jdeweese@primecontrols.com>
*/

#ifndef CLK_H7_PLL1_H
#define CLK_H7_PLL1_H

#include "clk/h7/hse.h" //TODO: drop the clk/h7/ dir prefix?
#include "clk/h7/hsi.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief The PLL input source. */
#if CLOCK_HSE
    #define CLOCK_PLL_SRC           CLOCK_HSE
#else
    #define CLOCK_PLL_SRC           CLOCK_HSI
#endif

#if CONFIG_CLOCK_PLL1_P

    #define CLOCK_PLL1_P            (CLOCK_PLL_SRC / CONFIG_CLOCK_PLL1_M * \
                                     CONFIG_CLOCK_PLL1_N / CONFIG_CLOCK_PLL1_P)

    #define CLOCK_PLL_P             CLOCK_PLL1_P
#endif

#if CONFIG_CLOCK_PLL1_Q

    #define CLOCK_PLL1_Q            (CLOCK_PLL_SRC / CONFIG_CLOCK_PLL1_M * \
                                     CONFIG_CLOCK_PLL1_N / CONFIG_CLOCK_PLL1_Q)

    #define CLOCK_PLL_Q             CLOCK_PLL1_Q
#endif

#if CONFIG_CLOCK_PLL1_R

    #define CLOCK_PLL1_R            (CLOCK_PLL_SRC / CONFIG_CLOCK_PLL1_M * \
                                     CONFIG_CLOCK_PLL1_N / CONFIG_CLOCK_PLL1_R)

    #define CLOCK_PLL_R             CLOCK_PLL1_R
#endif

#ifdef __cplusplus
}
#endif

#endif /* CLK_H7_PLL1_H */
/** @} */
