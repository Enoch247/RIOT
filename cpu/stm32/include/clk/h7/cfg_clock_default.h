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
 * @brief           Main header for STM32H7 clock configuration
 *
 * @author          Joshua DeWeese <jdeweese@primecontrols.com>
*/

#ifndef CFG_CLOCK_DEFAULT_H
#define CFG_CLOCK_DEFAULT_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_CLOCK_HSE_MAX
    #if defined(CPU_LINE_STM32H723xx)
        #define CONFIG_CLOCK_HSE_MAX                MHZ(50)
    #endif
#endif

#ifndef CONFIG_CLOCK_HSE_MIN
    #if defined(CPU_LINE_STM32H723xx)
        #define CONFIG_CLOCK_HSE_MIN                MHZ(4)
    #endif
#endif

#include "clk/h7/hse.h"

/** @brief The HSI RC oscillator frequency (before any dividers). */
#ifndef CONFIG_CLOCK_HSI
    #if defined(CPU_LINE_STM32H723xx)
        #define CONFIG_CLOCK_HSI                    MHZ(64)
    #endif
#endif

/** @brief The HSI RC divider. */
#ifndef CONFIG_CLOCK_HSI_DIV
    #if defined(CPU_LINE_STM32H723xx)
        #define CONFIG_CLOCK_HSI_DIV                1
    #endif
#endif

#include "clk/h7/hsi.h"

/** @brief The CSI RC oscillator frequency (before any dividers). */
#ifndef CONFIG_CLOCK_CSI
    #if defined(CPU_LINE_STM32H723xx)
        #define CONFIG_CLOCK_CSI                    MHZ(4)
    #endif
#endif

#include "clk/h7/csi.h"








#if CLOCK_HSE == MHZ(8)
    #define CONFIG_CLOCK_PLL1_M              4  //(3)   //(4)
    #define CONFIG_CLOCK_PLL1_N              275//(117) //(275)
    #define CONFIG_CLOCK_PLL1_P              2//(1)
    #define CONFIG_CLOCK_PLL1_Q              4
    #define CONFIG_CLOCK_PLL1_R              2
#elif CLOCK_HSI == MHZ(64)
    #define CONFIG_CLOCK_PLL1_M              32
    #define CONFIG_CLOCK_PLL1_N              275
    #define CONFIG_CLOCK_PLL1_P              1
    #define CONFIG_CLOCK_PLL1_Q              4
    #define CONFIG_CLOCK_PLL1_R              2
#else
    #error "PLL source does not match a known configuration"
#endif

#include "clk/h7/pll1.h"



#ifndef CONFIG_CLOCK_CORECLOCK_DIV
    #define CONFIG_CLOCK_CORECLOCK_DIV      1
#endif

#include "clk/h7/coreclock.h"



#ifndef CONFIG_CLOCK_AHB_DIV
    #define CONFIG_CLOCK_AHB_DIV            2
#endif

#include "clk/h7/ahb.h"



#ifndef CONFIG_CLOCK_APB1_DIV
    #define CONFIG_CLOCK_APB1_DIV           2
#endif

#ifndef CONFIG_CLOCK_APB2_DIV
    #define CONFIG_CLOCK_APB2_DIV           2
#endif

#ifndef CONFIG_CLOCK_APB3_DIV
    #define CONFIG_CLOCK_APB3_DIV           2
#endif

#ifndef CONFIG_CLOCK_APB4_DIV
    #define CONFIG_CLOCK_APB4_DIV           2
#endif

#include "clk/h7/apb1.h"



#ifdef __cplusplus
}
#endif

#endif /* CFG_CLOCK_DEFAULT_H */
/** @} */
