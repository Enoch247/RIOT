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
 * @brief           Shared ABP1 clock definition for STM32H7 family TODO
 *
 * @author          Joshua DeWeese <jdeweese@primecontrols.com>
*/

#ifndef CLK_H7_APB1_H //TODO
#define CLK_H7_APB1_H

#include "clk/h7/ahb.h"
#include "cpu_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(APB1PERIPH_BASE)
    #define HAS_APB1                1
#endif

#if defined(APB2PERIPH_BASE)
    #define HAS_APB2                1
#endif

#if defined(RCC_APB3ENR_WWDG1EN)
    #define HAS_APB3                1
#endif

#if defined(RCC_APB4ENR_SYSCFGEN)
    #define HAS_APB4                1
#endif

//TODO: ensure if CONFIG_CLOCK_APBx_DIV is not defined and the CPU has a divider, we get an error
// - do this in cpu_common.c

#if HAS_APB1 && defined(CONFIG_CLOCK_APB1_DIV)
    #define CLOCK_APB1                  (CLOCK_AHB / CONFIG_CLOCK_APB1_DIV)
#else
    #define CLOCK_APB1                  (CLOCK_AHB)
#endif

#if HAS_APB2 && defined(CONFIG_CLOCK_APB2_DIV)
    #define CLOCK_APB2                  (CLOCK_AHB / CONFIG_CLOCK_APB2_DIV)
#else
    #define CLOCK_APB2                  (CLOCK_AHB)
#endif

#if HAS_APB3 && defined(CONFIG_CLOCK_APB3_DIV)
    #define CLOCK_APB3                  (CLOCK_AHB / CONFIG_CLOCK_APB3_DIV)
#else
    #define CLOCK_APB3                  (CLOCK_AHB)
#endif

#if HAS_APB4 && defined(CONFIG_CLOCK_APB4_DIV)
    #define CLOCK_APB4                  (CLOCK_AHB / CONFIG_CLOCK_APB4_DIV)
#else
    #define CLOCK_APB4                  (CLOCK_AHB)
#endif

#ifdef __cplusplus
}
#endif

#endif /* CLK_H7_APB1_H */
/** @} */
