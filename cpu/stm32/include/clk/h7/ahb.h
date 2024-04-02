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

#ifndef CLK_H7_AHB_H //TODO
#define CLK_H7_AHB_H

#include "coreclock.h"
#include "sysclk.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(AHB1PERIPH_BASE)
    #define HAS_AHB1                1
#endif

#if defined(AHB2PERIPH_BASE)
    #define HAS_AHB2                1
#endif

/*#if defined(RCC_APB3ENR_WWDG1EN)*/
/*    #define HAS_APB3                1*/
/*#endif*/

/*#if defined(RCC_APB4ENR_SYSCFGEN)*/
/*    #define HAS_APB4                1*/
/*#endif*/

/* some CPUs are clocked downstream of the AHB prescaler, other are upstream */
#if 1
    #define CLOCK_AHB_SRC               (CLOCK_CORECLOCK)
#else
    #define CLOCK_AHB_SRC               (CLOCK_SYSCLK)
#endif

#define CLOCK_AHB                       (CLOCK_AHB_SRC / CONFIG_CLOCK_AHB_DIV)

#ifdef __cplusplus
}
#endif

#endif /* CLK_H7_AHB_H */
/** @} */
