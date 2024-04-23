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
 * @brief           Shared CPU clock definition for STM32H7 family
 *
 * @author          Joshua DeWeese <jdeweese@primecontrols.com>
*/

#ifndef CLK_H7_CORECLOCK_H
#define CLK_H7_CORECLOCK_H

#include "ahb.h"
#include "sysclk.h"

#ifdef __cplusplus
extern "C" {
#endif

/* some CPUs are clocked downstream of the AHB prescaler, other are upstream */
#if 1
    #define CLOCK_CORECLOCK_SRC                 (CLOCK_SYSCLK)
#else
    #define CLOCK_CORECLOCK_SRC                 (CLOCK_AHB)
#endif

#if defined(CONFIG_CLOCK_CORECLOCK_DIV)
    #define CLOCK_CORECLOCK                     (CLOCK_CORECLOCK_SRC \
                                                / CONFIG_CLOCK_CORECLOCK_DIV)
#else
    #define CLOCK_CORECLOCK                     (CLOCK_CORECLOCK_SRC)
#endif

#ifdef __cplusplus
}
#endif

#endif /* CLK_H7_CORECLOCK_H */
/** @} */
