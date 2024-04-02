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
 * @brief           Shared HSE clock definition for STM32H7 family
 *
 * @author          Joshua DeWeese <jdeweese@primecontrols.com>
*/

#ifndef CLK_H7_HSE_H
#define CLK_H7_HSE_H

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(CONFIG_CLOCK_HSE_MAX)
    #error "HSE max undefined"
#endif

#if !defined(CONFIG_CLOCK_HSE_MIN)
    #error "HSE min undefined"
#endif

#if CONFIG_BOARD_HAS_HSE && !defined(CONFIG_CLOCK_HSE)
    #error "HSE speed is undefined"
#endif

/* ensure HSE is in range */
#ifdef CONFIG_CLOCK_HSE
    static_assert(CONFIG_CLOCK_HSE > CONFIG_CLOCK_HSE_MIN);
    static_assert(CONFIG_CLOCK_HSE < CONFIG_CLOCK_HSE_MAX);
#endif

#define CLOCK_HSE               CONFIG_CLOCK_HSE

#ifdef __cplusplus
}
#endif

#endif /* CLK_H7_HSE_H */
/** @} */
