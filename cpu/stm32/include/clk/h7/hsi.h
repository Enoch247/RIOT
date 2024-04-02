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
 * @brief           Shared HSI clock definition for STM32H7 family
 *
 * @author          Joshua DeWeese <jdeweese@primecontrols.com>
*/

#ifndef CLK_H7_HSI_H
#define CLK_H7_HSI_H

#include "cpu_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(RCC_CR_HSIDIV)
    #define HAS_CLOCK_HSIDIV         1
#endif

#if !defined(CONFIG_CLOCK_HSI)
    #error "HSI speed is undefined"
#endif

#if HAS_CLOCK_HSIDIV && !defined(CONFIG_CLOCK_HSI_DIV)
    #error "HSI clock divider is undefined"
#endif

#if !HAS_CLOCK_HSIDIV && defined(CONFIG_CLOCK_HSI_DIV)
    #error "HSI clock divider not supported"
#endif

/** @brief The High Speed Internal clock frequency. */
#if HAS_CLOCK_HSIDIV
    #define CLOCK_HSI               (CONFIG_CLOCK_HSI / CONFIG_CLOCK_HSI_DIV)
#else
    #define CLOCK_HSI               (CONFIG_CLOCK_HSI)
#endif

#ifdef __cplusplus
}
#endif

#endif /* CLK_H7_HSI_H */
/** @} */
