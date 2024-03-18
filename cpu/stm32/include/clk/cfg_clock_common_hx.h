/*
 * Copyright (c) 2024 Prime Controls, Inc.(R)
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32
 * @{
 *
 * @file
 * @brief       Base STM32Hx clock configuration
 *
 * @author      Joshua DeWeese <jdeweese@primecontrols.com>
 */

#ifndef CLK_CFG_CLOCK_COMMON_HX_H
#define CLK_CFG_CLOCK_COMMON_HX_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Clock common configuration (H7)
 * @{
 */

/** @brief The total number of clock sources selected. */
#define _USE_CLOCK_NUMOF \
      IS_ACTIVE(CONFIG_USE_CLOCK_HSI) \
    + IS_ACTIVE(CONFIG_USE_CLOCK_HSE) \
    + IS_ACTIVE(CONFIG_USE_CLOCK_CSI) \
    + IS_ACTIVE(CONFIG_USE_CLOCK_PLL)

/* defaults to PLL as clock source */
#if (_USE_CLOCK_NUMOF == 0)
    #define CONFIG_USE_CLOCK_PLL 1                  1
#elif (_USE_CLOCK_NUMOF > 1)
    #error "Multiple clock sources selected"
#endif

/** @brief The High Speed External clock frequency. */
#ifndef CONFIG_CLOCK_HSE
    #define CONFIG_CLOCK_HSE                        MHZ(8)
#endif

/** @brief The High Speed Internal clock frequency. */
#ifndef CONFIG_CLOCK_HSI
    #if defined(CPU_LINE_STM32H723xx)
        #define CONFIG_CLOCK_HSI                    MHZ(64)
    #endif
#endif

/** @brief The High Speed Internal 48 MHz clock frequency. */
#ifndef CONFIG_CLOCK_HSI48
    #define CONFIG_CLOCK_HSI48                      MHZ(48)
#endif

/** @brief The Low Power Internal clock frequency. */
#ifndef CONFIG_CLOCK_CSI
    #define CONFIG_CLOCK_CSI                        MHZ(4)
#endif

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CLK_CFG_CLOCK_COMMON_HX_H */
/** @} */
