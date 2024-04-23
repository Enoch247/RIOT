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
 * @brief           Shared CSI clock definition for STM32H7 family TODO
 *
 * @author          Joshua DeWeese <jdeweese@primecontrols.com>
*/

#ifndef CLK_H7_CSI_H //TODO
#define CLK_H7_CSI_H

#include "cpu_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/*#if defined(RCC_CR_CSION)*/
/*    #define HAS_CLOCK_CSI           1*/
/*#endif*/

/*#if HAS_CLOCK_CSI && !defined(CONFIG_CLOCK_CSI)*/
/*    #error "CSI speed is undefined"*/
/*#endif*/

/*#if HAS_CLOCK_CSI*/
/*    #define CLOCK_CSI               CONFIG_CLOCK_CSI*/
/*#endif*/

#ifdef __cplusplus
}
#endif

#endif /* CLK_H7_CSI_H */
/** @} */
