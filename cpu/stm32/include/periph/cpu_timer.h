/*
 * Copyright (C) 2016 Freie Universität Berlin
 *               2017 OTA keys S.A.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_stm32
 * @{
 *
 * @file
 * @brief           Timer CPU specific definitions for the STM32 family
 *
 * @author          Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author          Vincent Dupont <vincent@otakeys.com>
 */

#ifndef PERIPH_CPU_TIMER_H
#define PERIPH_CPU_TIMER_H

#include <stdint.h>

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   All STM timers have at most 4 capture-compare channels
 */
#define TIMER_CHANNEL_NUMOF (4U)

/**
 * @brief   The driver provides a relative set function
 */
#define PERIPH_TIMER_PROVIDES_SET

/**
 * @brief   Define a macro for accessing a timer channel
 */
#define TIM_CHAN(tim, chan) *(&dev(tim)->CCR1 + chan)

static const periph_t periph_timer1 = {
    #if   defined(RCC_APB1ENR_TIM1EN)
    .en_reg = &RCC->APB1ENR,
    .en_mask = RCC_APB1ENR_TIM1EN,
    #elif defined(RCC_APB1ENR1_TIM1EN)
    .en_reg = &RCC->APB1ENR1,
    .en_mask = RCC_APB1ENR1_TIM1EN,
    #endif
};

static const periph_t periph_timer2 = {
    #if   defined(RCC_APB1ENR_TIM2EN)
    .en_reg = &RCC->APB1ENR,
    .en_mask = RCC_APB1ENR_TIM2EN,
    #elif defined(RCC_APB1ENR1_TIM2EN)
    .en_reg = &RCC->APB1ENR1,
    .en_mask = RCC_APB1ENR1_TIM2EN,
    #elif defined(RCC_APB1LENR_TIM2EN)
    .en_reg = &RCC->APB1LENR,
    .en_mask = RCC_APB1LENR_TIM2EN,
    #elif defined(RCC_MC_APB1ENSETR_TIM2EN)
    #error "CPU_FAM_STM32MP1 not implmented yet"
    #endif
};

/**
 * @brief   Timer configuration
 */
typedef struct {
    TIM_TypeDef *dev;       /**< timer device */
    const periph_t *rcc_dev;
    uint32_t max;           /**< maximum value to count to (16/32 bit) */
    uint32_t rcc_mask;      /**< corresponding bit in the RCC register */
    uint8_t bus;            /**< APBx bus the timer is clock from */
    uint8_t irqn;           /**< global IRQ channel */
    uint8_t channel_numof;  /**< number of channels, 0 is alias for
                                 @ref TIMER_CHANNEL_NUMOF */
} timer_conf_t;

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CPU_TIMER_H */
/** @} */
