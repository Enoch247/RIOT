/*
 * Copyright (C) 2016 Freie Universität Berlin
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
 * @brief       Shared CPU specific function for the STM32 CPU family
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Joshua DeWeese <jdeweese@primecontrols.com>
 *
 * @}
 */

#include "macros/utils.h"
#include "periph_conf.h"
#include "periph_cpu.h"

#define ENABLE_DEBUG 0
#include "debug.h"

uint32_t periph_apb_clk(bus_t bus)
{
    switch (bus) {
        case APB1:
            return CLOCK_APB1;

#       ifdef CLOCK_APB2
        case APB2:
            return CLOCK_APB2;
#       endif

#       ifdef CLOCK_APB12
        case APB12:
            return CLOCK_APB12;
#       endif

#       ifdef CLOCK_APB3
/*        case APB3:*/
/*            return CLOCK_APB3;*/
#       endif

#       ifdef CLOCK_APB4
/*        case APB4:*/
/*            return CLOCK_APB4;*/
#       endif

        default:
            break;
    }

    return 0;
}

uint32_t periph_timer_clk(bus_t bus)
{
    const bool timpre =
        #if defined(RCC_DCKCFGR_TIMPRE)
        RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE
        #elif defined(RCC_DCKCFGR1_TIMPRE)
        RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE
        #elif defined(RCC_CFGR_TIMPRE)
        RCC->CFGR & RCC_CFGR_TIMPRE
        #else
        false
        #endif
        ;

    const unsigned multiplier_max = (timpre) ? 2 : 4;
    const unsigned periph_bus_divider = CLOCK_AHB / periph_apb_clk(bus);
    const unsigned multiplier = MIN(periph_bus_divider, multiplier_max);

    return periph_apb_clk(bus) * multiplier;
}

void periph_clk_en(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G4) || defined(CPU_FAM_STM32L5) || \
    defined(CPU_FAM_STM32U5) || defined(CPU_FAM_STM32WL)
            RCC->APB1ENR1 |= mask;
#elif defined(CPU_FAM_STM32G0)
            RCC->APBENR1 |= mask;
#elif defined(CPU_FAM_STM32MP1)
            RCC->MC_APB1ENSETR |= mask;
#else
            RCC->APB1LENR |= mask;
#endif
            break;
#if !defined(CPU_FAM_STM32G0)
        case APB2:
#if defined(CPU_FAM_STM32MP1)
            RCC->MC_APB2ENSETR |= mask;
#else
            RCC->APB2ENR |= mask;
#endif
            break;
#endif
#if defined(CPU_FAM_STM32WL) || defined(CPU_FAM_STM32U5)
        case APB3:
            RCC->APB3ENR |= mask;
            break;
#endif
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G4) || defined(CPU_FAM_STM32L5) || \
    defined(CPU_FAM_STM32U5) || defined(CPU_FAM_STM32WL)
        case APB12:
            RCC->APB1ENR2 |= mask;
            break;
#elif defined(CPU_FAM_STM32G0)
        case APB12:
            RCC->APBENR2 |= mask;
            break;
#endif
#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32G0)
        case AHB:
            RCC->AHBENR |= mask;
            break;
        case IOP:
            RCC->IOPENR |= mask;
            break;
#elif defined(CPU_FAM_STM32L1) || defined(CPU_FAM_STM32F1) || \
      defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F3)
        case AHB:
            RCC->AHBENR |= mask;
            break;
#elif defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
      defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32F7) || \
      defined(CPU_FAM_STM32WB) || defined(CPU_FAM_STM32G4) || \
      defined(CPU_FAM_STM32L5) || defined(CPU_FAM_STM32U5) || \
      defined(CPU_FAM_STM32WL)
        case AHB1:
            RCC->AHB1ENR |= mask;
            break;
/* STM32F410 RCC doesn't provide AHB2 and AHB3 */
#if !defined(CPU_LINE_STM32F410Rx)
        case AHB2:
#if defined(CPU_FAM_STM32U5)
            RCC->AHB2ENR1 |= mask;
#else
            RCC->AHB2ENR |= mask;
#endif
            break;
#if defined(CPU_FAM_STM32U5)
        case AHB22:
            RCC->AHB2ENR2 |= mask;
            break;
#endif
        case AHB3:
            RCC->AHB3ENR |= mask;
            break;
#endif
#endif
#if defined(CPU_FAM_STM32MP1) || defined(CPU_FAM_STM32H7)
        case AHB4:
            RCC->AHB4ENR |= mask;
            break;
#endif
        default: //TODO: rm
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
    /* stm32xx-errata: Delay after a RCC peripheral clock enable */
    __DSB();
}

#if 0
static const periph_t* _periph(const void *dev)
{
    #ifdef TIM1
    static_assert(ARRAY_SIZE(_timers) >= 1);
    if (TIM1 == dev)
    {
        return &_timers[0];
    }
    #endif

    #ifdef TIM2
    static_assert(ARRAY_SIZE(_timers) >= 2);
    if (TIM2 == dev)
    {
        return &_timers[1];
    }
    #endif

    return NULL;
}

void periph_clk_en2(const void *dev)
{
    const periph_t *periph = _periph(dev);

    ...
}
#endif

void periph_clk_en2(const periph_t *periph)
{
    assert(periph);

    const int irq_state = irq_disable();
    *(periph->en_reg) |= periph->en_mask;
    irq_restore(irq_state);

    /* stm32xx-errata: Delay after a RCC peripheral clock enable */
    __DSB();
}

void periph_clk_dis2(const periph_t *periph)
{
    assert(periph);

    const int irq_state = irq_disable();
    *(periph->en_reg) &= ~(periph->en_mask);
    irq_restore(irq_state);
}

void periph_clk_dis(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G4) || defined(CPU_FAM_STM32L5) || \
    defined(CPU_FAM_STM32U5) || defined(CPU_FAM_STM32WL)
            RCC->APB1ENR1 &= ~(mask);
#elif defined(CPU_FAM_STM32G0)
            RCC->APBENR1 &= ~(mask);
#elif defined(CPU_FAM_STM32MP1)
            /* Write 1 to clear */
            RCC->MC_APB1ENCLRR |= mask;
#else
            RCC->APB1LENR &= ~(mask);
#endif
            break;
#if !defined(CPU_FAM_STM32G0)
        case APB2:
#if defined(CPU_FAM_STM32MP1)
            RCC->MC_APB2ENCLRR |= mask;
#else
            RCC->APB2ENR &= ~(mask);
#endif
            break;
#endif
#if defined(CPU_FAM_STM32WL) || defined(CPU_FAM_STM32U5)
        case APB3:
            RCC->APB3ENR &= ~(mask);
            break;
#endif
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G4) || defined(CPU_FAM_STM32L5) || \
    defined(CPU_FAM_STM32U5) || defined(CPU_FAM_STM32WL)
        case APB12:
            RCC->APB1ENR2 &= ~(mask);
            break;
#elif defined(CPU_FAM_STM32G0)
        case APB12:
            RCC->APBENR2 &= ~(mask);
            break;
#endif
#if defined(CPU_FAM_STM32L0)
        case AHB:
            RCC->AHBENR &= ~(mask);
            break;
        case IOP:
            RCC->IOPENR &= ~(mask);
            break;
#elif defined(CPU_FAM_STM32L1) || defined(CPU_FAM_STM32F1) || \
      defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F3)
        case AHB:
            RCC->AHBENR &= ~(mask);
            break;
#elif defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
      defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32F7) || \
      defined(CPU_FAM_STM32WB) || defined(CPU_FAM_STM32G4) || \
      defined(CPU_FAM_STM32L5) || defined(CPU_FAM_STM32U5) || \
      defined(CPU_FAM_STM32WL)
        case AHB1:
            RCC->AHB1ENR &= ~(mask);
            break;
/* STM32F410 RCC doesn't provide AHB2 and AHB3 */
#if !defined(CPU_LINE_STM32F410Rx)
        case AHB2:
#if defined(CPU_FAM_STM32U5)
            RCC->AHB2ENR1 &= ~(mask);
#else
            RCC->AHB2ENR &= ~(mask);
#endif
            break;
#if defined(CPU_FAM_STM32U5)
        case AHB22:
            RCC->AHB2ENR2 &= ~(mask);
            break;
#endif
        case AHB3:
            RCC->AHB3ENR &= ~(mask);
            break;
#endif
#if defined(CPU_FAM_STM32WB)
        case AHB4:
            RCC->AHB3ENR &= ~(mask);
            break;
#endif
#endif
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}

#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32G4) || \
    defined(CPU_FAM_STM32L5) || defined(CPU_FAM_STM32U5)
void periph_lpclk_en(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
            RCC->APB1SMENR1 |= mask;
            break;
        case APB12:
            RCC->APB1SMENR2 |= mask;
            break;
        case APB2:
            RCC->APB2SMENR |= mask;
            break;
        case AHB1:
            RCC->AHB1SMENR |= mask;
            break;
        case AHB2:
#if defined(CPU_FAM_STM32U5)
            RCC->AHB2SMENR1 |= mask;
#else
            RCC->AHB2SMENR |= mask;
#endif
            break;
#if defined(CPU_FAM_STM32U5)
        case AHB22:
            RCC->AHB2SMENR2 |= mask;
            break;
#endif
        case AHB3:
            RCC->AHB3SMENR |= mask;
            break;
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}

void periph_lpclk_dis(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
            RCC->APB1SMENR1 &= ~(mask);
            break;
        case APB12:
            RCC->APB1SMENR2 &= ~(mask);
            break;
        case APB2:
            RCC->APB2SMENR &= ~(mask);
            break;
        case AHB1:
            RCC->AHB1SMENR &= ~(mask);
            break;
        case AHB2:
#if defined(CPU_FAM_STM32U5)
            RCC->AHB2SMENR1 &= ~(mask);
#else
            RCC->AHB2SMENR &= ~(mask);
#endif
            break;
#if defined(CPU_FAM_STM32U5)
        case AHB22:
            RCC->AHB2SMENR2 &= ~(mask);
            break;
#endif
        case AHB3:
            RCC->AHB3SMENR &= ~(mask);
            break;
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}
#elif defined(CPU_FAM_STM32F2) || \
      defined(CPU_FAM_STM32F4) || \
      defined(CPU_FAM_STM32F7)
void periph_lpclk_en(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
            RCC->APB1LPENR |= mask;
            break;
        case APB2:
            RCC->APB2LPENR |= mask;
            break;
        case AHB1:
            RCC->AHB1LPENR |= mask;
            break;
/* STM32F410 RCC doesn't provide AHB2 and AHB3 */
#if !defined(CPU_LINE_STM32F410Rx)
        case AHB2:
            RCC->AHB2LPENR |= mask;
            break;
        case AHB3:
            RCC->AHB3LPENR |= mask;
            break;
#endif
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}

void periph_lpclk_dis(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
            RCC->APB1LPENR &= ~(mask);
            break;
        case APB2:
            RCC->APB2LPENR &= ~(mask);
            break;
        case AHB1:
            RCC->AHB1LPENR &= ~(mask);
            break;
/* STM32F410 RCC doesn't provide AHB2 and AHB3 */
#if !defined(CPU_LINE_STM32F410Rx)
        case AHB2:
            RCC->AHB2LPENR &= ~(mask);
            break;
        case AHB3:
            RCC->AHB3LPENR &= ~(mask);
            break;
#endif
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}
#endif
