/*
 * Copyright (C) 2021 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Blinky application
 *
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 *
 * @}
 */

#include <stdio.h>

#include "clk.h"
#include "board.h"
#include "periph_conf.h"
#include "timex.h"
#include "ztimer.h"

static void delay(void)
{
    if (IS_USED(MODULE_ZTIMER)) {
        ztimer_sleep(ZTIMER_USEC, 1 * US_PER_SEC);
    }
    else {
        /*
         * As fallback for freshly ported boards with no timer drivers written
         * yet, we just use the CPU to delay execution and assume that roughly
         * 20 CPU cycles are spend per loop iteration.
         *
         * Note that the volatile qualifier disables compiler optimizations for
         * all accesses to the counter variable. Without volatile, modern
         * compilers would detect that the loop is only wasting CPU cycles and
         * optimize it out - but here the wasting of CPU cycles is desired.
         */
        uint32_t loops = coreclk() / 20;
        //uint32_t loops = 1;
        for (volatile uint32_t i = 0; i < loops; i++) { }
    }
}

int main(void)
{
    //gpio_init(LED0_PORT_NUM, LED0_PIN_NUM);
    //gpio_init(LED1_PORT_NUM, LED1_PIN_NUM);
    //gpio_init(LED2_PORT_NUM, LED2_PIN_NUM);

    LED0_ON;
    LED1_ON;
    LED2_OFF;

    while (1) {
        delay();
        LED1_ON;
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        puts("Blink! (No LED present or configured...)");
#endif
    }

    return 0;
}
