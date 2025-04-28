/*
 * Copyright (C) 2018 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the DS18B20 1-Wire temperature sensor.
 *
 * @author      Leandro Lanzieri <leandro.lanzieri@haw-hamburg.de>
 *
 * @}
 */

#include <stdio.h>
#include <inttypes.h>

#include "board.h"
#include "ds18.h"
/*#include "ds18_params.h"*/
#include "soft_onewire.h"
#include "xtimer.h"

#define SAMPLING_PERIOD     2

static soft_onewire_params_t soft_onewire_params[] = {
    {
#ifdef MODULE_ONEWIRE_MULTIDRIVER
        .super = { .driver = &soft_onewire_driver },
#endif
#if     defined(BOARD_NATIVE)
        .pin = GPIO_UNDEF,
#elif   defined(BOARD_STM32F429I_DISC1)
        .pin = GPIO_PIN(PORT_C, 0),
#endif
        .pin_imode = GPIO_IN_PU,
#ifdef MODULE_SOFT_ONEWIRE_HWTIMER
        .timer = TIMER_DEV(1),
#endif
    },
};

soft_onewire_t soft_onewire_devs[ARRAY_SIZE(soft_onewire_params)];

static ds18_params_t ds18_params[] = {
    {
        .bus = &soft_onewire_devs[0].super,
    },
};

int main(void)
{
    ds18_t dev;
    int result;

    puts("DS18B20 test application\n");

    printf("+------------Initializing------------+\n");
    soft_onewire_init(&soft_onewire_devs[0], &soft_onewire_params[0]);
    result = ds18_init(&dev, &ds18_params[0]);
    if (result == DS18_ERROR) {
        puts("[Error] The sensor pin could not be initialized");
        return 1;
    }

    printf("\n+--------Starting Measurements--------+\n");
    while (1) {
        int16_t temperature;

        /* Get temperature in centidegrees celsius */
        if (ds18_get_temperature(&dev, &temperature) == DS18_OK) {
            bool negative = (temperature < 0);
            if (negative) {
                temperature = -temperature;
            }

            printf("Temperature [ºC]: %c%d.%02d"
                   "\n+-------------------------------------+\n",
                   negative ? '-': ' ',
                   temperature / 100,
                   temperature % 100);
        }
        else {
            puts("[Error] Could not read temperature");
        }

        xtimer_sleep(SAMPLING_PERIOD);
    }

    return 0;
}
