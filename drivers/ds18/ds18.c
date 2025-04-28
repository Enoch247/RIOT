/*
 * Copyright (C) 2017 Frits Kuipers
 *               2018 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ds18
 * @{
 *
 * @file
 * @brief       Device driver implementation for the Maxim Integrated DS1822 and
 *              DS18B20 temperature sensors.
 *
 * @author      Frits Kuipers <frits.kuipers@gmail.com>
 * @author      Leandro Lanzieri <leandro.lanzieri@haw-hamburg.de>
 * @author      Joshua DeWeese <josh.deweese@gmail.com>
 * @}
 */

#include "ds18.h"
#include "ds18_internal.h"

#include "xtimer.h"

#define ENABLE_DEBUG 0
#include "debug.h"

int ds18_trigger(const ds18_t *dev)
{
    onewire_t *bus = dev->params->bus;
    int res;

    res = onewire_select(bus, NULL);
    if (res) {
        return DS18_ERROR;
    }

    /* Please note that this command triggers a conversion on all devices
     * connected to the bus. */
    res = onewire_write_byte(bus, DS18_CMD_CONVERT);
    if (res) {
        return DS18_ERROR;
    }

    return DS18_OK;
}

int ds18_read(const ds18_t *dev, int16_t *temperature)
{
    onewire_t *bus = dev->params->bus;
    int res;

    DEBUG("[DS18] Reset and read scratchpad\n");
    res = onewire_select(bus, NULL);
    if (res) {
        return DS18_ERROR;
    }

    res = onewire_write_byte(bus, DS18_CMD_RSCRATCHPAD);
    if (res) {
        return DS18_ERROR;
    }

    res = onewire_read_word(dev->params->bus, (uint16_t*)temperature);
    if (res) {
        DEBUG("[DS18] Error reading temperature\n");
        return DS18_ERROR;
    }

    *temperature = (100 * (*temperature)) >> 4;

    return DS18_OK;
}

int ds18_get_temperature(const ds18_t *dev, int16_t *temperature)
{

    DEBUG("[DS18] Convert T\n");
    if (ds18_trigger(dev)) {
        return DS18_ERROR;
    }

    DEBUG("[DS18] Wait for convert T\n");
    xtimer_usleep(DS18_DELAY_CONVERT);

    return ds18_read(dev, temperature);
}

int ds18_init(ds18_t *dev, const ds18_params_t *params)
{
    dev->params = params;

    return 0;
}
