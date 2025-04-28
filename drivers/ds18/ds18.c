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

#include "log.h"
#include "periph/gpio.h"
#include "xtimer.h"

#define ENABLE_DEBUG 0
#include "debug.h"

static int ds18_read_byte(const ds18_t *dev, uint8_t *byte)
{
    return onewire_read(dev->params->bus, byte, 1);
}

static void ds18_write_byte(const ds18_t *dev, uint8_t byte)
{
    onewire_write_byte(dev->params->bus, byte);
}

static int ds18_reset(const ds18_t *dev)
{
    return onewire_select(dev->params->bus, NULL);
}

int ds18_trigger(const ds18_t *dev)
{
    int res;

    res = ds18_reset(dev);
    if (res) {
        return DS18_ERROR;
    }

    /* Please note that this command triggers a conversion on all devices
     * connected to the bus. */
    //ds18_write_byte(dev, DS18_CMD_SKIPROM);
    ds18_write_byte(dev, DS18_CMD_CONVERT);

    return DS18_OK;
}

int ds18_read(const ds18_t *dev, int16_t *temperature)
{
    int res;
    uint8_t b1 = 0, b2 = 0;

    DEBUG("[DS18] Reset and read scratchpad\n");
    res = ds18_reset(dev);
    if (res) {
        return DS18_ERROR;
    }

    //ds18_write_byte(dev, DS18_CMD_SKIPROM);
    ds18_write_byte(dev, DS18_CMD_RSCRATCHPAD);

    if (ds18_read_byte(dev, &b1) != DS18_OK) {
        DEBUG("[DS18] Error reading temperature byte 1\n");
        return DS18_ERROR;
    }

    DEBUG("[DS18] Received byte: 0x%02x\n", b1);

    if (ds18_read_byte(dev, &b2) != DS18_OK) {
        DEBUG("[DS18] Error reading temperature byte 2\n");
        return DS18_ERROR;
    }

    DEBUG("[DS18] Received byte: 0x%02x\n", b2);

    int32_t measurement = (int16_t)(b2 << 8 | b1);
    *temperature = (int16_t)((100 * measurement) >> 4);

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
