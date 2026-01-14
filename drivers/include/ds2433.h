/*
 * Copyright (C) 2026 Prime Controls
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#pragma once

/**
 * @defgroup    drivers_ds2433 DS2433 EEPROM
 * @ingroup     drivers_onewire_devs
 * @ingroup     drivers_storage
 * @brief       DS2433 1-Wire EEPROM
 *
 * This module provides a driver for the 1-Wire based DS2433 EEPROM.
 * @{
 *
 * @file
 * @brief       Dallas / Analog Devices DS2433 driver module
 *
 * @author      Joshua DeWeese <jdeweese@primecontrols.com>
 */

#include <stddef.h>

#include "onewire.h"

#ifdef __cplusplus
extern "C" {
#endif

/** DS2433 1-wire ROM ID family code */
#define DS2433_FAMILY_CODE              (0x23)

/** size of EEPROM in bytes */
#define DS2433_EEPROM_SIZE              (512)

/**
 * @brief ds2433 parameter struct
 */
typedef struct {
    onewire_t *bus; /**< the 1-wire bus the device is attached to */
} ds2433_params_t;

/**
 * @brief ds2433 device descriptor
 */
typedef struct {
    const ds2433_params_t *params;  /**< the device's parameters */
    const onewire_rom_t *id;        /**< the device's 1-wire ID */
} ds2433_t;

/**
 * @brief Initialize a ds2433 device
 *
 * If this device is the only device on the bus, @p id may be NULL, and this
 * driver will skip sending the device address.
 *
 * @param[in] dev                   pointer to device descriptor to initialize
 * @param[in] params                pointer to device's parameters
 * @param[in] id                    device's 1-wire ROM ID
 *
 * @retval 0 on success
 */
int ds2433_init(ds2433_t *dev, const ds2433_params_t *params,
    const onewire_rom_t *id);

/**
 * @brief Read data from EEPROM.
 *
 * @param[in] dev                   pointer to device descriptor
 * @param[in] address               EEPROM address to read from
 * @param[out] buf                  buffer to write read bytes into
 * @param[in] size                  number of bytes to read
 *
 * @retval number of bytes read on success
 * @retval -ENODEV if no device detected on 1-wire bus
 * @retval -ERANGE if read would fall outside of the EEPROM's address range
 * @retval -EIO on all other failures
 */
int ds2433_read(ds2433_t *dev, uint16_t address, void* buf, size_t size);

/**
 * @brief Write data to EEPROM.
 *
 * @param[in] dev                   pointer to device descriptor
 * @param[in] address               EEPROM address to write to
 * @param[in] buf                   data to write
 * @param[in] size                  number of bytes to write
 *
 * @retval number of bytes written on success
 * @retval -ENODEV if no device detected on 1-wire bus
 * @retval -ERANGE if write would fall outside of the EEPROM's address range
 * @retval -EIO on all other failures
 */
int ds2433_write(ds2433_t *dev, uint16_t address, const void* buf, size_t size);

/**
 * @brief Verify data in EEPROM.
 *
 * Verify that the contents of the EEPROM match the buffer given.
 *
 * @param[in] dev       pointer to device descriptor
 * @param[in] address   EEPROM address of data to verify
 * @param[in] buf       data to verify
 * @param[in] size      number of bytes to verify
 *
 * @return the number of bytes read that matched @p buf
 * @retval -ENODEV if no device detected on 1-wire bus
 * @retval -EIO on all other failures
 */
int ds2433_verify(ds2433_t *dev, uint16_t address, const void* buf, size_t size);

#ifdef __cplusplus
}
#endif

/** @} */
