/*
 * Copyright (C) 2016-2020 Freie Universität Berlin
 * Copyright (C) 2023 Prime Controls
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#pragma once

/**
 * @defgroup    drivers_onewire 1-Wire Bus Interface
 * @ingroup     drivers_misc
 * @brief       Generic interface for 1-Wire bus drivers
 *
 * This is RIOT's driver interface for Dallas Semiconductor Corp (now Maxim
 * Integrated) specified 1-Wire Buses. 1-Wire slave device drivers should use it
 * to access the bus (and slave hardware). Drivers implementing 1-Wire bus
 * functionality should expose this functionality via this interface so that
 * slave drivers can operate independent of the bus hardware and its driver.
 *
 * The pseudomodule `onewire_oneslave`, when enabled, permits the assumption
 * that each 1-wire bus instance will only ever have a single slave device
 * connected. This turns calls to acquire and release the bus into noops.
 *
 * @see         https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
 * @see         https://www.maximintegrated.com/en/design/technical-documents/app-notes/7/74.html
 * @see         https://pdfserv.maximintegrated.com/en/an/AN937.pdf
 * @see         https://www.ti.com/lit/an/spma057c/spma057c.pdf?ts=1598599201996
 *
 * @{
 * @file
 * @brief       1-Wire driver interface
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Joshua DeWeese <jdeweese@primecontrols.com>
 */

#include <stdbool.h>
#include <stdint.h>

#include "byteorder.h"
#include "checksum/crc8.h"
#include "checksum/ucrc16.h"
#include "mutex.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Length of 1-Wire ROM code in bytes
 */
#define ONEWIRE_ROM_LEN         (8U)

/**
 * @brief   String length of a hex encoded 1-Wire ROM code (including \0)
 */
#define ONEWIRE_ROM_STR_LEN     (2 * ONEWIRE_ROM_LEN + 1)

/**
 * @brief   Custom value used to start a device search
 */
#define ONEWIRE_SEARCH_FIRST    (0)

/** forward declaration of 1-Wire bus descriptor */
typedef struct onewire_t onewire_t;

/**
 * @brief   1-Wire ROM code (device address) representation
 */
typedef struct {
    uint8_t u8[ONEWIRE_ROM_LEN];    /**< byte-wise access to address bytes */
} onewire_rom_t;

/**
 * @brief   1-Wire bus driver implementation
 */
typedef struct {

    /**
     * @brief   Callback to reset the bus
     *
     * When called, the bus driver should perform a bus reset and detect the
     * slave device present pulse.
     *
     * @param[in] bus       1-Wire bus descriptor
     *
     * @retval  0 if reset successfully and slave(s) were found
     * @retval  -ENXIO if no slave answered the reset sequence
     * @retval  -EIO on all other failures
     */
    int (*reset)(onewire_t *bus);

    /**
     * @brief   Callback to transfer bytes from the bus
     *
     * @param[in] bus       1-Wire bus descriptor
     * @param[out] buf      buffer to place received bits into
     * @param[in] len       number of bits to transfer
     *
     * @retval  0 on success
     * @retval  -EIO on failure
     */
    int (*read_bits)(onewire_t *bus, void *buf, size_t len);

    /**
     * @brief   Callback to transfer bytes to the bus
     *
     * @param[in] bus       1-Wire bus descriptor
     * @param[in] buf       buffer containing bits to send
     * @param[in] len       number of bits to transfer
     *
     * @retval  0 on success
     * @retval  -EIO on failure
     */
    int (*write_bits)(onewire_t *bus, const void *buf, size_t len);

} onewire_driver_t;

/**
 * @brief   1-Wire configuration parameters
 */
typedef struct {
    const onewire_driver_t *driver;     /**< driver for this bus */
} onewire_params_t;

/**
 * @brief   1-Wire bus device descriptor
 */
struct onewire_t {
    const onewire_params_t *params;     /**< bus's configuration params */
#ifndef MODULE_ONEWIRE_ONESLAVE
    mutex_t lock;                       /**< bus access lock */
#endif
};

/**
 * @brief   Initialize a 1-Wire bus
 *
 * @note This is a private function meant to be called by 1-Wire bus driver
 * implementations.
 *
 * @param[in] bus       1-Wire bus descriptor
 * @param[in] params    configuration parameters
 */
void _onewire_init(onewire_t *bus, const onewire_params_t *params);

/**
 * @brief   Acquire exclusive access to 1-Wire bus
 *
 * 1-Wire buses cannot multiplex transactions of data to multiple slave devices
 * at the same time. Therefore, transactions need to be serialized by acquiring
 * exclusive access to the bus before a transaction is started. At the end of
 * the transaction (or multiple transactions) the bus should be released so that
 * it may be shared amongst several slave devices.
 *
 * If it is known that only a single slave device is attached to the bus and
 * only a single instance of a slave driver will make use of the bus, then this
 * serialization may be omitted.
 *
 * @param[in] bus       1-Wire bus descriptor
 */
static inline void onewire_aquire(onewire_t *bus)
{
#ifdef MODULE_ONEWIRE_ONESLAVE
    (void)bus;
#else
    mutex_lock(&bus->lock);
#endif
}

/**
 * @brief   Release exclusive access to 1-Wire bus
 *
 * This call ends exclusive access to a bus which was previously acquired with a
 * call to @ref onewire_aquire().
 *
 * @param[in] bus       1-Wire bus descriptor
 */
static inline void onewire_release(onewire_t *bus)
{
#ifdef MODULE_ONEWIRE_ONESLAVE
    (void)bus;
#else
    mutex_unlock(&bus->lock);
#endif
}

/**
 * @brief   Search for devices on the given 1-Wire bus
 *
 * @see     https://pdfserv.maximintegrated.com/en/an/AN937.pdf page 51+52
 *
 * Use this function to discover devices connected to the given 1-Wire bus. To
 * carry out a full device discovery, this function has to be called multiple
 * times. For each iteration, the return value of the previous call as well as
 * the previous discovered ROM code have to be given as @p rom and @p ld
 * parameters to the subsequent call. When the function returns 0, the last
 * device has been found and there are no further devices to be discovered.
 *
 * @param[in] bus       1-Wire bus
 * @param[in,out] rom   next discovery ROM code, needs to hold previous value
 *                      when function is called multiple times
 * @param[in] ld        position of last discrepancy or ONEWIRE_SEARCH_FIRST for
 *                      initial call
 *
 * @return  bit position of last discrepancy
 * @retval  0 if discovered device was the last
 * @retval  -ENXIO if no device was found
 * @retval  -EBADMSG if check of discovered device's ROM CRC failed
 * @retval  -EIO on all other failures
 */
int onewire_search(onewire_t *bus, onewire_rom_t *rom, int ld);

/**
 * @brief   Read the ROM of a slave device
 *
 * This function reads the ROM from a device on the bus when only a single slave
 * device is present. If multiple devices are present, the read data will be
 * corrupted and the check of its CRC will fail.
 *
 * @param[in] bus       1-Wire bus device descriptor
 * @param[out] rom      1-Wire ROM code of target device
 *
 * @retval  0 on success
 * @retval  -ENXIO if no device was found
 * @retval  -EBADMSG if check of ROM's CRC failed
 * @retval  -EIO on all other failures
 */
int onewire_read_rom(onewire_t *bus, onewire_rom_t *rom);

/**
 * @brief   Select a device on the bus to communicate with
 *
 * This call resets the bus and then sends a Match ROM command followed by the
 * device's ROM pointed to by @p rom. If @p rom is NULL, then a Skip ROM command
 * is sent instead. In this case no single device is selected and all attached
 * devices will receive any further communication.
 *
 * @param[in] bus       1-Wire bus device descriptor
 * @param[in] rom       1-Wire ROM code of target device
 *
 * @retval  0 on success
 * @retval  -ENXIO if no slave answered the reset sequence
 */
int onewire_select(onewire_t *bus, const onewire_rom_t *rom);

/**
 * @brief   Read data from the bus
 *
 * @param[in] bus       1-Wire bus device descriptor
 * @param[out] data     buffer to write received bytes into
 * @param[in] len       number of bytes to read from the bus
 *
 * @retval  0 on success
 * @retval  -EIO on failure
 */
int onewire_read(onewire_t *bus, void *data, size_t len);

/**
 * @brief   Convenience function for reading a single byte from the bus
 *
 * @param[in] bus       1-Wire bus device descriptor
 * @param[out] data     byte read from the bus
 *
 * @retval  0 on success
 * @retval  -EIO on failure
 */
static inline int onewire_read_byte(onewire_t *bus, uint8_t *data)
{
    return onewire_read(bus, data, 1);
}

/**
 * @brief   Convenience function for reading a 16 bit word from the bus
 *
 * @param[in] bus       1-Wire bus device descriptor
 * @param[out] data     word read from the bus
 *
 * @retval  0 on success
 * @retval  -EIO on failure
 */
static inline int onewire_read_word(onewire_t *bus, uint16_t *data)
{
    le_uint16_t buffer;

    const int res = onewire_read(bus, &buffer, 2);
    *data = byteorder_ltohs(buffer);
    return res;
}

/**
 * @brief   Write data to the bus
 *
 * @param[in] bus       1-Wire bus device descriptor
 * @param[in] data      buffer holding the data that is written to the bus
 * @param[in] len       number of bytes to write to the bus
 *
 * @retval  0 on success
 * @retval  -EIO on failure
 */
int onewire_write(onewire_t *bus, const void *data, size_t len);

/**
 * @brief   Convenience function for writing a single byte to the bus
 *
 * @param[in] bus       1-Wire bus device descriptor
 * @param[in] data      byte to write to the bus
 *
 * @retval  0 on success
 * @retval  -EIO on failure
 */
static inline int onewire_write_byte(onewire_t *bus, uint8_t data)
{
    return onewire_write(bus, &data, 1);
}

/**
 * @brief   Convenience function for writing a 16 bit word to the bus
 *
 * @param[in] bus       1-Wire bus device descriptor
 * @param[in] data      word to write to the bus
 *
 * @retval  0 on success
 * @retval  -EIO on failure
 */
static inline int onewire_write_word(onewire_t *bus, uint16_t data)
{
    const le_uint16_t buffer = byteorder_htols(data);

    return onewire_write(bus, &buffer, 2);
}

/**
 * @brief   Calculate a 8 bit CRC using the 1-Wire specific polynomial
 *
 * @param[in] seed      the seed (starting value) for the checksum
 * @param[in] data      input data
 * @param[in] len       len of @p data in bytes
 *
 * @return  calculated 8 bit CRC
 */
static inline uint_fast8_t onewire_crc8(uint_fast16_t seed, const void *data,
    size_t len)
{
    return crc8(data, len, 0x80, seed);
}

/**
 * @brief   Calculate a 16 bit CRC using the 1-Wire specific polynomial
 *
 * @param[in] seed      the seed (starting value) for the checksum
 * @param[in] data      input data
 * @param[in] len       len of @p data in bytes
 *
 * @return  calculated 16 bit CRC
 */
static inline uint_fast16_t onewire_crc16(uint_fast16_t seed, const void *data,
    size_t len)
{
    return ucrc16_calc_le(data, len, 0xA001, seed);
}

/**
 * @brief   Read 1-Wire ROM code from string
 *
 * @param[out] rom      ROM code is parsed into this location
 * @param[in] str       String representation of a ROM code
 *
 * @retval  0 on success
 * @retval  -EINVAL on parsing error
 */
int onewire_rom_from_str(onewire_rom_t *rom, const char *str);

/**
 * @brief   Write a 1-Wire ROM code to a string
 *
 * @param[out] str      Output string, must be able to hold ONEWIRE_ROM_STR_LEN
 *                      characters
 * @param[in] rom       1-Wire ROM code to read
 */
void onewire_rom_to_str(char *str, const onewire_rom_t *rom);

/**
 * @brief   Print a 1-Wire ROM code to STDIO
 *
 * @param[in] rom       1-Wire ROM code
 */
void onewire_rom_print(const onewire_rom_t *rom);

/**
 * @brief   Validate a 1-Wire ROM code
 *
 * This function checks if a given 1-Wire ROM contains a valid device ID.
 *
 * @param[in] rom       1-Wire ROM code
 *
 * @retval  true if @p rom contains a valid ID
 * @retval  false if @p rom contains an invalid ID
 */
bool onewire_rom_is_valid(const onewire_rom_t *rom);

/**
 * @brief   Returns the 1-Wire family code of @p rom
 *
 * @param[in] rom       1-Wire ROM code
 *
 * @return  1-Wire device 8 bit family code
 */
uint_fast8_t onewire_rom_family_code(const onewire_rom_t *rom);

/**
 * @brief   Compare two 1-Wire ROMs for equality.
 *
 * @param[in] left      1-Wire ROM code to compare
 * @param[in] right     1-Wire ROM code to compare
 *
 * @retval  0 if @p left and @p right are equal
 */
int onewire_rom_compare(const onewire_rom_t *left, const onewire_rom_t *right);

#ifdef __cplusplus
}
#endif

/** @} */
