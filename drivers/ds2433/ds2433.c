#include "ds2433.h"

#include <assert.h>
#include <errno.h>

#include "ztimer.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#define PAGE_SIZE               32 /* bytes */

#define CMD_READ_MEMORY         0xf0
#define CMD_READ_SCRATCHPAD     0xaa
#define CMD_WRITE_SCRATCHPAD    0x0f
#define CMD_COPY_SCRATCHPAD     0x55

static unsigned _addr_to_page(uint16_t address)
{
    return address / PAGE_SIZE;
}

static uint16_t _page_to_addr(unsigned page)
{
    return page * PAGE_SIZE;
}

static uint16_t _page_to_end_addr(unsigned page)
{
    return _page_to_addr(page + 1) - 1;
}

/**
 * @brief Read the EEPROM's scratchpad buffer.
 *
 * @param[in] dev       pointer to device descriptor
 * @param[out] buf      buffer to write received bytes into
 * @param[in] size      number of bytes to read
 *
 * @return number of bytes read on success
 * @retval -ENODEV if no device detected on 1-wire bus
 * @retval -EIO on all other failures
 */
static int _read_scratchpad(ds2433_t *dev, void* buf, size_t size)
{
    onewire_t *bus = dev->params->bus;
    int res;

    res = onewire_select(bus, dev->id);
    if (res == -ENXIO) {
        return -ENODEV;
    }
    else if (res < 0) {
        return -EIO;
    }

    res = onewire_write_byte(bus, CMD_READ_SCRATCHPAD);
    if (res < 0) {
        return -EIO;
    }

    res = onewire_read(bus, buf, size);
    if (res < 0) {
        return -EIO;
    }

    return size;
}

/**
 * @brief Write to the EEPROM's scratchpad buffer.
 *
 * Writes to the scratchpad are limited to a single EEPROM page. Therefore any
 * write that crosses a page boundary will be truncated and this will be
 * reflected in the returned value.
 *
 * @note @p address does not refer to an address in the scratchpad. It refers to
 * the address in the EEPROM, where the scratchpad will be written when it is
 * transferred to the EEPROM.
 *
 * @param[in] dev       pointer to device descriptor
 * @param[in] address   EEPROM address where the scratchpad will be written
 * @param[in] buf       data to write
 * @param[in] size      number of bytes to write
 *
 * @return number of bytes written on success
 * @retval -ENODEV if no device detected on 1-wire bus
 * @retval -EIO on all other failures
 */
static int _write_scratchpad(ds2433_t *dev, uint16_t address, const void* buf,
    size_t size)
{
    onewire_t *bus = dev->params->bus;
    uint16_t end_address = address + size - 1;
    int res;

    /* the page we will be writing to */
    const unsigned page = _addr_to_page(address);

    /* the highest address within the page we are writing */
    const unsigned page_end_addr = _page_to_end_addr(page);

    /* limit size of write to stay within a single page */
    if (end_address > page_end_addr) {
        end_address = page_end_addr;
        size = end_address - address + 1;
    }

    DEBUG("writing page %u adress 0x%04x to 0x%04x\n",
        page, address, end_address);

    res = onewire_select(bus, dev->id);
    if (res == -ENXIO) {
        return -ENODEV;
    }
    else if (res < 0) {
        return -EIO;
    }

    res = onewire_write_byte(bus, CMD_WRITE_SCRATCHPAD);
    if (res < 0) {
        return -EIO;
    }

    res = onewire_write_word(bus, address);
    if (res < 0) {
        return -EIO;
    }

    res = onewire_write(bus, buf, size);
    if (res < 0) {
        return -EIO;
    }

    /* If we hit the end of the scratchpad, the device makes a CRC available to
       read back. In that case, lets use it to verify what we sent. */
    if (end_address == page_end_addr) {
        uint16_t crc_read = 0;
        uint16_t crc_calc = 0;

        /* read CRC from device */
        res = onewire_read_word(bus, &crc_read);
        if (res < 0) {
            return -EIO;
        }

        /* the CRC received is inverted, undo that */
        crc_read = ~crc_read;

        /* calculate our own CRC */
        uint8_t tmp = CMD_WRITE_SCRATCHPAD;
        crc_calc = onewire_crc16(crc_calc, &tmp, 1);
        tmp = address & 0x00ff;
        crc_calc = onewire_crc16(crc_calc, &tmp, 1);
        tmp = address >> 8;
        crc_calc = onewire_crc16(crc_calc, &tmp, 1);
        crc_calc = onewire_crc16(crc_calc, buf, size);

        DEBUG("crc read: 0x%04x crc calculated: 0x%04x\n", crc_read, crc_calc);

        if (crc_read != crc_calc) {
            return -EIO;
        }
    }

    return size;
}

/**
 * @brief Commit the scratchpad buffer to EEPROM.
 *
 * Copies the contents of the scratchpad buffer to the EEPROM at the EEPROM
 * address indicated at the time the scratchpad was written to.
 *
 * @param[in] dev       pointer to device descriptor
 *
 * @retval 0 on success
 * @retval -ENODEV if no device detected on 1-wire bus
 * @retval -EIO on all other failures
 */
static int _copy_scratchpad(ds2433_t *dev)
{
    onewire_t *bus = dev->params->bus;
    uint8_t key[3];
    int res;

    res = _read_scratchpad(dev, key, sizeof(key));
    if (res < 0) {
        return res;
    }

    DEBUG("%s: key = 0x%02x%02x%02x\n", DEBUG_FUNC, key[0], key[1], key[2]);

    res = onewire_select(bus, dev->id);
    if (res == -ENXIO) {
        return -ENODEV;
    }
    else if (res < 0) {
        return -EIO;
    }

    res = onewire_write_byte(bus, CMD_COPY_SCRATCHPAD);
    if (res < 0) {
        return -EIO;
    }

    res = onewire_write(bus, key, sizeof(key));
    if (res < 0) {
        return -EIO;
    }

    /* copy takes a maxiumum of 5 msec, durring which the bus must not fall
       below 2.8 volts */
    ztimer_sleep(ZTIMER_USEC, 5 * 1000);

    uint8_t byte = 0;
    res = onewire_read_byte(bus, &byte);
    if (res < 0) {
        return -EIO;
    }
    if (byte != 0x55 && byte != 0xaa) {
        return -EIO;
    }

    return 0;
}

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
static int _verify(ds2433_t *dev, uint16_t address, const void* buf,
    size_t size)
{
    onewire_t *bus = dev->params->bus;
    const uint8_t *data = buf;
    int res;

    res = onewire_select(bus, dev->id);
    if (res == -ENXIO) {
        return -ENODEV;
    }
    else if (res < 0) {
        return -EIO;
    }

    res = onewire_write_byte(bus, CMD_READ_MEMORY);
    if (res < 0) {
        return -EIO;
    }

    res = onewire_write_word(bus, address);
    if (res < 0) {
        return -EIO;
    }

    /* TODO: This could be more efficient if a onewire_verify(...) or
       onewire_expect(...) were implemented so that we could read more then a
       single byte at a time, without allocating a large RAM buffer. */
    for (unsigned i = 0; i < size; i++) {
        uint8_t byte;
        onewire_read_byte(bus, &byte);
        if (data[i] != byte) {
            return i;
        }
    }

    return size;
}

int ds2433_read(ds2433_t *dev, uint16_t address, void* buf, size_t size)
{
    assert(dev);
    assert(buf);

    if (address + size > DS2433_EEPROM_SIZE) {
        return -ERANGE;
    }

    onewire_t *bus = dev->params->bus;
    int res;

    onewire_aquire(bus);

    res = onewire_select(bus, dev->id);
    if (res == -ENXIO) {
        res = -ENODEV;
        goto fail;
    }
    else if (res < 0) {
        res = -EIO;
        goto fail;
    }

    res = onewire_write_byte(bus, CMD_READ_MEMORY);
    if (res < 0) {
        res = -EIO;
        goto fail;
    }

    res = onewire_write_word(bus, address);
    if (res < 0) {
        res = -EIO;
        goto fail;
    }

    res = onewire_read(bus, buf, size);
    if (res < 0) {
        res = -EIO;
        goto fail;
    }

    onewire_release(bus);
    return size;

    fail:
    onewire_release(bus);
    return res;
}

int ds2433_write(ds2433_t *dev, uint16_t address, const void* buf, size_t size)
{
    assert(dev);
    assert(buf);

    if (address + size > DS2433_EEPROM_SIZE) {
        return -ERANGE;
    }

    onewire_t *bus = dev->params->bus;
    const uint8_t *data = buf;
    int res;

    onewire_aquire(bus);

    unsigned offset = 0;
    while (offset < size) {

        /* fill the scratchpad */
        res = _write_scratchpad(dev, address + offset, &data[offset],
            size - offset);
        if (res < 0) {
            goto fail;
        }

        const int bytes_written = res;

        /* save the scratchpad to EEPROM */
        res = _copy_scratchpad(dev);
        if (res < 0) {
            goto fail;
        }

        /* verify data just written */
        res = _verify(dev, address + offset, &data[offset], bytes_written);
        if (res < 0) {
            goto fail;
        }
        else if (res != bytes_written) {
            res = -EIO;
            goto fail;
        }

        offset += bytes_written;
    }

    onewire_release(bus);
    return size;

    fail:
    onewire_release(bus);
    return res;
}

int ds2433_verify(ds2433_t *dev, uint16_t address, const void* buf, size_t size)
{
    assert(dev);
    assert(buf);

    if (address + size > DS2433_EEPROM_SIZE) {
        return -ERANGE;
    }

    onewire_t *bus = dev->params->bus;

    onewire_aquire(bus);
    const int res = _verify(dev, address, buf, size);
    onewire_release(bus);

    return res;
}

int ds2433_init(ds2433_t *dev, const ds2433_params_t *params,
    const onewire_rom_t *id)
{
    assert(dev);
    assert(params);

    dev->params = params;
    dev->id = id;

    return 0;
}
