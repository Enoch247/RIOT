#include "ds2433.h"
#include "ds2433_params.h"

#include <assert.h>
#include <errno.h>

#include "checksum/ucrc16.h"
#include "ztimer.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#define FAMILIY_CODE 0x23

//TODO: consolidate these two?
#define PAGE_SIZE       32 // 32 bytes
#define SCRATCHPAD_SIZE 32 // 32 bytes

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

static int _aquire(ds2433_t *dev)
{
    int res;
    onewire_t *bus = dev->params->bus;

    onewire_aquire(bus);

    res = onewire_select(bus, NULL);
    if (res == -ENXIO)
    {
        return res;
    }
    else if (res < 0)
    {
        return -EIO;
    }

    return 0;
}

static int _release_and_return(ds2433_t *dev, int rtnval)
{
    onewire_t *bus = dev->params->bus;

    onewire_release(bus);

    return rtnval;
}

int ds2433_init(ds2433_t *dev, const ds2433_params_t *params)
{
    int res;
    onewire_t *bus = params->bus;
    onewire_rom_t id_rom;

    assert(dev);
    assert(params);

/*    if (!dev || !params) {*/
/*        return -EINVAL;*/
/*    }*/

    dev->params = params;

    res = _aquire(dev);
    if (res < 0)
    {
        return _release_and_return(dev, res);
    }

    // invalidate the id_rom
    //TODO: check returned of onewire_read_rom() instead
    id_rom.u8[0] = !FAMILIY_CODE;

    // read the id_rom
    onewire_read_rom(bus, &id_rom);

    // validate the id_rom
    if (onewire_rom_family_code(&id_rom) != FAMILIY_CODE)
    {
        return _release_and_return(dev, -ENODEV);
    }

    //TODO: check ROM CRC?
    //if (onewire_rom_valid(&id_rom)) { }

    //TODO: rm once onewire driver has the ability to run read ROM cmd?
#if ENABLE_DEBUG
    char id_str[ONEWIRE_ROM_STR_LEN];
    onewire_rom_to_str(id_str, &id_rom);
    DEBUG("%s: ROM ID: %s\n", DEBUG_FUNC, id_str);
#endif

    return _release_and_return(dev, 0);
}

int ds2433_read(ds2433_t *dev, uint16_t address, void* buf, size_t size)
{
    int res;
    onewire_t *bus = dev->params->bus;

    //TODO: check address?

    if (address + size > DS2433_EEPROM_SIZE)
    {
        //size = DS2433_EEPROM_SIZE - address;
        return -ERANGE; //TODO: assert instead
    }

    onewire_aquire(bus);

    res = onewire_select(bus, NULL);
    if (res < 0)
    {
        onewire_release(bus);
        return -EIO; //TODO
    }

    onewire_write_byte(bus, CMD_READ_MEMORY);
    onewire_write_word(bus, address);

    onewire_read(bus, buf, size);

    onewire_release(bus);
    return size;
}

/*static int _read_scratchpad(ds2433_t *dev, uint16_t *address, const void* buf,*/
/*    size_t size) TODO*/
static int _read_scratchpad(ds2433_t *dev, void* buf, size_t size)
{
    int res;
    onewire_t *bus = dev->params->bus;

    //TODO: check size param

    res = onewire_select(bus, NULL);
    if (res < 0)
    {
        onewire_release(bus);
        return -EIO; //TODO
    }

    onewire_write_byte(bus, CMD_READ_SCRATCHPAD);
    onewire_read(bus, buf, size);

    return size;
}

// Note, the address param does not refer to the address in the scratchpad. It
// referes to the address in the EEPROM, where the scratchpad wil be written
// when transfered to the EEPROM.
static int _write_scratchpad(ds2433_t *dev, uint16_t address, const void* buf,
    size_t size)
{
    int res;
    onewire_t *bus = dev->params->bus;
    uint16_t end_address = address + size - 1;

    // the page we will be writing to
    const unsigned page = _addr_to_page(address);

    // the highest address within the page we are writing
    const unsigned page_end_addr = _page_to_end_addr(page);

    // limit size of write to stay within a single page
    if (end_address > page_end_addr)
    {
        end_address = page_end_addr;
        size = end_address - address + 1;
    }

    DEBUG("writing page %u adress 0x%04x to 0x%04x\n",
        page, address, end_address);

    res = onewire_select(bus, NULL);
    if (res < 0)
    {
        onewire_release(bus);
        return -EIO; //TODO
    }

    onewire_write_byte(bus, CMD_WRITE_SCRATCHPAD);
    onewire_write_word(bus, address);
    onewire_write(bus, buf, size);

    //TODO: re-word
    // If writing the entire scratchpad, the hardware will make a 16 bit CRC
    // available to read back. So make use of it when possible.
    if (end_address == page_end_addr)
    {
        uint16_t crc_read = 0, crc_calc = 0;

        onewire_read_word(bus, &crc_read);
        crc_read = ~crc_read; //TODO

        uint8_t tmp = CMD_WRITE_SCRATCHPAD;
        crc_calc = onewire_crc16(crc_calc, &tmp, 1);
        tmp = address & 0x00ff;
        crc_calc = onewire_crc16(crc_calc, &tmp, 1);
        tmp = address >> 8;
        crc_calc = onewire_crc16(crc_calc, &tmp, 1);
        crc_calc = onewire_crc16(crc_calc, buf, size);

        DEBUG("crc read: 0x%04x crc calculated: 0x%04x\n", crc_read, crc_calc);

        if (crc_read != crc_calc)
        {
            return -EIO; //TODO
        }
    }

    return size;
}

static int _copy_scratchpad(ds2433_t *dev)
{
    int res;
    onewire_t *bus = dev->params->bus;
    uint8_t key[3];

    _read_scratchpad(dev, key, sizeof(key));

    DEBUG("%s: key = 0x%02x%02x%02x\n", DEBUG_FUNC, key[0], key[1], key[2]);

    res = onewire_select(bus, NULL);
    if (res < 0)
    {
        onewire_release(bus);//TODO: rm
        return -EIO; //TODO
    }

    onewire_write_byte(bus, CMD_COPY_SCRATCHPAD);
    onewire_write(bus, key, sizeof(key));

    ztimer_sleep(ZTIMER_USEC, 100 * 1000); //TODO

    uint8_t byte = 0;
    while (byte != 0x55 && byte != 0xaa)
    {
        //TODO: this is an opertunity to do some erorr checking here
        onewire_read_byte(bus, &byte);
    }

    return 0;
}

static int _verify(ds2433_t *dev, uint16_t address, const void* buf,
    size_t size)
{
    int res;
    onewire_t *bus = dev->params->bus;
    const uint8_t *data = buf;

/*    //TODO: rm?*/
/*    if (address + size > DS2433_EEPROM_SIZE)*/
/*    {*/
/*        size = DS2433_EEPROM_SIZE - address;*/
/*    }*/

    //onewire_aquire(bus);

    res = onewire_select(bus, NULL);
    if (res < 0)
    {
        onewire_release(bus);
        return -EIO; //TODO
    }

    onewire_write_byte(bus, CMD_READ_MEMORY);
    onewire_write_word(bus, address);

    for (unsigned i = 0; i < size; i++)
    {
        uint8_t byte;
        onewire_read_byte(bus, &byte);
        if (data[i] != byte)
        {
            return -1; //TODO
        }
    }

    return 0;
}

int ds2433_write(ds2433_t *dev, uint16_t address, const void* buf, size_t size)
{
    DEBUG("%s\n", DEBUG_FUNC);

    int res;
    onewire_t *bus = dev->params->bus;
    const uint8_t *data = buf;

    //TODO: check size param

    onewire_aquire(bus);

    unsigned offset = 0;
    while (offset < size)
    {
        // fill the scratchpad
        res = _write_scratchpad(dev, address + offset, &data[offset],
            size - offset);
        if (res < 0)
        {
            return -EIO; //TODO
        }

        int bytes_written = res;

        // save the scratchpad to EEPROM
        res = _copy_scratchpad(dev);
        if (res < 0)
        {
            return -EIO; //TODO
        }

        // verify data just written
        res = _verify(dev, address + offset, &data[offset], bytes_written);
        if (res < 0)
        {
            return -EIO; //TODO
        }

        offset += bytes_written;
    }

    onewire_release(bus);
    return size;
}
