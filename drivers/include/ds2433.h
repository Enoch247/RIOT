#ifndef DS2433_H
#define DS2433_H

#include <stddef.h>

#include "onewire.h"

#define DS2433_FAMILY_CODE 0x23 // onewire ROM ID family code

#define DS2433_EEPROM_SIZE 512 // 512 bytes

typedef struct {
    onewire_t *bus;
} ds2433_params_t;

typedef struct {
    const ds2433_params_t *params;
    const onewire_rom_t *id;
} ds2433_t;

extern ds2433_t ds2433[];

int ds2433_init(ds2433_t *dev, const ds2433_params_t *params,
    const onewire_rom_t *id);

int ds2433_read(ds2433_t *dev, uint16_t address, void* buf, size_t size);
int ds2433_write(ds2433_t *dev, uint16_t address, const void* buf, size_t size);
int ds2433_flush(ds2433_t *dev);

#endif
