#ifndef DS2433_H
#define DS2433_H

#include <stddef.h>

#include "onewire.h"

#define DS2433_EEPROM_SIZE 512 // 512 bytes

typedef struct {
    onewire_t *bus;
} ds2433_params_t;

typedef struct {
    const ds2433_params_t *params;
} ds2433_t;

extern ds2433_t ds2433[];

int ds2433_init(ds2433_t *dev, const ds2433_params_t *params);

int ds2433_read(ds2433_t *dev, uint16_t address, void* buf, size_t size);
int ds2433_write(ds2433_t *dev, uint16_t address, const void* buf, size_t size);
int ds2433_flush(ds2433_t *dev);

#endif
