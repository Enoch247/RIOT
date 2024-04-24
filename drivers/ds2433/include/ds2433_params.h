#ifndef DS2433_PARAMS_H
#define DS2433_PARAMS_H

#include "board.h"

#include "ds2433.h"
#include "kernel_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DS2433_PARAMS_BUS
#define DS2433_PARAMS_BUS (NULL)
#endif

#ifndef DS2433_PARAMS
#define DS2433_PARAMS \
    { \
        .bus = DS2433_PARAMS_BUS, \
    }
#endif

static const ds2433_params_t ds2433_params[] = {
    DS2433_PARAMS
};

#define DS2433_NUMOF ARRAY_SIZE(ds2433_params)

#ifdef __cplusplus
}
#endif

#endif /* DS2433_PARAMS_H */
