#ifndef OS_PORTABILITY_H
#define OS_PORTABILITY_H

#include "mbed_assert.h"
#include <stdint.h>

#if defined(MBED_OS)
#include "mbed_assert.h"
#define PORTABLE_ASSERT MBED_ASSERT

#elif defined(ESP_IDF)
#include <assert.h>
#define PORTABLE_ASSERT assert

#else
#error Need to define either MBED_OS or ESP_IDF
#endif

#endif /* OS_PORTABILITY_H */
