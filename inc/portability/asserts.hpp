#ifndef ASSERTS_HPP
#define ASSERTS_HPP

#if defined(MBED_OS)
#include "mbed.h"
#define PORTABLE_ASSERT MBED_ASSERT

#elif defined(ESP_IDF)
#include <assert.h>
using PORTABLE_ASSERT = assert;

#else
#error Need to define either MBED_OS or ESP_IDF
#endif

#endif /* ASSERTS_HPP */