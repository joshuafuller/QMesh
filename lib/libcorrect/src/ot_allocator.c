#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "mbed_assert.h"
#include <stdio.h>

/**
 * Onetime allocator. Behaves like malloc, but makes a single, onetime
 *  allocation from a static memory pool.
 *  @param size Size in bytes of the desired allocation.
 */
 static const int pool_size = 0x1C000;
 static char mpool[0x1C000] __attribute__((section (".DtcmRamSection")));
 static int mpool_pos = 0;
 void *malloc_ot(size_t size) {
    // Make the allocation aligned on a 32-bit boundary
    size &= 0xFFFFFFFC;
    size += 4;
    if(mpool_pos+size >= pool_size) {
        MBED_ASSERT(false);
        return NULL;
    }
    void *mpool_ptr = (void *) &mpool[mpool_pos];
    mpool_pos += size;
    return mpool_ptr;
 }


 /**
  * Calloc variant of the onetime allocator.
  * @param size Size, in bytes, of the desired allocation.
  */
 void *calloc_ot(size_t nmemb, size_t size) {
    return memset(malloc_ot(size*nmemb), 0, size*nmemb);
 }


 /** 
  * Free version of the onetime allocator. Doesn't do anything besides throw an assert.
  * @param ptr Pointer to the memory to be freed.
  */
void free_ot(void * ptr) {
    //MBED_ASSERT(false);
}


#ifdef __cplusplus
}
#endif