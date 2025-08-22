#ifndef __CODEC_COMMON_H__
#define __CODEC_COMMON_H__
#include "mem_pub.h"

#define _malloc_wrapper(size)                   (os_malloc(size))
//#define _zalloc_wrapper(num, size)              (os_zalloc(size))
#define _realloc_wrapper(old_mem, size)         (os_realloc(old_mem, size))
#define _psram_malloc_wrapper(size)             (os_malloc(size))
//#define _psram_zalloc_wrapper(num, size)        (NULL)
//#define _psram_realloc_wrapper(old_mem, size)   (NULL)
#define _free_wrapper(p)                        (os_free(p))
#define _memcpy_wrapper(out, in, n)             (os_memcpy(out, in, n))
//#define _memcpy_word_wrapper(out, in, n)      (os_memcpy(out, in, n))
#define _memset_wrapper(b, c, len)              (os_memset(b, c, len))
#define _memmove_wrapper(out, in, n)            (os_memmove(out, in, ))
#define _memset_word_wrapper(b, c, n)           (os_memset(b, c, n))

#endif // __CODEC_COMMON_H__