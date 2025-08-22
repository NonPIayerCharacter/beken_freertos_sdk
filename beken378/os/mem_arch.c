// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include.h"
#include "arm_arch.h"
#include <string.h>
#include "sys_rtos.h"
#include "uart_pub.h"
#include "mem_pub.h"
#include "rtos_pub.h"

INT32 os_memcmp(const void *s1, const void *s2, UINT32 n)
{
    return memcmp(s1, s2, (unsigned int)n);
}

void *os_memmove(void *out, const void *in, UINT32 n)
{
    return memmove(out, in, n);
}

void *os_memcpy(void *out, const void *in, UINT32 n)
{
    return memcpy(out, in, n);
}

void *os_memset(void *b, int c, UINT32 len)
{
    return (void *)memset(b, c, (unsigned int)len);
}

int os_memcmp_const(const void *a, const void *b, size_t len)
{
    return memcmp(a, b, len);
}

#if CFG_OS_FREERTOS
void *os_realloc(void *ptr, size_t size)
{
    #ifdef FIX_REALLOC_ISSUE
    return pvPortRealloc(ptr, size);
    #else
    void *tmp;

    if(platform_is_in_interrupt_context())
    {
        os_printf("realloc_risk\r\n");
    }

    tmp = (void *)pvPortMalloc(size);
    if(tmp)
    {
        os_memcpy(tmp, ptr, size);
        vPortFree(ptr);
    }

    return tmp;
    #endif
}

#if !OSMALLOC_STATISTICAL && !CFG_MEM_DEBUG
void *os_malloc(size_t size)
{
    if(platform_is_in_interrupt_context())
    {
        os_printf("malloc_risk\r\n");
    }

    #if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7252N))
    void *ptr = psram_malloc(size);
    if (ptr)
    {
        return ptr;
    }
    #endif

    return (void *)pvPortMalloc(size);
}

void * os_zalloc(size_t size)
{
    void *n = (void *)os_malloc(size);

    if (n)
        os_memset(n, 0, size);
    return n;
}

void os_free(void *ptr)
{
    if(platform_is_in_interrupt_context())
    {
        os_printf("free_risk\r\n");
    }

    if(ptr)
    {
        vPortFree(ptr);
    }
}
#endif
#endif
// EOF

