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

#include "tls_config.h"

#ifdef MBEDTLS_PLATFORM_MEMORY
#include "mem_pub.h"

void *tls_mbedtls_mem_calloc(size_t n, size_t size)
{
    unsigned int len = n * size;
    if(len == 0) {
        return 0;
    }
    return os_zalloc( len );
}

void tls_mbedtls_mem_free(void *ptr)
{
    os_free(ptr);
}

#endif /* !MBEDTLS_PLATFORM_MEMORY */
