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
#include "rtos_pub.h"
#include "BkDriverRng.h"
#include "drv_model_pub.h"
#include "error.h"
#include "uart_pub.h"
#if !(SOC_BK7252N == CFG_SOC_NAME)
#include "irda_pub.h"
#else
#include "irda_pub_bk7252n.h"
#endif
#include <stdlib.h>
#include "mem_pub.h"

#if (SOC_BK7231 == CFG_SOC_NAME)
int bk_rand(void)
{
    int i = (int)prandom_get();
    return (i & RAND_MAX);
}

#else
int bk_rand(void)
{
    int i = 0;

    BkRandomNumberRead(&i, sizeof(i));
    return (i & RAND_MAX);
}

OSStatus BkRandomNumberRead( void *inBuffer, int inByteCount )
{
    uint32_t i;
    uint32_t param = 0;


    ASSERT(inBuffer);
    for (i = 0; i < inByteCount; i += sizeof(param)) {
        sddev_control(IRDA_DEV_NAME, TRNG_CMD_GET, &param);
        os_memcpy((uint8_t *)inBuffer+i, (uint8_t *)&param, sizeof(param) > (inByteCount - i) ? (inByteCount - i) : sizeof(param));
    }

    return 0;
}
#endif
// eof

