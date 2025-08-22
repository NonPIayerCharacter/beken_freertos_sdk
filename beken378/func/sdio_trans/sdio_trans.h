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

#include "tx_swdesc.h"

#if  CFG_SDIO_TRANS

#define SDIO_TRANS_DEBUG

#ifdef SDIO_TRANS_DEBUG
#define SDIO_TRANS_PRT       os_printf
#define SDIO_TRANS_WPRT      warning_prf
#else
#define SDIO_TRANS_PRT       os_null_printf
#define SDIO_TRANS_WPRT      warning_prf
#endif

#define SDIO_TRANS_FAILURE        ((UINT32)-1)
#define SDIO_TRANS_SUCCESS        (0)

typedef struct _stm32_frame_hdr
{
    UINT16 len;
    #if FOR_SDIO_BLK_512
    UINT8 type;
    UINT8 seq;
    #else
    UINT16 type;
    #endif
} STM32_FRAME_HDR;

#endif
