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

#ifndef _DRIVER_AUDIO_IF_H_
#define _DRIVER_AUDIO_IF_H_
#include "sys_rtos.h"
#include "rtos_pub.h"

#define AUDIO_BUFF_LEN 	       (6144)

#define DRIVER_AUDIO_DEBUG

#ifdef DRIVER_AUDIO_DEBUG
#define DRIVER_AUDIO_PRT       os_printf
#else
#define DRIVER_AUDIO_PRT       os_null_printf
#endif

typedef struct _driver_ringbuff_s
{
    uint16_t buffer_len;
    uint16_t buffer_fill;
    uint16_t   wptr;
    uint16_t   rptr;
    uint8_t   *buffp;
} driver_ringbuff_t;

typedef struct
{
    uint8_t *data_buff;
    driver_ringbuff_t   aud_rb;
    uint32_t  channels;
} AUDIO_CTRL_BLK;
#endif
// eof

