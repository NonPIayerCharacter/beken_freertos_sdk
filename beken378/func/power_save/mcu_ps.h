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

#ifndef _MCU_PS_H_
#define _MCU_PS_H_

//#define MCU_PS_DEBUG

#ifdef MCU_PS_DEBUG
#define MCU_PS_PRT                 os_printf

#else
#define MCU_PS_PRT                 os_null_printf

#endif

typedef struct {
    #if (CFG_SUPPORT_ALIOS)
    UINT64 first_tick;
    #else
    UINT32 first_tick;
    #endif
    UINT64 first_tsf;
} MCU_PS_TSF;
typedef struct {
    #if (CFG_SUPPORT_ALIOS)
    UINT64 fclk_tick;
    #else
    UINT32 fclk_tick;
    #endif
    UINT32 machw_tm;
} MCU_PS_MACHW_TM;

uint32_t ps_may_sleep(void);
void ps_send_null(void);

#endif

