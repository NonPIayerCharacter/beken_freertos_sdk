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

#ifndef _BEKEN_AIR_KISS_H_
#define _BEKEN_AIR_KISS_H_

#include "uart_pub.h"
#include "rtos_pub.h"

#define BK_AIRKISS_DEBUG           1
#if BK_AIRKISS_DEBUG
#define BK_AIRKISS_PRT             os_printf
#define BK_AIRKISS_WARN            warning_prf
#define BK_AIRKISS_FATAL           fatal_prf
#else
#define BK_AIRKISS_PRT             null_prf
#define BK_AIRKISS_WARN            null_prf
#define BK_AIRKISS_FATAL           null_prf
#endif

#define AIRKISS_SUCCESS                0
#define AIRKISS_FAILURE                1

#define MAX_CHANNEL_NUM                13
#define BK_AIRKISS_TIMEOUT             (1000 * 90)
#define BK_AIRKISS_SWITCH_TIMER        50     // ms
#define BK_AIRKISS_CYCLE_TIMER         20000  // 20s
#define MAX_UDP_RANDOM_SEND            20

#define BK_AIRKISS_THD_PRIORITY        BEKEN_DEFAULT_WORKER_PRIORITY


extern uint32_t bk_airkiss_get_decode_status(void);
extern uint32_t bk_airkiss_decode_isnot_over(void);

#endif // _BEKEN_AIR_KISS_H_
// eof
