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

#ifndef _FAKE_CLOCK_PUB_H_
#define _FAKE_CLOCK_PUB_H_

#include "include.h"

typedef enum
{
    BK_TIMER_ID0     = 0,//BKTIMER0
    BK_TIMER_ID1,
    BK_TIMER_ID2,
    BK_TIMER_ID3,
    BK_TIMER_ID4,
    BK_TIMER_ID5,
    BK_PWM_TIMER_ID0     = (6),//PWM0
    BK_PWM_TIMER_ID1,
    BK_PWM_TIMER_ID2,
    BK_PWM_TIMER_ID3,
    BK_PWM_TIMER_ID4,
    BK_PWM_TIMER_ID5,
    BK_TIMER_ID_MAX_COUNT
} BK_HW_TIMER_INDEX;

#define FCLK_PWM_ID           PWM0

#if (CFG_SOC_NAME != SOC_BK7231)
#define CAL_TIMER_ID           BKTIMER2
#endif

#if (CFG_SUPPORT_RTT)
#define FCLK_DURATION_MS      (1000 / RT_TICK_PER_SECOND)
#define FCLK_SECOND           (RT_TICK_PER_SECOND)
#else
#define FCLK_DURATION_MS      2
#define FCLK_SECOND           (1000/FCLK_DURATION_MS)
#endif

#define TICK_PER_SECOND       FCLK_SECOND

#define BK_MS_TO_TICKS(x)     ((x) / (FCLK_DURATION_MS))
#define BK_TICKS_TO_MS(x)     ((x) * (FCLK_DURATION_MS))

#if !(CFG_SOC_NAME == SOC_BK7221U)
#define         ONE_CAL_TIME        15000
#else
#define         ONE_CAL_TIME        1000
#endif

#if (0 == CFG_LOW_VOLTAGE_PS)
typedef struct
{
    UINT32 fclk_tick;
    UINT32 tmp1;
} CAL_TICK_T;
#else
typedef struct
{
    UINT64 fclk_tick;
    UINT64 time_us;
} CAL_TICK_T;
void fclk_cal_tick(void);
#endif

extern UINT64 fclk_get_tick(void);
extern UINT32 fclk_get_second(void);
extern void fclk_reset_count(void);
extern void fclk_init(void);
extern UINT32 fclk_from_sec_to_tick(UINT32 sec);
extern UINT32 fclk_cal_endvalue(UINT32 mode);
BK_HW_TIMER_INDEX fclk_get_tick_id(void);;

#endif // _FAKE_CLOCK_PUB_H_
// eof

