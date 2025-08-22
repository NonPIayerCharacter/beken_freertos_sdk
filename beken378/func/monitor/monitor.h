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

#ifndef _MONITOR_MAIN_H_
#define _MONITOR_MAIN_H_

#define MONITOR_MAX_CHANNELS	13
#define MONITOR_SWITCH_TIMER	110// ms
#if CFG_AP_MONITOR_COEXIST_TBTT
#define MONITOR_TBTT_DUR_TIMER	10//ms
/*Minimal presence duration on a channel + channel switch time*/
#define MONITOR_CHANNEL_SWITCH_TIMER_MARGIN	8//ms
#endif

#define MONITOR_DEBUG	0

#if MONITOR_DEBUG
#define MONITOR_INFO            os_printf
#define MONITOR_PRT             os_printf
#define MONITOR_FATAL           fatal_prf
#else
#define MONITOR_INFO            null_prf
#define MONITOR_PRT             os_printf
#define MONITOR_FATAL           fatal_prf
#endif

typedef struct
{
    u16 channel_list[MONITOR_MAX_CHANNELS + 1];
    u8 cur_channel_idx;
    u8 all_channel_nums;
    u16 softap_channel;
} monitor_channel_t;
#endif // _MONITOR_MAIN_H_
// eof
