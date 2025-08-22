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

#ifndef _RTC_REG_PUB_H_
#define _RTC_REG_PUB_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#define RTC_REG_DEV_NAME        "rtc_reg"
#define RTC_CMD_MAGIC           (0xeee0000)
enum
{
    CMD_RTC_INIT = RTC_CMD_MAGIC + 1,
    CMD_RTC_START,
    CMD_RTC_TMR_PROG,
    CMD_RTC_TMR_CLEAR,
};
extern UINT32 rtc_reg_ctrl(UINT32 cmd, void *param);

extern void rtc_reg_init(void);
extern void rtc_reg_start(void);
extern void rtc_reg_exit(void);
extern uint64_t rtc_reg_get_time_us(void);

#define cal_get_time_us()       rtc_reg_get_time_us()
#endif

#endif // _RTC_REG_PUB_H_
// eof