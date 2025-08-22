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

#ifndef _BK_TIMER_H_
#define _BK_TIMER_H_
#include "pwm.h"

#define BK_TIMER_FAILURE                (1)
#define BK_TIMER_SUCCESS                (0)

#if (CFG_SOC_NAME != SOC_BK7231)
#define TIMER0_CNT                                     (PWM_NEW_BASE + 0 * 4)

#define TIMER1_CNT                                     (PWM_NEW_BASE + 1 * 4)

#define TIMER2_CNT                                     (PWM_NEW_BASE + 2 * 4)

#define TIMER0_2_CTL                                   (PWM_NEW_BASE + 3 * 4)
#define TIMERCTL0_EN_BIT                               (0x01UL << 0)
#define TIMERCTL1_EN_BIT                               (0x01UL << 1)
#define TIMERCTL2_EN_BIT                               (0x01UL << 2)
#define TIMERCTLA_CLKDIV_POSI                          (3)
#define TIMERCTLA_CLKDIV_MASK                          (0x07)
#define TIMERCTL0_INT_BIT                              (0x01UL << 7)
#define TIMERCTL1_INT_BIT                              (0x01UL << 8)
#define TIMERCTL2_INT_BIT                              (0x01UL << 9)
#define TIMERCTLA_INT_POSI                             (7)
#define REG_TIMERCTLA_PERIOD_ADDR(n)                    (PWM_NEW_BASE +  0x04 * (n))

#if (CFG_SOC_NAME != SOC_BK7221U)
#define TIMER0_2_READ_CTL                             (PWM_NEW_BASE + 4 * 4)
#define TIMER0_2_READ_OP_BIT                          (1<<0)
#define TIMER0_2_READ_INDEX_POSI                       (2)
#define TIMER0_2_READ_INDEX_MASK                       (0x3)
#define TIMER0_2_READ_INDEX_0                          (0)
#define TIMER0_2_READ_INDEX_1                          (1)
#define TIMER0_2_READ_INDEX_2                          (2)

#define TIMER0_2_READ_VALUE                           (PWM_NEW_BASE + 5 * 4)
#endif

#define TIMER3_CNT                                     (PWM_NEW_BASE + 0x10 * 4)
#define TIMER4_CNT                                     (PWM_NEW_BASE + 0x11 * 4)
#define TIMER5_CNT                                     (PWM_NEW_BASE + 0x12 * 4)

#define TIMER3_5_CTL                                   (PWM_NEW_BASE + 0x13 * 4)
#define TIMERCTL3_EN_BIT                               (0x01UL << 0)
#define TIMERCTL4_EN_BIT                               (0x01UL << 1)
#define TIMERCTL5_EN_BIT                               (0x01UL << 2)
#define TIMERCTLB_CLKDIV_POSI                          (3)
#define TIMERCTLB_CLKDIV_MASK                          (0x07)
#define TIMERCTL3_INT_BIT                              (0x01UL << 7)
#define TIMERCTL4_INT_BIT                              (0x01UL << 8)
#define TIMERCTL5_INT_BIT                              (0x01UL << 9)
#define TIMERCTLB_INT_POSI                             (7)
#define REG_TIMERCTLB_PERIOD_ADDR(n)                   (PWM_NEW_BASE + 0x10 * 4 + 0x04 * (n - 3))

#define REG_TIMERCTL_PERIOD_ADDR(group, id)            (PWM_NEW_BASE + 0x10 * group * 4 + 0x04 * (id - group * 3))

#if (CFG_SOC_NAME != SOC_BK7221U)
#define TIMER3_5_READ_CTL                             (PWM_NEW_BASE + 0x14 * 4)
#define TIMER3_5_READ_OP_BIT                          (1<<0)
#define TIMER3_5_READ_INDEX_POSI                       (2)
#define TIMER3_5_READ_INDEX_MASK                       (0x3)
#define TIMER3_5_READ_INDEX_3                          (0)
#define TIMER3_5_READ_INDEX_4                          (1)
#define TIMER3_5_READ_INDEX_5                          (2)

#define TIMER3_5_READ_VALUE                           (PWM_NEW_BASE + 0x15 * 4)
#endif

#define TIMER_CHANNEL_NO                                  6
#endif
UINT32 bk_timer_ctrl(UINT32 cmd, void *param);

#endif //_TIMER_H_


