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

#ifndef _CHARGE_H_
#define _CHARGE_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#define CHARGE_ADDR_BASE                        (0x008001C0)

/*
 * reg0x0, RW(def 0)
 * bit[7:0]: charge0~7 interrupt enable
 */
#define CHARGE_REG0X0                           (CHARGE_ADDR_BASE + 0*4)
#define CHARGE_REG0X0_VAL                       (*((volatile uint32_t *)(CHARGE_REG0X0)))
#define CHARGE_REG0X0_CHARGE_INT_EN             (1)
#define CHARGE_REG0X0_CHARGE_INT_DIS            (0)
#define CHARGE_REG0X0_CHARGE_X_INT_EN(n)        (CHARGE_REG0X0_VAL | (1<<n))
#define CHARGE_REG0X0_CHARGE_X_INT_DIS(n)       (CHARGE_REG0X0_VAL & ~(1<<n))
#define CHARGE_REG0X0_CHARGE_X_INT_GET(n)       (CHARGE_REG0X0_VAL & (1<<n))

/*
 * reg0x1, RW(def 0)
 * bit[15:0]: charge0~7 interrupt type
 * 0x0: low level
 * 0x1: high level
 * 0x2: posedge
 * 0x3: negedge
 */
#define CHARGE_REG0X1                           (CHARGE_ADDR_BASE + 1*4)
#define CHARGE_REG0X1_VAL                       (*((volatile uint32_t *)(CHARGE_REG0X1)))
#define CHARGE_REG0X1_CHARGE_INT_TYPE_LO_LV     (0x0)
#define CHARGE_REG0X1_CHARGE_INT_TYPE_HI_LV     (0x1)
#define CHARGE_REG0X1_CHARGE_INT_TYPE_P_EDGE    (0x2)
#define CHARGE_REG0X1_CHARGE_INT_TYPE_N_EDGE    (0x3)
#define CHARGE_REG0X1_CHARGE_INT_TYPE_MASK      (0x3)
#define CHARGE_REG0X1_CHARGE_X_INT_POS(n)       (2*n)
#define CHARGE_REG0X1_CHARGE_X_INT_MASK(n)      (0x3<<(2*n))
#define CHARGE_REG0X1_CHARGE_X_INT_CLR(n)       (CHARGE_REG0X1_VAL & ~(0x3<<(2*n)))
#define CHARGE_REG0X1_CHARGE_X_INT_GET(n)       (CHARGE_REG0X1_VAL & (0x3<<(2*n)))
#define CHARGE_REG0X1_CHARGE_X_INT_SET(n,m)     ((CHARGE_REG0X1_VAL & ~(0x3<<(2*n))) | (m<<(2*n)))

/*
 * reg0x2
 * R, bit[15:8]: charge0~7 input state
 * RW, bit[7:0]: charge0~7 interrupt status, write '1' for clear interrupt
 */
#define CHARGE_REG0X2                           (CHARGE_ADDR_BASE + 2*4)
#define CHARGE_REG0X2_VAL                       (*((volatile uint32_t *)(CHARGE_REG0X2)))
#define CHARGE_REG0X2_CHARGE_X_STA_POS(n)       (1<<(n+8))
#define CHARGE_REG0X2_CHARGE_X_STA_GET(n)       (CHARGE_REG0X2_VAL & (1<<(n+8)))
#define CHARGE_REG0X2_CHARGE_INT_STA_CLR        (1)
#define CHARGE_REG0X2_CHARGE_X_INT_STA_POS(n)   (1<<n)
#define CHARGE_REG0X2_CHARGE_X_INT_STA_GET(n)   (CHARGE_REG0X2_VAL & (1<<n))
#define CHARGE_REG0X2_CHARGE_X_INT_STA_CLR(n)   (CHARGE_REG0X2_VAL & ~(1<<n))

extern UINT32 charge_ctrl(UINT32 cmd, void *param);

#endif

#endif // _CHARGE_H_
// eof


