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

#ifndef _LA_H_
#define _LA_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#define LA_ADDR_BASE                            (0x00A05000)

#define LA_REG0X0_DEV_ID                        (LA_ADDR_BASE + 0*4)

#define LA_REG0X1_VER_ID                        (LA_ADDR_BASE + 1*4)

#define LA_REG0X2_ADDR                          (LA_ADDR_BASE + 2*4)
#define LA_REG0X2_SOFT_RST                      (1 << 0)
#define LA_REG0X2_BPS_CLKGATE                   (1 << 1)

#define LA_REG0X3_ADDR                          (LA_ADDR_BASE + 3*4)
#define LA_REG0X3_LA_SMP_EN                     (1 << 0)
#define LA_REG0X3_LA_SMP_CLK_INV                (1 << 1)
#define LA_REG0X3_LA_SMP_MOD_POSI               (2)
#define LA_REG0X3_LA_SMP_MOD_MASK               (0x3)
#define LA_REG0X3_LA_SMP_INT_EN_POSI            (4)
#define LA_REG0X3_LA_SMP_INT_EN_MASK            (0x3)
#define LA_REG0X3_MEM_SECU_ATTR                 (1 << 6)
#define LA_REG0X3_LA_SMP_SOURCE_POSI            (8)
#define LA_REG0X3_LA_SMP_SOURCE_MASK            (0x7)
#define LA_REG0X3_LA_SMP_LEN_POSI               (12)
#define LA_REG0X3_LA_SMP_LEN_MASK               (0xFFFFF)

#define LA_REG0X4_LA_SMP_VALUE                  (LA_ADDR_BASE + 4*4)

#define LA_REG0X5_LA_SMP_MASK                   (LA_ADDR_BASE + 5*4)

#define LA_REG0X6_LA_SMP_DATA                   (LA_ADDR_BASE + 6*4)

#define LA_REG0X7_LA_SMP_START_ADDR             (LA_ADDR_BASE + 7*4)

#define LA_REG0X8_ADDR                          (LA_ADDR_BASE + 8*4)
#define LA_REG0X8_LA_SMP_INT_STATUS_POSI        (0)
#define LA_REG0X8_LA_SMP_INT_STATUS_MASK        (0x3)
#define LA_REG0X8_BUS_ERR_FLAG                  (1 << 2)
#define LA_REG0X8_BUS_ERR_FLAG_POSI             (2)
#define LA_REG0X8_BUS_ERR_FLAG_MASK             (0x1)

extern UINT32 la_ctrl(UINT32 cmd, void *param);

#endif

#endif //_LA_H_
