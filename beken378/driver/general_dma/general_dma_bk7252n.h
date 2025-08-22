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

#ifndef __GENER_DMA_BK7252N_H__
#define __GENER_DMA_BK7252N_H__

#if (CFG_SOC_NAME == SOC_BK7252N)

#include "sys_config.h"

#if CFG_GENERAL_DMA

#include "uart_pub.h"
#ifdef GENER_DMA_DEBUG
#define GENER_DMA_PRT                   os_printf
#define GENER_DMA_WPRT                  os_printf
#else
#define GENER_DMA_PRT                   null_prf
#define GENER_DMA_WPRT                  null_prf
#endif

#define DMA_LL_TRANS_LEN_MAX                    (0x10000)

#define GENER_DMA_BASE                          (0x00809000)

#define GENER_DMA_REG0_DEV_ID                   (GENER_DMA_BASE + 0x00*4)
#define GENER_DMA_REG1_VER_ID                   (GENER_DMA_BASE + 0x01*4)

#define GENER_DMA_REG2_ADDR                     (GENER_DMA_BASE + 0x02*4)
#define GENER_DMA_REG2_SOFT_RESET               (1 << 0)
#define GENER_DMA_REG2_BPS_CLKGATE              (1 << 1)
#define GENER_DMA_REG2_PRIO_MODE                (1 << 2)

#define GENER_DMA_REG4_ADDR                     (GENER_DMA_BASE + 0x04*4)
#define GENER_DMA_REG4_SECURE_ATTR_POSI         (0)
#define GENER_DMA_REG4_SECURE_ATTR_MASK         (0xFFF)

#define GENER_DMA_REG5_ADDR                     (GENER_DMA_BASE + 0x05*4)
#define GENER_DMA_REG5_PRIVILEGED_ATTR_POSI     (0)
#define GENER_DMA_REG5_PRIVILEGED_ATTR_MASK     (0xFFF)

#define GENER_DMA_REG6_ADDR                     (GENER_DMA_BASE + 0x06*4)
#define GENER_DMA_REG6_INT0_STATUS_POSI         (0)
#define GENER_DMA_REG6_INT0_STATUS_MASK         (0xFFF)

#define GENER_DMA_REG10_ADDR                    (GENER_DMA_BASE + 0x0A*4)
#define GENER_DMA_REG10_INT_ALLOCATE_POSI       (0)
#define GENER_DMA_REG10_INT_ALLOCATE_MASK       (0xFFFFFF)

/*
 * CHANNEL_X: 0~11
 * CH0:  REG16  ~ REG28
 * CH1:  REG32  ~ REG44
 * CH2:  REG48  ~ REG60
 * CH3:  REG64  ~ REG76
 * CH4:  REG80  ~ REG92
 * CH5:  REG96  ~ REG108
 * CH6:  REG112 ~ REG124
 * CH7:  REG128 ~ REG140
 * CH8:  REG144 ~ REG156
 * CH9:  REG160 ~ REG172
 * CH10: REG176 ~ REG188
 * CH11: REG192 ~ REG204
 */

#define GENER_DMA_CH0_CTRL_ADDR             (GENER_DMA_BASE + 0x10*4)
#define GDMA_DMA_EN                         (1 << 0)
#define GDMA_FINISH_INTEN                   (1 << 1)
#define GDMA_HALF_FINISH_INTEN              (1 << 2)
#define GDMA_DMA_MODE                       (1 << 3)
#define GDMA_SRC_DATA_WIDTH_POSI            (4)
#define GDMA_SRC_DATA_WIDTH_MASK            (0x3)
#define GDMA_DEST_DATA_WIDTH_POSI           (6)
#define GDMA_DEST_DATA_WIDTH_MASK           (0x3)
#define GDMA_SRC_ADDR_INC                   (1 << 8)
#define GDMA_DEST_ADDR_INC                  (1 << 9)
#define GDMA_SRC_ADDR_LOOP                  (1 << 10)
#define GDMA_DEST_ADDR_LOOP                 (1 << 11)
#define GDMA_CHN_PRIOPRITY_POSI             (12)
#define GDMA_CHN_PRIOPRITY_MASK             (0x7)
#define GDMA_TRANS_LEN_POSI                 (16)
#define GDMA_TRANS_LEN_MASK                 (0xFFFF)

#define GENER_DMA_CH0_DEST_START_ADDR       (GENER_DMA_BASE + 0x11*4)
#define GENER_DMA_CH0_SOURCE_START_ADDR     (GENER_DMA_BASE + 0x12*4)
#define GENER_DMA_CH0_DEST_LOOP_END_ADDR    (GENER_DMA_BASE + 0x13*4)
#define GENER_DMA_CH0_DEST_LOOP_START_ADDR  (GENER_DMA_BASE + 0x14*4)
#define GENER_DMA_CH0_SRC_LOOP_END_ADDR     (GENER_DMA_BASE + 0x15*4)
#define GENER_DMA_CH0_SRC_LOOP_START_ADDR   (GENER_DMA_BASE + 0x16*4)

#define GENER_DMA_CH0_REQ_MUX_ADDR          (GENER_DMA_BASE + 0x17*4)
#define GDMA_SRC_REG_MUX_POSI               (0)
#define GDMA_SRC_REG_MUX_MASK               (0x1F)
#define GDMA_DEST_REQ_MUX_POSI              (5)
#define GDMA_DEST_REQ_MUX_MASK              (0x1F)
#define GDMA_SRC_RD_INTLV_POSI              (12)
#define GDMA_SRC_RD_INTLV_MASK              (0xF)
#define GDMA_DEST_WR_INTLV_POSI             (16)
#define GDMA_DEST_WR_INTLV_MASK             (0xF)
#define GDMA_SRC_SECU_ATTR                  (1 << 20)
#define GDMA_DEST_SECU_ATTR                 (1 << 21)
#define GDMA_BUS_ERR_INTEN                  (1 << 22)
#define GDMA_SRC_BURST_LEN_POSI             (24)
#define GDMA_SRC_BURST_LEN_MASK             (0x3)
#define GDMA_DEST_BURST_LEN_POSI            (26)
#define GDMA_DEST_BURST_LEN_MASK            (0x3)

#define GENER_DMA_CH0_SRC_PAUSE_ADDR        (GENER_DMA_BASE + 0x18*4)
#define GENER_DMA_CH0_DEST_PAUSE_ADDR       (GENER_DMA_BASE + 0x19*4)
#define GENER_DMA_CH0_SRC_RD_ADDR           (GENER_DMA_BASE + 0x1A*4)
#define GENER_DMA_CH0_DEST_WR_ADDR          (GENER_DMA_BASE + 0x1B*4)

#define GENER_DMA_CH0_STATUS_ADDR           (GENER_DMA_BASE + 0x1C*4)
#define GDMA_REMAIN_LEN_POSI                (0)
#define GDMA_REMAIN_LEN_MASK                (0x1FFFF)
#define GDMA_FLUSH_SRC_BUFF                 (1 << 17)
#define GDMA_INT_FINISH                     (1 << 18)
#define GDMA_INT_FINISH_POSI                (18)
#define GDMA_INT_HALF_FINISH                (1 << 19)
#define GDMA_INT_HALF_FINISH_POSI           (19)
#define GDMA_INT_BUS_ERR                    (1 << 20)
#define GDMA_REPEAT_WR_PAUSE                (1 << 22)
#define GDMA_REPEAT_RD_PAUSE                (1 << 23)
#define GDMA_INTCNT_FINISH_POSI             (24)
#define GDMA_INTCNT_FINISH_MASK             (0xF)
#define GDMA_INTCNT_HALF_FINISH_POSI        (28)
#define GDMA_INTCNT_HALF_FINISH_MASK        (0xF)

UINT32 gdma_ctrl(UINT32 cmd, void *param);

#endif // CFG_GENERAL_DMA

#endif

#endif // __GENER_DMA_BK7252N_H__
