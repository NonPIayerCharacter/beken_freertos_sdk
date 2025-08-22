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

#ifndef _IRDA_BK7252N_H_
#define _IRDA_BK7252N_H_

#define CONFIG_TRNG_PATCH    1

#if (SOC_BK7252N == CFG_SOC_NAME)
#define IRDA_DEBUG

#ifdef IRDA_DEBUG
#define IRDA_PRT      os_printf
#define IRDA_WARN     warning_prf
#define IRDA_FATAL    fatal_prf
#else
#define IRDA_PRT      null_prf
#define IRDA_WARN     null_prf
#define IRDA_FATAL    null_prf
#endif

#define IRDA_BASE                               (0x00802400)

#define IRDA_REG0X0_DEV_ID                      (IRDA_BASE + 0 * 4)
#define IRDA_REG0X1_VER_ID                      (IRDA_BASE + 1 * 4)
#define IRDA_REG0X2_CLK_RST                     (IRDA_BASE + 2 * 4)
#define IRDA_REG0X2_CLK_RST_SOFT_RESET          (1 << 0)
#define IRDA_REG0X2_CLK_RST_CLKG_BYPASS         (1 << 1)
#define IRDA_REG0X3_DEV_STA                     (IRDA_BASE + 3 * 4)

#define IRDA_REG0X4_CONFIG                      (IRDA_BASE + 4 * 4)
#define IRDA_REG0X4_CONFIG_RX_EN                (1 << 0)
#define IRDA_REG0X4_CONFIG_RX_INITIAL_LV        (1 << 1)
#define IRDA_REG0X4_CONFIG_TX_EN                (1 << 2)
#define IRDA_REG0X4_CONFIG_TX_INITIAL_LV        (1 << 3)
#define IRDA_REG0X4_CONFIG_TX_START             (1 << 4)
#define IRDA_REG0X4_CONFIG_TX_IRDA_PWD          (1 << 5)
#define IRDA_REG0X4_CONFIG_CLK_FREQ_IN_POS      (8)
#define IRDA_REG0X4_CONFIG_CLK_FREQ_IN_MASK     (0x7F)
#define IRDA_REG0X4_CONFIG_RSTN                 (1 << 15)
#define IRDA_REG0X4_CONFIG_TX_DATA_NUM_POS      (16)
#define IRDA_REG0X4_CONFIG_TX_DATA_NUM_MASK     (0x3FF)

#define IRDA_REG0X5_INT                         (IRDA_BASE + 5 * 4)
#define IRDA_REG0X5_INT_TX_FIFO_THR_POS         (0)
#define IRDA_REG0X5_INT_TX_FIFO_THR_MASK        (0xFF)
#define IRDA_REG0X5_INT_RX_FIFO_THR_POS         (8)
#define IRDA_REG0X5_INT_RX_FIFO_THR_MASK        (0xFF)
#define IRDA_REG0X5_INT_RX_TIMEOUT_CNT_POS      (16)
#define IRDA_REG0X5_INT_RX_TIMEOUT_CNT_MASK     (0xFFFF)

#define IRDA_REG0X6_FIFO                        (IRDA_BASE + 6 * 4)
#define IRDA_REG0X6_FIFO_TX_FIFO_CNT_POS        (0)
#define IRDA_REG0X6_FIFO_TX_FIFO_CNT_MASK       (0xFF)
#define IRDA_REG0X6_FIFO_RX_FIFO_CNT_POS        (8)
#define IRDA_REG0X6_FIFO_RX_FIFO_CNT_MASK       (0xFF)
#define IRDA_REG0X6_FIFO_TX_FIFO_FULL           (1 << 16)
#define IRDA_REG0X6_FIFO_TX_FIFO_EMPTY          (1 << 17)
#define IRDA_REG0X6_FIFO_RX_FIFO_FULL           (1 << 18)
#define IRDA_REG0X6_FIFO_RX_FIFO_EMPTY          (1 << 19)
#define IRDA_REG0X6_FIFO_TX_FIFO_WR_READY       (1 << 20)
#define IRDA_REG0X6_FIFO_RX_FIFO_RD_READY       (1 << 21)
#define IRDA_REG0X6_FIFO_RXDATA_NUM_POS         (22)
#define IRDA_REG0X6_FIFO_RXDATA_NUM_MASK        (0x3FF)

#define IRDA_REG0X7_DATA                        (IRDA_BASE + 7 * 4)
#define IRDA_REG0X7_DATA_FIFO_DATA_RX_POS       (0)
#define IRDA_REG0X7_DATA_FIFO_DATA_RX_MASK      (0xFFFF)
#define IRDA_REG0X7_DATA_FIFO_DATA_TX_POS       (16)
#define IRDA_REG0X7_DATA_FIFO_DATA_TX_MASK      (0xFFFF)

#define IRDA_REG0X8_INT_MASK                    (IRDA_BASE + 8 * 4)
#define IRDA_REG0X8_INT_MASK_TX_NEED_WR_MASK    (1 << 0)
#define IRDA_REG0X8_INT_MASK_RX_NEED_RD_MASK    (1 << 1)
#define IRDA_REG0X8_INT_MASK_TX_DONE_MASK       (1 << 2)
#define IRDA_REG0X8_INT_MASK_RX_TIMEOUT_MASK    (1 << 3)
#define IRDA_REG0X8_INT_MASK_RX_OVERFLOW_MASK   (1 << 4)

#define IRDA_REG0X9_INT_STA                     (IRDA_BASE + 9 * 4)
#define IRDA_REG0X9_INT_STA_TX_NEED_WR_STA      (1 << 0)
#define IRDA_REG0X9_INT_STA_RX_NEED_ED_STA      (1 << 1)
#define IRDA_REG0X9_INT_STA_TX_DONE_STA         (1 << 2)
#define IRDA_REG0X9_INT_STA_RX_DONE_STA         (1 << 3)
#define IRDA_REG0X9_INT_STA_RX_OVERFLOW_STA     (1 << 4)

#define IRDA_REG0XA_CONFIG2                     (IRDA_BASE + 10 * 4)
#define IRDA_REG0XA_CONFIG2_CARRIER_PERIOD_POS  (0)
#define IRDA_REG0XA_CONFIG2_CARRIER_PERIOD_MASK (0xFF)
#define IRDA_REG0XA_CONFIG2_CARRIER_DUTY_DOS    (8)
#define IRDA_REG0XA_CONFIG2_CARRIER_DUTY_MASK   (0xFF)
#define IRDA_REG0XA_CONFIG2_CARRIER_POLARITY    (1 << 16)
#define IRDA_REG0XA_CONFIG2_CARRIER_ENABLE      (1 << 17)
#define IRDA_REG0XA_CONFIG2_RX_START_THR_POS    (20)
#define IRDA_REG0XA_CONFIG2_RX_START_THR_MASK   (0xFFF)

#define IRDA_REG0XB_CONFIG3                     (IRDA_BASE + 11 * 4)
#define IRDA_REG0XB_CONFIG3_GLITCH_ENABLE       (1 << 0)
#define IRDA_REG0XB_CONFIG3_GLITCH_THR_POS      (16)
#define IRDA_REG0XB_CONFIG3_GLITCH_THR_MASK     (0xFFF)

#define TRNG_BASE                       (0x00802480)

#define TRNG_CTRL                       (TRNG_BASE + 0 * 4)
#define TRNG_EN                         (0x01UL << 0)

#define TRNG_DATA                       (TRNG_BASE + 1 * 4)

#endif
#endif
