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

#ifndef _IRDA_H_
#define _IRDA_H_

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


#define IRDA_BASE                     (0x00802400)

#define IRDA_CTRL                     (IRDA_BASE + 0 * 4)
#define IRDA_NEC_EN                   (0x01UL << 0)
#define IRDA_POLARITY                 (0x01UL << 1)
#define UART_RX_ENABLE                (0x01UL << 2)
#define UART_LEN_POSI                 (3)
#define UART_LEN_MASK                 (0x03)
#define UART_PAR_EN                   (0x01UL << 5)
#define UART_PAR_MODE                 (0x01UL << 6)
#define UART_STOP_LEN                 (0x01UL << 7)
#define CLK_DIVID_POSI                (8)
#define CLK_DIVID_MASK                (0xFFFF)

#define IRDA_INT_MASK                 (IRDA_BASE + 1 * 4)
#define IRDA_END_INT_MASK             (0x01UL << 0)
#define IRDA_RIGHT_INT_MASK           (0x01UL << 1)
#define IRDA_REPEAT_INT_MASK          (0x01UL << 2)
#define FIFO_WREN_INT_MASK            (0x01UL << 3)
#define PARITY_ERR_INT_MASK           (0x01UL << 4)
#define STOP_ERR_INT_MASK             (0x01UL << 5)
#define INT_MASK_EN					(0x3f)

#define IRDA_INT                      (IRDA_BASE + 2 * 4)
#define IRDA_END_INT                  (0x01UL << 0)
#define IRDA_RIGHT_INT                (0x01UL << 1)
#define IRDA_REPEAT_INT               (0x01UL << 2)
#define FIFO_WREN_INT                 (0x01UL << 3)
#define PARITY_ERR_INT                (0x01UL << 4)
#define STOP_ERR_INT                  (0x01UL << 5)

#define RX_FIFO_DOUT                  (IRDA_BASE + 3 * 4)

#define TRNG_BASE                     (0x00802480)

#define TRNG_CTRL                     (TRNG_BASE + 0 * 4)
#define TRNG_EN                       (0x01UL << 0)

#define TRNG_DATA                     (TRNG_BASE + 1 * 4)


#endif
