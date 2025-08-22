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

#pragma once

#define SPI_DEBUG

#ifdef SPI_DEBUG
#define SPI_PRT      os_printf
#define SPI_WARN     warning_prf
#define SPI_FATAL    fatal_prf
#else
#define SPI_PRT      null_prf
#define SPI_WARN     null_prf
#define SPI_FATAL    null_prf
#endif

#define SPI_BASE                           (0x00802700)

#define SPI_CTRL                           (SPI_BASE + 0 * 4)
#define TXINT_MODE_POSI                    (0)
#define TXINT_MODE_MASK                    (0x3)
#define RXINT_MODE_POSI                    (2)
#define RXINT_MODE_MASK                    (0x3)
#define TXOVR_EN                           (0x01UL << 4)
#define RXOVR_EN                           (0x01UL << 5)
#define TXINT_EN                           (0x01UL << 6)
#define RXINT_EN                           (0x01UL << 7)
#define SPI_CKR_POSI                       (8)
#define SPI_CKR_MASK                       (0xFF)
#define NSSMD_POSI                         (16)
#define NSSMD_MASK                         (0x3)
#define BIT_WDTH                           (0x01UL << 18)
#define CKPOL                              (0x01UL << 20)
#define CKPHA                              (0x01UL << 21)
#define MSTEN                              (0x01UL << 22)
#define SPIEN                              (0x01UL << 23)

#define SPI_STAT                           (SPI_BASE + 1 * 4)
#define TXFIFO_EMPTY                       (0x01UL << 0)
#define TXFIFO_FULL                        (0x01UL << 1)
#define RXFIFO_EMPTY                       (0x01UL << 2)
#define RXFIFO_FULL                        (0x01UL << 3)
#define TXINT                              (0x01UL << 8)
#define RXINT                              (0x01UL << 9)
#define MODF                               (0x01UL << 10)
#define TXOVR                              (0x01UL << 11)
#define RXOVR                              (0x01UL << 12)
#define SLVSEL                             (0x01UL << 14)
#define SPIBUSY                            (0x01UL << 15)

#define SPI_DAT                            (SPI_BASE + 2 * 4)
#define SPI_DAT_POSI                       (0)
#define SPI_DAT_MASK                       (0xFFFF)

#define SPI_SLAVE_CTRL                     (SPI_BASE + 3 * 4)
#define SPI_S_CS_UP_INT_EN                 (0x01UL << 1)
#define SPI_S_CS_UP_INT_STATUS             (0x01UL << 4)

/*******************************************************************************
* Function Declarations
*******************************************************************************/
UINT32 spi_ctrl(UINT32 cmd, void *param);
// eof

