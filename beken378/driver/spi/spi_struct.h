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

#include "rtos_pub.h"
#include "spi_pub.h"

struct spi_mdev {
    SPI_CFG_ST cfg;
    UINT8 *tx_ptr;
    UINT32 tx_req_len;
    UINT32 tx_remain_cnt;
    beken_semaphore_t tx_sem;
    beken_semaphore_t rx_sem;

    UINT8 *rx_ptr;
    UINT32 rx_len;
    UINT32 rx_offset;
    UINT32 rx_drop;
    UINT32 rx_overflow_cnt;

    UINT32 total_len;
    UINT32 flag;

    beken_mutex_t mutex;
};

struct spi_rx_fifo {
    UINT8 *buffer;
    UINT16 put_index;
    UINT16 get_index;
    UINT32 is_full;
};

struct spi_sdev {
    SPI_CFG_ST cfg;
    UINT32 flag;

    beken_semaphore_t tx_sem;
    UINT8 *tx_ptr;
    UINT32 tx_len;

    beken_semaphore_t rx_sem;
    struct spi_rx_fifo *rx_fifo;

    beken_mutex_t mutex;
};

// eof

