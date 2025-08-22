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

#ifndef _SDIO_PUB_H_
#define _SDIO_PUB_H_

#include "doubly_list.h"


//#define SDIO_TEST

#define SDIO_DEV_NAME       "sdio"

#define SDIO_FAILURE        ((UINT32)-1)
#define SDIO_SUCCESS        (0)

#define BLOCK_LEN           (512 * 8)

#define H2S_RD_SYNC         (1)
#define H2S_WR_SYNC         (2)
#define H2S_RD_SPECIAL      (5)

#define S2H_WR_SPECIAL      (6)
#define S2H_RD_SYNC         (3)
#define S2H_WR_SYNC         (4)

#define SDIO_DUMMY_BUFF_ADDR             (0x100)
#define SDIO_DUMMY_LENGTH                (0x100)

#define SDIO_CMD_GET_FREE_NODE           (0xBB)
#define SDIO_CMD_REG_RX_CALLBACK         (0xBC)
#define SDIO_CMD_PUSH_FREE_NODE          (0xBD)
#define SDIO_CMD_GET_CNT_FREE_NODE       (0xBE)
#define SDIO_CMD_CLEAR_TX_VALID          (0xC0)
#define SDIO_CMD_SET_TX_VALID            (0xC1)
#define SDIO_CMD_PEEK_H2S_COUNT          (0xC2)
#define SDIO_CMD_PEEK_S2H_COUNT          (0xC3)

typedef void (*SDIO_FUNC)(void *Lparam, void *Rparam);

typedef struct _sdio_mem_node_
{
    LIST_HEADER_T node_list;

    UINT8 *orig_addr;

    UINT8 *addr;
    UINT32 length;

    SDIO_FUNC callback;
    void *Lparam;
    void *Rparam;
} SDIO_NODE_T, *SDIO_NODE_PTR;

/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern void sdio_init(void);
extern void sdio_exit(void);

#endif // _SDIO_PUB_H_
