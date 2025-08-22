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

#ifndef __DRIVER_USB_H__
#define __DRIVER_USB_H__

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include "include.h"
#include "ffconf.h"


#if CFG_USE_USB_HOST
#define USB_RET_OK			              0
#define USB_RET_ERROR 		              1
#define USB_RET_CONNECT 	              2
#define USB_RET_DISCONNECT                3
#define USB_RET_READ_OK 	              4
#define USB_RET_WRITE_OK 	              5

typedef struct __driver_udisk_s
{
    uint32        total_block;
    uint16		  block_size;
    uint16		  InitFlag;
} driver_udisk_t;

extern uint8 udisk_init(void);
extern void udisk_uninit(void);
extern uint32 udisk_get_size(void);
extern uint8 udisk_is_attached(void);
extern int udisk_rd_blk_sync(uint32 first_block, uint32 block_num, uint8 *dest );
extern int udisk_wr_blk_sync(uint32 first_block, uint32 block_num, uint8 *dest);
#endif


#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif      /* __DRIVER_USB_H__ */
