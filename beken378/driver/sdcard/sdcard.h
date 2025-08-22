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

#ifndef _SDCARD_H_
#define _SDCARD_H_

#if CFG_USE_SDCARD_HOST
#include "sdcard_pub.h"

typedef void (*SD_DETECT_FUN)(void);

typedef struct _sdcard_
{
    UINT32  total_block;
    UINT16  block_size;
    UINT16  card_rca;
    UINT16	Addr_shift_bit;
    UINT8  	init_flag;
    UINT8 	clk_cfg;
} SDCARD_S, *SDCARD_PTR;

/* API */
extern void sdcard_get_card_info(SDCARD_S *card_info);

extern UINT32 sdcard_open(UINT32 op_falag);

extern UINT32 sdcard_close(void);

extern UINT32 sdcard_read(char *user_buf, UINT32 count, UINT32 op_flag);

extern UINT32 sdcard_write(char *user_buf, UINT32 count, UINT32 op_flag);

extern UINT32 sdcard_ctrl(UINT32 cmd, void *parm);

#endif // CFG_USE_SDCARD_HOST

#endif
