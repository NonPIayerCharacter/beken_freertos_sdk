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

#include "drv_model_pub.h"
#include "BkDriverFlash.h"

#define HTTP_WR_TO_FLASH        1

typedef struct http_data_st {
    UINT32 http_total;
    UINT8 do_data;
    #if HTTP_WR_TO_FLASH
    UINT8 *wr_buf ;
    UINT8 *wr_tmp_buf;
    UINT16 wr_last_len ;
    UINT32 flash_address;
    bk_logic_partition_t *pt;
    #endif
    DD_HANDLE flash_hdl;
} HTTP_DATA_ST;

#define TCP_LEN_MAX             1460
#define HTTP_FLASH_ADDR         0xff000

extern  void store_block (unsigned block, uint8_t * src, unsigned len);
#define WR_BUF_MAX 1048

extern HTTP_DATA_ST *bk_http_ptr;
