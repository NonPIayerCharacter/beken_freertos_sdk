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

#include "include.h"
#include "sdcard.h"
#include "sdcard_pub.h"
#include "sd_card.h"
#include "sdio_driver.h"
#define SDCARD_READ_FAIL_RETRY_CNT (3)
#if (CFG_SOC_NAME == SOC_BK7252N)
#define SDCARD_WRITE_FAIL_RETRY_CNT (1)
#else
#define SDCARD_WRITE_FAIL_RETRY_CNT (3)
#endif
void sdcard_operation_err_reset(void)
{
    bk_sd_card_deinit();
    bk_sd_card_init();
}

UINT32 sdcard_open(UINT32 op_flag)
{
    return bk_sd_card_init();
}

UINT32 sdcard_close(void)
{
    bk_sd_card_deinit();
    return SDCARD_SUCCESS;
}


SDIO_Error sdcard_read_multi_block(UINT8 *read_buffer, int first_block, int block_num)
{
    bk_err_t result;

    result = bk_sd_card_read_blocks(read_buffer, first_block, block_num);
    if(result != BK_OK)
    {

        for(int i = 0; i < SDCARD_READ_FAIL_RETRY_CNT; i++)
        {
            bk_printf("sdcard_read_multi_block retry:%d\r\n",i+1);
            sdcard_operation_err_reset();
            result = bk_sd_card_read_blocks(read_buffer, first_block, block_num);
            if(result != BK_OK)
            {
                bk_printf("%s ERROR result:%d\r\n", __func__, result);
            }
            else
            {
                return SD_OK;
            }
        }
    }
    else
    {
        return SD_OK;
    }

    return SD_ERROR;
}

SDIO_Error sdcard_write_multi_block(UINT8 *write_buff, UINT32 first_block, UINT32 block_num)
{

    bk_err_t result;

    result = bk_sd_card_write_blocks((const uint8_t *)write_buff, first_block, block_num);
    if(result != BK_OK)
    {
        for(int i = 0; i < SDCARD_WRITE_FAIL_RETRY_CNT; i++)
        {
            bk_printf("bk_sd_card_write_blocks retry:%d\r\n",i+1);
            sdcard_operation_err_reset();
            result = bk_sd_card_write_blocks((const uint8_t *)write_buff, first_block, block_num);
            if(result != BK_OK)
            {
                bk_printf("%s ERROR result:%d\r\n", __func__, result);
            }
            else
            {
                return SD_OK;
            }
        }
    }
    else
    {
        return SD_OK;
    }

    return SD_ERROR;
}

void sdcard_get_card_info(SDCARD_S *card_info)
{
    card_info->block_size = 512;
    card_info->total_block = bk_sd_card_get_card_size();
}