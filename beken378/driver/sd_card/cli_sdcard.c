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
#include "sdcard_config.h"
#include <sd_card.h>
#include "uart_pub.h"
#include "str_pub.h"
#include "mem_pub.h"
#include "wlan_cli_pub.h"
#include "bk_log.h"

#define SD_CARD_READ_BUFFER_SIZE 512

#if (CFG_ENABLE_SDIO_DEV)
#if CONFIG_SDCARD
/*
sdtest I 0 --
sdtest R secnum
sdtest W secnum
*/
static void cli_sd_card_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    bk_err_t ret;

    if (os_strcmp(argv[1], "init") == 0) {
        //bk_gpio_ctrl_external_ldo(GPIO_CTRL_LDO_MODULE_SDIO, SDCARD_LDO_CTRL_GPIO, GPIO_OUTPUT_STATE_HIGH);
        BK_LOG_ON_ERR(bk_sd_card_init());
        os_printf("sd card init ok\r\n");
    } else if(os_strcmp(argv[1], "deinit") == 0) {
        BK_LOG_ON_ERR(bk_sd_card_deinit());
        //bk_gpio_ctrl_external_ldo(GPIO_CTRL_LDO_MODULE_SDIO, SDCARD_LDO_CTRL_GPIO, GPIO_OUTPUT_STATE_LOW);
        os_printf("sd card deinit ok\r\n");
    } else if (os_strcmp(argv[1], "read") == 0) {
        uint32_t block_num = os_strtoul(argv[2], NULL, 10);
        uint8_t *buf = os_malloc(SD_CARD_READ_BUFFER_SIZE * block_num);
        if (buf == NULL) {
            os_printf("sd card buf malloc failed\r\n");
            return;
        }
        if(0 == block_num) {
            os_printf("input block count\r\n");
            return;
        }
        ret = bk_sd_card_read_blocks(buf, 0, block_num);
        if(BK_OK == ret) {
            print_hex_dump("read content:", buf, SD_CARD_READ_BUFFER_SIZE * block_num);
        } else {
            os_printf("read exceptionally\r\n");
        }
        if (buf) {
            os_free(buf);
            buf = NULL;
        }
        os_printf("sd card read over\r\n");
    } else if (os_strcmp(argv[1], "write") == 0) {
        uint32_t block_num = os_strtoul(argv[2], NULL, 10);
        uint8_t *buf = os_malloc(SD_CARD_READ_BUFFER_SIZE * block_num);
        if (buf == NULL) {
            os_printf("sd card buf malloc failed\r\n");
            return;
        }
        if(0 == block_num) {
            os_printf("input block num\r\n");
            return;
        }
        for (int i = 0; i < SD_CARD_READ_BUFFER_SIZE * block_num; i++) {
            buf[i] = i & 0xff;
        }
        BK_LOG_ON_ERR(bk_sd_card_write_blocks(buf, 0, block_num));
        if (buf) {
            os_free(buf);
            buf = NULL;
        }
        os_printf("sd card write ok\r\n");
    } else if (os_strcmp(argv[1], "erase") == 0) {
        uint32_t block_num = os_strtoul(argv[2], NULL, 10);
        BK_LOG_ON_ERR(bk_sd_card_erase(0, block_num));
        while (bk_sd_card_get_card_state() != SD_CARD_TRANSFER);
        os_printf("sd card erase ok\r\n");
    } else if (os_strcmp(argv[1], "cmp") == 0) {
        BK_LOG_ON_ERR(bk_sd_card_erase(0, 2));
        while (bk_sd_card_get_card_state() != SD_CARD_TRANSFER);

        uint8_t *write_buf = os_malloc(SD_CARD_READ_BUFFER_SIZE * 2);
        if (write_buf == NULL) {
            os_printf("sd card write_buf malloc failed\r\n");
            return;
        }
        for (int i = 0; i < SD_CARD_READ_BUFFER_SIZE * 2; i++) {
            write_buf[i] = i & 0xff;
        }
        BK_LOG_ON_ERR(bk_sd_card_write_blocks(write_buf, 0, 2));
        while (bk_sd_card_get_card_state() != SD_CARD_TRANSFER);

        uint8_t *read_buf = os_malloc(SD_CARD_READ_BUFFER_SIZE * 2);
        if (read_buf == NULL) {
            os_printf("sd card read_buf malloc failed\r\n");
            return;
        }
        BK_LOG_ON_ERR(bk_sd_card_read_blocks(read_buf, 0, 2));
        while (bk_sd_card_get_card_state() != SD_CARD_TRANSFER);

        int ret = os_memcmp(write_buf, read_buf, SD_CARD_READ_BUFFER_SIZE * 2);
        if (ret == 0) {
            os_printf("sd card test ok\r\n");
        }

        if (write_buf) {
            os_free(write_buf);
            write_buf = NULL;
        }

        if (read_buf) {
            os_free(read_buf);
            read_buf = NULL;
        }
    }
}

#define SD_CMD_CNT (sizeof(s_sd_commands) / sizeof(struct cli_command))
static const struct cli_command s_sd_commands[] = {
    {"sd_card", "sd_card {init|deinit|read|write|erase|cmp|}", cli_sd_card_cmd},
};

int cli_sd_init(void)
{
    return cli_register_commands(s_sd_commands, SD_CMD_CNT);
}
#else

int cli_sd_init(void)
{
    return 0;
}
#endif
#endif // (CFG_ENABLE_SDIO_DEV)
// eof

