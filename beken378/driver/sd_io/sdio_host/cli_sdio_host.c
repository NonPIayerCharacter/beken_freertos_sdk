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
#include <os.h>
#include "sdio_config.h"
#include <sdio_host.h>
#include "uart_pub.h"
#include "str_pub.h"
#include "wlan_cli_pub.h"
#include "bk_log.h"

#define SD_CMD_GO_IDLE_STATE 0
#define SD_BLOCK_SIZE 512
#define CMD_TIMEOUT_200K	5000	//about 5us per cycle (25ms)
#define DATA_TIMEOUT_13M	6000000 //450ms

static void cli_sdio_host_help(void)
{
    os_printf("sdio_host_driver {init|deinit}\r\n");
    os_printf("sdio {init|deinit}\r\n");
    os_printf("sdio send_cmd Index Arg(hex-decimal) RSP_Type Timeout_Value\r\n");
    os_printf("sdio set clock clock_dig_value [7 means 100K, 0 means 13M, 9 means 80M, 11 means 40M, 14 means 20M]\r\n");
}

static void cli_sdio_host_driver_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (argc < 2) {
        cli_sdio_host_help();
        return;
    }

    if (os_strcmp(argv[1], "init") == 0) {
        BK_LOG_ON_ERR(bk_sdio_host_driver_init());
        os_printf("sdio_host driver init\n");
    } else if (os_strcmp(argv[1], "deinit") == 0) {
        BK_LOG_ON_ERR(bk_sdio_host_driver_deinit());
        os_printf("sdio_host driver deinit\n");
    } else {
        cli_sdio_host_help();
        return;
    }
}

static void cli_sdio_host_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (argc < 2) {
        cli_sdio_host_help();
        return;
    }

    if (os_strcmp(argv[1], "init") == 0) {
        sdio_host_config_t sdio_cfg = {0};

        #if (CONFIG_SDIO_V2P0)
        sdio_cfg.clock_freq = SDIO_HOST_CLK_XTAL_128DIV;
        #else
        sdio_cfg.clock_freq = CONFIG_SDIO_HOST_DEFAULT_CLOCK_FREQ;
        #endif

        #if CONFIG_SDCARD_BUSWIDTH_4LINE
        sdio_cfg.bus_width = SDIO_HOST_BUS_WIDTH_4LINE;
        #else
        sdio_cfg.bus_width = SDIO_HOST_BUS_WIDTH_1LINE;
        #endif
        BK_LOG_ON_ERR(bk_sdio_host_init(&sdio_cfg));
        os_printf("sdio host init\r\n");
    } else if (os_strcmp(argv[1], "deinit") == 0) {
        BK_LOG_ON_ERR(bk_sdio_host_deinit());
        os_printf("sdio host deinit\r\n");
    } else if (os_strcmp(argv[1], "send_cmd") == 0) {
        bk_err_t error_state = BK_OK;
        sdio_host_cmd_cfg_t cmd_cfg = {0};

        //modify to send cmd by parameter,then we can easy to debug any cmds.
        cmd_cfg.cmd_index = os_strtoul(argv[2], NULL, 10);			//CMD0,CMD-XXX,SD_CMD_GO_IDLE_STATE
        cmd_cfg.argument = os_strtoul(argv[3], NULL, 16);			//0x123456xx
        cmd_cfg.response = os_strtoul(argv[4], NULL, 10);			//SDIO_HOST_CMD_RSP_NONE,SHORT,LONG
        cmd_cfg.wait_rsp_timeout = os_strtoul(argv[5], NULL, 10);	//CMD_TIMEOUT_200K;

        bk_sdio_host_send_command(&cmd_cfg);
        error_state = bk_sdio_host_wait_cmd_response(cmd_cfg.cmd_index);
        if (error_state != BK_OK) {
            os_printf("sdio:cmd %d err:-%x\r\n", cmd_cfg.cmd_index, -error_state);
        }
    } else if (os_strcmp(argv[1], "config_data") == 0) {
        sdio_host_data_config_t data_config = {0};

        data_config.data_timeout = DATA_TIMEOUT_13M;
        data_config.data_len = SD_BLOCK_SIZE * 1;
        data_config.data_block_size = SD_BLOCK_SIZE;
        data_config.data_dir = SDIO_HOST_DATA_DIR_RD;

        BK_LOG_ON_ERR(bk_sdio_host_config_data(&data_config));
        os_printf("sdio host config data ok\r\n");
    } else if (os_strcmp(argv[1], "set") == 0) {
        if (os_strcmp(argv[2], "clock") == 0) {
            sdio_host_clock_freq_t freq = os_strtoul(argv[3], NULL, 10);
            bk_sdio_host_set_clock_freq(freq);
        }
    } else {
        cli_sdio_host_help();
        return;
    }
}

#define SDIO_HOST_CMD_CNT (sizeof(s_sdio_host_commands) / sizeof(struct cli_command))
static const struct cli_command s_sdio_host_commands[] = {
    {"sdio_host_driver", "sdio_host_driver {init|deinit}", cli_sdio_host_driver_cmd},
    {"sdio", "sdio {init|deinit|send_cmd|config_data|set}", cli_sdio_host_cmd},
};

int cli_sdio_host_init(void)
{
    BK_LOG_ON_ERR(bk_sdio_host_driver_init());
    return cli_register_commands(s_sdio_host_commands, SDIO_HOST_CMD_CNT);
}

