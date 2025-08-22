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

#ifndef _CMD_RX_SENSITIVITY_H_
#define _CMD_RX_SENSITIVITY_H_

#include "command_table.h"

typedef enum {
    RXSENS_RTYPTE_CLEAN    = 0,
    RXSENS_RTYPTE_20M      = 1,
    RXSENS_RTYPTE_40M      = 2,
    RXSENS_RTYPTE_SIG_RES  = 3,
    RXSENS_RTYPTE_MAX
} RXSENS_RESULT_TYPE;

typedef enum {
    RXSENS_G_STOP_LASTRX   = 0,
    RXSENS_G_REBOOT        = 1,
    RXSENS_G_STOP_TIME     = 2,
    RXSENS_G_MAX
} RXSENS_G_TYPE;

extern int do_rx_sensitivity(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[]);

#define CMD_RX_SENSITIVITY_MAXARG              (8)
#define RXSENS_DEFUALT_MODE                    (0)
#define RXSENS_DEFUALT_DURATION                (0)
#define RXSENS_DEFUALT_CHANNEL                 (6)
#define RXSENS_DEFUALT_BLE_CHANNEL             (2)


#define ENTRY_CMD_RX_SENSITIVITY               \
	ENTRY_CMD(rxsens,                          \
				CMD_RX_SENSITIVITY_MAXARG,     \
				1,                             \
				do_rx_sensitivity,             \
				"rxsens [-m mode] [-d duration] [-c channel] [-l lost]\r\n"         \
				"      test rx sensitivity, and get statistic result about fcs, per and so on\r\n", \
				"Options:\r\n" \
				"      -m mode                  0:PHY_CHNL_BW_20  1:PHY_CHNL_BW_40 \r\n"\
				"      -d duration              duration: n second\r\n"\
				"      -c channel: 1,2,...,14   channel number\r\n"\
				"      -l don't show rxsens log any more\r\n"\
				"\r\n")

#endif // _CMD_RX_SENSITIVITY_H_
