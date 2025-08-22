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

#ifndef _CMD_EVM_H_
#define _CMD_EVM_H_

#include "include.h"
#include "command_table.h"

extern int do_evm(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[]);

#define CMD_EVM_MAXARG                          10

#define EVM_DEFUALT_MODE                       (1)
#define EVM_VIAMAC_TPC_MODE                    (0)
#define EVM_VIAMAC_NOTPC_MODE                  (2)
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
#define EVM_DEFUALT_PACKET_LEN                 (1024)
#define EVM_DEFUALT_B_PACKET_LEN               (1024)
#else
#define EVM_DEFUALT_PACKET_LEN                 (100)
#define EVM_DEFUALT_B_PACKET_LEN               (100)
#endif
#define EVM_DEFUALT_RATE                       (54)
#define EVM_DEFUALT_B_RATE                     (11)
#define EVM_MIN_MCS_RATE                       (128)
#define EVM_MAX_MCS_RATE                       (135) //MCS7
#define EVM_DEFUALT_BLE_RATE                   (158)
#define EVM_DEFUALT_CHANNEL                    (6)
#define EVM_DEFUALT_BLE_CHANNEL                (2402)
#define EVM_DEFUALT_BW                         (0)
#define EVM_DEFUALT_PWR_MOD                    (2)
#define EVM_DEFUALT_BLE_PWR_MOD                (4)
#define EVM_DEFUALT_PWR_PA                     (8)
#define EVM_DEFUALT_MODUL_FORMAT               (0)
#define EVM_DEFUALT_GI_TYPE                    (0)
#define EVM_DEFUALT_SINGLE_CARRIER             (0)

#define EVM_TEST_MODE_NONE                     (0)
#define EVM_TEST_MODE_FCC                      (1)
#define EVM_TEST_MODE_SRRC                     (2)
#define EVM_TEST_MODE_CE                       (3)

#define ENTRY_CMD_EVM               \
	ENTRY_CMD(txevm,                          \
				CMD_EVM_MAXARG,     \
				1,                             \
				do_evm,             \
				"txevm [-m mode] [-c channel] [-l packet-length] [-r physical-rate]\r\n",         \
				"Options:\r\n"\
				"     -m mode: 1,0                       1: tx packet bypass mac, 0: via mac\r\n"\
				"     -c channel: 1,2,...,14             channel number\r\n"\
				"     -l packet-length: 0--4095          legacy:0--4095 ht:0--65535 vht:0--1048575\r\n"\
				"     -r ppdu-rate: 1,2,5,6,9,11,12,18,24,36,48,54    Mbps\r\n"\
				)

#endif // _CMD_EVM_H_
