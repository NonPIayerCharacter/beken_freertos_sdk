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

#ifndef _CMD_REG_H_
#define _CMD_REG_H_

#include "command_table.h"

#define REG_RD_MAX_CNT                      4
#define REG_MASK                            3

#define CREG_DEBUG

#ifdef CREG_DEBUG
#define CREG_PRT       os_printf
#define CREG_WPRT      warning_prf
#else
#define CREG_PRT       os_null_printf
#define CREG_WPRT      os_null_printf
#endif

extern int do_reg(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[]);

#define CMD_REG_MAXARG                              6

#define ENTRY_CMD_REG                       \
	ENTRY_CMD(reg,                          \
				CMD_REG_MAXARG,             \
				1,                          \
				do_reg,                     \
				"reg [-r register register...] [-w register value]\r\n",\
				"\r\n"\
				"	read register, or write register\r\n"\
				"Options:\r\n"\
				"     -r register register...            register: hex\r\n"\
				"     -w register value                  register: hex, value: hex\r\n"\
				"\r\n")

#endif // _CMD_REG_H_
// eof

