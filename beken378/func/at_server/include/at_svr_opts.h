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

#ifndef _AT_SVR_OPTS_H_
#define _AT_SVR_OPTS_H_
#include "atsvr_comm.h"


#define ATSVR_VERSION_NUM            "ATSVR-01.01"

#define ATSVR_MAX_COMMANDS           250
#define ATSVR_MAX_ARG                16

#define ATSVR_ADD_ESCAPE_CFG         1
#define ATSVR_INPUT_BUFF_MAX_SIZE           2048

#define ATSVR_POWER_UP_READY_DELAY          400

#define ATSVR_CMDRSP_HEAD                   "CMDRSP:"
#define ATSVR_READY_MSG                     "\r\nready\r\n"
#define ATSVR_CMD_RSP_SUCCEED               "OK\r\n"
#define ATSVR_CMD_RSP_ERROR                 "ERROR\r\n"
#define ATSVR_CMDMSG_ERROR_RSP              "ERROR\r\n"
#define ATSVR_RET_CHAR                      '\r'
#define ATSVR_END_CHAR		                '\n'

#define ATSVR_EVENT_HEAD                    "EVT:"
#define ATSVR_EVT_WLAN_DISCONNECTED         "WFI DISCONNECTED\r\n"
#define ATSVR_EVT_WLAN_CONNECTED            "WIFI CONNECTED\r\n"
#define ATSVR_EVT_GOT_IP                    "WIFI GOT IP\r\n"


#ifndef __weak__
#define __weak__              __attribute__((weak))
#endif

extern void bk_printf(const char *fmt, ...);

#ifdef AT_DEBUG
#define ATSVRLOG(fmt, ...)   bk_printf("\nFUNC_ENTRY:   %s L#%d "fmt"\n",__FUNCTION__, __LINE__,##__VA_ARGS__);
#else
#define ATSVRLOG(fmt, ...)
#endif

#endif
