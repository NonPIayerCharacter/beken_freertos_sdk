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

#ifndef __ATSVR_NET_CNMD_H_
#define __ATSVR_NET_CNMD_H_

#include "network_interface.h"
#if CFG_USE_TCPUDP
typedef struct _pm_socket_info
{
    int linkid;
    char remoteip[16];
    int port;
    NETWORK_TYPE type;
    int keepalive;
    int udpmode;
    char localip[16];
    unsigned int localport;
    int linktype; //0:clinet 1:server
} pm_socket_info_t;

void _atsvr_net_cmd_init(_atsvr_env_t *env);
void _atsvr_net_cmd_deinit(_atsvr_env_t *env);

extern beken_queue_t network_msg_que;
#endif
#endif