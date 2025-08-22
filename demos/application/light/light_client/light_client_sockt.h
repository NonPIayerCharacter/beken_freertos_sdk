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

#ifndef _LIGHT_CLIENT_SOCKET_H_
#define _LIGHT_CLIENT_SOCKET_H_

#include "light_client_app_demo.h"

#ifdef LIGHT_CLIENT_APP_DEMO

int tcp_client_send_remote_server_data(int skt_fd, char *buf, int len);
int light_net_connect_start(void);

#endif /*LIGHT_CLIENT_APP_DEMO*/
#endif /*_LIGHT_CLIENT_SOCKET_H_*/

