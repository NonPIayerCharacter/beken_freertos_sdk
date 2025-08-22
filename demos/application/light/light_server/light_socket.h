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

#ifndef _light_Socket_H_
#define _light_Socket_H_

#include "light_server_app.h"

#ifdef LIGHT_SERVER_APPLICATION

#ifdef TCP_SERVER_LIGHT
#include "compiler.h"
#include "doubly_list.h"

typedef struct
{
    LIST_HEADER_T list;
    int fd;
} light_socket_handler_meg_T;

int light_tcp_fd_find_connect(int fd);
int light_socket_application_start(void);

extern LIST_HEADER_T light_tcp_link_node_list;

#endif /*TCP_SERVER_LIGHT*/

#endif  /*_LIGHT_Application_*/
#endif /*_light_Socket_H_*/

