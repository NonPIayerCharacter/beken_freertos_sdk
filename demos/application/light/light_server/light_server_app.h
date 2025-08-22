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

#ifndef _LIGHT_SERVER_APP_H_
#define _LIGHT_SERVER_APP_H_

#define TCP_SERVER_LIGHT
#define SERVER_SOCKET_MSG (0x20000000)
#define LIGHT_SOCKET_MSG_QUEUE_LENGTH ( 30 )

void light_sck_cs_txdata_sender(int fd, unsigned char *buf, int len);
int light_sck_cs_rxdat_sender(int fd, char *buf, int len);
extern int demo_start(void);

#endif /*_LIGHT_SERVER_APP_H_*/

//EoF

