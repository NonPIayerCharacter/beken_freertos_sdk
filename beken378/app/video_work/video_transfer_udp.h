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

#ifndef __APP_DEMO_UDP_H__
#define __APP_DEMO_UDP_H__

UINT32 app_demo_udp_init(void);
void app_demo_udp_deinit(void);
int app_demo_udp_send_packet(UINT8 *data, UINT32 len);
void app_demo_disconnect_cmd_udp(void);

#endif
// eof

