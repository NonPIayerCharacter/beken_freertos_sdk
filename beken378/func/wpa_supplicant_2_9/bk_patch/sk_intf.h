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

#ifndef _SK_INTF_H_
#define _SK_INTF_H_

#include "common.h"
#include "fake_socket.h"

#define SK_INTF_MGMT_SOCKET_NUM              (PF_PACKET + SOCK_RAW + ETH_P_ALL)
#define SK_INTF_IOCTL_SOCKET_NUM             (PF_INET + SOCK_DGRAM + 0)
#define SK_INTF_DATA_SOCKET_NUM              (PF_PACKET + SOCK_RAW + ETH_P_PAE)

extern int ke_mgmt_peek_rxed_next_payload_size(int flag);
extern int ke_mgmt_packet_rx(unsigned char *buf, int len, int flag);
extern int ke_mgmt_packet_tx(unsigned char *buf, int len, int flag);
extern int ke_l2_packet_tx(unsigned char *buf, int len, int flag);
extern int ke_l2_packet_rx(unsigned char *buf, int len, int flag);
extern int ke_data_peek_txed_next_payload_size(int flag);
extern int ke_data_peek_rxed_next_payload_size(int flag);
extern int ws_mgmt_peek_rxed_next_payload_size(int flag);
extern int ws_get_mgmt_packet(unsigned char *buf, int len, int flag);
extern int ws_data_peek_rxed_next_payload_size(int flag);
extern int ws_get_data_packet(unsigned char *buf, int len, int flag);
extern SOCKET mgmt_get_socket_num(u8 vif_idx);
extern SOCKET data_get_socket_num(u8 vif_idx);
extern SOCKET ioctl_get_socket_num(u8 vif_idx);
#endif
// eof

