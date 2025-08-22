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

#ifndef __VIDEO_UDP_SDP_PUB_H__
#define __VIDEO_UDP_SDP_PUB_H__

#include "include.h"
#include "uart_pub.h"

#define VUPD_SDP_DEBUG              1
#if VUPD_SDP_DEBUG
#define VUPD_SDP_PRT                warning_prf
#define VUPD_SDP_WARN               warning_prf
#define VUPD_SDP_FATAL              fatal_prf
#else
#define VUPD_SDP_PRT                null_prf
#define VUPD_SDP_WARN               null_prf
#define VUPD_SDP_FATAL              null_prf
#endif

typedef struct vudp_sdp_init_st
{
    UINT8 *adv_buf;
    UINT32 adv_buf_len;
    UINT16 local_port;
    UINT16 remote_port;
} VUDP_SDP_INIT_ST, *VUDP_SDP_INIT_PTR;

int vudp_sdp_pub_init(VUDP_SDP_INIT_PTR param);
int vudp_sdp_pub_deinit(void);
int vudp_sdp_change_adv_data(UINT8 *adv_data, UINT32 data_len);
int vudp_sdp_get_adv_data(UINT8 **adv_data, UINT32 *data_len);
int vudp_sdp_start_timer(UINT32 time_ms);
int vudp_sdp_stop_timer(void);

#endif //__VIDEO_UDP_SDP_PUB_H__
