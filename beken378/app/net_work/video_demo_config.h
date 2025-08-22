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

#ifndef __VIDEO_DEMO_CONFIG_H__
#define __VIDEO_DEMO_CONFIG_H__

#include "include.h"
#include "param_config.h"

#if (CFG_USE_APP_DEMO_VIDEO_TRANSFER)
#define APP_DEMO_VIDEO_TRANSFER           1
#else
#define APP_DEMO_VIDEO_TRANSFER           0
#endif

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
#undef  APP_DEMO_VIDEO_TRANSFER
#define APP_DEMO_VIDEO_TRANSFER           0
#endif

#if APP_DEMO_VIDEO_TRANSFER
#include "video_transfer_config.h"

#if (APP_DEMO_CFG_USE_UDP == 0) && (APP_DEMO_CFG_USE_TCP == 0)
#error "must enable one way of network: tcp or udp"
#endif // APP_DEMO_VIDEO_TRANSFER

#define VIDEO_TRANSFER_AP_MODE            1
#define VIDEO_TRANSFER_STA_MODE           2
#define VIDEO_TRANSFER_P2P_MODE           4
#define VIDEO_TRANSFER_CO_AP_P2P_MODE     8

#define APP_VIDEO_TRANSFER_MODE           (VIDEO_TRANSFER_STA_MODE | VIDEO_TRANSFER_AP_MODE | VIDEO_TRANSFER_P2P_MODE | VIDEO_TRANSFER_CO_AP_P2P_MODE)

#if SUPPORT_TIANZHIHENG_DRONE
#define APP_DEMO_SOFTAP_DEF_SSID          "WIFI_FPV_000000"
#define APP_DEMO_SOFTAP_DEF_NET_IP        "192.168.4.153"
#define APP_DEMO_SOFTAP_DEF_NET_MASK      "255.255.255.0"
#define APP_DEMO_SOFTAP_DEF_NET_GW        "192.168.4.153"
#define APP_DEMO_SOFTAP_DEF_CHANNEL       1
#else  // SUPPORT_TIANZHIHENG_DRONE
// for softap configuration
#define APP_DEMO_SOFTAP_DEF_SSID          "BEKEN_WIFI_000000"
#define APP_DEMO_SOFTAP_DEF_NET_IP        "192.168.1.1"
#define APP_DEMO_SOFTAP_DEF_NET_MASK      "255.255.255.0"
#define APP_DEMO_SOFTAP_DEF_NET_GW        "192.168.1.1"
#define APP_DEMO_SOFTAP_DEF_CHANNEL       DEFAULT_CHANNEL_AP
#endif  // SUPPORT_TIANZHIHENG_DRONE

#endif // APP_DEMO_VIDEO_TRANSFER

#endif // __VIDEO_DEMO_CONFIG_H__
