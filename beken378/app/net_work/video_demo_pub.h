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

#ifndef __VOID_DEMO_PUB_H__
#define __VOID_DEMO_PUB_H__

enum
{
    DAP_TIMER_POLL          = 0,
    DAP_WIFI_DISCONECTED,
    DAP_WIFI_CONECTED,
    DAP_APP_CONECTED,
    DAP_APP_DISCONECTED,
    DAP_EXIT,
    DAP_START_OTA,
};

typedef struct tvideo_ota_st
{
    const char *http_url;
    int    http_port;
    UINT32 http_timeout;
} TV_OTA_ST, *TV_OTA_PTR;

typedef struct temp_message
{
    u32 dmsg;
    u32 data;
} DRONE_MSG_T;

#endif  // __VOID_DEMO_PUB_H__