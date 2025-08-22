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

#ifndef __CAMERA_INTF_CONFIG_H__
#define __CAMERA_INTF_CONFIG_H__

#define CAMERA_INTF_DEBUG
#include "uart_pub.h"
#ifdef CAMERA_INTF_DEBUG
#define CAMERA_INTF_PRT             os_printf
#define CAMERA_INTF_WPRT            warning_prf
#define CAMERA_INTF_FATAL           fatal_prf
#define DEBUG_TIMRT_INTERVAL        (2000)
#define CAMERA_BITRATE_LOG_PRT
#else
#define CAMERA_INTF_PRT             null_prf
#define CAMERA_INTF_WPRT            null_prf
#define CAMERA_INTF_FATAL           null_prf
#endif



#define PAS6329_DEV             (0xABC00)
#define OV_7670_DEV             (0xABC01)
#define PAS6375_DEV             (0xABC02)
#define GC0328C_DEV             (0xABC03)
#define BF_2013_DEV             (0xABC04)
#define GC0308C_DEV             (0xABC05)
#define HM_1055_DEV             (0xABC06)
#define GC_2145_DEV             (0xABC07)

#define USE_CAMERA              GC0328C_DEV

#define PAS6329_DEV_ID          (0x40)
#define OV_7670_DEV_ID          (0x21)
#define PAS6375_DEV_ID          (0x40)
#define GC0328C_DEV_ID          (0x21)
#define BF_2013_DEV_ID          (0x6e)
#define GC0308C_DEV_ID          (0x21)
#define HM_1055_DEV_ID          (0x24)      //slave_address:0x48
#define GC_2145_DEV_ID          (0x3C)      //slave_address:0x78

#endif

