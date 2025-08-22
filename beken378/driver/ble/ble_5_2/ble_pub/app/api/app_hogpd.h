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

#ifndef APP_HOGPD_H_
#define APP_HOGPD_H_

#include "rwprf_config.h"     // SW configuration

#if (BLE_HID_DEVICE)
#include <stdint.h>          // Standard Integer Definition
#include "ble.h"
#include "ble_pub.h"
#include "ble_api_5_x.h"
#include "kernel_task.h"         // Kernel Task Definition

extern const struct app_subtask_handlers app_hogpd_table_handler;
extern void bk_hogpd_enable(void);
extern void bk_hogpd_init(void);

#endif

#endif



