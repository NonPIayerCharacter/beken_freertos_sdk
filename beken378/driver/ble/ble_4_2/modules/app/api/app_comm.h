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

#ifndef APP_COMM_H_
#define APP_COMM_H_

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_COMM)
#include <stdint.h>          // Standard Integer Definition
#include "ble.h"
#include "ble_pub.h"
#include "ble_api.h"
#include "kernel_task.h"         // Kernel Task Definition

extern const struct kernel_state_handler app_comm_table_handler;

#endif

#endif
