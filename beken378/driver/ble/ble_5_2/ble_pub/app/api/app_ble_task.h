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

#ifndef _APP_BLE_TASK_H_
#define _APP_BLE_TASK_H_

#include "rwip_config.h"     // SW configuration
#if (BLE_APP_PRESENT)

#include "app_ble.h"
#include "ble_ui.h"
#include "app_comm.h"                //  Application Module Definitions


/**
 ****************************************************************************************
 * @brief Start/stop advertising
 *
 * @param[in] start     True if advertising has to be started, else false
 ****************************************************************************************
 */
void appm_update_adv_state(char start);

/**
 ****************************************************************************************
 * @brief Send a disconnection request
 ****************************************************************************************
 */
void appm_disconnect(unsigned char reason);




#endif
#endif
