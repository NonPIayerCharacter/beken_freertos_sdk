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

#ifndef _COMM_H_
#define _COMM_H_

#include "rwip_config.h"
#include "rwprf_config.h"

#if (BLE_COMM_SERVER)
#include "ble.h"
#include "ble_pub.h"
#include "ble_api.h"
#include "comm_task.h"
#include "atts.h"
#include "prf_types.h"
#include "prf.h"
#include "ble_compiler.h"

enum bk_ble_state
{
    /// Idle state
    BLE_IDLE,
    /// busy state
    BLE_BUSY,
    /// Number of defined states.
    BLE_STATE_MAX
};

///  'Profile' Server environment variable
struct bk_ble_env_tag
{
    /// profile environment
    prf_env_t prf_env;

    /// On-going operation
    struct kernel_msg * operation;
    /// Services Start Handle
    uint16_t start_hdl;

    /// Number of attribute
    uint8_t att_db_nb;

    /// Profile id
    uint16_t id;

    /// BASS task state
    kernel_state_t state[BLE_CONN_IDX_MAX];

    uint16_t ntf_cfg[BLE_CONNECTION_MAX];

    uint16_t ind_cfg[BLE_CONNECTION_MAX];
};

const struct prf_task_cbs* bk_ble_prf_itf_get(void);

#endif
#endif
