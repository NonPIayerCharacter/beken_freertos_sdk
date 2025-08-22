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

#ifndef _APP_BLE_INIT_H_
#define _APP_BLE_INIT_H_

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT && (BLE_CENTRAL) && CFG_INIT_ENABLE)

///#include "ble_api.h"
#include "gap.h"
#include "ble_api_5_x.h"
#include "app_task.h"
#include "common_bt_defines.h"

/// Scan interval
#define APP_CONN_SCAN_INTV                 (100)
/// Scan window
#define APP_CONN_SCAN_WD                   (20)
/// Conn interval
#define APP_CONN_INTV                      (30) // 1.25MS

#define APP_CONN_KEEP_INTV                 (81) // 1.25MS

#define APP_SDP_KEEP_TIME                  (3000)

/// Conn window
#define APP_CONN_LATENCY                   (0)
#define APP_CONN_SUP_TO                    (500) // 10MS
#define APP_CONN_DRV_TO                    (10000)  //1mS

#define APP_CONN_IND_DELT_TIME             (3)

#define APP_INIT_SET_STOP_CONN_TIMER       1
#define APP_INIT_STOP_CONN_TIMER_EVENT     1

//////////////////////////////////////////////////////
typedef struct app_ble_initing_env_tag
{
    struct gap_bdaddr g_bdaddr;
    struct {
        unsigned char actv_idx;
#define BLE_INIT_IDX_NONE     0
#define BLE_INIT_IDX_USED     1
#define BLE_INIT_IDX_STOPED   2
        unsigned char state;   ///0:none,1.used,2:stoped
    } init_idx[BLE_CONNECTION_MAX];
} app_ble_initing_env_t;

extern int app_ble_master_appm_disconnect(uint8_t conidx);
extern void app_ble_initing_init(void);

#endif  ////BLE_CENTRAL

extern ble_err_t appm_start_connecting(uint8_t con_idx);
extern ble_err_t appm_stop_connencting(uint8_t con_idx);
extern ble_err_t set_app_ble_master_conn_dev_timeout(unsigned char con_idx,unsigned int n_10mS);
extern int appm_set_connect_dev_addr(unsigned char connidx,struct bd_addr *bdaddr,unsigned char addr_type);
extern ble_err_t appm_create_initing(uint8_t con_idx,unsigned short con_interval,
                                     unsigned short con_latency,unsigned short sup_to);
extern ble_err_t appm_delete_initing(uint8_t actv_idx);

#endif  ///_APP_BLE_INIT_H_

