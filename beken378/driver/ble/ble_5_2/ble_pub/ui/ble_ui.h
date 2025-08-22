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

#ifndef _BLE_UI_H_
#define _BLE_UI_H_

#include "rwip_config.h"             // SW configuration
#include <stdint.h>          // Standard Integer Definition
#include "ble_api_5_x.h"
#include "common_bt_defines.h"
#if (BLE_APP_PRESENT && BLE_CENTRAL)
#include "app_sdp.h"
#endif

typedef struct
{
    uint8_t    nb_att;
    gatt_att_t *p_atts;
} app_gattc_multi_t;            /// The gattc multiple read element

extern ble_notice_cb_t ble_event_notice;

///////////////////////
extern uint8_t ble_appm_get_dev_name(uint8_t *name, uint32_t buf_len);
extern uint8_t ble_appm_set_dev_name(uint8_t len, uint8_t *name);

#if (CFG_BLE_VERSION == BLE_VERSION_5_2)
ble_err_t bk_ble_gattc_read_by_type(uint8_t conn_idx, uint16_t start_handle, uint16_t end_handle, uint8_t uuid_type, uint8_t *uuid);
ble_err_t bk_ble_gattc_read_multiple(uint8_t conn_idx, app_gattc_multi_t *read_multi);
ble_err_t bk_ble_gattc_register_for_notify(uint8_t conidx, uint16_t handle);
ble_err_t bk_ble_gattc_register_for_indicate(uint8_t conidx, uint16_t handle);
ble_err_t bk_ble_gattc_unregister_for_notify_or_indicate(uint8_t conidx, uint16_t handle);
#endif

#if (BLE_APP_PRESENT && (BLE_CENTRAL))
extern ble_err_t bk_ble_create_init(uint8_t con_idx,ble_cmd_cb_t callback);
extern ble_err_t bk_ble_init_start_conn(uint8_t con_idx,uint16_t con_dev_time,ble_cmd_cb_t callback);
extern ble_err_t bk_ble_init_stop_conn(uint8_t con_idx,ble_cmd_cb_t callback);
extern ble_err_t bk_ble_init_set_connect_dev_addr(unsigned char connidx,struct bd_addr *bdaddr,unsigned char addr_type);
extern ble_err_t bk_ble_delete_init(uint8_t actv_idx, ble_cmd_cb_t callback);
extern void bk_ble_sdp_register_filt_service_tab(unsigned char service_tab_nb,app_sdp_service_uuid *service_tab);
extern ble_err_t bk_ble_read_service_data_by_handle_req(uint8_t conidx,uint16_t handle);
extern ble_err_t bk_ble_write_service_data_req(uint8_t conidx,uint16_t handle,uint16_t data_len,uint8_t *data);
#endif  ///#if (BLE_APP_PRESENT && (BLE_CENTRAL))

#endif
