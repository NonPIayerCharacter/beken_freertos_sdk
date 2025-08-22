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

#ifndef _APP_SDP_H_
#define _APP_SDP_H_

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT && (BLE_CENTRAL) && (BLE_SDP_CLIENT))
#include "att.h"
#include "gattc_task.h"

typedef enum {
    CHARAC_NOTIFY,
    CHARAC_INDICATE,
    CHARAC_READ,
    CHARAC_READ_DONE,
    CHARAC_WRITE_DONE,
} CHAR_TYPE;

typedef void (*app_sdp_callback)(unsigned char conidx,uint16_t chars_val_hdl,unsigned char uuid_len,unsigned char *uuid);
typedef void (*app_sdp_charac_callback)(CHAR_TYPE type,uint8 conidx,uint16_t hdl,uint16_t len,uint8 *data);

typedef struct {
    /// Service UUID Length
    unsigned char  uuid_len;
    /// Service UUID
    unsigned char  uuid[ATT_UUID_128_LEN];
} app_sdp_service_uuid;


typedef struct {
    /////Filter non-selected service tables
    unsigned char filtration;
    ///////service tables number
    unsigned char service_tab_nb;
    ////service tables:Only this table will be registered.But you need to set "filtration" flag
    app_sdp_service_uuid *service_tab;

    ////Tell the properties of the registration
    app_sdp_callback sdp_cb;

    ////Attribute data callback
    app_sdp_charac_callback charac_cb;
} app_sdp_env_tag;


///Internal function
/*
 * Only the UUID of the PROFILE and the corresponding handle are told on master
 * You do not need to care about the handle number of the server, which can be used to communicate correctly.
 * Because we did a mapping.
*/
extern void app_sdp_characteristic_callback_handler(unsigned char conidx,uint16_t chars_val_hdl,unsigned char uuid_len,unsigned char *uuid);
/*
 * We pass the data to the application layer through this callback.
 * You need to handle the connection number, handle, and escalation data to prevent data on the application from not matching.
*/
extern void app_sdp_charac_callback_handler(CHAR_TYPE type,uint8 conidx,uint16_t hdl,uint16_t len,uint8 *data);


/////////extern function
extern void register_app_sdp_characteristic_callback(app_sdp_callback cb);
extern void register_app_sdp_charac_callback(app_sdp_charac_callback cb);

extern uint8_t appc_write_service_data_req(uint8_t conidx,uint16_t handle,uint16_t data_len,uint8_t *data);
extern uint8_t appc_write_service_ntf_cfg_req(uint8_t conidx,uint16_t handle,uint16_t ntf_cfg,uint16_t seq_num);
extern uint8_t appm_read_service_data_by_uuid_req(uint8_t conidx,uint8_t uuid_len,uint8_t* uuid);
extern uint8_t appm_read_service_data_by_handle_req(uint8_t conidx,uint16_t handle);
extern uint8_t appm_read_service_ntf_ind_cfg_by_handle_req(uint8_t conidx,uint16_t handle);
extern uint8_t appm_read_service_userDesc_by_handle_req(uint8_t conidx,uint16_t handle);

extern uint8_t app_sdp_add_element_srv(uint8_t conidx,struct gattc_sdp_svc_ind const *ind);

extern void register_app_sdp_service_tab(unsigned char service_tab_nb,app_sdp_service_uuid *service_tab);
extern void app_sdp_service_filtration(int en);

#endif  ////BLE_CENTRAL
#endif  ///_APP_SDP_H_

