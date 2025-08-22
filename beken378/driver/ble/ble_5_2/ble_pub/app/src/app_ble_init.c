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

#include "rwip_config.h"             // SW configuration


#if ((BLE_APP_PRESENT) && BLE_CENTRAL)
#include "rwapp_config.h"
#include <string.h>

#include "rwip.h"
#include "app_sdp.h"
#include "gapm_msg.h"
#include "kernel_timer.h"
#include "gap.h"

#include "ble_api_5_x.h"
#include "app_ble.h"
#include "app_task.h"

#define APP_BLE_INIT_CHECK_CONN_IDX(conn_idx)	if (conn_idx >= BLE_CONNECTION_MAX) {\
													bk_printf("[%s]unknow conn_idx:%d\r\n",__FUNCTION__,conn_idx);\
													return ERR_UNKNOW_IDX;\
												}

extern struct app_env_tag app_ble_env;
extern const struct appm_create_conn_param default_init_par;

int appm_set_connect_dev_addr(unsigned char connidx,struct bd_addr *bdaddr,unsigned char addr_type)
{
    APP_BLE_INIT_CHECK_CONN_IDX(connidx);
    memcpy(app_ble_env.connections[connidx].peer_addr.addr,bdaddr->addr,BD_ADDR_LEN);
    app_ble_env.connections[connidx].peer_addr_type = addr_type;
    return 0;
}

struct bd_addr *appm_get_connect_dev_addr(unsigned char connidx)
{
    return &app_ble_env.connections[connidx].peer_addr;
}

unsigned char appm_get_connect_dev_addr_type(unsigned char connidx)
{
    return app_ble_env.connections[connidx].peer_addr_type;
}

void appm_set_gap_prefer_ext_connect_params(ext_conn_param_t *pref_par)
{
    memcpy(&app_ble_env.init_conn_par, pref_par, sizeof(ext_conn_param_t));
}

void app_ble_create_initing(uint8_t con_idx)
{
    // Prepare the GAPM_ACTIVITY_CREATE_CMD message
    struct gapm_activity_create_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
            TASK_BLE_GAPM,
            KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
            gapm_activity_create_cmd);

    // Set operation code
    p_cmd->operation = GAPM_CREATE_INIT_ACTIVITY;

    // Fill the allocated kernel message
    p_cmd->own_addr_type = GAPM_STATIC_ADDR;

    // Keep the current operation
    BLE_APP_MASTER_SET_IDX_STATE(con_idx,APP_INIT_STATE_CREATING);

    // Send the message
    kernel_msg_send(p_cmd);
}

ble_err_t appm_create_initing(uint8_t con_idx)
{
    APP_BLE_INIT_CHECK_CONN_IDX(con_idx);
    #if BLE_APP_SDP_DBG_CHECK(BLE_APP_SDP_WARN)
    bk_printf("[%s]con_idx:%d init_state:%d\r\n",__func__,con_idx,BLE_APP_MASTER_GET_IDX_STATE(con_idx));
    #endif

	if (BLE_APP_MASTER_GET_IDX_STATE(con_idx) == APP_INIT_STATE_IDLE)
	{
		// And the next expected operation code for the command completed event
		app_ble_env.connections[con_idx].conhdl = USED_CONN_HDL;
		app_ble_env.connections[con_idx].conn_op_mask = 1 << BLE_OP_CREATE_INIT_POS;
		app_ble_env.connections[con_idx].conn_op_cb = NULL;

		app_ble_create_initing(con_idx);
		return ERR_SUCCESS;
	}
	else
	{
		bk_printf("connections[%d] is not idle\r\n", con_idx);
		return ERR_INIT_STATE;
	}
}

static void appm_ble_set_init_param(uint8_t phy_type, struct gapm_conn_param *conn_par, struct gapm_scan_wd_op_param *scan_par)
{
    struct appm_create_conn_param *pref_par;

    if (phy_type == GAPM_INIT_PROP_1M_BIT) {
        pref_par = &app_ble_env.init_conn_par.conn_param_1m;
    } else if (phy_type == GAPM_INIT_PROP_2M_BIT) {
        pref_par = &app_ble_env.init_conn_par.conn_param_2m;
    } else if (phy_type == GAPM_INIT_PROP_CODED_BIT) {
        pref_par = &app_ble_env.init_conn_par.conn_param_coded;
    } else {
        return;
    }
    conn_par->ce_len_max = pref_par->ce_len_max ? pref_par->ce_len_max : default_init_par.ce_len_max;
    conn_par->ce_len_min = pref_par->ce_len_min ? pref_par->ce_len_min : default_init_par.ce_len_min;
    conn_par->conn_intv_max = pref_par->conn_intv_max ? pref_par->conn_intv_max : default_init_par.conn_intv_max;
    conn_par->conn_intv_min = pref_par->conn_intv_min ? pref_par->conn_intv_min : default_init_par.conn_intv_min;
    conn_par->conn_latency = pref_par->conn_latency ? pref_par->conn_latency : default_init_par.conn_latency;
    conn_par->supervision_to = pref_par->supervision_to ? pref_par->supervision_to : default_init_par.supervision_to;

    if (scan_par == NULL) {
        return;
    }

    scan_par->scan_intv = pref_par->scan_interval ? pref_par->scan_interval : default_init_par.scan_interval;
    scan_par->scan_wd = pref_par->scan_window ? pref_par->scan_window : default_init_par.scan_window;
}

ble_err_t appm_start_connecting(uint8_t con_idx,uint16_t con_dev_time)
{
    APP_BLE_INIT_CHECK_CONN_IDX(con_idx);
    #if BLE_APP_SDP_DBG_CHECK(BLE_APP_SDP_WARN)
    bk_printf("[%s],init_state:%d\r\n",__func__,BLE_APP_MASTER_GET_IDX_STATE(con_idx));
    #endif

    int ret = ERR_SUCCESS;
    ext_conn_param_t *pref_par = &app_ble_env.init_conn_par;
    struct bd_addr *bdaddr = appm_get_connect_dev_addr(con_idx);
    unsigned char addr_type = appm_get_connect_dev_addr_type(con_idx);

    if (BLE_APP_MASTER_GET_IDX_STATE(con_idx) == APP_INIT_STATE_CREATED)
    {
        // Prepare the GAPM_ACTIVITY_START_CMD message
        struct gapm_activity_start_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
                                                TASK_BLE_GAPM,
                                                KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
                                                gapm_activity_start_cmd);

        p_cmd->operation = GAPM_START_ACTIVITY;
        p_cmd->actv_idx = app_ble_env.connections[con_idx].gap_actv_idx;
        p_cmd->u_param.init_param.type = GAPM_INIT_TYPE_DIRECT_CONN_EST;

        #if BLE_APP_SDP_DBG_CHECK(BLE_APP_SDP_WARN)
        bk_printf("con address: %02x-%02x-%02x-%02x-%02x-%02x\r\n",
                  bdaddr->addr[0], bdaddr->addr[1], bdaddr->addr[2],
                  bdaddr->addr[3], bdaddr->addr[4], bdaddr->addr[5]);
        #endif
        memcpy(p_cmd->u_param.init_param.peer_addr.addr,bdaddr->addr,GAP_BD_ADDR_LEN);
        p_cmd->u_param.init_param.peer_addr.addr_type = addr_type;
        p_cmd->u_param.init_param.conn_to = 0;

        p_cmd->u_param.init_param.prop = pref_par->phy_mask;
        if (pref_par->phy_mask & GAPM_INIT_PROP_1M_BIT)
        {
            appm_ble_set_init_param(GAPM_INIT_PROP_1M_BIT, &p_cmd->u_param.init_param.conn_param_1m, &p_cmd->u_param.init_param.scan_param_1m);
        }
        if (pref_par->phy_mask & GAPM_INIT_PROP_2M_BIT)
        {
            appm_ble_set_init_param(GAPM_INIT_PROP_2M_BIT, &p_cmd->u_param.init_param.conn_param_2m, NULL);
        }
        if (pref_par->phy_mask & GAPM_INIT_PROP_CODED_BIT)
        {
            appm_ble_set_init_param(GAPM_INIT_PROP_CODED_BIT, &p_cmd->u_param.init_param.conn_param_coded, &p_cmd->u_param.init_param.scan_param_coded);
        }

        /////Keep the current operation
        BLE_APP_MASTER_SET_IDX_STATE(con_idx,APP_INIT_STATE_WAIT_CONECTTING);

        app_ble_env.connections[con_idx].conn_op_mask = 1 << BLE_OP_INIT_START_POS;
        app_ble_env.connections[con_idx].conn_op_cb = NULL;
        app_ble_env.connections[con_idx].u.master.conn_dev_to = con_dev_time;

        #if BLE_APP_SDP_DBG_CHECK(BLE_APP_SDP_WARN)
        bk_printf("conn_dev_to:%d\r\n",app_ble_env.connections[con_idx].u.master.conn_dev_to);
        #endif
        #if (APP_INIT_SET_STOP_CONN_TIMER && (!APP_INIT_STOP_CONN_TIMER_EVENT))
        kernel_timer_set(APP_INIT_CON_DEV_TIMEROUT_TIMER,
                         KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
                         con_dev_time ? con_dev_time : 0x7FFFFF);
        #elif (APP_INIT_SET_STOP_CONN_TIMER && (APP_INIT_STOP_CONN_TIMER_EVENT))
        struct app_task_start_timeout_event_ind *pt_cmd = KERNEL_MSG_ALLOC(APP_INIT_START_TIMEOUT_EVENT,
                KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
                KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
                app_task_start_timeout_event_ind);
        pt_cmd->timout_ms = con_dev_time ? con_dev_time : 0x7FFFFF;
        pt_cmd->task_id = KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx));
        kernel_msg_send(pt_cmd);
        #endif
        ///// Send the message
        kernel_msg_send(p_cmd);
    }
    else
    {
        ret = ERR_INIT_STATE;
    }

    return ret;
}

ble_err_t appm_stop_connencting(uint8_t con_idx)
{
    uint8_t ret = ERR_SUCCESS;

    APP_BLE_INIT_CHECK_CONN_IDX(con_idx);
    #if BLE_APP_SDP_DBG_CHECK(BLE_APP_SDP_WARN)
    bk_printf("[%s]status:%d\r\n",__func__,BLE_APP_MASTER_GET_IDX_STATE(con_idx));
    #endif

    if (BLE_APP_MASTER_GET_IDX_STATE(con_idx) == APP_INIT_STATE_CONECTTING)
    {
        /////Prepare the GAPM_ACTIVITY_STOP_CMD message
        struct gapm_activity_stop_cmd *cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
                                             TASK_BLE_GAPM,
                                             KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
                                             gapm_activity_stop_cmd);

        /////Fill the allocated kernel message
        cmd->operation = GAPM_STOP_ACTIVITY;
        cmd->actv_idx = app_ble_env.connections[con_idx].gap_actv_idx;
        app_ble_env.connections[con_idx].conn_op_mask = 1 << BLE_OP_INIT_STOP_POS;
        app_ble_env.connections[con_idx].conn_op_cb = NULL;

        /////Update advertising state
        BLE_APP_MASTER_SET_IDX_STATE(con_idx,APP_INIT_STATE_STOPPING);

        /////Send the message
        kernel_msg_send(cmd);
    }
    else
    {
        ret = ERR_INIT_STATE;
    }

    return ret;
}

ble_err_t appm_delete_initing(uint8_t con_idx)
{
    ble_err_t ret = ERR_SUCCESS;

    APP_BLE_INIT_CHECK_CONN_IDX(con_idx);
    if (BLE_APP_MASTER_GET_IDX_STATE(con_idx) == APP_INIT_STATE_CREATED) {
        struct gapm_activity_delete_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
                TASK_BLE_GAPM, KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
                gapm_activity_delete_cmd);

        if (p_cmd) {
            // Set operation code
            p_cmd->operation = GAPM_DELETE_ACTIVITY;
            p_cmd->actv_idx = app_ble_env.connections[con_idx].gap_actv_idx;
            // Send the message
            kernel_msg_send(p_cmd);
            ret = ERR_SUCCESS;
        } else {
            ret = ERR_NO_MEM;
        }
    } else {
        bk_printf("conidx [%d] is not created\r\n", con_idx);
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

int app_ble_master_appm_disconnect(uint8_t conidx)
{
    APP_BLE_INIT_CHECK_CONN_IDX(conidx);
    return app_ble_disconnect(conidx, COMMON_ERROR_REMOTE_USER_TERM_CON);
}
#endif


