/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "user_config.h"  

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)
#include "rwapp_config.h"
#include <string.h>

#include "rwip.h"
#include "app_task.h"                // Application task Definition
#include "app_ble.h"                     // Application Definition
#include "gap.h"                     // GAP Definition
#include "gap_int.h"
#include "gapm_msg.h"               // GAP Manager Task API
#include "gapm_int.h"
#include "gapc_msg.h"               // GAP Controller Task API
#include "gapc_int.h"
#include "gatt_msg.h"
#include "gatt_int.h"
#include "llc_int.h"
#include "common_bt.h"                   // Common BT Definition
#include "common_math.h"                 // Common Maths Definition
#include "gapc.h"
#include "kernel_mem.h"
#include "l2cap_hl_api.h"

#if BLE_GATT_CLI
#include "app_sdp.h"
#include "sdp_comm.h"
#endif

#if (BLE_APP_SEC)
#include "app_sec.h"
#endif

#if (NVDS_SUPPORT)
#include "nvds.h"                    // NVDS Definitions
#endif //(NVDS_SUPPORT)
/*
 * DEFINES
 ****************************************************************************************
 */

#define DEVICE_NAME        "BK DEVICE"

#define DEVICE_NAME_SIZE    sizeof(DEVICE_NAME)

/*
 * LOCAL VARIABLES DEFINITIONS
 ****************************************************************************************
 */

/// Application Task Descriptor
extern const struct kernel_task_desc TASK_BLE_DESC_APP;

/// Application Environment Structure
struct app_env_tag app_ble_env;

#define BLE_APP_CHECK_ACTVS_IDX(actv_idx) \
	if (actv_idx >= BLE_ACTIVITY_MAX) {\
		bk_printf("[%s]unknow actv_idx:%d\r\n",__FUNCTION__,actv_idx);\
		return ERR_UNKNOW_IDX;\
	}

#define BLE_APP_CHECK_CONN_IDX(conn_idx) \
	if (conn_idx >= BLE_CONNECTION_MAX) {\
		bk_printf("[%s]unknow conn_idx:%d\r\n",__FUNCTION__,conn_idx);\
		return ERR_UNKNOW_IDX;\
	}

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if BLE_CENTRAL
const struct appm_create_conn_param default_init_par =
{
    .conn_intv_max   = APP_CONN_INTV,
    .conn_intv_min   = APP_CONN_INTV,
    .conn_latency    = APP_CONN_LATENCY,
    .supervision_to  = APP_CONN_SUP_TO,
    .scan_interval   = APP_CONN_SCAN_INTV,
    .scan_window     = APP_CONN_SCAN_WD,
    .ce_len_min      = APP_CONN_CE_LEN_MIN,
    .ce_len_max      = APP_CONN_CE_LEN_MAX,
};
#endif

ble_status_t app_ble_env_state_get(void)
{
	return app_ble_env.app_status & BLE_APP_STATUS_MASK;
}

actv_state_t app_ble_actv_state_get(uint8_t actv_idx)
{
	return app_ble_env.actvs[actv_idx].actv_status;
}

uint8_t app_ble_actv_state_find(uint8_t status)
{
	uint8_t index;
	for (index = 0; index < BLE_ACTIVITY_MAX; index++) {
		if (app_ble_env.actvs[index].actv_status == status) {
			break;
		}
	}

	if (index == BLE_ACTIVITY_MAX) {
		bk_printf("Don't have free actv\r\n");
		return UNKNOW_ACT_IDX;
	}
	return index;
}

uint8_t app_ble_get_idle_actv_idx_handle(ACTV_TYPE type)
{
	uint8_t index;
	switch (type) {
		case ADV_ACTV:
		{
			if (app_ble_env.actv_cnt.adv_actv >= CFG_BLE_ADV_NUM) {
				return UNKNOW_ACT_IDX;
			}
		}break;
		case SCAN_ACTV:
		{
			if (app_ble_env.actv_cnt.scan_actv >= CFG_BLE_SCAN_NUM) {
				return UNKNOW_ACT_IDX;
			}
		}break;
		case PER_SYNC_ACTV:
		{
			if (app_ble_env.actv_cnt.per_sync_actv >= CFG_BLE_PER_SYNC) {
				return UNKNOW_ACT_IDX;
			}
		}break;
		default:
			return UNKNOW_ACT_IDX;
		break;
	}

	for (index = 0; index < BLE_ACTIVITY_MAX; index++) {
		if (app_ble_env.actvs[index].actv_status == ACTV_IDLE) {
			break;
		}
	}

	return index;
}

uint8_t app_ble_get_idle_conn_idx_handle(ACTV_TYPE type)
{
	uint8_t conn_idx;
	switch (type) {
		case INIT_ACTV:
		{
			if (app_ble_env.actv_cnt.init_actv >= CFG_BLE_INIT_NUM) {
				return BLE_CONNECTION_MAX;
			}
		}break;
		case CONN_ACTV:
		{
			if (app_ble_env.actv_cnt.conn_actv >= CFG_BLE_CONN_NUM) {
				return BLE_CONNECTION_MAX;
			}
		}break;
		default:
			return BLE_CONNECTION_MAX;
		break;
	}

	for (conn_idx = 0; conn_idx < BLE_CONNECTION_MAX; conn_idx++) {
		if (app_ble_env.connections[conn_idx].conhdl == UNKNOW_CONN_HDL) {
			break;
		}
	}

	return conn_idx;
}

uint8_t app_ble_find_conn_idx_handle(uint16_t conhdl)
{
	uint8_t conn_idx;

	for (conn_idx = 0; conn_idx < BLE_CONNECTION_MAX; conn_idx++) {
		if (app_ble_env.connections[conn_idx].conhdl == conhdl) {
			break;
		}
	}

	return conn_idx;
}

uint8_t app_ble_find_actv_idx_handle(uint16_t gap_actv_idx)
{
	uint8_t actv_idx;

	for (actv_idx = 0; actv_idx < BLE_ACTIVITY_MAX; actv_idx++) {
		if (app_ble_env.actvs[actv_idx].gap_advt_idx == gap_actv_idx) {
			break;
		}
	}

	return actv_idx;
}

uint8_t app_ble_get_connect_status(uint8_t con_idx)
{
	if (app_ble_env.connections[con_idx].conhdl != UNKNOW_CONN_HDL &&
		app_ble_env.connections[con_idx].conhdl != USED_CONN_HDL) {
			return 1;
		}
	return 0;
}

bool app_ble_init_is_created(uint8_t conn_idx)
{
	return (app_ble_env.connections[conn_idx].u.master.init_state > APP_INIT_STATE_IDLE);
}

uint8_t app_ble_get_connect_status_by_peer_addr(struct bd_addr *peer_addr)
{
	struct bd_addr addr;

	memcpy(&addr.addr[0], &peer_addr->addr[0], 6);

	uint8_t conn_idx = 0;
	for (; conn_idx < BLE_CONNECTION_MAX;conn_idx ++) {
		if (!memcmp(&app_ble_env.connections[conn_idx].peer_addr.addr[0], &peer_addr->addr[0], 6))
			break;
	}

	if (conn_idx == BLE_CONNECTION_MAX)
		return 0;

	if (app_ble_env.connections[conn_idx].conhdl >= USED_CONN_HDL)
		return 0;

	return 1;
}

uint8_t app_ble_find_conn_idx_by_peer_addr(struct bd_addr *peer_addr)
{
	struct bd_addr addr;

	memcpy(&addr.addr[0], &peer_addr->addr[0], 6);

	uint8_t conn_idx = 0;
	for (; conn_idx < BLE_CONNECTION_MAX;conn_idx ++) {
		if (!memcmp(&app_ble_env.connections[conn_idx].peer_addr.addr[0], &peer_addr->addr[0], 6))
			break;
	}

	return conn_idx;
}

uint8_t app_ble_get_connhdl(int conn_idx)
{
	return (app_ble_env.connections[conn_idx].conhdl);
}

void app_ble_run(uint8_t idx, ble_cmd_t cmd, uint32_t op_mask, ble_cmd_cb_t callback)
{
	app_ble_env.op_mask = op_mask;
	app_ble_env.op_cb = callback;
	app_ble_env.cmd = cmd;
	app_ble_env.app_status = (idx << BLE_APP_IDX_POS) | APP_BLE_CMD_RUNNING;
}

void app_ble_reset(void)
{
	app_ble_env.op_mask = 0;
	app_ble_env.op_cb = NULL;
	app_ble_env.cmd = BLE_CMD_NONE;
	app_ble_env.app_status = APP_BLE_READY;
}

ble_err_t app_ble_create_advertising(uint8_t actv_idx, struct adv_param *adv)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_IDLE) {
		struct gapm_activity_create_adv_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
										TASK_BLE_GAPM, TASK_BLE_APP,
										gapm_activity_create_adv_cmd);

		if (p_cmd) {
			/// Set operation code
			p_cmd->operation = GAPM_CREATE_ADV_ACTIVITY;

			// Fill the allocated kernel message
			p_cmd->own_addr_type = GAPM_STATIC_ADDR;
			p_cmd->adv_param.type = GAPM_ADV_TYPE_LEGACY;
			p_cmd->adv_param.prop = adv->prop;//GAPM_ADV_PROP_UNDIR_CONN_MASK;
			p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
			p_cmd->adv_param.prim_cfg.chnl_map = adv->channel_map;
			p_cmd->adv_param.prim_cfg.phy = GAP_PHY_LE_1MBPS;

			p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;
			p_cmd->adv_param.prim_cfg.adv_intv_min = adv->interval_min;
			p_cmd->adv_param.prim_cfg.adv_intv_max = adv->interval_max;

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not idle\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_create_extended_advertising(uint8_t actv_idx, ext_adv_param_cfg_t *param)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_IDLE) {
		struct gapm_activity_create_adv_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
										TASK_BLE_GAPM, TASK_BLE_APP,
										gapm_activity_create_adv_cmd);

		if (p_cmd) {
			/// Set operation code
			p_cmd->operation = GAPM_CREATE_ADV_ACTIVITY;

			// Fill the allocated kernel message
			p_cmd->own_addr_type = GAPM_STATIC_ADDR;
			p_cmd->adv_param.type = GAPM_ADV_TYPE_EXTENDED;
			p_cmd->adv_param.prop = param->prop;
			p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
			p_cmd->adv_param.prim_cfg.chnl_map = param->channel_map;
			p_cmd->adv_param.prim_cfg.phy = GAP_PHY_LE_1MBPS;

			p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;
			p_cmd->adv_param.prim_cfg.adv_intv_min = param->interval_min;
			p_cmd->adv_param.prim_cfg.adv_intv_max = param->interval_max;
			p_cmd->adv_param.second_cfg.phy = p_cmd->adv_param.prim_cfg.phy;
			p_cmd->adv_param.second_cfg.adv_sid = param->sid;

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not idle\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_start_advertising(uint8_t actv_idx, uint16 duration)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_ADV_CREATED) {
		// Prepare the GAPM_ACTIVITY_START_CMD message
		struct gapm_activity_start_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
									TASK_BLE_GAPM, TASK_BLE_APP,
									gapm_activity_start_cmd);

		if (p_cmd) {
			p_cmd->operation = GAPM_START_ACTIVITY;
			p_cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;
			p_cmd->u_param.adv_add_param.duration = duration;
			p_cmd->u_param.adv_add_param.max_adv_evt = 0;

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not created\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_stop_advertising(uint8_t actv_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_ADV_STARTED) {
		struct gapm_activity_stop_cmd *cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
									TASK_BLE_GAPM, TASK_BLE_APP,
									gapm_activity_stop_cmd);

		if (cmd) {
			// Fill the allocated kernel message
			cmd->operation = GAPM_STOP_ACTIVITY;
			cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			// Send the message
			kernel_msg_send(cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not started\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_get_con_rssi(uint8_t conn_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);
	if ((conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)) {
		struct gapc_get_info_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_GET_INFO_CMD,
									KERNEL_BUILD_ID(TASK_BLE_GAPC,conhdl),
									KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(conn_idx)),
									gapc_get_info_cmd);
		if (cmd) {
			cmd->operation = GAPC_GET_CON_RSSI;
			kernel_msg_send(cmd);
			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("unknow conhdl\r\n");
		ret = ERR_BLE_STATUS;
	}
	return ret;
}

ble_err_t app_ble_delete_advertising(uint8_t actv_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_ADV_CREATED) {
		// Prepare the GAPM_ACTIVITY_CREATE_CMD message
		struct gapm_activity_delete_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
									TASK_BLE_GAPM, TASK_BLE_APP,
									gapm_activity_delete_cmd);

		if (p_cmd) {
			// Set operation code
			p_cmd->operation = GAPM_DELETE_ACTIVITY;
			p_cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not created\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_set_adv_data(uint8_t actv_idx, unsigned char* adv_buff, uint16_t adv_len)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if ((app_ble_actv_state_get(actv_idx) == ACTV_ADV_CREATED)
		|| (app_ble_actv_state_get(actv_idx) == ACTV_ADV_STARTED)) {
		// Prepare the GAPM_SET_ADV_DATA_CMD message
		struct gapm_set_adv_data_cmd *p_cmd = KERNEL_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
										TASK_BLE_GAPM, TASK_BLE_APP,
										gapm_set_adv_data_cmd,
										adv_len);
		if (p_cmd) {
			// Fill the allocated kernel message
			p_cmd->operation = GAPM_SET_ADV_DATA;
			p_cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			memcpy(&p_cmd->data[0], adv_buff, adv_len);
			// Update advertising data length
			p_cmd->length = adv_len;

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not created\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_set_scan_rsp_data(uint8_t actv_idx, unsigned char* scan_buff, uint16_t scan_len)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if ((app_ble_actv_state_get(actv_idx) == ACTV_ADV_CREATED)
		|| (app_ble_actv_state_get(actv_idx) == ACTV_ADV_STARTED)) {
		// Prepare the GAPM_SET_ADV_DATA_CMD message
		struct gapm_set_adv_data_cmd *p_cmd = KERNEL_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
										TASK_BLE_GAPM, TASK_BLE_APP,
										gapm_set_adv_data_cmd,
										scan_len);

		if (p_cmd) {
			// Fill the allocated kernel message
			p_cmd->operation = GAPM_SET_SCAN_RSP_DATA;
			p_cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			p_cmd->length = scan_len;
			memcpy(&p_cmd->data[0], scan_buff, scan_len);

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not created\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_update_param(uint8_t conn_idx, struct gapc_conn_param *conn_param)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_CONN_IDX(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if ((conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)) {
		// Prepare the GAPC_PARAM_UPDATE_CMD message
		struct gapc_param_update_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
									 KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
									 KERNEL_BUILD_ID(TASK_BLE_APP, conn_idx),
									 gapc_param_update_cmd);

		if (cmd) {
			cmd->operation	= GAPC_UPDATE_PARAMS;
			cmd->intv_min	= conn_param->intv_min;
			cmd->intv_max	= conn_param->intv_max;
			cmd->latency	= conn_param->latency;
			cmd->time_out	= conn_param->time_out;

			// not used by a slave device
			cmd->ce_len_min = 0x10;  ///0xFFFF
			cmd->ce_len_max = 0x20;  ///0xFFFF

			// Send the message
			kernel_msg_send(cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("unknow conhdl");
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_disconnect(uint8_t conn_idx, uint8_t reason)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_CONN_IDX(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if ((conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)){
		struct gapc_disconnect_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_DISCONNECT_CMD,
									KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
									KERNEL_BUILD_ID(TASK_BLE_APP, conn_idx),
									gapc_disconnect_cmd);

		if (cmd) {
			cmd->operation = GAPC_DISCONNECT;
			cmd->reason    = reason;

			// Send the message
			kernel_msg_send(cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("unknow conhdl");
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_create_scaning(uint8_t actv_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_IDLE) {
		// Prepare the GAPM_ACTIVITY_CREATE_CMD message
		struct gapm_activity_create_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
									TASK_BLE_GAPM, TASK_BLE_APP,
									gapm_activity_create_cmd);

		if (p_cmd) {
			// Set operation code
			p_cmd->operation = GAPM_CREATE_SCAN_ACTIVITY;

			// Fill the allocated kernel message
			p_cmd->own_addr_type = GAPM_STATIC_ADDR;

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not idle\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_start_scaning(uint8_t actv_idx, uint16_t scan_intv, uint16_t scan_wd)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_SCAN_CREATED) {
		// Prepare the GAPM_ACTIVITY_START_CMD message
		struct gapm_activity_start_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
									TASK_BLE_GAPM, TASK_BLE_APP,
									gapm_activity_start_cmd);

		if (p_cmd) {
			p_cmd->operation = GAPM_START_ACTIVITY;

			p_cmd->u_param.scan_param.type = GAPM_SCAN_TYPE_OBSERVER;//GAPM_SCAN_TYPE_GEN_DISC;//GAPM_SCAN_TYPE_OBSERVER;//;
			p_cmd->u_param.scan_param.prop = GAPM_SCAN_PROP_PHY_1M_BIT ;//| GAPM_SCAN_PROP_ACTIVE_1M_BIT;
			p_cmd->u_param.scan_param.scan_param_1m.scan_intv = scan_intv;
			p_cmd->u_param.scan_param.scan_param_1m.scan_wd = scan_wd;
			p_cmd->u_param.scan_param.dup_filt_pol = 0;
			p_cmd->u_param.scan_param.duration = 0;
			p_cmd->u_param.scan_param.period = 10;

			p_cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not created\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_stop_scaning(uint8_t actv_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_SCAN_STARTED) {
		// Prepare the GAPM_ACTIVITY_STOP_CMD message
		struct gapm_activity_stop_cmd *cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
									TASK_BLE_GAPM, TASK_BLE_APP,
									gapm_activity_stop_cmd);

		if (cmd) {
			// Fill the allocated kernel message
			cmd->operation = GAPM_STOP_ACTIVITY;
			cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			// Send the message
			kernel_msg_send(cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not started\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_delete_scaning(uint8_t actv_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_SCAN_CREATED) {
		// Prepare the GAPM_ACTIVITY_CREATE_CMD message
		struct gapm_activity_delete_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
									TASK_BLE_GAPM, TASK_BLE_APP,
									gapm_activity_delete_cmd);

		if (p_cmd) {
			// Set operation code
			p_cmd->operation = GAPM_DELETE_ACTIVITY;
			p_cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not created\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

#if (CFG_BLE_PER_ADV)
ble_err_t app_ble_create_periodic_advertising(uint8_t actv_idx, struct per_adv_param *per_adv)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_IDLE) {
		struct gapm_activity_create_adv_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
										TASK_BLE_GAPM, TASK_BLE_APP,
										gapm_activity_create_adv_cmd);

		if (p_cmd) {
			/// Set operation code
			p_cmd->operation = GAPM_CREATE_ADV_ACTIVITY;

			// Fill the allocated kernel message
			p_cmd->own_addr_type = per_adv->own_addr_type;
			p_cmd->adv_param.type = GAPM_ADV_TYPE_PERIODIC;
			p_cmd->adv_param.prop = per_adv->adv_prop;
			p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
			p_cmd->adv_param.prim_cfg.chnl_map = per_adv->chnl_map;
			p_cmd->adv_param.prim_cfg.phy = per_adv->prim_phy;

			p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;
			p_cmd->adv_param.prim_cfg.adv_intv_min = per_adv->adv_intv_min;
			p_cmd->adv_param.prim_cfg.adv_intv_max = per_adv->adv_intv_max;

			p_cmd->adv_param.second_cfg.max_skip = 3;
			p_cmd->adv_param.second_cfg.phy = per_adv->second_phy;
			p_cmd->adv_param.second_cfg.adv_sid = actv_idx;

			p_cmd->adv_param.period_cfg.adv_intv_max = 80;
			p_cmd->adv_param.period_cfg.adv_intv_min = 40;
			p_cmd->adv_param.period_cfg.cte_count = 0;

			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not idle\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}
	return ret;
}

ble_err_t app_ble_set_periodic_adv_data(uint8_t actv_idx, unsigned char *per_adv_buff, uint16_t per_adv_len)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if ((app_ble_actv_state_get(actv_idx) == ACTV_ADV_CREATED)
		|| (app_ble_actv_state_get(actv_idx) == ACTV_ADV_STARTED)) {
		struct gapm_set_adv_data_cmd *p_cmd = KERNEL_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
										TASK_BLE_GAPM, TASK_BLE_APP,
										gapm_set_adv_data_cmd,
										per_adv_len);

		if (p_cmd) {
			p_cmd->operation = GAPM_SET_PERIOD_ADV_DATA;
			p_cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;
			memcpy(&p_cmd->data[0], per_adv_buff, per_adv_len);
			p_cmd->length = per_adv_len;

			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("actv[%d] is not created\r\n", actv_idx);
		ret = ERR_BLE_STATUS;
	}
	return ret;
}
#endif

#if (CFG_BLE_PER_SYNC)
ble_err_t app_ble_create_periodic_sync(uint8_t actv_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_IDLE ||
		app_ble_actv_state_get(actv_idx) == ACTV_PER_SYNC_CREATED) {
		// Prepare the GAPM_ACTIVITY_CREATE_CMD message
		struct gapm_activity_create_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
												TASK_BLE_GAPM, TASK_BLE_APP,
												gapm_activity_create_cmd);

		if (p_cmd) {
			memset(p_cmd, 0, sizeof(*p_cmd));
			p_cmd->operation = GAPM_CREATE_PERIOD_SYNC_ACTIVITY;
			p_cmd->own_addr_type = GAPM_STATIC_ADDR;

			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_start_periodic_sync(uint8_t actv_idx, ble_periodic_sync_param_t *param)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_PER_SYNC_CREATED) {
		// Prepare the GAPM_ACTIVITY_START_CMD message
		struct gapm_activity_start_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
												TASK_BLE_GAPM, TASK_BLE_APP,
												gapm_activity_start_cmd);

		if (p_cmd) {
			p_cmd->operation = GAPM_START_ACTIVITY;
			p_cmd->u_param.per_sync_param.skip = param->skip;
			p_cmd->u_param.per_sync_param.sync_to = param->sync_to;
			p_cmd->u_param.per_sync_param.type = param->per_sync_type;
			p_cmd->u_param.per_sync_param.report_en_bf = param->report_en_bf;
			p_cmd->u_param.per_sync_param.cte_type = param->cte_type;
			p_cmd->u_param.per_sync_param.adv_addr.adv_sid = param->adv_sid;
			p_cmd->u_param.per_sync_param.adv_addr.addr_type = param->adv_addr_type;
			memcpy(&p_cmd->u_param.per_sync_param.adv_addr.addr, &param->adv_addr, sizeof(param->adv_addr));
			p_cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_stop_periodic_sync(uint8_t actv_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_PER_SYNC_STARTED) {
		// Prepare the GAPM_ACTIVITY_STOP_CMD message
		struct gapm_activity_stop_cmd *cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
											TASK_BLE_GAPM, TASK_BLE_APP,
											gapm_activity_stop_cmd);

		if (cmd) {
			// Fill the allocated kernel message
			cmd->operation = GAPM_STOP_ACTIVITY;
			cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			// Send the message
			kernel_msg_send(cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_delete_periodic_sync(uint8_t actv_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_ACTVS_IDX(actv_idx);

	if (app_ble_actv_state_get(actv_idx) == ACTV_PER_SYNC_CREATED) {
		// Prepare the GAPM_ACTIVITY_DELETE_CMD message
		struct gapm_activity_delete_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
												TASK_BLE_GAPM, TASK_BLE_APP,
												gapm_activity_delete_cmd);

		if (p_cmd) {
			// Set operation code
			p_cmd->operation = GAPM_DELETE_ACTIVITY;
			p_cmd->actv_idx = app_ble_env.actvs[actv_idx].gap_advt_idx;

			// Send the message
			kernel_msg_send(p_cmd);

			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		ret = ERR_BLE_STATUS;
	}

	return ret;
}
#endif

#if (CFG_BLE_PER_ADV) | (CFG_BLE_PER_SYNC)
ble_err_t app_ble_periodic_adv_sync_transf(uint8_t actv_idx, uint16_t service_data)
{
	ble_err_t ret = ERR_SUCCESS;

	struct gapc_per_adv_sync_trans_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPC_PER_ADV_SYNC_TRANS_CMD,
											TASK_BLE_GAPC, TASK_BLE_APP,
											gapc_per_adv_sync_trans_cmd);

	if (p_cmd) {
		p_cmd->operation = GAPC_PER_ADV_SYNC_TRANS;
		p_cmd->actv_idx = actv_idx;
		p_cmd->service_data = service_data;

		// Send the message
		kernel_msg_send(p_cmd);

		ret = ERR_SUCCESS;
	} else {
		ret = ERR_NO_MEM;
	}

	return ret;
}
#endif

ble_err_t app_ble_set_le_pkt_size(uint8_t conn_idx,uint16_t pkt_size)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_CONN_IDX(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if (app_ble_env.connections[conn_idx].role == APP_BLE_MASTER_ROLE) {
		conn_idx = BLE_APP_INITING_INDEX(conn_idx);
	}

	if ((conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)) {
		struct gapc_set_le_pkt_size_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_SET_LE_PKT_SIZE_CMD,
									KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
									KERNEL_BUILD_ID(TASK_BLE_APP, conn_idx),
									gapc_set_le_pkt_size_cmd);

		if (cmd) {
			cmd->operation = GAPC_SET_LE_PKT_SIZE;
			cmd->tx_octets = pkt_size;
			cmd->tx_time = LE_MAX_TIME;
			kernel_msg_send(cmd);
			ret = ERR_SUCCESS;
		}else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("unknow conhdl");
		ret = ERR_BLE_STATUS;
	}
	return ret;
}

ble_err_t app_ble_get_peer_feature(uint8_t conn_idx)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_CONN_IDX(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if ((conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)) {
		struct gapc_get_info_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_GET_INFO_CMD,
									KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
									KERNEL_BUILD_ID(TASK_BLE_APP, conn_idx),
									gapc_get_info_cmd);

		if (cmd) {
			cmd->operation = GAPC_GET_PEER_FEATURES;

			kernel_msg_send(cmd);
			ret = ERR_SUCCESS;
		}else {
			ret = ERR_NO_MEM;
		}
	} else {
		bk_printf("unknow conhdl");
		ret = ERR_BLE_STATUS;
	}
	return ret;
}

ble_err_t app_ble_mtu_get(uint8_t conn_idx, uint16_t *p_mtu)
{
	ble_err_t ret = ERR_SUCCESS;
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if ((conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)) {
		*p_mtu =  gatt_bearer_mtu_get(conhdl,0);
	} else {
		ret = ERR_BLE_STATUS;
	}
	return ret;
}

ble_err_t app_ble_mtu_exchange(uint8_t conn_idx)
{
	#if BLE_GATT_CLI
	return sdp_update_gatt_mtu(conn_idx);
	#else
	return ERR_CMD_NOT_SUPPORT;
	#endif
}

ble_err_t app_ble_set_pref_mtu(uint8_t conn_idx,uint16_t mtu)
{
	ble_err_t ret = ERR_CMD_NOT_SUPPORT;

	#if (BLE_CON_MTU_INCLUDE)
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if (mtu > GAP_LE_MTU_MAX || mtu <= L2CAP_LE_MTU_MIN) {
		ret = ERR_INVALID_PARAM;
	} else {
		if (BLE_APP_CONHDL_IS_VALID(conhdl)) {
			struct gatt_set_pref_mtu_cmd *p_cmd = KERNEL_MSG_ALLOC(GATT_CMD,TASK_BLE_GATT,
																	KERNEL_BUILD_ID(TASK_BLE_APP, conn_idx),
																	gatt_set_pref_mtu_cmd);
			if (p_cmd) {
				p_cmd->cmd_code = GATT_SET_PREF_MTU;
				p_cmd->dummy = 0;
				p_cmd->conidx = conhdl;
				p_cmd->pref_mtu = mtu;

				kernel_msg_send(p_cmd);

				ret = ERR_SUCCESS;
			}
		} else {
			ret = ERR_INVALID_PARAM;
		}
	}
	#endif

	return ret;
}

ble_err_t app_ble_gap_read_phy(uint8_t conn_idx, ble_read_phy_t *phy)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_CONN_IDX(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if (BLE_APP_CONHDL_IS_VALID(conhdl)) {
		uint8_t link_id = BLE_CONHDL_TO_LINKID(gapc_get_conhdl(conhdl));

		if (hci_ble_read_phy(link_id, &phy->rx_phy, &phy->tx_phy)) {
			ret = ERR_CMD_NOT_SUPPORT;
		}
	} else {
		ret = ERR_INVALID_PARAM;
	}

	return ret;
}

ble_err_t app_ble_gap_set_phy(uint8_t conn_idx, ble_set_phy_t *phy)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_CONN_IDX(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if (app_ble_env.connections[conn_idx].role == APP_BLE_MASTER_ROLE) {
		conn_idx = BLE_APP_INITING_INDEX(conn_idx);
	}

	if (BLE_APP_CONHDL_IS_VALID(conhdl)) {
		// Prepare the GAPC_SET_PHY_CMD message
		struct gapc_set_phy_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_SET_PHY_CMD,
														KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
														KERNEL_BUILD_ID(TASK_BLE_APP, conn_idx),
														gapc_set_phy_cmd);

		if (cmd) {
			cmd->operation = GAPC_SET_PHY;
			cmd->phy_opt = phy->phy_opt;
			cmd->rx_phy = phy->rx_phy;
			cmd->tx_phy = phy->tx_phy;

			// Send the message
			kernel_msg_send(cmd);
			ret = ERR_SUCCESS;
		} else {
			ret = ERR_NO_MEM;
		}
	} else {
		ret = ERR_BLE_STATUS;
	}

	return ret;
}

ble_err_t app_ble_set_pref_slave_evt_dur(uint8_t conn_idx, uint8_t duration)
{
	ble_err_t ret = ERR_SUCCESS;
	BLE_APP_CHECK_CONN_IDX(conn_idx);
	uint8_t connhdl = app_ble_get_connhdl(conn_idx);

	if (app_ble_env.connections[conn_idx].role == APP_BLE_MASTER_ROLE) {
		return ERR_BLE_STATUS;
	}
	if (connhdl != UNKNOW_CONN_HDL) {
		struct gapc_set_pref_slave_evt_dur_cmd *cmd= KERNEL_MSG_ALLOC(GAPC_SET_PREF_SLAVE_EVT_DUR_CMD,
												KERNEL_BUILD_ID(TASK_BLE_GAPC, connhdl),
												KERNEL_BUILD_ID(TASK_BLE_APP, conn_idx),
												gapc_set_pref_slave_evt_dur_cmd);

		cmd->operation = GAPC_SET_PREF_SLAVE_EVT_DUR;
		cmd->duration = duration;

		// Send message
		kernel_msg_send(cmd);
	} else {
		ret = ERR_BLE_STATUS;
	}
	return ret;
}

void app_ble_send_conn_param_update_cfm(uint8_t con_idx,bool accept)
{
	uint8_t conidx = app_ble_get_connhdl(con_idx);

	// Send connection confirmation
	struct gapc_param_update_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
								KERNEL_BUILD_ID(TASK_BLE_GAPC, conidx), TASK_BLE_APP,
								gapc_param_update_cfm);

	cfm->accept = accept;
	cfm->ce_len_min = 10;
	cfm->ce_len_max = 20;

	// Send message
	kernel_msg_send(cfm);
}

#if (BLE_APP_SEC)
ble_err_t app_ble_get_bonded_device_num(uint8_t *dev_num)
{
    *dev_num = __builtin_popcount(app_sec_env.bonded);

    if (*dev_num > MAX_BOND_NUM) {
        *dev_num = 0xFF;
    }

    return ERR_SUCCESS;
}

ble_err_t app_ble_get_bonded_device_list(uint8_t *dev_num, bond_device_addr_t *dev_list)
{
    ble_err_t ret = ERR_SUCCESS;
    uint8_t exp_num = *dev_num;
    uint8_t act_num = 0;

    for (uint8_t i = 0; i < MAX_BOND_NUM; i++) {

        if ((app_sec_env.bonded >> i) & 1) {

            if (act_num++ < exp_num) {
                memcpy(dev_list->addr, app_sec_env.bond_info[i].peer_irk.addr.addr, GAP_BD_ADDR_LEN);
                dev_list->addr_type = app_sec_env.bond_info[i].peer_irk.addr.addr_type;
                dev_list->bond_idx = i;
                dev_list++;
            } else {
                ret = ERR_NO_MEM;
            }

        }

    }

    *dev_num = act_num;
    return ret;
}
#endif

ble_err_t app_ble_get_sendable_packets_num(uint16_t *pkt_total)
{
    *pkt_total = l2cap_chan_ll_buf_nb_get();
    return ERR_SUCCESS;
}

ble_err_t app_ble_get_cur_sendable_packets_num(uint16_t *pkt_curr)
{
    *pkt_curr = l2cap_chan_ll_buf_nb_avail_get();
    return ERR_SUCCESS;
}

ble_err_t app_ble_set_channels(bk_ble_channels_t *channels)
{
	ble_err_t ret = ERR_SUCCESS;

	struct gapm_set_channel_map_cmd *cmd = KERNEL_MSG_ALLOC(GAPM_SET_CHANNEL_MAP_CMD,
											TASK_BLE_GAPM, TASK_BLE_APP,
											gapm_set_channel_map_cmd);

	if (cmd){
		cmd->operation = GAPM_SET_CHANNEL_MAP;
		memcpy(cmd->chmap.map, channels->channels, BLE_CHANNELS_LEN);
		kernel_msg_send(cmd);
	} else {
		ret = ERR_NO_MEM;
	}

	return ret;
}

ble_err_t app_ble_get_wl_size_cmd(uint8_t *wl_size)
{
	*wl_size = BLE_WHITELIST_MAX;
	return ERR_SUCCESS;
}

ble_err_t app_ble_list_clear_wl_cmd(void)
{
	ble_err_t ret = ERR_SUCCESS;

	struct gapm_list_set_wl_cmd *cmd = KERNEL_MSG_ALLOC(GAPM_LIST_SET_CMD,
										TASK_BLE_GAPM, TASK_BLE_APP,
										gapm_list_set_wl_cmd);

	if (cmd) {
		cmd->operation = GAPM_SET_WL;
		cmd->size = 0;

		kernel_msg_send(cmd);
	} else {
		ret = ERR_NO_MEM;
	}
	return ret;
}

ble_err_t app_ble_update_wl_cmd(uint8_t add_remove, struct bd_addr *addr, uint8_t addr_type)
{
	ble_err_t ret = ERR_SUCCESS;

	struct gapm_list_update_cmd *cmd = KERNEL_MSG_ALLOC(GAPM_LIST_UPDATE_CMD,
											TASK_BLE_GAPM, TASK_BLE_APP,
											gapm_list_update_cmd);

	if (cmd) {
		if (add_remove) {
			cmd->operation = GAPM_LIST_ADD_DEVICE;
		} else {
			cmd->operation = GAPM_LIST_REMOVE_DEVICE;
		}

		cmd->list_type = GAP_WL;
		cmd->device_addr.addr_type = addr_type;
		memcpy(cmd->device_addr.addr, addr, GAP_BD_ADDR_LEN);

		kernel_msg_send(cmd);
	} else {
		ret = ERR_NO_MEM;
	}
	return ret;
}

ble_err_t app_ble_clear_per_adv_list_cmd(void)
{
	ble_err_t ret = ERR_SUCCESS;

	struct gapm_list_set_pal_cmd *cmd = KERNEL_MSG_ALLOC(GAPM_LIST_SET_CMD,
												TASK_BLE_GAPM, TASK_BLE_APP,
												gapm_list_set_pal_cmd);

	if (cmd) {
		cmd->operation = GAPM_SET_PAL;
		cmd->size = 0;

		kernel_msg_send(cmd);

		ret = ERR_SUCCESS;
	} else {
		ret = ERR_NO_MEM;
	}

	return ret;
}

ble_err_t app_ble_update_per_adv_list_cmd(uint8_t add_remove, gap_per_adv_bdaddr_t *p_pal_info)
{
	ble_err_t ret = ERR_SUCCESS;

	struct gapm_list_update_cmd *cmd = KERNEL_MSG_ALLOC(GAPM_LIST_UPDATE_CMD,
											TASK_BLE_GAPM, TASK_BLE_APP,
											gapm_list_update_cmd);

	if (cmd){
		if (add_remove) {
			cmd->operation = GAPM_LIST_ADD_DEVICE;
		} else {
			cmd->operation = GAPM_LIST_REMOVE_DEVICE;
		}

		cmd->list_type = GAP_PAL;
		cmd->device_addr.addr_type = p_pal_info->addr_type;
		cmd->adv_sid = p_pal_info->adv_sid;
		memcpy(cmd->device_addr.addr, p_pal_info->addr, GAP_BD_ADDR_LEN);

		kernel_msg_send(cmd);
	} else {
		ret = ERR_NO_MEM;
	}

	return ret;
}


#if (BLE_GATT_CLI)
ble_err_t app_ble_gattc_read_by_type(uint8_t conn_idx, uint16_t start_handle,
                                     uint16_t end_handle, const charac_uuid_t *p_uuid)
{
	struct sdp_env_tag *p_env = sdp_get_env_use_conidx(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if (p_env == NULL || !BLE_APP_CONHDL_IS_VALID(conhdl)) {
		return ERR_INVALID_PARAM;
	}

	struct gatt_cli_read_by_uuid_cmd *p_cmd = KERNEL_MSG_ALLOC(GATT_CMD, KERNEL_BUILD_ID(TASK_BLE_GATT, conhdl),
																KERNEL_BUILD_ID(TASK_BLE_APP, conn_idx),
																gatt_cli_read_by_uuid_cmd);

	if (p_cmd) {
		p_cmd->cmd_code = GATT_CLI_READ_BY_UUID;
		p_cmd->dummy = GATT_ROLE_CLIENT;
		p_cmd->user_lid = p_env->user_lib;
		p_cmd->conidx = conhdl;
		p_cmd->start_hdl = start_handle;
		p_cmd->end_hdl = end_handle;
		p_cmd->uuid_type = p_uuid->uuid_type;
		memcpy(p_cmd->uuid, p_uuid->uuid, GATT_UUID_128_LEN);

		kernel_msg_send(p_cmd);

		return ERR_SUCCESS;
	} else {
		return ERR_NO_MEM;
	}
}

ble_err_t app_ble_gattc_read_multiple(uint8_t conn_idx, app_gattc_multi_t *read_multi)
{
	struct sdp_env_tag *p_env = sdp_get_env_use_conidx(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if (p_env == NULL || !BLE_APP_CONHDL_IS_VALID(conhdl)) {
		return ERR_INVALID_PARAM;
	}

	struct gatt_cli_read_multiple_cmd *p_cmd = KERNEL_MSG_ALLOC_DYN(GATT_CMD, KERNEL_BUILD_ID(TASK_BLE_GATT, conhdl),
																KERNEL_BUILD_ID(TASK_BLE_APP, conn_idx),
																gatt_cli_read_multiple_cmd,
																read_multi->nb_att * sizeof(struct gatt_att));

	if (p_cmd) {
		p_cmd->cmd_code = GATT_CLI_READ_MULTIPLE;
		p_cmd->dummy = GATT_ROLE_CLIENT;
		p_cmd->user_lid = p_env->user_lib;
		p_cmd->conidx = conhdl;
		p_cmd->nb_att = read_multi->nb_att;
		memcpy(p_cmd->atts, read_multi->p_atts, read_multi->nb_att * sizeof(struct gatt_att));

		kernel_msg_send(p_cmd);

		return ERR_SUCCESS;
	} else {
		return ERR_NO_MEM;
	}
}
#endif

void app_ble_next_operation(uint8_t idx, uint8_t status)
{
	uint8_t op_idx;
	ble_cmd_param_t cmd_ret;
	ble_cmd_cb_t cmd_op_cb = app_ble_env.op_cb;
	ble_cmd_t cmd = app_ble_env.cmd;

	for (op_idx = 0; op_idx < 32; op_idx++) {
		if ((app_ble_env.op_mask >> op_idx) & 0x1)
			break;
	}

	switch (cmd) {
		case BLE_CREATE_ADV:
		case BLE_SET_ADV_DATA:
		case BLE_SET_RSP_DATA:
		case BLE_START_ADV:
		case BLE_STOP_ADV:
		case BLE_DELETE_ADV:
		case BLE_CREATE_SCAN:
		case BLE_START_SCAN:
		case BLE_STOP_SCAN:
		case BLE_DELETE_SCAN:
		case BLE_CREATE_PERIODIC_SYNC:
		case BLE_START_PERIODIC_SYNC:
		case BLE_STOP_PERIODIC_SYNC:
		case BLE_DELETE_PERIODIC_SYNC:
			app_ble_reset();
			if (cmd_op_cb) {
				cmd_ret.cmd_idx = idx;
				cmd_ret.status = status;
				cmd_op_cb(cmd, &cmd_ret);
			}
			break;
		case BLE_INIT_ADV:
			if ((app_ble_env.op_mask) && (status == ERR_SUCCESS)) {
				switch (op_idx) {
				case BLE_OP_SET_ADV_DATA_POS:
					app_ble_set_adv_data(idx, app_ble_env.actvs[idx].param.adv.advData, app_ble_env.actvs[idx].param.adv.advDataLen);
					break;
				case BLE_OP_SET_RSP_DATA_POS:
					app_ble_set_scan_rsp_data(idx, app_ble_env.actvs[idx].param.adv.respData, app_ble_env.actvs[idx].param.adv.respDataLen);
					break;
				case BLE_OP_START_ADV_POS:
					app_ble_start_advertising(idx, app_ble_env.actvs[idx].param.adv.duration);
					break;
				default:
					bk_printf("Cmd[%d] have err operation[%d]\r\n", app_ble_env.cmd, op_idx);
					break;
				}
			} else {
				if (status != ERR_SUCCESS) {
					if (app_ble_actv_state_get(idx) == ACTV_ADV_CREATED) {
						app_ble_delete_advertising(idx);
					} else {
						app_ble_reset();
					}
				} else {
					app_ble_reset();
				}
				if (cmd_op_cb) {
					app_ble_env.op_mask = 0;
					app_ble_env.op_cb = NULL;
					cmd_ret.cmd_idx = idx;
					cmd_ret.status = status;
					cmd_op_cb(cmd, &cmd_ret);
				}
			}
			break;
		case BLE_DEINIT_ADV:
			if (app_ble_env.op_mask) {
				if (op_idx == BLE_OP_DEL_ADV_POS) {
					app_ble_delete_advertising(idx);
				} else {
					bk_printf("Cmd[%d] have err operation[%d]\r\n", app_ble_env.cmd, op_idx);
				}
			} else {
				app_ble_reset();
				if (cmd_op_cb) {
					cmd_ret.cmd_idx = idx;
					cmd_ret.status = status;
					cmd_op_cb(cmd, &cmd_ret);
				}
			}
			break;
		case BLE_INIT_SCAN:
			if ((app_ble_env.op_mask) && (status == ERR_SUCCESS)) {
				if (op_idx == BLE_OP_START_SCAN_POS) {
					app_ble_start_scaning(idx, app_ble_env.actvs[idx].param.scan.interval, app_ble_env.actvs[idx].param.scan.window);
				} else {
					bk_printf("Cmd[%d] have err operation[%d]\r\n", app_ble_env.cmd, op_idx);
				}
			} else {
				if (cmd_op_cb) {
					app_ble_env.op_mask = 0;
					app_ble_env.op_cb = NULL;
					cmd_ret.cmd_idx = idx;
					cmd_ret.status = status;
					cmd_op_cb(cmd, &cmd_ret);
				}
				if (status != ERR_SUCCESS) {
					if (app_ble_actv_state_get(idx) == ACTV_SCAN_CREATED) {
						app_ble_delete_scaning(idx);
					} else {
						app_ble_reset();
					}
				} else {
					app_ble_reset();
				}
			}
			break;
		case BLE_DEINIT_SCAN:
			if (app_ble_env.op_mask) {
				if (op_idx == BLE_OP_DEL_SCAN_POS) {
					app_ble_delete_scaning(idx);
				} else {
					bk_printf("Cmd[%d] have err operation[%d]\r\n", app_ble_env.cmd, op_idx);
				}
			} else {
				app_ble_reset();
				if (cmd_op_cb) {
					cmd_ret.cmd_idx = idx;
					cmd_ret.status = status;
					cmd_op_cb(cmd, &cmd_ret);
				}
			}
			break;
		case BLE_INIT_CREATE:
			bk_printf("Cmd[%d]operation[%d]BLE_INIT_CREATE\r\n", app_ble_env.cmd, op_idx);
			app_ble_reset();
			if (cmd_op_cb) {
				cmd_ret.cmd_idx = idx;
				cmd_ret.status = status;
				cmd_op_cb(cmd, &cmd_ret);
			}
			break;
		case BLE_INIT_START_CONN:
			bk_printf("Cmd[%d]operation[%d]BLE_INIT_START_CONN\r\n", app_ble_env.cmd, op_idx);
			app_ble_reset();
			if (cmd_op_cb) {
				cmd_ret.cmd_idx = idx;
				cmd_ret.status = status;
				cmd_op_cb(cmd, &cmd_ret);
			}
			break;
		case BLE_INIT_STOP_CONN:
			bk_printf("Cmd[%d]operation[%d]BLE_INIT_STOP_CONN\r\n", app_ble_env.cmd, op_idx);
			app_ble_reset();
			if (cmd_op_cb) {
				cmd_ret.cmd_idx = idx;
				cmd_ret.status = status;
				cmd_op_cb(cmd, &cmd_ret);
			}
			break;
		case BLE_INIT_DELETE:
			bk_printf("Cmd[%d]operation[%d]BLE_INIT_DELETE\r\n", app_ble_env.cmd, op_idx);
			app_ble_reset();
			if (cmd_op_cb) {
				cmd_ret.cmd_idx = idx;
				cmd_ret.status = status;
				cmd_op_cb(cmd, &cmd_ret);
			}
			break;
		default:
			bk_printf("unknow ble app command:%d\r\n", app_ble_env.cmd);
			break;
	}
}

ble_err_t app_ble_gatts_remove_service(uint8_t user_lid, uint16_t start_handle)
{
	struct gatt_db_svc_remove_cmd * p_cmd = KERNEL_MSG_ALLOC(GATT_CMD, TASK_BLE_GATT, TASK_BLE_APP,
															gatt_db_svc_remove_cmd);

	if (p_cmd) {
		p_cmd->cmd_code = GATT_DB_SVC_REMOVE;
		p_cmd->dummy = start_handle;
		p_cmd->user_lid = user_lid;
		p_cmd->start_hdl = start_handle;

		kernel_msg_send(p_cmd);

		return ERR_SUCCESS;
	} else {
		return ERR_NO_MEM;
	}
}

ble_err_t app_ble_gatts_app_unregister(uint8_t user_lid, uint16_t service_handle)
{
	struct gatt_user_unregister_cmd * p_cmd = KERNEL_MSG_ALLOC(GATT_CMD, TASK_BLE_GATT, TASK_BLE_APP,
																gatt_user_unregister_cmd);

	if (p_cmd) {
		p_cmd->cmd_code = GATT_USER_UNREGISTER;
		p_cmd->dummy = service_handle;
		p_cmd->user_lid = user_lid;

		kernel_msg_send(p_cmd);

		return ERR_SUCCESS;
	} else {
		return ERR_NO_MEM;
	}
}

ble_err_t app_ble_gatts_read_response(app_gatts_rsp_t *rsp)
{
	gatt_db_svc_t *p_svc = NULL;
	uint8_t conhdl = app_ble_get_connhdl(rsp->con_idx);

	if (!BLE_APP_CONHDL_IS_VALID(conhdl) || gatt_db_svc_find(rsp->attr_handle, &p_svc)) {
		return ERR_INVALID_PARAM;
	}

	struct gatt_srv_att_read_get_cfm_st * cmd = KERNEL_MSG_ALLOC_DYN(GATT_CFM,
														KERNEL_BUILD_ID(TASK_BLE_GATT, conhdl),
														KERNEL_BUILD_ID(TASK_BLE_APP, rsp->con_idx),
														gatt_srv_att_read_get_cfm_st,
														rsp->value_length);

	if (cmd) {
		cmd->req_ind_code = GATT_SRV_ATT_READ_GET;
		cmd->user_lid = p_svc->user_lid;
		cmd->token = rsp->token;
		cmd->status = rsp->status;
		cmd->conidx = conhdl;
		cmd->att_length = rsp->att_length;
		cmd->value_length = rsp->value_length;
		memcpy(cmd->value, rsp->value, rsp->value_length);

		kernel_msg_send(cmd);
	} else {
		return ERR_NO_MEM;
	}

	return ERR_SUCCESS;
}

ble_err_t app_ble_gatts_set_attr_value(uint16_t attr_handle, uint16_t length, uint8_t *value)
{
	ble_err_t ret = ERR_SUCCESS;
	gatt_db_att_t *p_att = NULL;
	gatt_db_svc_t *p_svc = NULL;

	uint8_t status = gatt_db_att_get(attr_handle, &p_svc, &p_att);

	if(status == GAP_ERR_NO_ERROR) {
		// value can not be set for following parameters
		if ((p_att->uuid == GATT_DECL_PRIMARY_SERVICE) || (p_att->uuid == GATT_DECL_SECONDARY_SERVICE)
			|| (p_att->uuid == GATT_DECL_CHARACTERISTIC) || (p_att->uuid == GATT_DECL_INCLUDE)
			|| (p_att->uuid == GATT_DESC_CHAR_EXT_PROPERTIES) || (p_att->uuid == GATT_DESC_CLIENT_CHAR_CFG)
			|| (p_att->uuid == GATT_DESC_SERVER_CHAR_CFG)) {

			ret = ERR_REQUEST_NOT_SUPPORTED;
		} else {
			if (p_att->attr_val.value == NULL) {
				p_att->attr_val.value = kernel_malloc(length, KERNEL_MEM_ATT_DB);
			} else {
				kernel_free(p_att->attr_val.value);
				p_att->attr_val.value = kernel_malloc(length, KERNEL_MEM_ATT_DB);
			}

			p_att->attr_val.length = length;
			memcpy(p_att->attr_val.value, value, length);
		}
	}
	return ret;
}

ble_err_t app_ble_gatts_get_attr_value(uint16_t attr_handle, uint16_t *length, uint8_t **value)
{
	ble_err_t ret = ERR_SUCCESS;
	gatt_db_att_t *p_att = NULL;
	gatt_db_svc_t *p_svc = NULL;

	uint8_t status = gatt_db_att_get(attr_handle, &p_svc, &p_att);

	if (status == GAP_ERR_NO_ERROR) {
		*length = p_att->attr_val.length;
		*value = p_att->attr_val.value;
	} else {
		ret = ERR_INVALID_HANDLE;
	}
	return ret;
}

ble_err_t app_ble_gatts_svc_chg_ind_send(uint16_t start_handle, uint16_t end_handle)
{
	// Inform that database has been updated
	gapc_svc_db_updated(start_handle, end_handle);
	return ERR_SUCCESS;
}

/////////////// other /////////////////////////////////////////////
void ble_appm_send_gapm_reset_cmd(void)
{
	//// Reset the stack
	struct gapm_reset_cmd *p_cmd = KERNEL_MSG_ALLOC(GAPM_RESET_CMD,
							TASK_BLE_GAPM,
							TASK_BLE_APP,
							gapm_reset_cmd);

	if (p_cmd) {
		p_cmd->operation = GAPM_RESET;
		kernel_msg_send(p_cmd);
	}
}

void appm_init( void )
{
	// Reset the application manager environment
	memset(&app_ble_env, 0, sizeof(struct app_env_tag));
#if BLE_CENTRAL
	app_ble_env.init_conn_par.phy_mask = GAPM_INIT_PROP_1M_BIT;
	memcpy(&app_ble_env.init_conn_par.conn_param_1m, &default_init_par, sizeof(struct appm_create_conn_param));
#endif

	for (int i = 0; i < BLE_CONNECTION_MAX; i++) {
		app_ble_env.connections[i].conhdl = UNKNOW_CONN_HDL;
		app_ble_env.connections[i].role = APP_BLE_NONE_ROLE;
	}
	// Create APP task
	kernel_task_create(TASK_BLE_APP, &TASK_BLE_DESC_APP);

	// Initialize Task state
	kernel_state_set(TASK_BLE_APP, APPM_INIT);
	/*------------------------------------------------------
	* INITIALIZE ALL MODULES
	*------------------------------------------------------*/
	#if (BLE_GATT_CLI)
	sdp_common_init();
	#endif

	#if (BLE_APP_SEC)
	app_sec_init();
	#endif

	// Reset the stack
	ble_appm_send_gapm_reset_cmd();
}

#endif //(BLE_APP_PRESENT)

/// @} APP
