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
#include "gapm_msg.h"               // GAP Manager Task API
#include "gapc_msg.h"               // GAP Controller Task API
#include "gatt_msg.h"
#include "gatt_int.h"

#include "common_bt.h"                   // Common BT Definition
#include "common_math.h"                 // Common Maths Definition

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

uint8_t app_ble_get_idle_actv_idx_handle(void)
{
	uint8_t index;

	for (index = 0; index < BLE_ACTIVITY_MAX; index++) {
		if (app_ble_env.actvs[index].actv_status == ACTV_IDLE) {
			break;
		}
	}

	if (index == BLE_ACTIVITY_MAX) {
		bk_printf("Don't have free actv\r\n");
		return UNKNOW_ACT_IDX;
	}

	return index;
}

uint8_t app_ble_get_idle_conn_idx_handle(void)
{
	uint8_t conn_idx;

	for (conn_idx = 0; conn_idx < BLE_CONNECTION_MAX; conn_idx++) {
		if (app_ble_env.connections[conn_idx].conhdl == UNKNOW_CONN_HDL) {
			break;
		}
	}

	if(conn_idx >= BLE_CONNECTION_MAX)
		return UNKNOW_CONN_HDL;

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
									 KERNEL_BUILD_ID(TASK_BLE_APP, BLE_APP_INITING_INDEX(conn_idx)),
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
									KERNEL_BUILD_ID(TASK_BLE_APP, BLE_APP_INITING_INDEX(conn_idx)),
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

ble_err_t app_ble_set_le_pkt_size(uint8_t conn_idx,uint16_t pkt_size)
{
	ble_err_t ret = ERR_SUCCESS;

	BLE_APP_CHECK_CONN_IDX(conn_idx);
	uint8_t conhdl = app_ble_get_connhdl(conn_idx);

	if ((conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)) {
		struct gapc_set_le_pkt_size_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_SET_LE_PKT_SIZE_CMD,
									KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
									KERNEL_BUILD_ID(TASK_BLE_APP, BLE_APP_INITING_INDEX(conn_idx)),
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
									KERNEL_BUILD_ID(TASK_BLE_APP, BLE_APP_INITING_INDEX(conn_idx)),
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

ble_err_t app_ble_gap_set_phy(uint8_t conn_idx, ble_set_phy_t * phy_info)
{
	ble_err_t ret = ERR_SUCCESS;
	BLE_APP_CHECK_CONN_IDX(conn_idx);
	if (app_ble_get_connhdl(conn_idx) != UNKNOW_CONN_HDL) {
		// Prepare the GAPC_GET_INFO_CMD message
		struct gapc_set_phy_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_SET_PHY_CMD,
														KERNEL_BUILD_ID(TASK_BLE_GAPC, app_ble_get_connhdl(conn_idx)),
														KERNEL_BUILD_ID(TASK_BLE_APP, BLE_APP_INITING_INDEX(conn_idx)),
														gapc_set_phy_cmd);
		if (cmd) {
			cmd->operation = GAPC_SET_PHY;
			cmd->phy_opt = phy_info->phy_opt;
			cmd->rx_phy = phy_info->rx_phy;
			cmd->tx_phy = phy_info->tx_phy;
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
		case BLE_CONN_UPDATE_MTU:
		case BLE_CONN_UPDATE_PARAM:
		case BLE_CONN_DIS_CONN:
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
		case BLE_INIT_DIS_CONN:
			break;
		case BLE_INIT_READ_CHAR:
		case BLE_INIT_WRITE_CHAR:
			bk_printf("Cmd[%d]operation[%d] READ/Write CHAR\r\n", app_ble_env.cmd, op_idx);
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
