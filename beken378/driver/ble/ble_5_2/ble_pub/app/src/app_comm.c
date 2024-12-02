#include "rwip_config.h"     // SW configuration

#if (BLE_APP_COMM)
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "app_comm.h"                //  Application Module Definitions
#include "app_ble.h"                     // Application Definitions
#include "app_task.h"                // application task definitions
#include "comm_task.h"               // health thermometer functions
#include "common_bt.h"
#include "prf_types.h"               // Profile common types definition
#include "architect.h"                    // Platform Definitions
#include "prf.h"
#include "prf_utils.h"
#include "comm.h"
#include "kernel_timer.h"
#include "ble_ui.h"

ble_err_t bk_ble_create_db (struct bk_ble_db_cfg *ble_db_cfg)
{
	ble_err_t ret = ERR_SUCCESS;

	bk_printf("ble create new db\r\n");
	if (kernel_state_get(TASK_BLE_APP) == APPM_READY) {
		struct bk_ble_db_cfg *db_cfg;

		struct gapm_profile_task_add_cmd *req = KERNEL_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
													TASK_BLE_GAPM,TASK_BLE_APP,
													gapm_profile_task_add_cmd,
													sizeof(struct bk_ble_db_cfg));
		// Fill message
		req->operation = GAPM_PROFILE_TASK_ADD;
		req->sec_lvl = ble_db_cfg->svc_perm;
		req->prf_api_id = TASK_BLE_ID_COMMON + ble_db_cfg->prf_task_id;
		req->app_task = TASK_BLE_APP;
		req->user_prio = 0;
		req->start_hdl = ble_db_cfg->start_hdl; //req->start_hdl = 0; dynamically allocated

		//Set parameters
		db_cfg = (struct bk_ble_db_cfg* ) req->param;
		memcpy(db_cfg, ble_db_cfg, sizeof(struct bk_ble_db_cfg));

		kernel_state_set(TASK_BLE_APP, APPM_CREATE_DB);
		//Send the message
		kernel_msg_send(req);
	} else {
		ret = ERR_CREATE_DB;
	}

	return ret;
}

ble_err_t bk_ble_send_ntf_value(uint32_t len,uint8_t *buf,uint16_t prf_id,uint16_t att_idx)
{
	ble_err_t ret = ERR_SUCCESS;
	uint16_t prf_task_id = prf_id + TASK_BLE_ID_COMMON;

	prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(prf_task_id);

	if (ble_prf) {
		// Allocate the message
		struct bk_ble_ntf_upd_req * req = KERNEL_MSG_ALLOC_DYN(BK_BLE_NTF_UPD_REQ,
														ble_prf->prf_task,TASK_BLE_APP,
														bk_ble_ntf_upd_req,len);

		req->length = len;
		memcpy(req->value,buf,len);
		req->att_id = att_idx;
		req->conidx = 0;

		kernel_msg_send(req);
	} else {
		ret = ERR_PROFILE;
	}

	return ret;
}

ble_err_t bk_ble_conidx_send_ntf(uint8_t conidx,uint32_t len,uint8_t *buf,uint16_t prf_id,uint16_t att_idx)
{
	ble_err_t ret = ERR_SUCCESS;
	uint16_t prf_task_id = prf_id + TASK_BLE_ID_COMMON;

	prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(prf_task_id);
	uint8_t conhdl = app_ble_get_connhdl(conidx);

	if (ble_prf && (conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)) {
		// Allocate the message
		struct bk_ble_ntf_upd_req * req = KERNEL_MSG_ALLOC_DYN(BK_BLE_NTF_UPD_REQ,
										ble_prf->prf_task,TASK_BLE_APP,
										bk_ble_ntf_upd_req,len);

		req->length = len;
		memcpy(req->value,buf,len);
		req->att_id = att_idx;
		req->conidx = conhdl;

		kernel_msg_send(req);
	} else {
		ret = ERR_PROFILE;
	}

	return ret;
}

ble_err_t bk_ble_send_ind_value(uint32_t len,uint8_t *buf,uint16_t prf_id,uint16_t att_idx)
{
	ble_err_t ret = ERR_SUCCESS;
	uint16_t prf_task_id = prf_id + TASK_BLE_ID_COMMON;

	prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(prf_task_id);

	if (ble_prf) {
		// Allocate the message
		struct bk_ble_ind_upd_req * req = KERNEL_MSG_ALLOC_DYN(BK_BLE_IND_UPD_REQ,
									ble_prf->prf_task,TASK_BLE_APP,
									bk_ble_ind_upd_req,len);
		req->length = len;
		memcpy(req->value, buf, len);
		req->att_id = att_idx;
		req->conidx = 0;
		kernel_msg_send(req);
	} else {
		ret = ERR_PROFILE;
	}
	return ret;
}

ble_err_t bk_ble_conidx_send_ind(uint8_t conidx,uint32_t len,uint8_t *buf,uint16_t prf_id,uint16_t att_idx)
{
	ble_err_t ret = ERR_SUCCESS;
	uint16_t prf_task_id = prf_id + TASK_BLE_ID_COMMON;

	prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(prf_task_id);
	uint8_t conhdl = app_ble_get_connhdl(conidx);

	if (ble_prf && (conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)) {
		// Allocate the message
		struct bk_ble_ind_upd_req * req = KERNEL_MSG_ALLOC_DYN(BK_BLE_IND_UPD_REQ,
									ble_prf->prf_task,TASK_BLE_APP,
									bk_ble_ind_upd_req,len);
		req->length = len;
		memcpy(req->value, buf, len);
		req->att_id = att_idx;
		req->conidx = conhdl;
		kernel_msg_send(req);
	} else {
		ret = ERR_PROFILE;
	}
	return ret;
}

#endif

