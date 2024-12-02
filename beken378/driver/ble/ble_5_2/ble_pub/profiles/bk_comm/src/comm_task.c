#include "rwip_config.h"

#if (BLE_COMM_SERVER)
#include "string.h"
#include "gap.h"
#include "common_utils.h"
#include "kernel_mem.h"
#include "comm.h"
#include "comm_task.h"
#include "kernel_msg.h"
#include "prf.h"
#include "app_ble.h"

static uint8_t bk_ble_ntf_val(struct prf_data *prf_data,struct bk_ble_ntf_upd_req const *param)
{
	struct bk_ble_env_tag *ble_env = (struct bk_ble_env_tag*)prf_data->p_env;

	struct gatt_srv_event_send_cmd * p_cmd = KERNEL_MSG_ALLOC_DYN(GATT_CMD,TASK_BLE_GATT,
															prf_data->prf_task,
															gatt_srv_event_send_cmd,param->length);
	if (p_cmd) {
		p_cmd->cmd_code = GATT_SRV_EVENT_SEND;
		p_cmd->dummy = GATT_NOTIFY;
		p_cmd->user_lid = ble_env->user_lid;
		p_cmd->conidx = param->conidx;
		p_cmd->evt_type = GATT_NOTIFY;
		p_cmd->hdl = (ble_env->start_hdl + param->att_id);
		p_cmd->value_length = param->length;
		memcpy(p_cmd->value,param->value,param->length);

		kernel_msg_send(p_cmd);

		return COMMON_BUF_ERR_NO_ERROR;
	} else {
		return COMMON_BUF_ERR_INSUFFICIENT_RESOURCE;
	}
}

static uint8_t bk_ble_ind_val(struct prf_data *prf_data,struct bk_ble_ind_upd_req const *param)
{
	struct bk_ble_env_tag *ble_env = (struct bk_ble_env_tag*)prf_data->p_env;

	struct gatt_srv_event_send_cmd * p_cmd = KERNEL_MSG_ALLOC_DYN(GATT_CMD,TASK_BLE_GATT,
															prf_data->prf_task,
															gatt_srv_event_send_cmd,param->length);
	if (p_cmd) {
		p_cmd->cmd_code = GATT_SRV_EVENT_SEND;
		p_cmd->dummy = GATT_INDICATE;
		p_cmd->user_lid = ble_env->user_lid;
		p_cmd->conidx = param->conidx;
		p_cmd->evt_type = GATT_INDICATE;
		p_cmd->hdl = (ble_env->start_hdl + param->att_id);
		p_cmd->value_length = param->length;
		memcpy(p_cmd->value,param->value,param->length);

		kernel_msg_send(p_cmd);

		return COMMON_BUF_ERR_NO_ERROR;
	} else {
		return COMMON_BUF_ERR_INSUFFICIENT_RESOURCE;
	}
}

static int bk_ble_ntf_upd_req_handler(kernel_msg_id_t const msgid,
										struct bk_ble_ntf_upd_req const *param,
										kernel_task_id_t const dest_id,
										kernel_task_id_t const src_id)
{
	int msg_status = KERNEL_MSG_CONSUMED;
	// retrieve handle information
	prf_data_t *prf_data = prf_data_get_by_prf_task(dest_id);

	if (prf_data) {
		bk_ble_ntf_val(prf_data, param);
		msg_status =  KERNEL_MSG_CONSUMED;
	}
	return msg_status;
}

static int bk_ble_ind_upd_req_handler(kernel_msg_id_t const msgid,
										struct bk_ble_ind_upd_req const *param,
										kernel_task_id_t const dest_id,
										kernel_task_id_t const src_id)
{
	int msg_status = KERNEL_MSG_CONSUMED;
	// retrieve handle information
	prf_data_t *prf_data = prf_data_get_by_prf_task(dest_id);

	if (prf_data) {
		bk_ble_ind_val(prf_data, param);
		msg_status =  KERNEL_MSG_CONSUMED;
	}
	return msg_status;
}

static int bk_gatt_cmp_evt_handler(kernel_msg_id_t const msgid,
										struct gatt_proc_cmp_evt const *param,
										kernel_task_id_t const dest_id,
										kernel_task_id_t const src_id)
{
	uint8_t conn_idx = app_ble_find_conn_idx_handle(param->conidx);
	bk_printf("bk_gatt_cmp_evt_handler conn_idx:%d,cmd_code:%d,dummy:%d,user_lid:%d,status:0x%x\r\n",
		conn_idx,param->cmd_code,param->dummy,param->user_lid,param->status);
	return KERNEL_MSG_CONSUMED;
}

static int app_msg_handler(kernel_msg_id_t const msgid,
							void *param,
							kernel_task_id_t const dest_id,
							kernel_task_id_t const src_id)
{
	bk_printf("[%s]msgid:0x%x,dest_id:%d,src_id:%d\r\n",__func__,msgid,dest_id,src_id);
	return (KERNEL_MSG_CONSUMED);
}

/// Default State handlers definition
const struct kernel_msg_handler bk_ble_default_handler[] =
{
	{GATT_CMP_EVT,					(kernel_msg_func_t) bk_gatt_cmp_evt_handler},
	{BK_BLE_NTF_UPD_REQ,			(kernel_msg_func_t) bk_ble_ntf_upd_req_handler},
	{BK_BLE_IND_UPD_REQ,			(kernel_msg_func_t) bk_ble_ind_upd_req_handler},
	{KERNEL_MSG_DEFAULT_HANDLER,	(kernel_msg_func_t) app_msg_handler},
};

void comm_task_init(struct kernel_task_desc *task_desc, kernel_state_t *state)
{
	// Get the address of the environment
	task_desc->msg_handler_tab = bk_ble_default_handler;
	task_desc->msg_cnt = ARRAY_LEN(bk_ble_default_handler);
#if (BLE_HL_MSG_API)
	task_desc->state = NULL;
	task_desc->idx_max = BLE_CONN_IDX_MAX;
#endif
}

#endif
