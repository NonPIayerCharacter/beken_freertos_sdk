#include <string.h>
#include "rwprf_config.h"
#if (BLE_DIS_SERVER)
#include "kernel_task.h"
#include "ble_api_5_x.h"
#include "prf_types.h"               // Profile common types definition
#include "architect.h"                    // Platform Definitions
#include "prf.h"
#include "prf_utils.h"
#include "diss.h"
#include "diss_msg.h"
#include "app_ble.h"
#include "app_task.h"
#include "app_diss.h"
#include "ble_ui.h"
#include "sched.h"

ble_err_t bk_diss_create_db(struct diss_db_cfg *g_diss_db_cfg)
{
	ble_err_t ret = ERR_SUCCESS;

	if (kernel_state_get(TASK_BLE_APP) == APPM_READY) {
		struct diss_db_cfg *db_cfg;

		struct gapm_profile_task_add_cmd *req = KERNEL_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
													TASK_BLE_GAPM,TASK_BLE_APP,
													gapm_profile_task_add_cmd,
													sizeof(struct diss_db_cfg));
		// Fill message
		req->operation = GAPM_PROFILE_TASK_ADD;
		req->sec_lvl = 0;
		req->prf_api_id = TASK_BLE_ID_DISS;
		req->app_task = TASK_BLE_APP;
		req->user_prio = 0;
		req->start_hdl = 0;

		//Set parameters
		db_cfg = (struct diss_db_cfg* ) req->param;
		memcpy(db_cfg, g_diss_db_cfg, sizeof(struct diss_db_cfg));

		kernel_state_set(TASK_BLE_APP, APPM_CREATE_DB);
		//Send the message
		kernel_msg_send(req);
	} else {
		ret = ERR_CREATE_DB;
	}

	return ret;
}
void bk_diss_init(void)
{
    struct diss_db_cfg dis_db_cfg;
    dis_db_cfg.features = DIS_ALL_FEAT_SUP;
    bk_diss_create_db(&dis_db_cfg);
}

ble_err_t bk_diss_set(uint8_t val_id,uint8_t len,uint8_t *buf)
{
	ble_err_t ret = ERR_SUCCESS;
	prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(TASK_BLE_ID_DISS);
	struct diss_set_value_req * req = KERNEL_MSG_ALLOC(DISS_SET_VALUE_REQ,
									ble_prf->prf_task,TASK_BLE_APP,
									diss_set_value_req);
	req->val_id = val_id;
	req->length = len;
	memcpy(req->data, buf, len);
	kernel_msg_send(req);
	return ret;
}

__STATIC int diss_set_value_rsp_handler(kernel_msg_id_t const msgid, struct diss_set_value_rsp const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] val_id=%d\r\n",__func__,p_param->val_id);
	return (KERNEL_MSG_CONSUMED);
}

__STATIC int diss_value_req_ind_handler(kernel_msg_id_t const msgid, struct diss_value_req_ind const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] the attribute value with val_id %d is not set\r\n",__func__,p_param->val_id);
	struct diss_value_cfm * req = KERNEL_MSG_ALLOC(DISS_VALUE_CFM,
									src_id,dest_id,diss_value_cfm);
	req->token = p_param->token;
	req->val_id = p_param->val_id;
	kernel_msg_send(req);
	return (KERNEL_MSG_CONSUMED);
}

/// Default State handlers definition
__STATIC const struct kernel_msg_handler app_diss_msg_handler_tab[] =
{
	// Note: all messages must be sorted in ID ascending ordser

	{DISS_SET_VALUE_RSP,               (kernel_msg_func_t) diss_set_value_rsp_handler},
	{DISS_VALUE_REQ_IND,               (kernel_msg_func_t) diss_value_req_ind_handler},
};

const struct app_subtask_handlers app_diss_table_handler =
	{&app_diss_msg_handler_tab[0], (sizeof(app_diss_msg_handler_tab)/sizeof(struct kernel_msg_handler))};
#endif
