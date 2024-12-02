#include <string.h>
#include "rwprf_config.h"
#if (BLE_BATT_SERVER)
#include "kernel_task.h"
#include "ble_api_5_x.h"
#include "prf_types.h"               // Profile common types definition
#include "architect.h"                    // Platform Definitions
#include "prf.h"
#include "prf_utils.h"
#include "bass.h"
#include "app_ble.h"
#include "app_task.h"
#include "app_bass.h"
#include "ble_ui.h"

ble_err_t bk_bass_create_db(struct bass_db_cfg *g_bass_db_cfg)
{
	ble_err_t ret = ERR_SUCCESS;

	if (kernel_state_get(TASK_BLE_APP) == APPM_READY) {
		struct bass_db_cfg *db_cfg;

		struct gapm_profile_task_add_cmd *req = KERNEL_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
													TASK_BLE_GAPM,TASK_BLE_APP,
													gapm_profile_task_add_cmd,
													sizeof(struct bass_db_cfg));
		// Fill message
		req->operation = GAPM_PROFILE_TASK_ADD;
		req->sec_lvl = 0;
		req->prf_api_id = TASK_BLE_ID_BASS;
		req->app_task = TASK_BLE_APP;
		req->user_prio = 0;
		req->start_hdl = 0;

		//Set parameters
		db_cfg = (struct bass_db_cfg* ) req->param;
		memcpy(db_cfg, g_bass_db_cfg, sizeof(struct bass_db_cfg));

		kernel_state_set(TASK_BLE_APP, APPM_CREATE_DB);
		//Send the message
		kernel_msg_send(req);
	} else {
		ret = ERR_CREATE_DB;
	}

	return ret;
}
void bk_bass_init(void)
{
    struct bass_db_cfg bas_db_cfg;

	bas_db_cfg.bas_nb = 1;
	bas_db_cfg.features[0] = BAS_BATT_LVL_NTF_SUP;
	bk_bass_create_db(&bas_db_cfg);
}

ble_err_t bk_bass_enable(uint8_t conidx)
{
	ble_err_t ret = ERR_SUCCESS;
		// Allocate the message
	prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(TASK_BLE_ID_BASS);
	struct bass_enable_req * req = KERNEL_MSG_ALLOC(BASS_ENABLE_REQ,
									ble_prf->prf_task,TASK_BLE_APP,
									bass_enable_req);
	req->conidx = conidx;
	req->ntf_cfg = PRF_CLI_START_NTF;
	req->old_batt_lvl[0] = 100;
	kernel_msg_send(req);
	return ret;
}
ble_err_t bk_bass_ntf_send(uint8_t batt_level)
{
	ble_err_t ret = ERR_SUCCESS;
		// Allocate the message
	prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(TASK_BLE_ID_BASS);
	struct bass_batt_level_upd_req * req = KERNEL_MSG_ALLOC(BASS_BATT_LEVEL_UPD_REQ,
									ble_prf->prf_task,TASK_BLE_APP,
									bass_batt_level_upd_req);
	req->bas_instance = 0;
	req->batt_level = batt_level;
	kernel_msg_send(req);
	return ret;
}
__STATIC int bass_enable_rsp_handler(kernel_msg_id_t const msgid, struct bass_enable_rsp const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s]\r\n",__func__);
	return (KERNEL_MSG_CONSUMED);
}

__STATIC int bass_batt_level_ntf_cfg_ind_handler(kernel_msg_id_t const msgid, struct bass_batt_level_ntf_cfg_ind const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s]\r\n",__func__);
	return (KERNEL_MSG_CONSUMED);
}

__STATIC int bass_batt_level_ntf_rsp_handler(kernel_msg_id_t const msgid, struct bass_batt_level_upd_rsp const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s]\r\n",__func__);
	return (KERNEL_MSG_CONSUMED);
}

/// Default State handlers definition
__STATIC const struct kernel_msg_handler app_bass_msg_handler_tab[] =
{
	// Note: all messages must be sorted in ID ascending ordser

	{BASS_ENABLE_RSP,               (kernel_msg_func_t) bass_enable_rsp_handler},
	{BASS_BATT_LEVEL_NTF_CFG_IND,   (kernel_msg_func_t) bass_batt_level_ntf_cfg_ind_handler},
	{BASS_BATT_LEVEL_UPD_RSP,       (kernel_msg_func_t) bass_batt_level_ntf_rsp_handler},
};

const struct app_subtask_handlers app_bass_table_handler =
	{&app_bass_msg_handler_tab[0], (sizeof(app_bass_msg_handler_tab)/sizeof(struct kernel_msg_handler))};
#endif
