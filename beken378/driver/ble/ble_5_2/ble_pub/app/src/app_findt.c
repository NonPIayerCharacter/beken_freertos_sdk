#include "rwprf_config.h"     // SW configuration

#if (BLE_FINDME_TARGET)
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "prf_types.h"               // Profile common types definition
#include "architect.h"                    // Platform Definitions
#include "prf.h"
#include "prf_utils.h"
#include "findt.h"
#include "app_ble.h"
#include "app_task.h"
#include "app_findt.h"
#include "ble_ui.h"

ble_err_t bk_findt_create_db(struct findt_db_cfg *g_findt_db_cfg)
{
	ble_err_t ret = ERR_SUCCESS;

	if (kernel_state_get(TASK_BLE_APP) == APPM_READY) {
		struct findt_db_cfg *db_cfg;

		struct gapm_profile_task_add_cmd *req = KERNEL_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
													TASK_BLE_GAPM,TASK_BLE_APP,
													gapm_profile_task_add_cmd,
													sizeof(struct findt_db_cfg));
		// Fill message
		req->operation = GAPM_PROFILE_TASK_ADD;
		req->sec_lvl = 0;
		req->prf_api_id = TASK_BLE_ID_FINDT;
		req->app_task = TASK_BLE_APP;
		req->user_prio = 0;
		req->start_hdl = 0;

		//Set parameters
		db_cfg = (struct findt_db_cfg* ) req->param;
		memcpy(db_cfg, g_findt_db_cfg, sizeof(struct findt_db_cfg));

		kernel_state_set(TASK_BLE_APP, APPM_CREATE_DB);
		//Send the message
		kernel_msg_send(req);
	} else {
		ret = ERR_CREATE_DB;
	}

	return ret;
}

void bk_findt_init(void)
{
	struct findt_db_cfg g_findt_db_cfg;
	g_findt_db_cfg.dummy = 0;
	bk_findt_create_db(&g_findt_db_cfg);
}
__STATIC int findt_alert_ind_handler(kernel_msg_id_t const msgid, struct findt_alert_ind const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] alert_lvl=%d\r\n",__func__,p_param->alert_lvl);
	return (KERNEL_MSG_CONSUMED);
}

/// Default State handlers definition
__STATIC const struct kernel_msg_handler app_findt_msg_handler_tab[] =
{
	// Note: all messages must be sorted in ID ascending ordser
	{FINDT_ALERT_IND,               (kernel_msg_func_t) findt_alert_ind_handler},
};

const struct app_subtask_handlers app_findt_table_handler =
	{&app_findt_msg_handler_tab[0], (sizeof(app_findt_msg_handler_tab)/sizeof(struct kernel_msg_handler))};

#endif

