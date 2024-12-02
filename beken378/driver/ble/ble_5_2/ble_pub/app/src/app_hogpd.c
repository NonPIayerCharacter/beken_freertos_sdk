#include <string.h>
#include "rwprf_config.h"     // SW configuration
#if (BLE_HID_DEVICE)
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "prf_types.h"               // Profile common types definition
#include "architect.h"                    // Platform Definitions
#include "prf.h"
#include "prf_utils.h"
#include "hogpd.h"
#include "app_ble.h"
#include "app_task.h"
#include "ble_ui.h"
#include "hogp_common.h"
#include "kernel_timer.h"
#define HIDS_KB_REPORT_ID       	1
#define HIDS_MOUSE_REPORT_ID    	2

const uint8_t gHIDReportDescriptor[] =
{
/**
* ----------------------------------------------------------------------------
* Keyboard
* ----------------------------------------------------------------------------
*/
    0x05, 0x01,// usage page(Generic Desktop)
    0x09, 0x06,// usage(keyboard)
    0xa1, 0x01,// collection(Application)
    0x85, HIDS_KB_REPORT_ID,// REPORT ID (1)
    0x05, 0x07,// usage page(key code)
    0x19, 0xe0,// Usage Minimum (224)
    0x29, 0xe7,// Usage Maximum (231)
    0x15, 0x00,// Logical Minimum (0)
    0x25, 0x01,// Logical Maximum (1)
    0x75, 0x01,// Report Size (1)
    0x95, 0x08,// Report Count (8)
    0x81, 0x02,// Input (Data, Variable,Absolute)
    0x95, 0x01,// Report count (1)
    0x75, 0x08,// Report size (8)
    0x81, 0x01,// Input (Constant)
    0x95, 0x05,// Report count (5)
    0x75, 0x01,// Report size (1)
    0x05, 0x08,// usage page(LED)
    0x19, 0x01,// Usage Minimum (1)
    0x29, 0x05,// Usage Maximum (5)
    0x91, 0x02,// output (data, varable, absolute)
    0x95, 0x01,// Report Count (1)
    0x75, 0x03,// Report Size (3)
    0x91, 0x01,// output (Constant)
    0x95, 0x06,// Report Count (6)
    0x75, 0x08,// Report Size (8)
    0x15, 0x00,// Logical Minimum (0)
    0x25, 0xff,// Logical Maximum (255)
    0x05, 0x07,// usage page(key code)
    0x19, 0x00,// Usage Minimum (0)
    0x29, 0xff,// Usage maximum (255)
    0x81, 0x00,// Input (Data, array)
    0xc0,      // end collection
/**
* ----------------------------------------------------------------------------
* Mouse
* ----------------------------------------------------------------------------
*/
    0x05, 0x01,  /// USAGE PAGE (Generic Desktop)
    0x09, 0x02,  /// USAGE (Mouse)
    0xa1, 0x01,  /// COLLECTION (Application)
    0x85, HIDS_MOUSE_REPORT_ID, /// REPORT ID (2)
    0x09, 0x01,  /// USAGE (Pointer)
    0xa1, 0x00,  /// COLLECTION (Physical)
    0x05, 0x09,  /// USAGE PAGE (Buttons)
    0x19, 0x01,  /// Usage Minimum (01) -Button 1
    0x29, 0x03,  /// Usage Maximum (03) -Button 3
    0x15, 0x00,  /// Logical Minimum (0)
    0x25, 0x01,  /// Logical Maximum (1)
    0x95, 0x03,  /// Report Count (3)
    0x75, 0x01,  /// Report Size (1)
    0x81, 0x02,  /// Input (Data, Variable,Absolute) - Button states
    0x95, 0x01,  /// Report Count (1)
    0x75, 0x05,  /// Report Size (5)
    0x81, 0x01,  /// Input (Constant) - Padding or Reserved bits
    0x05, 0x01,  /// USAGE PAGE (Generic Desktop)
    0x09, 0x30,  /// USAGE (X)
    0x09, 0x31,  /// USAGE (Y)
    0x15, 0x81,  /// LOGICAL MINIMUM (-127)
    0x25, 0x7f,  /// LOGICAL MAXIMUM (127)
    0x75, 0x08,  /// REPORT SIZE (8)
    0x95, 0x02,  /// REPORT COUNT (2)
    0x81, 0x06,  /// INPUT
    0xc0,        /// end collection
    0xc0,        /// end collection
};
ble_err_t bk_hogpd_create_db(struct hogpd_db_cfg *g_hogpd_db_cfg)
{
	ble_err_t ret = ERR_SUCCESS;
	if (kernel_state_get(TASK_BLE_APP) == APPM_READY) {
		struct hogpd_db_cfg *db_cfg;
		struct gapm_profile_task_add_cmd *req = KERNEL_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
													TASK_BLE_GAPM,TASK_BLE_APP,
													gapm_profile_task_add_cmd,
													sizeof(struct hogpd_db_cfg));
		// Fill message
		req->operation = GAPM_PROFILE_TASK_ADD;
		req->sec_lvl = SVC_SEC_LVL(AUTH);
		req->prf_api_id = TASK_BLE_ID_HOGPD;
		req->app_task = TASK_BLE_APP;
		req->user_prio = 0;
		req->start_hdl = 0;
		//Set parameters
		db_cfg = (struct hogpd_db_cfg* ) req->param;
		memcpy(db_cfg, g_hogpd_db_cfg, sizeof(struct hogpd_db_cfg));
		kernel_state_set(TASK_BLE_APP, APPM_CREATE_DB);
		//Send the message
		kernel_msg_send(req);
	} else {
		ret = ERR_CREATE_DB;
	}
	return ret;
}

void bk_hogpd_init(void)
{
	struct hogpd_db_cfg g_hogpd_db_cfg;
	g_hogpd_db_cfg.hids_nb = 1;
	g_hogpd_db_cfg.cfg[0].svc_features = 0x07;//keyboard | mouse | protocol mode:00000111
	g_hogpd_db_cfg.cfg[0].report_nb = 2;
	g_hogpd_db_cfg.cfg[0].report_char_cfg[0] = HOGPD_CFG_REPORT_IN;
	g_hogpd_db_cfg.cfg[0].report_id[0] = 1;//keyboard
	g_hogpd_db_cfg.cfg[0].report_char_cfg[1] = HOGPD_CFG_REPORT_IN;
	g_hogpd_db_cfg.cfg[0].report_id[1] = 2;//mouse
	//HID information
	g_hogpd_db_cfg.cfg[0].hid_info.bcdHID = 0x0111;//HID version 1.11
	g_hogpd_db_cfg.cfg[0].hid_info.bCountryCode = 0x00;
	g_hogpd_db_cfg.cfg[0].hid_info.flags = HIDS_REMOTE_WAKE_CAPABLE | HIDS_NORM_CONNECTABLE;
	bk_hogpd_create_db(&g_hogpd_db_cfg);
}
//client can operate on notify cccd.
void bk_hogpd_enable(void)
{
	struct hogpd_enable_req *p_req;
	prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(TASK_BLE_ID_HOGPD);
	// send req to HOGPD
	p_req = KERNEL_MSG_ALLOC(HOGPD_ENABLE_REQ, ble_prf->prf_task, TASK_BLE_APP, hogpd_enable_req);
	p_req->conidx = 0;
	p_req->ntf_cfg[0] = 0x0000;
	kernel_msg_send(p_req);
}

void bk_hogpd_send_report(uint8_t *data, uint8_t len)
{
	prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(TASK_BLE_ID_HOGPD);
	struct hogpd_report_upd_req * req = KERNEL_MSG_ALLOC_DYN(HOGPD_REPORT_UPD_REQ,
														ble_prf->prf_task,
														TASK_BLE_APP,
														hogpd_report_upd_req,len);
	if(len == HOGP_BOOT_REPORT_MAX_LEN)
		req->report_idx = 0;//keyboard
	else
		req->report_idx = 1;//mouse
	req->conidx  = 0;
	req->hid_idx  = 0;
	req->report_type = HOGPD_REPORT;
	req->report.length = len;
	memcpy(&req->report.value[0], &data[0], len);
	kernel_msg_send(req);
}

__STATIC int hogpd_enable_rsp_handler(kernel_msg_id_t const msgid, struct hogpd_enable_rsp const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] status = 0x%x\r\n",__func__, p_param->status);
	return (KERNEL_MSG_CONSUMED);
}

__STATIC int hogpd_report_upd_rsp_handler(kernel_msg_id_t const msgid, struct hogpd_report_upd_rsp const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] status = 0x%x\r\n",__func__, p_param->status);
	return (KERNEL_MSG_CONSUMED);
}

__STATIC int hogpd_report_read_req_ind_handler(kernel_msg_id_t const msgid, struct hogpd_report_read_req_ind const *param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] report_type = 0x%x\r\n",__func__,param->report_type);
	if ((param->report_type == HOGPD_REPORT_MAP))
	{
		struct hogpd_report_read_cfm *req = KERNEL_MSG_ALLOC_DYN(HOGPD_REPORT_READ_CFM,
											src_id,
											TASK_BLE_APP,
											hogpd_report_read_cfm,
											param->max_length);

		uint16_t map_size = sizeof(gHIDReportDescriptor);
		uint16_t valid_len = ( (map_size - param->offset) > param->max_length ) ? param->max_length : (map_size - param->offset);
		req->conidx = param->conidx;
		/// Status of the request
		req->status = GAP_ERR_NO_ERROR;
		req->token = param->token;
		req->tot_length = valid_len;
		/// Report Length (uint8_t)
		req->report.length = valid_len;
		/// Report Instance - 0 for boot reports and report map
		memcpy(&req->report.value[0], &gHIDReportDescriptor[0] + param->offset, req->report.length);
		// Send the message
		kernel_msg_send(req);
	}
	else {
		struct hogpd_report_read_cfm *req = KERNEL_MSG_ALLOC_DYN(HOGPD_REPORT_READ_CFM,
											src_id,
											TASK_BLE_APP,
											hogpd_report_read_cfm,
											HOGP_BOOT_REPORT_MAX_LEN);
		req->conidx = param->conidx;
		/// Status of the request
		req->status = GAP_ERR_NO_ERROR;
		req->token = param->token;
		req->tot_length = param->max_length;
		if(param->report_type == HOGPD_BOOT_KEYBOARD_INPUT_REPORT || param->report_idx == 0)
			req->report.length = 8;
		else if(param->report_type == HOGPD_BOOT_MOUSE_INPUT_REPORT || param->report_idx == 1)
			req->report.length = 3;
		/// Report data
		memset(&req->report.value[0], 0, req->report.length);
		kernel_msg_send(req);
	}
	return (KERNEL_MSG_CONSUMED);
}

__STATIC int hogpd_report_write_req_ind_handler(kernel_msg_id_t const msgid, struct hogpd_report_write_req_ind const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] report_idx = %d\r\n",__func__,p_param->report_idx);
	struct hogpd_report_write_cfm *p_req;
	p_req = KERNEL_MSG_ALLOC(HOGPD_REPORT_WRITE_CFM, src_id, dest_id, hogpd_report_write_cfm);
	p_req->conidx = p_param->conidx;
	p_req->status = GAP_ERR_NO_ERROR;
	p_req->token = p_param->token;
	kernel_msg_send(p_req);
	return (KERNEL_MSG_CONSUMED);
}

__STATIC int hogpd_proto_mode_req_ind_handler(kernel_msg_id_t const msgid, struct hogpd_proto_mode_req_ind const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] p_param->proto_mode = %d\r\n",__func__,p_param->proto_mode);
	struct hogpd_proto_mode_cfm *p_req;
	p_req = KERNEL_MSG_ALLOC(HOGPD_PROTO_MODE_CFM, src_id, dest_id, hogpd_proto_mode_cfm);
	p_req->conidx = p_param->conidx;
	p_req->token = p_param->token;
	p_req->status = GAP_ERR_NO_ERROR;
	p_req->hid_idx = p_param->hid_idx;
	p_req->proto_mode = p_param->proto_mode;
	kernel_msg_send(p_req);
	return (KERNEL_MSG_CONSUMED);
}

__STATIC int hogpd_ntf_cfg_ind_handler(kernel_msg_id_t const msgid, struct hogpd_ntf_cfg_ind const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] \r\n",__func__);
	return (KERNEL_MSG_CONSUMED);
}

__STATIC int hogpd_ctnl_pt_ind_handler(kernel_msg_id_t const msgid, struct hogpd_ctnl_pt_ind const *p_param,
									kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	bk_printf("[%s] \r\n",__func__);
	return (KERNEL_MSG_CONSUMED);
}

/// Default State handlers definition
__STATIC const struct kernel_msg_handler app_hogpd_msg_handler_tab[] =
{
	// Note: all messages must be sorted in ID ascending ordser
	{HOGPD_ENABLE_RSP,               (kernel_msg_func_t) hogpd_enable_rsp_handler},
	{HOGPD_REPORT_UPD_RSP,           (kernel_msg_func_t) hogpd_report_upd_rsp_handler},
	{HOGPD_REPORT_READ_REQ_IND,      (kernel_msg_func_t) hogpd_report_read_req_ind_handler},
	{HOGPD_REPORT_WRITE_REQ_IND,     (kernel_msg_func_t) hogpd_report_write_req_ind_handler},
	{HOGPD_PROTO_MODE_REQ_IND,       (kernel_msg_func_t) hogpd_proto_mode_req_ind_handler},
	{HOGPD_NTF_CFG_IND,              (kernel_msg_func_t) hogpd_ntf_cfg_ind_handler},
	{HOGPD_CTNL_PT_IND,              (kernel_msg_func_t) hogpd_ctnl_pt_ind_handler},
};
const struct app_subtask_handlers app_hogpd_table_handler =
	{&app_hogpd_msg_handler_tab[0], (sizeof(app_hogpd_msg_handler_tab)/sizeof(struct kernel_msg_handler))};

#endif


