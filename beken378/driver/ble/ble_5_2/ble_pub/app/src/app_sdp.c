#include <string.h>
#include "rwip_config.h"             // SW configuration
#include "rwprf_config.h"

#if (BLE_APP_PRESENT && BLE_GATT_CLI)
#include "app_sdp.h"
#include "app_ble.h"
#include "sdp_comm.h"
#include "app_task.h"
#include "prf_types.h"
#include "common.h"

app_sdp_env_tag app_sdp_env;
sdp_notice_cb_t sdp_event_notice = NULL;
sdp_discovery_cb_t sdp_discovery_notice = NULL;
void register_app_sdp_characteristic_callback(app_sdp_callback cb)
{
	app_sdp_env.sdp_cb = cb;
}

void register_app_sdp_charac_callback(app_sdp_charac_callback cb)
{
	app_sdp_env.charac_cb = cb;
}

void register_app_sdp_common_callback(app_sdp_comm_callback comm_cb)
{
	app_sdp_env.comm_cb = comm_cb;
}

void register_app_sdp_service_tab(uint8_t service_tab_nb,app_sdp_service_uuid *service_tab)
{
	app_sdp_env.service_tab_nb = service_tab_nb;
	app_sdp_env.service_tab = service_tab;
}

void app_sdp_service_filtration(uint8_t en)
{
	app_sdp_env.filtration = (en > 0) ? 1 : 0;
}

void sdp_set_notice_cb(sdp_notice_cb_t func)
{
	sdp_event_notice = func;
}

void sdp_set_discovery_svc_cb(sdp_discovery_cb_t func)
{
	sdp_discovery_notice = func;
}

uint8_t sdp_svc_write_characteristic(uint8_t con_idx,uint16_t handle,uint16_t data_len,uint8_t *data)
{
	struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);
	uint8_t conhdl = app_ble_get_connhdl(con_idx);

	if (p_env == NULL || (conhdl == UNKNOW_CONN_HDL) || (conhdl == USED_CONN_HDL)) {
		return COMMON_BUF_ERR_INVALID_PARAM;
	}

	uint8_t charac_type = 0;
	void * p_chars;
	p_chars = sdp_get_db_use_handle(con_idx,handle,&charac_type);
	if (p_chars == NULL) {
		return COMMON_BUF_ERR_INVALID_PARAM;
	}

	uint8_t write_type = 0;
	if (charac_type == SDP_CHARACTERISTIC_VALUE_TYPE) {
		struct bk_prf_char_def *chars = (struct bk_prf_char_def *)p_chars;
		if (chars->val_hdl != handle) {
			return COMMON_BUF_ERR_INVALID_PARAM;
		}
		if (chars->prop & PROP(WR)) {
			write_type = GATT_WRITE;
		} else if (chars->prop & PROP(WC)) {
			write_type = GATT_WRITE_NO_RESP;
		} else {
			return COMMON_BUF_ERR_INVALID_PARAM;
		}
	} else if (charac_type == SDP_CHARACTERISTIC_CCCD_TYPE) {
		struct bk_prf_desc_def *desc = (struct bk_prf_desc_def *)p_chars;
		if (desc->desc_hdl != handle) {
			return COMMON_BUF_ERR_INVALID_PARAM;
		}
		write_type = GATT_WRITE;
	}

	struct gatt_cli_write_cmd *p_cmd = KERNEL_MSG_ALLOC_DYN(GATT_CMD,TASK_BLE_GATT,
															TASK_BLE_SDP,
															gatt_cli_write_cmd,data_len);
	if (p_cmd) {
		p_cmd->cmd_code = GATT_CLI_WRITE;
		p_cmd->dummy = p_env->user_lib | (write_type << 8);
		p_cmd->user_lid = p_env->user_lib;
		p_cmd->conidx = conhdl;
		p_cmd->write_type = write_type;
		p_cmd->hdl = handle;
		p_cmd->value_length = data_len;
		memcpy(p_cmd->value,data,data_len);

		kernel_msg_send(p_cmd);

	} else {
		return COMMON_BUF_ERR_INSUFFICIENT_RESOURCE;
	}

	return COMMON_BUF_ERR_NO_ERROR;
}

uint8_t sdp_svc_read_characteristic(uint8_t con_idx,uint16_t handle,uint16_t offset,uint16_t length)
{
	struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);
	uint8_t conhdl = app_ble_get_connhdl(con_idx);

	if (p_env == NULL || (conhdl == UNKNOW_CONN_HDL) || (conhdl == USED_CONN_HDL)) {
		return COMMON_BUF_ERR_INVALID_PARAM;
	}

	struct gatt_cli_read_cmd *p_cmd = KERNEL_MSG_ALLOC(GATT_CMD,TASK_BLE_GATT,
														TASK_BLE_SDP,
														gatt_cli_read_cmd);
	if (p_cmd) {
		p_cmd->cmd_code = GATT_CLI_READ;
		p_cmd->dummy = 0;
		p_cmd->user_lid = p_env->user_lib;
		p_cmd->conidx = conhdl;
		p_cmd->hdl = handle;
		p_cmd->offset = offset;
		p_cmd->length = length;

		kernel_msg_send(p_cmd);

	} else {
		return COMMON_BUF_ERR_INSUFFICIENT_RESOURCE;
	}
	return COMMON_BUF_ERR_NO_ERROR;
}

uint8_t sdp_update_gatt_mtu(uint8_t con_idx)
{
	struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);
	uint8_t conhdl = app_ble_get_connhdl(con_idx);

	if (p_env == NULL || (conhdl == UNKNOW_CONN_HDL) || (conhdl == USED_CONN_HDL)) {
		return COMMON_BUF_ERR_INVALID_PARAM;
	}

	if ((conhdl != UNKNOW_CONN_HDL) && (conhdl != USED_CONN_HDL)) {
		struct gatt_cli_mtu_update_cmd *p_cmd = KERNEL_MSG_ALLOC(GATT_CMD,TASK_BLE_GATT,
																TASK_BLE_SDP,
																gatt_cli_mtu_update_cmd);
		if (p_cmd) {
			p_cmd->cmd_code = GATT_CLI_MTU_UPDATE;
			p_cmd->dummy = p_env->user_lib;
			p_cmd->user_lid = p_env->user_lib;
			p_cmd->conidx = conhdl;

			kernel_msg_send(p_cmd);

			return COMMON_BUF_ERR_NO_ERROR;
		} else {
			return COMMON_BUF_ERR_INSUFFICIENT_RESOURCE;
		}
	} else {
		return COMMON_BUF_ERR_INSUFFICIENT_RESOURCE;
	}

	return COMMON_BUF_ERR_NO_ERROR;
}

ble_err_t sdp_get_att_infor(uint8_t con_idx,struct sdp_att_event_t *param)
{
	uint8_t conhdl = app_ble_get_connhdl(con_idx);

	if((conhdl == UNKNOW_CONN_HDL) || (conhdl == USED_CONN_HDL)){
		return COMMON_BUF_ERR_INVALID_PARAM;
	}

	if (app_ble_env.connections[con_idx].sdp_end) {
		struct sdp_att_event_t *p_cmd = KERNEL_MSG_ALLOC(APP_INIT_GET_SDP_INFO,
												KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
												KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
												sdp_att_event_t);
		p_cmd->type = param->type;
		p_cmd->uuid_len = param->uuid_len;
		memcpy(p_cmd->uuid, param->uuid, 16);
		p_cmd->start_hdl = param->start_hdl;
		p_cmd->end_hdl = param->end_hdl;
		p_cmd->svr_id = param->svr_id;
		kernel_msg_send(p_cmd);
	}
	else
	{
		if(app_ble_env.connections[con_idx].sdp_ing == 0){
			struct sdp_att_event_t *p_cmd = KERNEL_MSG_ALLOC(APP_INIT_START_SDP,
										KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
										KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
										sdp_att_event_t);
			p_cmd->type = param->type;
			p_cmd->uuid_len = param->uuid_len;
			memcpy(p_cmd->uuid, param->uuid, 16);
			p_cmd->start_hdl = param->start_hdl;
			p_cmd->end_hdl = param->end_hdl;
			p_cmd->svr_id = param->svr_id;
			kernel_msg_send(p_cmd);
		}
	}
	return COMMON_BUF_ERR_NO_ERROR;
}

uint16_t sdp_get_att_hdl(uint8_t con_idx,uint8_t srv_index,uint8_t char_index,uint8_t desc_index)
{
	uint16_t hdl=0xffff;
	struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);
	if((srv_index == 0)||(p_env == NULL))
	{
		return hdl;
	}

	struct sdp_db *p_db = (struct sdp_db *)common_list_pick(&p_env->svr_list);
	struct db *p_svr = NULL;
	uint8_t srv_idx=1;
	while(p_db){
		p_svr = &p_db->svr;

		if(srv_idx == srv_index)
		{
			if((char_index != 0)&&(char_index <= p_svr->chars_nb))
			{
				if(desc_index != 0)
				{
					int first_desc_hdl=0;
					for(int i=0;i<p_svr->descs_nb;i++)
					{
						if((p_svr->chars[char_index-1].val_hdl+1) == p_svr->descs[i].desc_hdl)
						{
							first_desc_hdl = p_svr->descs[i].desc_hdl;
						}
						if((p_svr->chars[char_index-1].val_hdl+desc_index)==p_svr->descs[i].desc_hdl)
						{
							if((p_svr->descs[i].desc_hdl - first_desc_hdl)==(desc_index-1))
								hdl = p_svr->descs[i].desc_hdl;
						}
					}
				}else
				{

					hdl = p_svr->chars[char_index-1].val_hdl;
				}
			}
			break;
		}

		p_db = (struct sdp_db *)common_list_next(&p_db->hdr);
		srv_idx++;
	}
	bk_printf("hdl:%d\r\n",hdl);
	return hdl;
}

uint8_t sdp_get_att_table(uint8_t con_idx,struct sdp_att_event_t const *param)
{
	struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);
	uint8_t srv_idx=1;
	if (p_env == NULL) {
		return COMMON_BUF_ERR_INVALID_PARAM;
	}
	struct sdp_db *p_db = (struct sdp_db *)common_list_pick(&p_env->svr_list);
	struct db *p_svr = NULL;
	bk_printf("sdp_get_att_table param->type=%d\r\n",param->type);

	while (p_db) {
		p_svr = &p_db->svr;
		if (param->type == SDP_ATT_GET_SVR_UUID_ALL) {
			//struct bk_prf_svc svc;
			if (sdp_discovery_notice) {
				sdp_discovery_notice(con_idx,SDP_ATT_GET_SVR_UUID_ALL,p_svr);
			}
		}

		if (param->type == SDP_ATT_SVR_ATT_BY_SVR_UUID) {
			if(param->svr_id == srv_idx )
			{
				if (sdp_discovery_notice) {
					sdp_discovery_notice(con_idx,SDP_ATT_SVR_ATT_BY_SVR_UUID,p_svr);
				}
			}
		}

		if (param->type == SDP_ATT_GET_ATT_UUID_ALL) {
			struct bk_prf_char_def chars;
			for (uint8_t index = 0; index < p_svr->chars_nb ; index++) {
				memcpy(&chars,&p_svr->chars[index], sizeof(struct bk_prf_char_def));
				if (param->start_hdl <= chars.val_hdl && param->end_hdl >= chars.val_hdl) {
					if (sdp_discovery_notice) {
						sdp_discovery_notice(con_idx,SDP_ATT_GET_ATT_UUID_ALL,&chars);
					}
				}
			}
		}

		if (param->type == SDP_ATT_GET_ATT_DESC_UUID_ALL) {
			struct bk_prf_desc_def desc;
			for (uint8_t index = 0; index < p_svr->descs_nb ; index++) {
				memcpy(&desc,&p_svr->descs[index], sizeof(struct bk_prf_desc_def));
				if (param->start_hdl <= desc.desc_hdl && param->end_hdl >= desc.desc_hdl) {
					if (sdp_discovery_notice) {
						sdp_discovery_notice(con_idx,SDP_ATT_GET_ATT_DESC_UUID_ALL,&desc);
					}
				}
			}
		}
		p_db = (struct sdp_db *)common_list_next(&p_db->hdr);
		srv_idx++;
	}
	//report att end
	if (sdp_discovery_notice) {
		sdp_discovery_notice(con_idx,SDP_ATT_COMPLETE,NULL);
	}

	return COMMON_BUF_ERR_NO_ERROR;
}

uint8_t sdp_get_all_service(uint8_t con_idx)
{
	struct sdp_att_event_t msg_event;

	memset(&msg_event, 0, sizeof(msg_event));
	msg_event.type = SDP_ATT_GET_SVR_UUID_ALL;

	return sdp_get_att_infor(con_idx, &msg_event);
}

uint8_t sdp_get_all_char(uint8_t con_idx, uint16_t start_hdl, uint16_t end_hdl)
{
	struct sdp_att_event_t msg_event;

	memset(&msg_event, 0, sizeof(msg_event));
	msg_event.type = SDP_ATT_GET_ATT_UUID_ALL;
	msg_event.start_hdl = start_hdl;
	msg_event.end_hdl = end_hdl;

	return sdp_get_att_infor(con_idx, &msg_event);
}

uint8_t sdp_get_all_desc(uint8_t con_idx, uint16_t start_hdl, uint16_t end_hdl)
{
	struct sdp_att_event_t msg_event;

	memset(&msg_event, 0, sizeof(msg_event));
	msg_event.type = SDP_ATT_GET_ATT_DESC_UUID_ALL;
	msg_event.start_hdl = start_hdl;
	msg_event.end_hdl = end_hdl;

	return sdp_get_att_infor(con_idx, &msg_event);
}

#endif

