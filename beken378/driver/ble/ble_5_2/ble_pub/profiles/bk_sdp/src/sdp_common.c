#include <string.h>
#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT && BLE_GATT_CLI)
#include "sdp_comm.h"
#include "app_sdp.h"
#include "kernel_mem.h"
#include "app_ble.h"
#include "gatt_msg.h"
#include "app_task.h"

struct sdp_env_tag sdp_env[BLE_CONNECTION_MAX];
extern sdp_notice_cb_t sdp_event_notice;

struct sdp_env_tag * sdp_get_free_env(void)
{
	uint8_t con_idx;

	for (con_idx = 0;con_idx < BLE_CONNECTION_MAX;con_idx++) {
		if( sdp_env[con_idx].con_idx == SDP_INVALID_CONIDX) {
			break;
		}
	}
	if( con_idx == BLE_CONNECTION_MAX) {
		return NULL;
	}
	return &sdp_env[con_idx];
}

struct sdp_env_tag * sdp_get_env_use_conidx(uint8_t con_idx)
{
	struct sdp_env_tag * p_sdp = NULL;

	for (uint8_t idx = 0;idx < BLE_CONNECTION_MAX;idx++) {
		if(sdp_env[idx].con_idx == con_idx) {
			p_sdp = &sdp_env[idx];
			break;
		}
	}
	return p_sdp;
}

void * sdp_get_db_use_handle(uint8_t con_idx,uint16_t handle,uint8_t *charac_type)
{
	struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);

	if (p_env == NULL) {
		return NULL;
	}

	struct sdp_db *p_db = (struct sdp_db *)common_list_pick(&p_env->svr_list);
	struct db *p_svr = NULL;

	while (p_db) {
		p_svr = &p_db->svr;
		if (p_svr->svc.start_hdl <= handle && handle <= p_svr->svc.end_hdl) {
			break;
		}
		p_db = (struct sdp_db *)common_list_next(&p_db->hdr);
	}

	if (p_db == NULL) {
		return NULL;
	}

	uint8_t g_char;
	void * g_db = NULL;

	for (g_char = 0; g_char < p_svr->chars_nb; g_char++) {
		if (p_svr->chars[g_char].val_hdl == handle) {
			g_db = &(p_svr->chars[g_char]);
			*charac_type = SDP_CHARACTERISTIC_VALUE_TYPE;
			break;
		}
	}

	if( g_db == NULL) {
		for (g_char = 0; g_char < p_svr->descs_nb; g_char++) {
			if( p_svr->descs[g_char].desc_hdl == handle ) {
				g_db = &(p_svr->descs[g_char]);
				*charac_type = SDP_CHARACTERISTIC_CCCD_TYPE;
				break;
			}
		}
	}

	return g_db;
}

struct sdp_db * sdp_extract_svc_info(uint8_t nb_att, const gatt_svc_att_t* p_atts)
{
	uint8_t chars_nb = 0;
	uint8_t descs_nb = 0;
	uint8_t att;

	for (att=0 ;att < nb_att; att++) {
		if(p_atts[att].att_type == GATT_ATT_CHAR) {
			chars_nb++;
		}else if(p_atts[att].att_type == GATT_ATT_DESC) {
			descs_nb++;
		}
	}

	uint32_t malloc_size = sizeof(struct bk_prf_char_def) * chars_nb +
						sizeof(struct bk_prf_desc_def) * descs_nb +
						sizeof(struct sdp_db);
	bk_printf("[sdp_add_svr]chars_nb:%d,descs_nb:%d,malloc_size:%d\r\n",chars_nb,descs_nb,malloc_size);
	if (!kernel_check_malloc(malloc_size,KERNEL_MEM_NON_RETENTION)) {
		return NULL;
	}

	struct bk_prf_char_def *p_chars = NULL;
	struct bk_prf_desc_def *p_descs = NULL;
	struct sdp_db *p_db = NULL;

	p_db = (struct sdp_db *) kernel_malloc(sizeof(struct sdp_db),KERNEL_MEM_NON_RETENTION);
	memset(p_db,0,sizeof(struct sdp_db));
	if( chars_nb ) {
		p_chars = (struct bk_prf_char_def *) kernel_malloc(sizeof(struct bk_prf_char_def) * chars_nb,KERNEL_MEM_NON_RETENTION);
		p_db->svr.chars_nb = chars_nb;
		p_db->svr.chars = p_chars;
	}
	if( descs_nb ) {
		p_descs = (struct bk_prf_desc_def *) kernel_malloc(sizeof(struct bk_prf_desc_def) * descs_nb,KERNEL_MEM_NON_RETENTION);
		p_db->svr.descs_nb = descs_nb;
		p_db->svr.descs = p_descs;
	}

	uint8_t g_chars = 0;
	uint8_t g_descs = 0;
	struct db *p_svr = NULL;

	p_svr = &p_db->svr;
	for (att=0 ;att < nb_att; att++) {	
		if(p_atts[att].att_type == GATT_ATT_PRIMARY_SVC) {
			p_svr->svc.uuid_type = p_atts[att].uuid_type;
			p_svr->svc.start_hdl = p_atts[att].info.svc.start_hdl;
			p_svr->svc.end_hdl = p_atts[att].info.svc.end_hdl;
			memcpy(p_svr->svc.uuid,p_atts[att].uuid,16);
		} else if(p_atts[att].att_type == GATT_ATT_CHAR) {
			p_chars[g_chars].prop = p_atts[att].info.charac.prop;
			p_chars[g_chars].val_hdl = p_atts[att].info.charac.val_hdl;
			p_chars[g_chars].uuid_type = p_atts[att].uuid_type;
			memcpy(p_chars[g_chars].uuid,p_atts[att].uuid,16);
			g_chars++;

			//Find All Characteristic Descriptor
			uint8_t gd = att+1;
			do{
				if (gd >= nb_att) {
					break;
				}
				if (p_atts[gd].att_type == GATT_ATT_DESC) {
					p_descs[g_descs].desc_hdl = p_svr->svc.start_hdl + gd;
					p_descs[g_descs].prop = p_atts[gd].info.charac.prop;
					p_descs[g_descs].uuid_type = p_atts[gd].uuid_type;
					memcpy(p_descs[g_descs].uuid,p_atts[gd].uuid,16);
					g_descs++;
				}
				gd++;
			} while ((gd <nb_att) && (p_atts[gd].att_type != GATT_ATT_CHAR));
		}
	}

	if((chars_nb != g_chars) || (descs_nb != g_descs)) {
		bk_printf("[warning]add profile error:%d %d %d %d!!!\r\n",chars_nb,g_chars,descs_nb,g_descs);
		if (chars_nb) {
			kernel_free(p_db->svr.chars);
		}
		if (descs_nb) {
			kernel_free(p_db->svr.descs);
		}
		kernel_free(p_db);
		return NULL;
	}

	return p_db;
}


static void sdp_discover_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
	uint8_t con_idx = app_ble_find_conn_idx_handle(conidx);
	struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);

	if (p_env == NULL) {
		return;
	}

	//Find All Notify/Indicate Characteristic and register it
	struct sdp_db *p_db = (struct sdp_db *)common_list_pick(&p_env->svr_list);
	struct bk_prf_char_def *p_char = NULL;
	while (p_db) {
		if (p_db->svr.chars_nb) {
			for(uint8_t g = 0;g < p_db->svr.chars_nb;g++) {
				p_char = &p_db->svr.chars[g];
				if (p_char->prop & PROP(I) || p_char->prop & PROP(N)) {
					bk_printf("[%s][register]start_hdl:0x%x,end_hdl:0x%x\r\n",__func__,p_db->svr.svc.start_hdl,p_db->svr.svc.end_hdl);
					gatt_cli_event_register(conidx,user_lid,p_db->svr.svc.start_hdl,p_db->svr.svc.end_hdl);
					break;
				}
			}
		}
		p_db = (struct sdp_db *)common_list_next(&p_db->hdr);
	}
	sdp_event_t sdp_event;
	sdp_event.con_idx = con_idx;
	sdp_event.status = 0;

	if (sdp_event_notice) {
		sdp_event_notice(SDP_DISCOVER_SVR_DONE,&sdp_event);
	}

	//tell app_task sdp is done
	void *p_cmd = kernel_msg_alloc(APP_INIT_END_SDP,
									KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
									KERNEL_BUILD_ID(TASK_BLE_APP,BLE_APP_INITING_INDEX(con_idx)),
									0);
	kernel_msg_send(p_cmd);

}

static void sdp_read_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
	bk_printf("[%s]status:%d\r\n",__func__,status);
	sdp_event_t sdp_event;
	memset(&sdp_event,0,sizeof(sdp_event_t));
	sdp_event.con_idx = app_ble_find_conn_idx_handle(conidx);
	sdp_event.status = status;
	if (sdp_event_notice) {
		sdp_event_notice(SDP_CHARAC_READ_DONE,&sdp_event);
	}
}

static void sdp_write_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
	bk_printf("[%s]status:%d\r\n",__func__,status);
	sdp_event_t sdp_event;
	memset(&sdp_event,0,sizeof(sdp_event_t));
	sdp_event.con_idx = app_ble_find_conn_idx_handle(conidx);
	sdp_event.status = status;
	sdp_event.dummy = dummy;
	if (sdp_event_notice) {
		sdp_event_notice(SDP_CHARAC_WRITE_DONE,&sdp_event);
	}
}

static void sdp_att_val_get_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t dummy,
								uint16_t hdl, uint16_t offset, uint16_t max_length)
{
	bk_printf("%s\r\n",__func__);
}

static void sdp_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint8_t disc_info,
						uint8_t nb_att, const gatt_svc_att_t* p_atts)
{
	#if BLE_APP_SDP_DBG_CHECK(BLE_APP_SDP_WARN)
	bk_printf("[BK]%s,hdl:0x%x,disc_info:%d,nb_att:%d,user_lid:%d\r\n",__func__,hdl,disc_info,nb_att,user_lid);
	for(uint8_t j=0;j<nb_att;j++) {
		if( p_atts[j].uuid_type == GATT_UUID_16 ) {
			bk_printf("att_type:%d,uuid_type:%d,UUID:%x,start_hdl:0x%x,end_hdl:0x%x,val_hdl:0x%x,prop:0x%x\r\n",p_atts[j].att_type,p_atts[j].uuid_type, p_atts[j].uuid[0]| p_atts[j].uuid[1]<<8,p_atts[j].info.svc.start_hdl,
			p_atts[j].info.svc.end_hdl,p_atts[j].info.charac.val_hdl,p_atts[j].info.charac.prop);
		} else if( p_atts->uuid_type == GATT_UUID_128 ) {
			bk_printf("att_type:%d,uuid_type:%d,UUID:",p_atts[j].att_type,p_atts[j].uuid_type);
			for( uint8_t i=0;i<16;i++) {
				bk_printf("%x ", p_atts[j].uuid[i]);
			}
			bk_printf("\r\n");
		}
	}
	#endif

	uint8_t con_idx = app_ble_find_conn_idx_handle(conidx);
	if (con_idx >= BLE_CONNECTION_MAX) {
		return;
	}

	struct sdp_db *p_db = sdp_extract_svc_info(nb_att,p_atts);

	if (p_db) {
		bk_printf("sdp_svc_cb push db\r\n");
		struct sdp_env_tag *sdp_env = sdp_get_env_use_conidx(con_idx);
		common_list_push_back(&sdp_env->svr_list,&p_db->hdr);
	}

	sdp_event_t sdp_event;
	sdp_event.con_idx = con_idx;
	sdp_event.status = 0;
	if (sdp_event_notice) {
		sdp_event_notice(SDP_DISCOVER_SVR,&sdp_event);
	}
}

static void sdp_svc_info_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t start_hdl, uint16_t end_hdl,
							uint8_t uuid_type, const uint8_t* p_uuid)
{
	bk_printf("%s\r\n",__func__);
}

static void sdp_char_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t char_hdl, uint16_t val_hdl,
						uint8_t prop, uint8_t uuid_type, const uint8_t* p_uuid)
{
	bk_printf("%s\r\n",__func__);
}
static void sdp_desc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t desc_hdl,
						uint8_t uuid_type, const uint8_t* p_uuid)
{
	bk_printf("%s\r\n",__func__);
}
static void sdp_att_val_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint16_t offset,
							common_buf_t* p_data)
{
	uint8_t conn_idx = app_ble_find_conn_idx_handle(conidx);
	uint8_t *p_buf = common_buf_data(p_data);
	#if BLE_APP_SDP_DBG_CHECK(BLE_APP_SDP_WARN)
	bk_printf("[%s]conn_idx:%d,user_lid:%d,hdl:0x%x\r\n",__func__,conn_idx,user_lid,hdl);
	if (p_buf) {
		bk_printf("read:");
		for(uint16_t g = 0;g < p_data->data_len;g++) {
			bk_printf("0x%x ",p_buf[g]);
		}
		bk_printf("\r\n");
	}
	#endif
	sdp_event_t sdp_event;
	memset(&sdp_event,0,sizeof(sdp_event_t));
	sdp_event.con_idx = conn_idx;
	sdp_event.hdl = hdl;
	sdp_event.value = p_buf;
	sdp_event.value_length = p_data->data_len;
	if (sdp_event_notice) {
		sdp_event_notice(SDP_CHARAC_READ,&sdp_event);
	}
}
static void sdp_att_val_evt_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint8_t evt_type, bool complete, 
							uint16_t hdl, common_buf_t* p_data)
{
	uint8_t conn_idx = app_ble_find_conn_idx_handle(conidx);
	uint8_t *p_buf = common_buf_data(p_data);
	#if BLE_APP_SDP_DBG_CHECK(BLE_APP_SDP_WARN)
	bk_printf("[%s]conn_idx:%d,user_lid:%d,hdl:0x%x,evt_type:%d,complete:%d\r\n",__func__,conn_idx,user_lid,hdl,evt_type,complete);
	if (p_buf) {
		bk_printf("ntf/ind:");
		for(uint16_t g = 0;g < p_data->data_len;g++) {
			bk_printf("0x%x ",p_buf[g]);
		}
		bk_printf("\r\n");
	}
	#endif

	sdp_event_t sdp_event;
	memset(&sdp_event,0,sizeof(sdp_event_t));
	sdp_event.con_idx = conn_idx;
	sdp_event.hdl = hdl;
	sdp_event.value = p_buf;
	sdp_event.value_length = p_data->data_len;

	if (evt_type == GATT_NOTIFY) {
		if (sdp_event_notice) {
			sdp_event_notice(SDP_CHARAC_NOTIFY_EVENT,&sdp_event);
		}
	} else if(evt_type == GATT_INDICATE) {
		if (sdp_event_notice) {
			sdp_event_notice(SDP_CHARAC_INDICATE_EVENT,&sdp_event);
		}
	}
	gatt_cli_att_event_cfm(conidx, user_lid, token);
}

/// Client callback hander
static const gatt_cli_cb_t sdp_cb =
{
	.cb_discover_cmp    = sdp_discover_cmp_cb,
	.cb_read_cmp        = sdp_read_cmp_cb,
	.cb_write_cmp       = sdp_write_cmp_cb,
	.cb_att_val_get     = sdp_att_val_get_cb,
	.cb_svc             = sdp_svc_cb,
	.cb_svc_info        = sdp_svc_info_cb,
	.cb_inc_svc         = NULL,
	.cb_char            = sdp_char_cb,
	.cb_desc            = sdp_desc_cb,
	.cb_att_val         = sdp_att_val_cb,
	.cb_att_val_evt     = sdp_att_val_evt_cb,
	.cb_svc_changed     = NULL,
};

uint8_t sdp_common_create(uint8_t con_idx,uint16_t mtu)
{
	uint8_t ret;
	struct sdp_env_tag * p_env = sdp_get_free_env();
	BLE_ASSERT_ERR(p_env != NULL);

	p_env->con_idx = con_idx;
	p_env->user_lib = SDP_INVALID_USER_LID;
	p_env->mtu = mtu;
	ret = gatt_user_cli_register(p_env->mtu, 0, &sdp_cb, &p_env->user_lib);
	common_list_init(&p_env->svr_list);

	return ret;
}

uint8_t sdp_common_cleanup(uint8_t con_idx)
{
	uint16_t status = COMMON_BUF_ERR_NO_ERROR;

	struct sdp_env_tag * p_env = sdp_get_env_use_conidx(con_idx);

	if (p_env == NULL) {
		return COMMON_BUF_ERR_INVALID_PARAM;
	}

	while (!(common_list_is_empty(&p_env->svr_list))) {
		struct sdp_db *p_db = (struct sdp_db *)common_list_pop_front(&p_env->svr_list);
		if (p_db->svr.chars_nb) {
			bk_printf("[sdp_clean]chars:%p,chars_nb:%d\r\n",p_db->svr.chars,p_db->svr.chars_nb);
			kernel_free(p_db->svr.chars);
		}
		if (p_db->svr.descs_nb) {
			bk_printf("[sdp_clean]descs:%p,descs_nb:%d\r\n",p_db->svr.descs,p_db->svr.descs_nb);
			kernel_free(p_db->svr.descs);
		}
		bk_printf("[sdp_clean]sdp_db:%p\r\n",p_db);
		kernel_free(p_db);
	}
	status = gatt_user_unregister(p_env->user_lib);
	p_env->con_idx = SDP_INVALID_CONIDX;

	return status;
}

uint8_t sdp_discover_all_service(uint8_t con_idx)
{
	uint16_t status = COMMON_BUF_ERR_NO_ERROR;
	uint8_t conhdl = app_ble_get_connhdl(con_idx);

	struct sdp_env_tag *sdp_env = sdp_get_env_use_conidx(con_idx);

	if (sdp_env == NULL || (conhdl == UNKNOW_CONN_HDL) || (conhdl == USED_CONN_HDL)) {
		return COMMON_BUF_ERR_INVALID_PARAM;
	}

	if (status == GAP_ERR_NO_ERROR) {
		gatt_cli_discover_svc_cmd_t *p_cmd = KERNEL_MSG_ALLOC(GATT_CMD,TASK_BLE_GATT,
															TASK_BLE_SDP,
															gatt_cli_discover_svc_cmd);
		p_cmd->cmd_code = GATT_CLI_DISCOVER_SVC;
		p_cmd->dummy = BLE_APP_INITING_INDEX(conhdl);
		p_cmd->user_lid = sdp_env->user_lib;
		p_cmd->conidx = conhdl;
		p_cmd->disc_type = GATT_DISCOVER_SVC_PRIMARY_ALL;
		p_cmd->full = true;
		p_cmd->start_hdl = GATT_MIN_HDL;
		p_cmd->end_hdl = GATT_MAX_HDL;
		p_cmd->uuid_type = GATT_UUID_16;
		kernel_msg_send(p_cmd);
	} else {
		bk_printf("[sdp_discover_all_service]status:%d\r\n",status);
		return COMMON_BUF_ERR_INSUFFICIENT_RESOURCE;
	}
	return COMMON_BUF_ERR_NO_ERROR;
}

static int sdp_gatt_cmp_evt_handler(kernel_msg_id_t const msgid,
								struct gatt_proc_cmp_evt const *param,
								kernel_task_id_t const dest_id,
								kernel_task_id_t const src_id)
{
	bk_printf("[%s]cmd_code:%d,status:%d\r\n",__func__,param->cmd_code,param->status);
	return (KERNEL_MSG_CONSUMED);
}

static int sdp_default_msg_handler(kernel_msg_id_t const msgid,
								void const *param,
								kernel_task_id_t const dest_id,
								kernel_task_id_t const src_id)
{
	bk_printf("[%s]msgid:0x%x\r\n",__func__,msgid);
	return (KERNEL_MSG_CONSUMED);
}

KERNEL_MSG_HANDLER_TAB(sdp)
{
	{GATT_CMP_EVT,					(kernel_msg_func_t)sdp_gatt_cmp_evt_handler},
	{KERNEL_MSG_DEFAULT_HANDLER,	(kernel_msg_func_t)sdp_default_msg_handler},
};

uint8_t sdp_state;
const struct kernel_task_desc TASK_BLE_SDP_APP = {sdp_msg_handler_tab, &sdp_state, 1, ARRAY_LEN(sdp_msg_handler_tab)};

void sdp_common_init(void)
{
	uint8_t con_idx;

	for (con_idx = 0;con_idx < BLE_CONNECTION_MAX;con_idx++) {
		memset(&sdp_env[con_idx],0,sizeof(struct sdp_env_tag));
		sdp_env[con_idx].con_idx = SDP_INVALID_CONIDX;
		sdp_env[con_idx].user_lib = SDP_INVALID_USER_LID;
		sdp_env[con_idx].mtu = L2CAP_LE_MTU_MIN;
		common_list_init(&sdp_env[con_idx].svr_list);
	}

	kernel_task_create(TASK_BLE_SDP, &TASK_BLE_SDP_APP);
}

#endif

