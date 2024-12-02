#include "rwip_config.h"
#include <string.h>
#if (BLE_COMM_SERVER)
#include "comm.h"
#include "comm_task.h"
#include "prf_utils.h"
#include "prf.h"
#include "kernel_mem.h"
#include "app_ble.h"

void comm_event_sent(uint8_t conidx,uint8_t user_lid,uint16_t dummy,uint16_t status)
{
	if ((status == GAP_ERR_NO_ERROR) && ((dummy == GATT_INDICATE) || (dummy == GATT_NOTIFY))) {
		if (ble_event_notice) {
			ble_cmd_param_t cmd;

			cmd.cmd_idx = dummy;
			cmd.status = status;
			ble_event_notice(BLE_5_TX_DONE,&cmd);
		}
	} else {
		bk_printf("[%s]conidx:%d,user_lid:%d,dummy:%d,status:%d\r\n",__func__,conidx,user_lid,dummy,status);
	}
}

static void comm_cb_att_read_get(uint8_t conidx,uint8_t user_lid,uint16_t token,uint16_t hdl,uint16_t offset,
									uint16_t max_length)
{
	struct bk_ble_env_tag *ble_env = NULL;
	common_buf_t *p_buf = NULL;

	read_req_t read_req;
	uint16_t status = GAP_ERR_NO_ERROR;

	// retrieve handle information
	prf_data_t *prf_data = prf_data_get_by_prf_handler(hdl);

	if (prf_data) {
		// retrieve handle information
		ble_env = (struct bk_ble_env_tag*)(prf_data->p_env);

		read_req.conn_idx = app_ble_find_conn_idx_handle(conidx);
		read_req.value = kernel_malloc(BLE_CHAR_DATA_LEN, KERNEL_MEM_KERNEL_MSG);
		read_req.size = BLE_CHAR_DATA_LEN;
		read_req.att_idx = hdl - ble_env->start_hdl;
		read_req.prf_id = ble_env->id;

		if (ble_event_notice)
			ble_event_notice(BLE_5_READ_EVENT,&read_req);

		status = common_buf_alloc(&p_buf,GATT_BUFFER_HEADER_LEN,read_req.length,GATT_BUFFER_TAIL_LEN);

		if(status == GAP_ERR_NO_ERROR) {
			common_buf_copy_data_from_mem(p_buf,read_req.value,read_req.length);
		} else {
			status = ATT_ERR_INSUFF_RESOURCE;
			kernel_free(read_req.value);
		}
	} else {
		status = ATT_ERR_INVALID_HANDLE;
	}

	if (p_buf) {
		gatt_srv_att_read_get_cfm(conidx,user_lid,token,status,p_buf->data_len,p_buf);
		kernel_free(read_req.value);
		common_buf_release(p_buf);
	} else {
		bk_printf("[%s]status:%d\r\n",__func__,status);
	}
}

void comm_att_event_get(uint8_t conidx,uint8_t user_lid,uint16_t token,uint16_t dummy,uint16_t hdl,
						uint16_t max_length)
{
	bk_printf("[%s]\r\n",__func__);
}

static void comm_cb_att_info_get(uint8_t conidx,uint8_t user_lid,uint16_t token,uint16_t hdl)
{
	bk_printf("[%s]\r\n",__func__);
}

static void comm_cb_att_val_set(uint8_t conidx,uint8_t user_lid,uint16_t token,uint16_t hdl,uint16_t offset,
									common_buf_t *p_buf)
{
	struct bk_ble_env_tag *ble_env = NULL;
	prf_data_t *prf_data = NULL;

	// retrieve handle information
	prf_data = prf_data_get_by_prf_handler(hdl);

	if (prf_data) {
		ble_env = (struct bk_ble_env_tag*)(prf_data->p_env);
		write_req_t write_req;

		write_req.conn_idx = app_ble_find_conn_idx_handle(conidx);
		write_req.prf_id = ble_env->id;
		write_req.att_idx = hdl - ble_env->start_hdl;
		write_req.len = common_buf_data_len(p_buf);
		write_req.value = common_buf_data(p_buf);

		if (ble_event_notice)
			ble_event_notice(BLE_5_WRITE_EVENT,&write_req);
	}

	gatt_srv_att_val_set_cfm(conidx,user_lid,token,GAP_ERR_NO_ERROR);
}


/// Service callback hander
static const gatt_srv_cb_t comm_cb =
{
	.cb_event_sent    = comm_event_sent,
	.cb_att_read_get  = comm_cb_att_read_get,
	.cb_att_event_get = comm_att_event_get,
	.cb_att_info_get  = comm_cb_att_info_get,
	.cb_att_val_set   = comm_cb_att_val_set,
};

static uint16_t bk_ble_service_init(prf_data_t *p_env,uint16_t *p_start_hdl,uint8_t sec_lvl,uint8_t user_prio,
									struct bk_ble_db_cfg *p_params, const void *p_cb)
{
	uint8_t user_lid = GATT_INVALID_USER_LID;
	uint8_t status = GAP_ERR_NO_ERROR;
	uint16_t shdl;

	do {
		struct bk_ble_env_tag *ble_env = NULL;

		if (p_cb == NULL) {
			bk_printf("[%s]p_cb null\r\n",__func__);
		}

		// Service content flag
		uint16_t cfg_flag = 0xFFFF;

		shdl = *p_start_hdl;

		 // register DISS user
		status = gatt_user_srv_register(256,user_prio,&comm_cb,&user_lid);
		if (status != GAP_ERR_NO_ERROR) {
			break;
		}

		//Create FFF0 in the DB
		//------------------ create the attribute database for the profile -------------------
		status = gatt_db_svc_add(user_lid, sec_lvl,(uint8_t *)(p_params->uuid),p_params->att_db_nb,
								(uint8_t *)&cfg_flag,(gatt_att_desc_t *)(p_params->att_db),p_params->att_db_nb, &shdl);
		if (status != GAP_ERR_NO_ERROR) {
			break;
		}
		//-------------------- allocate memory required for the profile  ---------------------
		ble_env = (struct bk_ble_env_tag*)kernel_malloc(sizeof(struct bk_ble_env_tag),KERNEL_MEM_ATT_DB);

		if (ble_env != NULL) {
			memset(ble_env, 0 , sizeof(struct bk_ble_env_tag));

			// allocate BASS required environment variable
			p_env->p_env = (prf_hdr_t *) ble_env;
			*p_start_hdl = shdl;
			ble_env->start_hdl = *p_start_hdl;
			ble_env->user_lid  = user_lid;
			ble_env->id = p_params->prf_task_id;
			ble_env->att_db_nb = p_params->att_db_nb;

			// initialize environment variable
			p_env->api_id = p_params->prf_task_id;
			comm_task_init(&(p_env->desc), ble_env->state);

			bk_printf("ble_env->start_hdl = 0x%x,prf_task:%d\r\n",ble_env->start_hdl,p_env->prf_task);
		} else {
			status = GAP_ERR_INSUFF_RESOURCES;
		}
	}while(0);

	if ((status != GAP_ERR_NO_ERROR) && (user_lid != GATT_INVALID_USER_LID)) {
		gatt_user_unregister(user_lid);
	}

	return (status);
}

static uint16_t bk_ble_service_destroy(prf_data_t *p_env,uint8_t reason)
{
	struct bk_ble_env_tag *ble_env = (struct bk_ble_env_tag*)p_env->p_env;

	// clear on-going operation
	if (ble_env->operation != NULL) {
		kernel_free(ble_env->operation);
	}

	// free profile environment variables
	p_env->p_env = NULL;
	kernel_free(ble_env);

	return 0;
}

static void bk_ble_service_create(prf_data_t* p_env,uint8_t conidx,const gap_con_param_t* p_con_param)
{
	struct bk_ble_env_tag *ble_env = (struct bk_ble_env_tag*)p_env->p_env;

	BLE_ASSERT_ERR(conidx < BLE_CONNECTION_MAX);
	// force notification config to zero when peer device is connected
	ble_env->ntf_cfg[conidx] = 0;
	ble_env->ind_cfg[conidx] = 0;
}


static void bk_ble_service_cleanup(prf_data_t* p_env, uint8_t conidx, uint16_t reason)
{
	struct bk_ble_env_tag *ble_env = (struct bk_ble_env_tag*)p_env->p_env;

	BLE_ASSERT_ERR(conidx < BLE_CONNECTION_MAX);
	// force notification config to zero when peer device is disconnected
	ble_env->ntf_cfg[conidx] = 0;
	ble_env->ind_cfg[conidx] = 0;
}


static void bk_ble_service_con_upd(prf_data_t* p_env,uint8_t conidx,const gap_con_param_t* p_con_param)
{
	bk_printf("[%s]con_interval:%d,con_latency:%d,sup_to:%d\r\n",__func__,p_con_param->con_interval,p_con_param->con_latency,p_con_param->sup_to);
}

///  Task interface required by profile manager
const prf_task_cbs_t bk_ble_itf =
{
	(prf_init_cb) bk_ble_service_init,
	bk_ble_service_destroy,
	bk_ble_service_create,
	bk_ble_service_cleanup,
	bk_ble_service_con_upd,
};

const prf_task_cbs_t* bk_ble_prf_itf_get(void)
{
	return &bk_ble_itf;
}

#endif

