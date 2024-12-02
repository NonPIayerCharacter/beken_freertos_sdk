#include "_atsvr_func.h"
#include "_at_server.h"
#include "atsvr_comm.h"
#include "string.h"
#include "stdio.h"
#include "mem_pub.h"
#include "BkDriverFlash.h"
#include "sys.h"
#include "manual_ps_pub.h"
#include "atsvr_misc.h"
#include "atsvr_port.h"
#include "uart_pub.h"
#include "at_server.h"
#include "ble_config.h"
#include "utils_httpc.h"
#include "utils_timer.h"
#include "rtos_pub.h"
#include <stdlib.h>
#include "wlan_ui_pub.h"
#include "atsvr_airkiss_cmd.h"

#include "app_ble.h"
#include "app_sdp.h"
#include "app_ble_init.h"
#if CFG_USE_DISTRIBUTION_NETWORK
extern beken_queue_t ble_queue;
#define BK_BLE_ATT_DECL_PRIMARY_SERVICE_128     {0x00,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_BLE_ATT_DECL_CHARACTERISTIC_128      {0x03,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_BLE_ATT_DESC_CLIENT_CHAR_CFG_128     {0x02,0x29,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

#define BLE_WRITE_REQ_CHARACTERISTIC_128        {0x01,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}
#define BLE_INDICATE_CHARACTERISTIC_128         {0x02,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}
#define BLE_NOTIFY_CHARACTERISTIC_128           {0x03,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}

static const uint8_t ble_test_svc_uuid[16] = {0xFF,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0};

enum
{
	BK_TEST_IDX_SVC,
	BK_TEST_IDX_FF01_VAL_CHAR,
	BK_TEST_IDX_FF01_VAL_VALUE,
	BK_TEST_IDX_FF02_VAL_CHAR,
	BK_TEST_IDX_FF02_VAL_VALUE,
	BK_TEST_IDX_FF02_VAL_IND_CFG,
	BK_TEST_IDX_FF03_VAL_CHAR,
	BK_TEST_IDX_FF03_VAL_VALUE,
	BK_TEST_IDX_FF03_VAL_NTF_CFG,
	BK_TEST_IDX_NB,
};

bk_attm_desc_t bk_test_att_db[BK_TEST_IDX_NB] =
{
	//  Service Declaration
	[BK_TEST_IDX_SVC]              = {BK_BLE_ATT_DECL_PRIMARY_SERVICE_128, PROP(RD), 0},

	//  Level Characteristic Declaration
	[BK_TEST_IDX_FF01_VAL_CHAR]    = {BK_BLE_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
	//  Level Characteristic Value
	[BK_TEST_IDX_FF01_VAL_VALUE]   = {BLE_WRITE_REQ_CHARACTERISTIC_128,    PROP(WR), 128|OPT(NO_OFFSET)},

	[BK_TEST_IDX_FF02_VAL_CHAR]    = {BK_BLE_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
	//  Level Characteristic Value
	[BK_TEST_IDX_FF02_VAL_VALUE]   = {BLE_INDICATE_CHARACTERISTIC_128,     PROP(I), 128|OPT(NO_OFFSET)},

	//  Level Characteristic - Client Characteristic Configuration Descriptor

	[BK_TEST_IDX_FF02_VAL_IND_CFG] = {BK_BLE_ATT_DESC_CLIENT_CHAR_CFG_128, PROP(RD)|PROP(WR),OPT(NO_OFFSET)},

	[BK_TEST_IDX_FF03_VAL_CHAR]    = {BK_BLE_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
	//  Level Characteristic Value
	[BK_TEST_IDX_FF03_VAL_VALUE]   = {BLE_NOTIFY_CHARACTERISTIC_128,       PROP(N), 128|OPT(NO_OFFSET)},

	//  Level Characteristic - Client Characteristic Configuration Descriptor

	[BK_TEST_IDX_FF03_VAL_NTF_CFG] = {BK_BLE_ATT_DESC_CLIENT_CHAR_CFG_128, PROP(RD)|PROP(WR), OPT(NO_OFFSET)},
};

ble_err_t bk_ble_db_create(void)
{
	ble_err_t status = ERR_SUCCESS;

	struct bk_ble_db_cfg ble_db_cfg;

	ble_db_cfg.att_db = bk_test_att_db;
	ble_db_cfg.att_db_nb = BK_TEST_IDX_NB;
	ble_db_cfg.prf_task_id = 0;
	ble_db_cfg.start_hdl = 0;
	ble_db_cfg.svc_perm = BK_PERM_SET(SVC_UUID_LEN, UUID_16);
	memcpy(&(ble_db_cfg.uuid[0]), &ble_test_svc_uuid[0], 16);

	status = bk_ble_create_db(&ble_db_cfg);
	ATSVRLOG("status=%d\r\n",status);
	return status;
}

void bk_ble_cmd_cb(ble_cmd_t cmd, ble_cmd_param_t *param)
{
	ATSVRLOG("cmd:%d idx:%d status:%d\r\n", cmd, param->cmd_idx, param->status);
}

uint8_t stop_actv_idx = 0;
uint8_t ble_adv_init(void)
{
	uint8_t actv_idx;
	uint8_t adv_data[31];
	actv_idx = app_ble_get_idle_actv_idx_handle();
	bk_ble_create_advertising(actv_idx, 7, 160, 160, bk_ble_cmd_cb);
	rtos_delay_milliseconds(100);

	#if( (CFG_SOC_NAME == SOC_BK7238))
	adv_data[0] = 0x0A;
	adv_data[1] = 0x09;
	memcpy(&adv_data[2], "7238_BLE", 9);
	bk_ble_set_adv_data(actv_idx, adv_data, 0xB, bk_ble_cmd_cb);
	rtos_delay_milliseconds(100);

	uint8_t adv_data_len;
	adv_data[0] = 0x06;
	adv_data[1] = 0x08;
	memcpy(&adv_data[2], "7238", 5);
	adv_data_len = 0x7;
	bk_ble_set_scan_rsp_data(actv_idx, adv_data, adv_data_len, bk_ble_cmd_cb);
	rtos_delay_milliseconds(100);
	#elif( (CFG_SOC_NAME == SOC_BK7231N))

	adv_data[0] = 0x02;
	adv_data[1] = 0x01;
	adv_data[2] = 0x06;
	adv_data[3] = 0x0B;
	adv_data[4] = 0x09;
	memcpy(&adv_data[5], "7231N_BLE", 10);
	bk_ble_set_adv_data(actv_idx, adv_data, 0xF, bk_ble_cmd_cb);
	rtos_delay_milliseconds(100);

	adv_data[0] = 0x07;
	adv_data[1] = 0x08;
	memcpy(&adv_data[2], "7231N", 6);
	bk_ble_set_scan_rsp_data(actv_idx, adv_data, 0x8, bk_ble_cmd_cb);
	rtos_delay_milliseconds(100);

	#endif

	stop_actv_idx = actv_idx;
	bk_ble_start_advertising(actv_idx, 0, bk_ble_cmd_cb);
	return actv_idx;
}

void ble_adv_deinit(uint8_t actv_idx)
{
	bk_ble_adv_stop(actv_idx, bk_ble_cmd_cb);
}


void ble_msg_cmd_push_to_queue(int type,char *data,int len)
{
	ble_data_t ble_data;
	OSStatus err;

	if(ble_queue==NULL)
		return;

	memset(&ble_data, 0x00, sizeof(ble_data));
	ble_data.type=type;
	if(data!=NULL && len>0)
	{
		ble_data.length = len;
		memcpy(ble_data.buffer, data, len);
	}
	ATSVRLOG("ble_data.buffer =%s\r\n",ble_data.buffer);
	err=rtos_push_to_queue(&ble_queue, &ble_data, BEKEN_NEVER_TIMEOUT);
	if(err != kNoErr)
	{
		ATSVRLOG("Received data from queue failed:Err = %d\r\n", err);
	}
}


void bk_ble_notice_cb(ble_notice_t notice, void *param)
{
	switch (notice) {
	case BLE_5_STACK_OK:
		bk_printf("ble stack ok");
		break;
	case BLE_5_WRITE_EVENT:
	{
		write_req_t *w_req = (write_req_t *)param;
		bk_printf("write_cb:conn_idx:%d, prf_id:%d, add_id:%d, len:%d, data[0]:%02x\r\n",
			w_req->conn_idx, w_req->prf_id, w_req->att_idx, w_req->len, w_req->value[0]);

		ble_msg_cmd_push_to_queue(TYPE_MSG_DATA,(char *)w_req->value,w_req->len);
		break;
	}
	case BLE_5_READ_EVENT:
	{
		read_req_t *r_req = (read_req_t *)param;
		bk_printf("read_cb:conn_idx:%d, prf_id:%d, add_id:%d\r\n",
			r_req->conn_idx, r_req->prf_id, r_req->att_idx);
		r_req->value[0] = 0x12;
		r_req->value[1] = 0x34;
		r_req->value[2] = 0x56;
		r_req->length = 3;
		break;
	}
	case BLE_5_REPORT_ADV:
	{
		recv_adv_t *r_ind = (recv_adv_t *)param;

		bk_printf("[%s]r_ind:actv_idx:%d,evt_type:%d adv_addr:%02x:%02x:%02x:%02x:%02x:%02x,rssi:%d\r\n",
			((r_ind->evt_type&0x7) == 3)?"scan-rsp":((r_ind->evt_type&0x7) == 1)?"adv":"unknow",
			r_ind->actv_idx,r_ind->evt_type, r_ind->adv_addr[0], r_ind->adv_addr[1], r_ind->adv_addr[2],
			r_ind->adv_addr[3], r_ind->adv_addr[4], r_ind->adv_addr[5],r_ind->rssi);
		break;
	}
	case BLE_5_MTU_CHANGE:
	{
		mtu_change_t *m_ind = (mtu_change_t *)param;
		bk_printf("m_ind:conn_idx:%d, mtu_size:%d\r\n", m_ind->conn_idx, m_ind->mtu_size);
		break;
	}
	case BLE_5_CONNECT_EVENT:
	{
		conn_ind_t *c_ind = (conn_ind_t *)param;
		bk_printf("c_ind:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
			c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
			c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);
		break;
	}
	case BLE_5_DISCONNECT_EVENT:
	{
		discon_ind_t *d_ind = (discon_ind_t *)param;
		bk_printf("d_ind:conn_idx:%d,reason:%d\r\n", d_ind->conn_idx,d_ind->reason);
		break;
	}
	case BLE_5_ATT_INFO_REQ:
	{
		att_info_req_t *a_ind = (att_info_req_t *)param;
		bk_printf("a_ind:conn_idx:%d\r\n", a_ind->conn_idx);
		a_ind->length = 128;
		a_ind->status = ERR_SUCCESS;
		break;
	}
	case BLE_5_CREATE_DB:
	{
		create_db_t *cd_ind = (create_db_t *)param;
		bk_printf("cd_ind:prf_id:%d, status:%d\r\n", cd_ind->prf_id, cd_ind->status);
		break;
	}
	#if (BLE_CENTRAL)
	case BLE_5_INIT_CONNECT_EVENT:
	{
		conn_ind_t *c_ind = (conn_ind_t *)param;
		#if (CFG_SOC_NAME == SOC_BK7238)
		app_ble_get_peer_feature(c_ind->conn_idx);
		app_ble_set_le_pkt_size(c_ind->conn_idx,LE_MAX_OCTETS);
		app_ble_mtu_exchange(c_ind->conn_idx);
		sdp_discover_all_service(c_ind->conn_idx);
		#endif
		bk_printf("BLE_5_INIT_CONNECT_EVENT:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
			c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
			c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);
		break;
	}
	case BLE_5_INIT_DISCONNECT_EVENT:
	{
		discon_ind_t *d_ind = (discon_ind_t *)param;
		#if (CFG_SOC_NAME == SOC_BK7238)
		sdp_common_cleanup(d_ind->conn_idx);
		#endif
		bk_printf("BLE_5_INIT_DISCONNECT_EVENT:conn_idx:%d,reason:0x%x\r\n", d_ind->conn_idx,d_ind->reason);
		break;
	}
	#endif
	case BLE_5_INIT_CONN_PARAM_UPDATE_REQ_EVENT:
	{
		conn_param_t *d_ind = (conn_param_t *)param;
		bk_printf("BLE_5_INIT_CONN_PARAM_UPDATE_REQ_EVENT:conn_idx:%d,intv_min:%d,intv_max:%d,time_out:%d\r\n",d_ind->conn_idx,
			d_ind->intv_min,d_ind->intv_max,d_ind->time_out);
		app_ble_send_conn_param_update_cfm(d_ind->conn_idx,true);
	}break;
	
	case BLE_5_INIT_CONN_PARAM_UPDATE_IND_EVENT:
	{
		conn_update_ind_t *d_ind = (conn_update_ind_t *)param;
		bk_printf("BLE_5_INIT_CONN_PARAM_UPDATE_IND_EVENT:conn_idx:%d,interval:%d,time_out:%d,latency\r\n",d_ind->conn_idx,
			d_ind->interval,d_ind->time_out,d_ind->latency);
		
	}
	break;
	case BLE_5_SDP_REGISTER_FAILED:
		bk_printf("BLE_5_SDP_REGISTER_FAILED\r\n");
		break;
	default:
		break;
	}
}
#endif

