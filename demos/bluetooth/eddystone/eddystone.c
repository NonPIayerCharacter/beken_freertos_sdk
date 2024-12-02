#include "include.h"
#include "string.h"
#include "error.h"
#include "rtos_pub.h"
#include "uart_pub.h"
#include "rtos_pub.h"
#if EDDYSTONE_DEMO
#include "eddystone.h"
#include "drv_model_pub.h"
#include "gatt.h"
#include "ble_api_5_x.h"
#include "app_ble.h"
#include "mem_pub.h"
#include "gatt.h"
#include "kernel_mem.h"
#include "eddystone_core.h"
#include "wlan_cli_pub.h"

static beken_queue_t eddystone_msg_que = NULL;

void eddystone_notice_cb(ble_notice_t notice, void *param)
{
	switch (notice) {
		case BLE_5_STACK_OK:
			bk_printf("ble stack ok");
			break;
		case BLE_5_WRITE_EVENT:
		{
			write_req_t *w_req = (write_req_t *)param;
			ble_eddystone_post_msg(EDDYSTONE_WRITE_EVENT, w_req, sizeof(*w_req));
			break;
		}
		case BLE_5_READ_EVENT:
		{
			bk_ble_read_event_private((read_req_t *)param);
			break;
		}
		case BLE_5_CONNECT_EVENT:
		{
			conn_ind_t *c_ind = (conn_ind_t *)param;
			ble_eddystone_post_msg(EDDYSTONE_CONNECT_EVENT,c_ind, sizeof(*c_ind));
			break;
		}
		case BLE_5_DISCONNECT_EVENT:
		{
			discon_ind_t *d_ind = (discon_ind_t *)param;
			ble_eddystone_post_msg(EDDYSTONE_DISCONNECT_EVENT, d_ind, sizeof(*d_ind));
			break;
		}
		case BLE_5_CREATE_DB:
		{
			create_db_t *cd_ind = (create_db_t *)param;
			bk_printf("cd_ind:prf_id:%d, status:%d\r\n", cd_ind->prf_id, cd_ind->status);
			break;
		}
		case BLE_5_INIT_CONN_PARAM_UPDATE_REQ_EVENT:
		{
			conn_param_t *p_ind = (conn_param_t *)param;
			ble_eddystone_post_msg(EDDYSTONE_INIT_CONN_PARAM_UPDATE_REQ_EVENT, p_ind, sizeof(*p_ind));
			break;
		}
		default:
			break;
		}
}

ble_err_t ble_eddystone_post_msg(uint16_t msg_id, void *data, uint32_t len)
{
	EDDYSTONE_MSG_T msg;
	if (eddystone_msg_que) {
		msg.msg_id = msg_id;
		msg.data = os_malloc(len);
		os_memcpy(msg.data, data, len);
		if (rtos_push_to_queue(&eddystone_msg_que, &msg, BEKEN_NO_WAIT) != 0) {
			bk_printf("%s cant enqueue\n", __func__);
			if (msg.data) {
				os_free(msg.data);
			}
			return BK_ERR_BLE_FAIL;
		}
	} else {
		bk_printf("%s queue is not ready\n", __func__);
		return BK_ERR_BLE_FAIL;
	}
	return BK_ERR_BLE_SUCCESS;
}

static void eddystone_app_event_handler(EDDYSTONE_MSG_T *msg)
{
	switch (msg->msg_id) {
		case EDDYSTONE_WRITE_EVENT:
			bk_ble_write_event_private(msg->data);
			break;
		case EDDYSTONE_CONNECT_EVENT:
			bk_ble_connect_event_private(msg->data);
			break;
		case EDDYSTONE_DISCONNECT_EVENT:
			bk_ble_disconnect_event_private(msg->data);
			break;
		case EDDYSTONE_INIT_CONN_PARAM_UPDATE_REQ_EVENT:
			bk_ble_init_conn_papam_update_data_req_event_private(msg->data);
			break;
		case EDDYSTONE_ADV_EVENT:
			bk_ble_adv_event_private(msg->data);
			break;
		default:
			break;
	};
}

void eddystone_thread(beken_thread_arg_t arg)
{
	os_printf( "\r\n\r\n eddystone testing!!!!!!!!!!\r\n\r\n" );
	ble_set_notice_cb(eddystone_notice_cb);
	bk_eddystone_init();
	eddystone_services_init();
	//adv 0:non connectable + non scannable
	struct adv_param adv_info_0;
	adv_info_0.channel_map = 7;
	adv_info_0.duration = 0;
	adv_info_0.prop = 0;
	adv_info_0.interval_min = 160;
	adv_info_0.interval_max = 160;
	adv_info_0.advData[0] = 0x03;
	adv_info_0.advData[1] = 0x03;
	adv_info_0.advData[2] = 0xaa;
	adv_info_0.advData[3] = 0xfe;
	adv_info_0.advData[4] = 0x11;//length
	adv_info_0.advData[5] = 0x16;
	adv_info_0.advData[6] = 0xaa;//(little_endline)
	adv_info_0.advData[7] = 0xfe;
	adv_info_0.advData[8] = 0x10;//URL
	memcpy(&adv_info_0.advData[9], eddystone_default_data, 13);
	adv_info_0.advDataLen = 22;
	bk_ble_adv_start_first(0, &adv_info_0, ble_eddystone_cmd_cb);
	rtos_delay_milliseconds(1000);//1s
	//adv 1:connectable + scannable
	struct adv_param adv_info_1;
	adv_info_1.channel_map = 7;
	adv_info_1.duration = 0;
	adv_info_1.prop = (1 << ADV_PROP_CONNECTABLE_POS) | (1 << ADV_PROP_SCANNABLE_POS);
	adv_info_1.interval_min = 160;
	adv_info_1.interval_max = 160;
	adv_info_1.advData[0] = 0x03;
	adv_info_1.advData[1] = 0x03;
	adv_info_1.advData[2] = 0xaa;
	adv_info_1.advData[3] = 0xfe;
	adv_info_1.advDataLen = 4;
	adv_info_1.respData[0] = 0x11;
	adv_info_1.respData[1] = 0x07;
	memcpy(&adv_info_1.respData[2], eddystone_uuid, 16);
	adv_info_1.respData[18] = 0x09;
	adv_info_1.respData[19] = 0x08;
	memcpy(&adv_info_1.respData[20], "nRF5x_E", 8);
	adv_info_1.respDataLen = 28;
	bk_ble_adv_start(1, &adv_info_1, ble_eddystone_cmd_cb);

	while(1)
	{
		bk_err_t err;
		EDDYSTONE_MSG_T msg;
		err = rtos_pop_from_queue(&eddystone_msg_que, &msg, BEKEN_WAIT_FOREVER);
		if (0 == err) {
			eddystone_app_event_handler( &msg );
			if (msg.data > 0) {
				os_free(msg.data);
				msg.data = NULL;
			}
		}
	}
	rtos_deinit_queue(&eddystone_msg_que);
	eddystone_msg_que = NULL;
	rtos_delete_thread(NULL);
}

int demo_start(void)
{
	OSStatus err = kNoErr;
	if (eddystone_msg_que == NULL) {
		err = rtos_init_queue(&eddystone_msg_que,
								"eddystone_msg_que",
								sizeof(EDDYSTONE_MSG_T),
								128);
		if (err != kNoErr) {
			os_printf("eddystone msg que failed\r\n");
			return err;
		}
	}
	/* Start eddystone application thread*/
	err = rtos_create_thread( NULL, BEKEN_APPLICATION_PRIORITY,
							"eddystone_thread",
							eddystone_thread,
							0x400,
							0);
	if (err != kNoErr) {
		rtos_deinit_queue(&eddystone_msg_que);
		eddystone_msg_que = NULL;
		os_printf("eddystone thread failed\r\n");
	}
	return err;
}
#endif // EDDYSTONE_DEMO
