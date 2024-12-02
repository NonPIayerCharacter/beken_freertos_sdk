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
#include "atsvr_airkiss_cmd.h"
#include "utils_httpc.h"
#include "utils_timer.h"
#include "rtos_pub.h"
#include <stdlib.h>
#include "wlan_ui_pub.h"
#include "ble_config.h"
#include "generic.h"
#include "stdlib.h"
#include "common.h"
#include "atsvr_wlan.h"
#include "rw_msg_pub.h"

#if CFG_USE_DISTRIBUTION_NETWORK

#define BLE_CONNECT_TIMEOUT				1*60*1000

extern int sti(char *str);
extern u32 airkiss_process(u8 start);
beken_queue_t ble_queue = NULL;
extern uint8_t stop_actv_idx;

extern beken_semaphore_t ak_semaphore;

char netconfig_type = 2;//配网类型，默认为2，airkiss配网，1是ble配网，3是ble+airkiss配网组合


static void wifi_rw_event_func(void *new_evt)
{
	rw_evt_type evt_type = *((rw_evt_type *)new_evt);

	os_printf("========evt_type:%d=====\r\n",evt_type);

	if(evt_type == RW_EVT_STA_CONNECTED)
	{//联网成功之后就退出接收模式
		ATSVR_SIZEOF_OUTPUT_STRRING(ATSVR_EVT_WLAN_CONNECTED);
		app_ble_env.actvs[stop_actv_idx].actv_status = ACTV_ADV_STARTED;
		stop_ble_config();
	}
	else if(evt_type == RW_EVT_STA_DISCONNECTED)
	{
		ATSVR_SIZEOF_OUTPUT_STRRING(ATSVR_EVT_WLAN_DISCONNECTED);
	}
	else if(evt_type == RW_EVT_STA_PASSWORD_WRONG)
	{
		ATSVR_SIZEOF_OUTPUT_STRRING(ATSVR_EVT_WLAN_PASSWORD_ERROR);
	}
	else if(evt_type == RW_EVT_STA_NO_AP_FOUND)
	{
		ATSVR_SIZEOF_OUTPUT_STRRING(ATSVR_EVT_WLAN_NO_AP);
	}
	else if(evt_type == RW_EVT_STA_GOT_IP)
	{
		ATSVR_SIZEOF_OUTPUT_STRRING(ATSVR_EVT_GOT_IP);
	}
}
void bk_store_ssid_toflash(char *ssid,char *pwd)
{
	char updateflag = 0;
	if(strcmp(g_env_param.stainfo.con_ssid, ssid)!=0){
		strcpy(g_env_param.stainfo.con_ssid,ssid);
		updateflag =1;
	}
	if(strcmp(g_env_param.stainfo.con_key, pwd)!=0){
		strcpy(g_env_param.stainfo.con_key,pwd);
		updateflag =1;
	}
	if(updateflag == 1)
	{
		write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param);
	}
}

static void ble_config_thread(beken_thread_function_t ble_config_run)
{
	OSStatus err;
	ble_data_t ble_data_received;
	char ssid[32];
	char password[64];
	uint8_t ssid_len=0,password_len=0;
	bool exit_flag = false;
	uint8_t actv_idx;

	err = rtos_init_queue(&ble_queue, "ble_queue", sizeof(ble_data_t), 3);
	if(err != kNoErr)
	{
		bk_printf("init queue failed err:%d!!!\r\n",err);
		return;
	}

	bk_wlan_status_register_cb((FUNC_1PARAM_PTR)wifi_rw_event_func);
	ble_set_notice_cb(bk_ble_notice_cb);
	bk_ble_db_create();
	actv_idx=ble_adv_init();
	*((bool*)ble_config_run) = true;

	while (1)
	{
		err = rtos_pop_from_queue(&ble_queue, &ble_data_received, BLE_CONNECT_TIMEOUT);
		if(err == kNoErr)
		{
			bk_printf("ble_data_received.type=%d\r\n",ble_data_received.type);
			switch(ble_data_received.type)
			{
				case MSG_EXIT_NET_CONFIG:
					exit_flag = true;
					break;
				case TYPE_MSG_DATA:
					switch (((uint8_t)ble_data_received.buffer[0] - '0'))
					{
						case E_DEV_MSG_SET_WIFI_INFO:
							// 2 byte ssid len + N bytes ssid + 2 byte pwd len + N bytes pwd
							ssid_len = ((uint8_t)ble_data_received.buffer[1] - '0')*10 + ((uint8_t)ble_data_received.buffer[2] - '0') ;
							password_len = ((uint8_t)ble_data_received.buffer[2+ssid_len+1] - '0')*10 + ((uint8_t)ble_data_received.buffer[2+ssid_len+2] - '0') ;
							ssid[ssid_len] = password[password_len] = '\0';
							strncpy((char*)ssid, &ble_data_received.buffer[3], ssid_len);
							strncpy((char*)password, &ble_data_received.buffer[2+ssid_len+3], password_len);

							ATSVRLOG("netconfig_type=%d\r\n",netconfig_type);
							if(netconfig_type == 3)
							{//如果是两种配网都开启，当收到蓝牙数据，则退出airkiss配网
								ATSVRLOG("quit\r\n");
								if(ak_semaphore)
								{
									rtos_set_semaphore(&ak_semaphore);
								}
								bk_wlan_stop_monitor();
								bk_wlan_register_monitor_cb(NULL);
								airkiss_process(0);
							}

							ATSVRLOG("ssid:%s\r\n", ssid);
							ATSVRLOG("password:%s\r\n",password);
							bk_store_ssid_toflash(ssid,password);
							wlan_start_station_connect(ssid,password);
						break;
					}
					break;
			}

		}

		if(exit_flag == true)
			break;

	}

	rtos_deinit_queue(&ble_queue);
	app_ble_reset();
	ble_adv_deinit(actv_idx);
	ble_set_notice_cb(NULL);

	app_ble_env.actvs[actv_idx].actv_status = ACTV_ADV_CREATED;

	*((bool*)ble_config_run) = false;
	bk_printf("[ble_config_thread] exit\r\n\n");

	rtos_delete_thread( NULL );
}

int start_ble_config(void)
{
	int ret = 0;
	static bool ble_config_run = false;
	ret = rtos_create_thread(NULL,
	THD_APPLICATION_PRIORITY,
	"ble_task",
	(beken_thread_function_t)ble_config_thread,
	4*1024,
	(beken_thread_arg_t)ble_config_run);
	if(ret != kNoErr)
	{
		bk_printf("create thread fail!!\r\n");
	}
	return ret;
}

int stop_ble_config(void)
{
	ble_msg_cmd_push_to_queue(MSG_EXIT_NET_CONFIG,NULL,0);
	return 0;
}

void _atsvr_startnetwork_handle(int argc, char **argv)
{
	if(argc < 3)
	{
		atsvr_cmd_rsp_error();
		return ;
	}
	char netmode = 0;

	netmode = atoi(argv[1]);
	netconfig_type = netmode;
	atsvr_cmd_rsp_ok();
	bk_wlan_stop(BK_STATION);
	switch(netmode)
	{
		case 1:			//ble
			start_ble_config();
		break;
		case 2:			//airkiss
			airkiss_process(1);
		break;
		case 3:			//airkiss+ble
			start_ble_config();
			airkiss_process(1);

		break;
		default:	//airkiss
			airkiss_process(1);
		break;
	}
}
void _atsvr_stopnetwork_handle(int argc, char **argv)
{
	if(argc > 1)
	{
		atsvr_cmd_rsp_error();
		return ;
	}
	atsvr_cmd_rsp_ok();

	bk_wlan_stop(BK_STATION);
	switch(netconfig_type)
	{
		case 1:			//quit ble
			stop_ble_config();
		break;
		case 2:			//quit airkiss
			airkiss_process(0);
		break;
		case 3:			//quit ble+airkiss
			stop_ble_config();
			airkiss_process(0);
		break;
		default:	//quit ble+airkiss
			stop_ble_config();
			airkiss_process(0);
		break;
	}
}

const struct _atsvr_command _atsvc_airkisscmds_table[] = {

	_ATSVR_CMD_HADLER("AT+CWSTARTSMART","AT+CWSTARTSMART=<type>,<auth floor>",_atsvr_startnetwork_handle),
	_ATSVR_CMD_HADLER("AT+CWSTOPSMART","AT+CWSTOPSMART",_atsvr_stopnetwork_handle),

};



void atsvr_airkiss_init()
{
	atsvr_register_commands(_atsvc_airkisscmds_table,sizeof(_atsvc_airkisscmds_table) / sizeof(struct _atsvr_command));
}
#endif
