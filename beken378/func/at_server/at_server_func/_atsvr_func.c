#include "_atsvr_func.h"
#include "_at_server.h"
#include "_at_server_port.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "at_server.h"
#include "BkDriverFlash.h"
#include "sys.h"
#include "manual_ps_pub.h"
#include "atsvr_misc.h"
#include "atsvr_port.h"
#include "uart_pub.h"
#include "atsvr_wlan.h"
#include "sys_ctrl_pub.h"

#include <stdlib.h>
#include "param_config.h"

#include "common.h"
#include "cmd_evm.h"
#if CFG_USE_DEFUALT_CMD
static void _atsvr_at_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
	_at_svr_ctrl_env_t *p_env = NULL;

#if ATSVR_HANDLER_ENV
	p_env = env;
#else
	if(argc < ATSVR_MAX_ARG) {
		p_env = (_at_svr_ctrl_env_t*)argv[argc];
	}
#endif
	if(argc != 1){
		_atsvr_cmd_rsp_error(p_env);
		return;
	}
	_atsvr_cmd_rsp_ok(p_env);
}

//extern int log_enable();
static void _atsvr_at_close_echo_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
	_at_svr_ctrl_env_t *p_env = NULL;

#if ATSVR_HANDLER_ENV
	p_env = env;
#else
	if(argc < ATSVR_MAX_ARG) {
		p_env = (_at_svr_ctrl_env_t*)argv[argc];
	}
#endif
	if(argc != 1){
		_atsvr_cmd_rsp_error(p_env);
		return;
	}
	log_output_state(FALSE);
	_set_atsvr_echo_mode(p_env,_ATSVR_ECHO_NONE);
	_atsvr_cmd_rsp_ok(p_env);
}

static void _atsvr_at_open_echo_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
	_at_svr_ctrl_env_t *p_env = NULL;

#if ATSVR_HANDLER_ENV
	p_env = env;
#else
	if(argc < ATSVR_MAX_ARG) {
		p_env = (_at_svr_ctrl_env_t*)argv[argc];
	}
#endif
	if(argc != 1){
		_atsvr_cmd_rsp_error(p_env);
		return;
	}
	log_output_state(TRUE);
	_set_atsvr_echo_mode(p_env,_ATSVR_ECHO_ALL);
	_atsvr_cmd_rsp_ok(p_env);
}

int user_restore_func(void)
{
	g_env_param.sysstore = 1;
	g_env_param.workmode = 0;
	g_env_param.wifimode.autoconnect=1;
	g_env_param.wifimode.mode = 1;
	strcpy(g_env_param.apinfo.ap_ssid,ATSVR_WLAN_AP_SSID);
	strcpy(g_env_param.apinfo.ap_key,ATSVR_WLAN_AP_KEY);
	g_env_param.apinfo.channel=1;
	g_env_param.apinfo.ap_enc=8;
	g_env_param.apinfo.hidden=0;
	g_env_param.apinfo.proto=0;
	strcpy(g_env_param.apinfo.ap_local_ip,ATSVR_WLAN_DEFAULT_IP);
	strcpy(g_env_param.apinfo.ap_mask,ATSVR_WLAN_DEFAULT_MASK);
	strcpy(g_env_param.apinfo.ap_gate,ATSVR_WLAN_DEFAULT_GW);
	//strcpy(g_env_param.apinfo.ap_dns,ATSVR_WLAN_DEFAULT_DNS);
	g_env_param.apinfo.dhcp=1;

	strcpy(g_env_param.stainfo.con_ssid,ATSVR_WLAN_STA_SSID);
	strcpy(g_env_param.stainfo.con_key,ATSVR_WLAN_STA_KEY);
	g_env_param.stainfo.dhcp = 1;
	strcpy(g_env_param.stainfo.sta_local_ip,ATSVR_WLAN_STA_DEFAULT_IP);
	strcpy(g_env_param.stainfo.sta_mask,ATSVR_WLAN_STA_DEFAULT_MASK);
	strcpy(g_env_param.stainfo.sta_gate,ATSVR_WLAN_STA_DEFAULT_GW);
	//strcpy(g_env_param.stainfo.sta_dns1,ATSVR_WLAN_STA_DEFAULT_DNS);
	//strcpy(g_env_param.stainfo.sta_dns2,ATSVR_WLAN_STA_DEFAULT_DNS2);
	g_env_param.uartinfo.baudrate=115200;
	g_env_param.uartinfo.databits=BK_DATA_WIDTH_8BIT;
	g_env_param.uartinfo.stopbits=BK_STOP_BITS_1;
	g_env_param.uartinfo.parity=0;
	g_env_param.uartinfo.flow_control = 0;

	g_env_param.net_transmission_mode = 0;
	strcpy(g_env_param.deviceinfo.product_id,ATSVR_DEVINFO_PRODUCTID);
	strcpy(g_env_param.deviceinfo.device_name,ATSVR_DEVINFO_DEVNAME);
	//strcpy(g_env_param.deviceinfo.psk_secret,"2fe2b750a610dab52fa24374c0cb90a2e52b4c02f947107f83f4c11d81f74681");
	strcpy(g_env_param.deviceinfo.region,"china");
	g_env_param.ntpinfo.enable=1;
	strcpy(g_env_param.ntpinfo.hostname,"cn.pool.ntp.org");
	g_env_param.ntpinfo.timezone=8;

	g_env_param.dnsinfo.enable=1;
	strcpy(g_env_param.dnsinfo.dns1,ATSVR_WLAN_DEFAULT_DNS1);
	strcpy(g_env_param.dnsinfo.dns2,ATSVR_WLAN_DEFAULT_DNS2);
	strcpy(g_env_param.dnsinfo.dns3,ATSVR_WLAN_DEFAULT_DNS3);
	g_env_param.ntpinfo.timezone=8;

	g_env_param.pskinfo.linkid=5;
	strcpy(g_env_param.pskinfo.psk,ATSVR_PSKINFO_PSK);
	strcpy(g_env_param.pskinfo.hint,ATSVR_PSKINFO_HINT);

	strcpy(g_env_param.ble_param.ble_device_name,"BK_BLE");
	g_env_param.ble_param.adv_type = 0;
	g_env_param.ble_param.interval_min = 160;
	g_env_param.ble_param.interval_max  = 160;
	g_env_param.ble_param.channel_map = 7;

	g_env_param.ble_param.uuid_len = 2;
	memset(g_env_param.ble_param.uuid,0,sizeof(g_env_param.ble_param.uuid));
	g_env_param.ble_param.uuid[0]  = 0xff;
	g_env_param.ble_param.uuid[1]  = 0xee;
	g_env_param.ble_param.manufacturer_data_len = 2;
	memset(g_env_param.ble_param.manufacturer_data,0,sizeof(g_env_param.ble_param.manufacturer_data));
	g_env_param.ble_param.manufacturer_data[0] = 0x11;
	g_env_param.ble_param.manufacturer_data[1] = 0x22;
	g_env_param.ble_param.include_power = 1;

	g_env_param.ble_param.advDataLen = 10;
	memset(g_env_param.ble_param.advData,0,sizeof(g_env_param.ble_param.advData));
	g_env_param.ble_param.advData[0] = 0x09;
	g_env_param.ble_param.advData[1] = 0x09;
	g_env_param.ble_param.advData[2] = 0x37;
	g_env_param.ble_param.advData[3] = 0x32;
	g_env_param.ble_param.advData[4] = 0x3;
	g_env_param.ble_param.advData[5] = 0x38;
	g_env_param.ble_param.advData[6] = 0x5F;
	g_env_param.ble_param.advData[7] = 0x42;
	g_env_param.ble_param.advData[8] = 0x4C;
	g_env_param.ble_param.advData[9] = 0x45;
	g_env_param.ble_param.respDataLen = 6;
	memset(g_env_param.ble_param.respData,0,sizeof(g_env_param.ble_param.respData));
	g_env_param.ble_param.respData[0] = 0x05;
	g_env_param.ble_param.respData[1] = 0x08;
	g_env_param.ble_param.respData[2] = 0x37;
	g_env_param.ble_param.respData[3] = 0x32;
	g_env_param.ble_param.respData[4] = 0x33;
	g_env_param.ble_param.respData[5] = 0x38;

	g_env_param.ble_param.scan_type = 0;

	g_env_param.ble_param.own_addr_type = 0;
	g_env_param.ble_param.filter_policy = 0;
	g_env_param.ble_param.scan_intvl = 100;
	g_env_param.ble_param.scan_wd = 50;
	g_env_param.ble_param.con_interval = 87;

	g_env_param.ble_param.con_latency = 0;
	g_env_param.ble_param.sup_to = 500;

	if(write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param)==0)
	{
		return 1;
	}
	return 0;
}

static void _atsvr_restore_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
	_at_svr_ctrl_env_t *p_env = NULL;

#if ATSVR_HANDLER_ENV
	p_env = env;
#else
	if(argc < ATSVR_MAX_ARG) {
		p_env = (_at_svr_ctrl_env_t*)argv[argc];
	}
#endif
	if(argc != 1){
		_atsvr_cmd_rsp_error(p_env);
		return;
	}
	if(user_restore_func())
	{
		rtos_delay_milliseconds(10);
		_atsvr_cmd_rsp_ok(p_env);
		reboot_the_system();
	return ;
	}
	_atsvr_cmd_rsp_error(p_env);
	return;
}

static void _atsvr_at_USERRAM_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
	_at_svr_ctrl_env_t *p_env = NULL;
	unsigned int ramspace=0;
#if ATSVR_HANDLER_ENV
	p_env = env;
#else
	if(argc < ATSVR_MAX_ARG) {
		p_env = (_at_svr_ctrl_env_t*)argv[argc];
	}
#endif
	if(argc != 1){
		_atsvr_cmd_rsp_error(p_env);
	return;
}

#if CFG_OS_FREERTOS
	//cmd_printf("free memory %d\r\n", xPortGetFreeHeapSize());
	ramspace = xPortGetFreeHeapSize();
#elif CFG_SUPPORT_LITEOS
	LOS_MEM_POOL_STATUS status;
	(void)LOS_MemInfoGet(m_aucSysMem0, &status);
	ramspace = status.totalFreeSize;
#else
#endif
	char output[100];
	int n;

	n = snprintf(output,100,"+USERRAM:%d\r\n",ramspace);
	n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
	_atsvr_output_msg(p_env,output,n);
	return;
}

static void  _atsvr_at_SYSFLALSH_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{

	_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif
		if(argc != 1){
			_atsvr_cmd_rsp_error(p_env);
			return;
			}

		//_atsvr_output_msg(p_env,"enter SYSFLALSH\r\n",strlen("enter SYSFLALSH\r\n"));
		bk_partition_t i;
		bk_logic_partition_t *partition;
		char output[500]={0};
		int n=0;

		for( i = BK_PARTITION_BOOTLOADER; i <= BK_PARTITION_MAX; i++ )
		{
			partition = bk_flash_get_info( i );
			if (partition == NULL)
			{

				continue;
			}

		n+= snprintf(output+n,500-n,"%4d | %11s |  Dev:%d  | 0x%08lx | 0x%08lx |\r\n", i,
					partition->partition_description, partition->partition_owner,
					partition->partition_start_addr, partition->partition_length);
		}

		if(n>0)
		{
		n += snprintf(output+n,500 - n,ATSVR_CMD_RSP_SUCCEED);
		_atsvr_output_msg(p_env,output,n);
		}
		else
		{
		_atsvr_cmd_rsp_error(p_env);
		}
		return;
}

extern long  user_get_ntp_time();
extern char ntp_time_need_updata_get_fg();
extern void ntp_time_need_updata_set_fg(char fg);

static void  _atsvr_at_SYSTIMESTAMP_QUERY_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{

		_at_svr_ctrl_env_t *p_env = NULL;
		beken_time_t  timestamp = 0;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif
		if(argc == 1){

			// unsigned int timestamp;
			beken_time_t timestamp;

			if(ntp_time_need_updata_get_fg() == 1)
			{
				ntp_time_need_updata_set_fg(0);
				timestamp = user_get_ntp_time();
				beken_time_set_time_s(&timestamp);
			}
			else
			{
				beken_time_get_time_s(&timestamp);
			}
			char output[100];
			int n;

			n = snprintf(output,100,"+SYSTIMESTAMP:%d\r\n",timestamp);
			n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
			_atsvr_output_msg(p_env,output,n);

		}
		else{

		timestamp = strtol(argv[1],NULL,10);
		beken_time_set_time_s(&timestamp);
		_atsvr_cmd_rsp_ok(p_env);

		}

		return;
}

static void  _atsvr_at_SYSTIMESTAMP_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{

		_at_svr_ctrl_env_t *p_env = NULL;
		beken_time_t  timestamp = 0;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif
		if(argc == 1){

			// unsigned int timestamp;
			beken_time_t timestamp;

			if(ntp_time_need_updata_get_fg() == 1)
			{
				ntp_time_need_updata_set_fg(0);
				timestamp = user_get_ntp_time();
				beken_time_set_time_s(&timestamp);
			}
			else
			{
				beken_time_get_time_s(&timestamp);
			}

			char output[100];
			int n;

			n = snprintf(output,100,"+SYSTIMESTAMP:%d\r\n",timestamp);
			n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
			_atsvr_output_msg(p_env,output,n);

		}
		else{

		timestamp = strtol(argv[1],NULL,10);
		beken_time_set_time_s(&timestamp);
		_atsvr_cmd_rsp_ok(p_env);

		}

		return;
}

static void  _atsvr_at_SLEEPPWCFG_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{

		_at_svr_ctrl_env_t *p_env = NULL;
		PS_DEEP_CTRL_PARAM deep_sleep_param;
		int waketype  = 0;
		int param1 =0;
		int param2= 0;

#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif

		if (argc <3)
		{
			_atsvr_cmd_rsp_error(p_env);
			return;
		}
		else if(argc ==3)
		{

		ATSVRLOG("3 1\r\n");
		waketype = strtol(argv[1],NULL,10);
		param1 = strtol(argv[2],NULL,10);


			// compatible  ESP
			if(waketype ==0) {

				waketype = PS_DEEP_WAKEUP_RTC;   //time wakeup
			}
			else if(waketype ==2) {

				waketype = PS_DEEP_WAKEUP_GPIO; //gpio wakeup
			}
			else{

				_atsvr_cmd_rsp_error(p_env);
				return;
			}

			_atsvr_cmd_rsp_ok(p_env);


			deep_sleep_param.sleep_time 			= param1;
			deep_sleep_param.wake_up_way			= waketype;
			deep_sleep_param.gpio_index_map      	= 0;
			deep_sleep_param.gpio_edge_map       	= 0;
			deep_sleep_param.gpio_last_index_map 	= 0;
			deep_sleep_param.gpio_last_edge_map  	= 0;

			#if (CFG_SOC_NAME != SOC_BK7271)
			bk_enter_deep_sleep_mode(&deep_sleep_param);
			#endif
			return;

		}
		else if(argc == 4)
		{
			UINT32 gpio_index_map =0;
			UINT32 gpio_edge_map =0;
			waketype = strtol(argv[1],NULL,10);
			// compatible  ESP
			if(waketype !=2) {

				_atsvr_cmd_rsp_error(p_env);
				return;
			}
			if(waketype ==2) {

				waketype = PS_DEEP_WAKEUP_GPIO; //gpio wakeup
			}

		param1 = strtol(argv[2],NULL,10);
		param2 = strtol(argv[3],NULL,10);
		gpio_index_map = (1 << param1);

		if(param2 == 0)
		{
				gpio_edge_map = param2;
		}
		else
			{
				gpio_edge_map = (1 << param1);
			}
			if(param1 >=0 && param1 <=28)
			{
				deep_sleep_param.sleep_time             = 0;
				deep_sleep_param.wake_up_way     		= waketype;
				deep_sleep_param.gpio_index_map      	= gpio_index_map;
				deep_sleep_param.gpio_edge_map       	= gpio_edge_map;
				deep_sleep_param.gpio_last_index_map 	= 0;
				deep_sleep_param.gpio_last_edge_map  	= 0;

					_atsvr_cmd_rsp_ok(p_env);

				#if (CFG_SOC_NAME != SOC_BK7271)
					bk_enter_deep_sleep_mode(&deep_sleep_param);
				#endif
			}

			return;
		}
		else
		{
			_atsvr_cmd_rsp_error(p_env);
		}

		return;
}

static void _atsvr_at_SYSSTORE_Query_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{

		_at_svr_ctrl_env_t *p_env = NULL;
		//unsigned int	result = 0;
		uint8 result = 0;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}

#endif

	read_env_from_flash(TAG_SYSSTORE_OFFSET,LEN_SYSSTORE_VALUE,(uint8*)&result);
	char output[100];
	int n;
	ATSVRLOG("[%s] read  SYSSTORE :%d\r\n",__FUNCTION__,result);
	n = snprintf(output,100,"+SYSSTORE:%d\r\n",result);
	n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
	_atsvr_output_msg(p_env,output,n);
	return;
}

static void  _atsvr_at_SYSSTORE_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
		_at_svr_ctrl_env_t *p_env = NULL;
		//unsigned int  result = 0;
		uint8 result = 0;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif
		if(argc == 1){

			read_env_from_flash(TAG_SYSSTORE_OFFSET,LEN_SYSSTORE_VALUE,(uint8*)&result);
			char output[100];
			int n;
			ATSVRLOG("[%s] read  SYSSTORE :%d\r\n",__FUNCTION__,result);
			n = snprintf(output,100,"+SYSSTORE:%d\r\n",result);
			n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
			_atsvr_output_msg(p_env,output,n);
			return;

		}
		else if(argc ==2){

			result =(uint8) strtol(argv[1],NULL,16);
			if(result >1){
				_atsvr_cmd_rsp_error(p_env);
				return;
			}
			g_env_param.sysstore = result;
			if(write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param)==0)
			{
				rtos_delay_milliseconds(10);
				_atsvr_cmd_rsp_ok(p_env);
			return ;
			}
		}
		else
		{
		_atsvr_cmd_rsp_error(p_env);
		}

		return;
}

static void _atsvr_at_UART_DEF_Query_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
			_at_svr_ctrl_env_t *p_env = NULL;

#if ATSVR_HANDLER_ENV
			p_env = env;
#else
			if(argc < ATSVR_MAX_ARG) {
				p_env = (_at_svr_ctrl_env_t*)argv[argc];
			}
#endif

	char output[200];
	int n;
	uint8 uart_databits = 0;
	uint8 uart_stopbits = 0;
	switch(g_env_param.uartinfo.databits)
	{
		case BK_DATA_WIDTH_5BIT:
			uart_databits = 5;
			break;
		case BK_DATA_WIDTH_6BIT:
			uart_databits = 6;
			break;
		case BK_DATA_WIDTH_7BIT:
			uart_databits = 7;
			break;
		case BK_DATA_WIDTH_8BIT:
			uart_databits = 8;
			break;
		default:
			uart_databits = 8;
	}
	switch(g_env_param.uartinfo.stopbits)
	{
		case BK_STOP_BITS_1:
			uart_stopbits = 1;
			break;
		case BK_STOP_BITS_2:
			uart_stopbits = 2;
			break;
		default:
			uart_stopbits = 1;
	}

	n = snprintf(output,200,"+UART_DEF:%d,%d,%d,%d,%d\r\n",g_env_param.uartinfo.baudrate,uart_databits,uart_stopbits,g_env_param.uartinfo.parity,g_env_param.uartinfo.flow_control);
	n += snprintf(output+n,200 - n,ATSVR_CMD_RSP_SUCCEED);
	_atsvr_output_msg(p_env,output,n);
}

static void  _atsvr_at_UART_CUR_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
		_at_svr_ctrl_env_t *p_env = NULL;
		bk_uart_config_t uart_cfg;
		int datawidth = 0;
		int stopbit = 0;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif
		if(argc == 6){
			uart_cfg.baud_rate=atoi(argv[1]);
			datawidth =atoi(argv[2]);
			stopbit=atoi(argv[3]);
			uart_cfg.parity=atoi(argv[4]);
			if(uart_cfg.parity >2){
				_atsvr_cmd_rsp_error(p_env);
				return;

				}
			uart_cfg.flow_control=atoi(argv[5]);
			if(uart_cfg.flow_control>3){
				_atsvr_cmd_rsp_error(p_env);
					return;
			}
			uart_cfg.flags = 0;

			switch(datawidth)
			{
				case 5:
				uart_cfg.data_width = BK_DATA_WIDTH_5BIT;
					break;
				case 6:
				uart_cfg.data_width = BK_DATA_WIDTH_6BIT;
					break;
				case 7:
				uart_cfg.data_width = BK_DATA_WIDTH_7BIT;
					break;
				case 8:
				uart_cfg.data_width = BK_DATA_WIDTH_8BIT;
					break;
				default:
				//uart_cfg.data_width = BK_DATA_WIDTH_8BIT;
				_atsvr_cmd_rsp_error(p_env);
				return;
			}

			switch(stopbit)
			{
				case 1:
				uart_cfg.stop_bits = BK_STOP_BITS_1;
					break;
				case 2:
				uart_cfg.stop_bits = BK_STOP_BITS_2;
					break;
				default:
				//uart_cfg.stop_bits = BK_STOP_BITS_1;
				_atsvr_cmd_rsp_error(p_env);
				return;
			}

			ATSVRLOG("baud_rate:%d,data_width:%d,stop_bits:%d,parity:%d,flow_control:%d\r\n",uart_cfg.baud_rate,uart_cfg.data_width,uart_cfg.stop_bits,uart_cfg.parity,uart_cfg.flow_control);
			bk_uart_initialize(AT_UART_PORT_CFG,&uart_cfg,NULL);
			_atsvr_cmd_rsp_ok(p_env);
		}
		else
		{
			_atsvr_cmd_rsp_error(p_env);
		}

		return;
}

static void  _atsvr_at_UART_DEF_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
		_at_svr_ctrl_env_t *p_env = NULL;
		bk_uart_config_t uart_cfg;
		int datawidth = 0;
		int stopbit = 0;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif
		if(argc == 6){
			uart_cfg.baud_rate=atoi(argv[1]);
			datawidth =atoi(argv[2]);
			stopbit=atoi(argv[3]);
			uart_cfg.parity=atoi(argv[4]);
			uart_cfg.flow_control=atoi(argv[5]);
			uart_cfg.flags = 0;

			switch(datawidth)
			{
				case 5:
				uart_cfg.data_width = BK_DATA_WIDTH_5BIT;
					break;
				case 6:
				uart_cfg.data_width = BK_DATA_WIDTH_6BIT;
					break;
				case 7:
				uart_cfg.data_width = BK_DATA_WIDTH_7BIT;
					break;
				case 8:
				uart_cfg.data_width = BK_DATA_WIDTH_8BIT;
					break;
				default:
				uart_cfg.data_width = BK_DATA_WIDTH_8BIT;
			}

			switch(stopbit)
			{
				case 1:
				uart_cfg.stop_bits = BK_STOP_BITS_1;
					break;
				case 2:
				uart_cfg.stop_bits = BK_STOP_BITS_2;
					break;
				default:
				uart_cfg.stop_bits = BK_STOP_BITS_1;
			}

			ATSVRLOG("baud_rate:%d,data_width:%d,stop_bits:%d,parity:%d,flow_control:%d\r\n",uart_cfg.baud_rate,uart_cfg.data_width,uart_cfg.stop_bits,uart_cfg.parity,uart_cfg.flow_control);
			bk_uart_initialize(AT_UART_PORT_CFG,&uart_cfg,NULL);
			memcpy(&g_env_param.uartinfo,&uart_cfg,sizeof(uart_cfg));
			if(write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param)==0)
			{
			_atsvr_cmd_rsp_ok(p_env);
			return ;
			}
		}

		_atsvr_cmd_rsp_error(p_env);
		return;
}

#if ATSVR_CMD_HELP
static void _atsvr_help_command(
#if ATSVR_HANDLER_ENV
	void* env,
#endif
	int argc, char **argv);
#endif

static void _atsvr_at_verion(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
	_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
	p_env = env;
#else
	if(argc < ATSVR_MAX_ARG) {
		p_env = (_at_svr_ctrl_env_t*)argv[argc];
	}
#endif
	if(argc != 1){
		_atsvr_cmd_rsp_error(p_env);
		return;
	}
	char output[300];
	int n;

	n = snprintf(output,300,"<ATVER:%s>\r\n",AT_VERSION);
	n += snprintf(output+n,300-n,"<SDKVER:%s>\r\n",BEKEN_SDK_REV);
	#if buildtime
	n += snprintf(output+n,300-n,"<buildtime:%s\r\n>",buildtime);
	#endif
	n += snprintf(output+n,300 - n,ATSVR_CMD_RSP_SUCCEED);
	_atsvr_output_msg(p_env,output,n);

	return;
}

static void atsvr_reset_system(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)

{
		_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif

	if(argc != 1){
		_atsvr_cmd_rsp_error(p_env);
		return;
	}
	read_env_from_flash(TAG_SYSSTORE_OFFSET,sizeof(ENV_PARAM)+1,(uint8*)&g_env_param);
	int i = 0;
	for(i=0;i<16;i++)
	{
		ATSVRLOG("%d \r\n",((uint8*)&g_env_param)[i]);

	}
	_atsvr_cmd_rsp_ok(p_env);
	reboot_the_system();
	return;
}

static void atsvr_GSLP_system(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)

{
		_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif

	if(argc < 2){
		_atsvr_cmd_rsp_error(p_env);
		return;
	}

	if(argc == 2){
		unsigned int sleeptime = atoi(argv[1]);
		if(sleeptime <1000)
		{
			ATSVRLOG("atsvr_GSLP_system para error\r\n");
			_atsvr_cmd_rsp_error(p_env);
			return;

		}
		_atsvr_cmd_rsp_ok(p_env);
		PS_DEEP_CTRL_PARAM deep_sleep_param;

		deep_sleep_param.gpio_index_map      	= 0;
		deep_sleep_param.gpio_edge_map       	= 0;
		deep_sleep_param.gpio_stay_lo_map 	    = 0;

		deep_sleep_param.gpio_last_index_map 	= 0;
		deep_sleep_param.gpio_last_edge_map  	= 0;
		deep_sleep_param.gpio_stay_hi_map  	    = 0;

		deep_sleep_param.sleep_time     		= sleeptime/1000;
		deep_sleep_param.lpo_32k_src     		= 0;

		deep_sleep_param.wake_up_way     		= PS_DEEP_WAKEUP_RTC;

		bk_enter_deep_sleep_mode(&deep_sleep_param);
		_atsvr_cmd_rsp_ok(p_env);

	}

	_atsvr_cmd_rsp_error(p_env);
	return;

}

static void atsvr_workmode_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)
{
		_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif

	if(argc == 1){

		char output[100];
		int n;

		n = snprintf(output,100,"+WORKMODE:%d\r\n",g_env_param.workmode);
		n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
		_atsvr_output_msg(p_env,output,n);
		return;
	}
	else if(argc == 2){

	int mode = atoi(argv[1]);
	if(mode < 0 || mode >1){

		_atsvr_cmd_rsp_error(p_env);
		return ;
	}

	g_env_param.workmode = mode;
	if(g_env_param.sysstore){
		if(write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param)==0){
		_atsvr_cmd_rsp_ok(p_env);
		return ;
		}
	}
	_atsvr_cmd_rsp_ok(p_env);
	return;

	}
	else{
		
		_atsvr_cmd_rsp_error(p_env);
		return;
	}

	return;
}

static void atsvr_workmode_query_command(
#if ATSVR_HANDLER_ENV
		void* env,
#endif
		int argc, char **argv)

{
		_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
		p_env = env;
#else
		if(argc < ATSVR_MAX_ARG) {
			p_env = (_at_svr_ctrl_env_t*)argv[argc];
		}
#endif

	if(argc == 1){
		
		char output[100];
		int n;

		n = snprintf(output,100,"+WORKMODE:%d\r\n",g_env_param.workmode);
		n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
		_atsvr_output_msg(p_env,output,n);
		return;
	}
}

static void _atsvr_at_PRODUCTID_Query_command(

#if ATSVR_HANDLER_ENV
				void* env,
#endif
				int argc, char **argv)
{

				_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
				p_env = env;
#else
				if(argc < ATSVR_MAX_ARG) {
					p_env = (_at_svr_ctrl_env_t*)argv[argc];
				}
#endif


	char output[100];
	int n;
	n = snprintf(output,100,"+PRODUCTID:%s\r\n",g_env_param.deviceinfo.product_id);
	n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
	_atsvr_output_msg(p_env,output,n);
	return ;
}

static void	_atsvr_at_PRODUCTID_command(
#if ATSVR_HANDLER_ENV
				void* env,
#endif
				int argc, char **argv)

		{
				_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
				p_env = env;
#else
				if(argc < ATSVR_MAX_ARG) {
					p_env = (_at_svr_ctrl_env_t*)argv[argc];
				}
#endif
		if(argc == 1){
			char output[100];
			int n;
			n = snprintf(output,100,"+PRODUCTID:%s\r\n",g_env_param.deviceinfo.product_id);
			n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
			_atsvr_output_msg(p_env,output,n);
			return ;
		}
		else if(argc == 2){
			if(strlen(argv[1]) > 0){
				
				ATSVRLOG("org product :%s\r\n",argv[1]);
			// sscanf(argv[1],"\"%[^\"]\"",argv[1]);
				memset(g_env_param.deviceinfo.product_id,0,MAX_SIZE_OF_PRODUCT_ID + 1);
				strcpy(g_env_param.deviceinfo.product_id,argv[1]);
				ATSVRLOG("%s\r\n",g_env_param.deviceinfo.product_id);
				if(g_env_param.sysstore){
					if(write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param)==0){
					_atsvr_cmd_rsp_ok(p_env);
					return ;
					}
				}
				_atsvr_cmd_rsp_ok(p_env);
			}
		}
		else
		{
			_atsvr_cmd_rsp_error(p_env);
		}
	return ;
}

static void		_atsvr_at_DEVICENAME_Query_command(

#if ATSVR_HANDLER_ENV
				void* env,
#endif
				int argc, char **argv)
{
				_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
				p_env = env;
#else
				if(argc < ATSVR_MAX_ARG) {
					p_env = (_at_svr_ctrl_env_t*)argv[argc];
				}
#endif

	char output[100];
	int n;

	n = snprintf(output,100,"+DEVICENAME:%s\r\n",g_env_param.deviceinfo.device_name);
	n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
	_atsvr_output_msg(p_env,output,n);

	return ;
}

static void	_atsvr_at_DEVICENAME_command(
#if ATSVR_HANDLER_ENV
				void* env,
#endif
				int argc, char **argv)

		{
				_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
				p_env = env;
#else
				if(argc < ATSVR_MAX_ARG) {
					p_env = (_at_svr_ctrl_env_t*)argv[argc];
				}
#endif
		if(argc == 1){
			char output[100];
			int n;
			n = snprintf(output,100,"+DEVICENAME:%s\r\n",g_env_param.deviceinfo.device_name);
			n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
			_atsvr_output_msg(p_env,output,n);
			return ;
		}
		else if(argc == 2){
			if(strlen(argv[1]) > 0){
				strcpy(g_env_param.deviceinfo.device_name,argv[1]);
				ATSVRLOG("%s \r\n",g_env_param.deviceinfo.device_name);
				if(g_env_param.sysstore){
					if(write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param)==0){
					_atsvr_cmd_rsp_ok(p_env);
					return ;
					}
				}
				_atsvr_cmd_rsp_ok(p_env);
			}
		}
		else{
			_atsvr_cmd_rsp_error(p_env);
			return;
		}
	return ;
}

static void	 _atsvr_at_REGIONQuery_command(
#if ATSVR_HANDLER_ENV
				void* env,
#endif
				int argc, char **argv)
{
				_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
				p_env = env;
#else
				if(argc < ATSVR_MAX_ARG) {
					p_env = (_at_svr_ctrl_env_t*)argv[argc];
				}
#endif

	char output[100];
	int n;
	n = snprintf(output,100,"+REGION:%s\r\n",g_env_param.deviceinfo.region);
	n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
	_atsvr_output_msg(p_env,output,n);

	return ;
}

static void	_atsvr_at_REGION_command(
#if ATSVR_HANDLER_ENV
				void* env,
#endif
				int argc, char **argv)

		{
				_at_svr_ctrl_env_t *p_env = NULL;
#if ATSVR_HANDLER_ENV
				p_env = env;
#else
				if(argc < ATSVR_MAX_ARG) {
					p_env = (_at_svr_ctrl_env_t*)argv[argc];
				}
#endif

		if(argc == 1){
			char output[100];
			int n;
			n = snprintf(output,100,"+REGION:%s\r\n",g_env_param.deviceinfo.region);
			n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
			_atsvr_output_msg(p_env,output,n);
			return ;
		}
		else if(argc == 2){
			if(strlen(argv[1]) > 0){
				strcpy(g_env_param.deviceinfo.region,argv[1]);
				ATSVRLOG("%s \r\n",g_env_param.deviceinfo.region);
				if(g_env_param.sysstore){
					if(write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param)==0){
					_atsvr_cmd_rsp_ok(p_env);
					return ;
					}
				}
				_atsvr_cmd_rsp_ok(p_env);
			}
		}
	return ;
}

const struct _atsvr_command _atsvc_cmds_table[] = {
	_ATSVR_CMD_HADLER("AT","AT",_atsvr_at_command),
#if defined(ATSVR_CMD_HELP) && ATSVR_CMD_HELP
	//_ATSVR_CMD_HADLER("AT+HELP","AT+HELP",_atsvr_help_command),
	_ATSVR_CMD_HADLER("AT+HELP?","AT+HELP?",_atsvr_help_command),
#endif
	//_ATSVR_CMD_HADLER("AT+VERSION","AT+VERSION",_atsvr_at_verion),
	_ATSVR_CMD_HADLER("AT+GMR","AT+GMR",_atsvr_at_verion),

	_ATSVR_CMD_HADLER("AT+RST","AT+RST",atsvr_reset_system),

	_ATSVR_CMD_HADLER("AT+GSLP","AT+GSLP",atsvr_GSLP_system),

	_ATSVR_CMD_HADLER("AT+WORKMODE","AT+WORKMODE=<mode>",atsvr_workmode_command),
	_ATSVR_CMD_HADLER("AT+WORKMODE?","AT+WORKMODE?",atsvr_workmode_query_command),

	//_ATSVR_CMD_HADLER("AT+ECHO","AT+ECHO=<mode>",_atsvr_at_echo_command),

	_ATSVR_CMD_HADLER("ATE0","disalbe show AT command",_atsvr_at_close_echo_command),
	_ATSVR_CMD_HADLER("ATE1","enable show AT command",_atsvr_at_open_echo_command),


	_ATSVR_CMD_HADLER("AT+RESTORE","AT+RESTORE",_atsvr_restore_command),

//	_ATSVR_CMD_HADLER("AT+SYSREG","AT+SYSREG=<direct>,<address>[,<write_value>]",_atsvr_at_sysreg_command),

	//_ATSVR_CMD_HADLER("AT+USERRAM","AT+USERRAM",_atsvr_at_USERRAM_command),
	_ATSVR_CMD_HADLER("AT+USRRAM?","AT+USRRAM?",_atsvr_at_USERRAM_command),

	//_ATSVR_CMD_HADLER("AT+SYSFLALSH","AT+SYSFLASH",_atsvr_at_SYSFLALSH_command),
	_ATSVR_CMD_HADLER("AT+SYSFLASH?","AT+SYSFLASH?",_atsvr_at_SYSFLALSH_command),

	_ATSVR_CMD_HADLER("AT+SYSTIMESTAMP","AT+SYSTIMESTAMP=<TIMESTAMP>",_atsvr_at_SYSTIMESTAMP_command),
	_ATSVR_CMD_HADLER("AT+SYSTIMESTAMP?","AT+SYSTIMESTAMP?",_atsvr_at_SYSTIMESTAMP_QUERY_command),

	_ATSVR_CMD_HADLER("AT+SLEEPPWCFG","AT+SLEEPPWCFG=<wakeup source>,<param1>[,<param2>]",_atsvr_at_SLEEPPWCFG_command),

	_ATSVR_CMD_HADLER("AT+SYSSTORE","AT+SYSSTORE=<store mode>",_atsvr_at_SYSSTORE_command),

	_ATSVR_CMD_HADLER("AT+SYSSTORE?","AT+SYSSTORE?",_atsvr_at_SYSSTORE_Query_command),


	_ATSVR_CMD_HADLER("AT+UART_CUR","AT+UART_CUR=<baudrate>,<databits>,<stopbits>,<parity>,<flow control>",_atsvr_at_UART_CUR_command),

	_ATSVR_CMD_HADLER("AT+UART_DEF?","AT+UART_DEF?",_atsvr_at_UART_DEF_Query_command),

	_ATSVR_CMD_HADLER("AT+UART_DEF","AT+UART_DEF=<baudrate>,<databits>,<stopbits>,<parity>,<flow control>",_atsvr_at_UART_DEF_command),

	_ATSVR_CMD_HADLER("AT+PRODUCTID?","AT+PRODUCTID?",_atsvr_at_PRODUCTID_Query_command),

	_ATSVR_CMD_HADLER("AT+PRODUCTID","AT+PRODUCTID=<productid>",_atsvr_at_PRODUCTID_command),

	_ATSVR_CMD_HADLER("AT+DEVICENAME","AT+DEVICENAME=<devicename>",_atsvr_at_DEVICENAME_command),

	_ATSVR_CMD_HADLER("AT+DEVICENAME?","AT+DEVICENAME?",_atsvr_at_DEVICENAME_Query_command),

	_ATSVR_CMD_HADLER("AT+REGION?","AT+REGION?",_atsvr_at_REGIONQuery_command),

	_ATSVR_CMD_HADLER("AT+REGION","AT+REGION=<COUNTRY>",_atsvr_at_REGION_command),
};

#if ATSVR_CMD_HELP
static void _atsvr_help_command(
#if ATSVR_HANDLER_ENV
	void* env,
#endif
	int argc, char **argv)
{
	unsigned int i;
	int n,len;
	char *resultbuf;
	_at_svr_ctrl_env_t *p_env = NULL;

#if ATSVR_HANDLER_ENV
	p_env = env;
#else
	if(argc < ATSVR_MAX_ARG) {
		p_env = (_at_svr_ctrl_env_t*)argv[argc];
	}
#endif
	resultbuf = at_malloc(ATSVR_CMD_HELP_BUF_SIZE);
	if(resultbuf) {
		len = 0;
		if(p_env != NULL){
			for (i = 0, n = 0; i < ATSVR_MAX_COMMANDS && n < p_env->num_commands; i++){
				if (p_env->commands[i]->name) {
					len += snprintf(resultbuf+len,ATSVR_CMD_HELP_BUF_SIZE - len,
						"CMDRSP:cmd:%s,help:%s\r\n", p_env->commands[i]->name,
						p_env->commands[i]->help ? p_env->commands[i]->help : "");
					n++;
				}
			}
			len += snprintf(resultbuf+len,ATSVR_CMD_HELP_BUF_SIZE - len,"%s",ATSVR_CMD_RSP_SUCCEED);
		}else {
			len += snprintf(resultbuf+len,ATSVR_CMD_HELP_BUF_SIZE - len,"%s",ATSVR_CMD_RSP_ERROR);
		}
		if(p_env && p_env->output_func) {
			p_env->output_func(resultbuf,len);
		}
		at_free(resultbuf);
	}else {
		_atsvr_cmd_rsp_error(p_env);
	}
}
#endif

void _atsvr_def_cmd_init(_atsvr_env_t *env)
{
	_atsvr_register_commands(env,_atsvc_cmds_table, sizeof(_atsvc_cmds_table) / sizeof(struct _atsvr_command));
}

void _atsvr_def_cmd_deinit(_atsvr_env_t *env)
{
	_atsvr_unregister_commands(env,_atsvc_cmds_table, sizeof(_atsvc_cmds_table) / sizeof(struct _atsvr_command));
}
#endif
