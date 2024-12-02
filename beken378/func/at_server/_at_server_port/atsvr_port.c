#include "atsvr_port.h"
#include "atsvr_core.h"
#include "at_server.h"
#include "atsvr_cmd.h"
#include "atsvr_wlan.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "rtos_pub.h"
#include "include.h"
///#include "error.h"
#include "uart_pub.h"
#include "uart.h"
#include "BkDriverUart.h"
#include "mem_pub.h"
#include "atsvr_comm.h"
#include "network_app.h"


typedef struct{
	unsigned uart_overflow : 1;

	beken_queue_t msg_queue;
	beken_semaphore_t at_rx_sema;
	beken_semaphore_t at_tx_sema;

	beken_semaphore_t resources_protection;

	unsigned char rxbuf[ATSVR_INPUT_BUFF_MAX_SIZE];
	unsigned int bp;
}atsvr_port_env;

atsvr_port_env atsvr_port = {
	.uart_overflow = 0,
	.msg_queue = NULL,
	.at_rx_sema = NULL,
	.at_tx_sema = NULL,
	.resources_protection = NULL,
	.rxbuf={0},
	.bp = 0,
};

ENV_PARAM g_env_param={
.sysstore=1,
.workmode=0,   //0:factory mode,1:common mode
.wifimode={1,1},
.apinfo= {ATSVR_WLAN_AP_SSID,ATSVR_WLAN_AP_KEY,1,8,0,0,ATSVR_WLAN_DEFAULT_IP,ATSVR_WLAN_DEFAULT_MASK,ATSVR_WLAN_DEFAULT_GW,1},
.stainfo={ATSVR_WLAN_STA_SSID,ATSVR_WLAN_STA_KEY,1,ATSVR_WLAN_STA_DEFAULT_IP,ATSVR_WLAN_STA_DEFAULT_MASK,ATSVR_WLAN_STA_DEFAULT_GW},
.uartinfo={115200,BK_DATA_WIDTH_8BIT,BK_STOP_BITS_1,0,0},
.net_transmission_mode = 0,
.deviceinfo = {ATSVR_DEVINFO_PRODUCTID, ATSVR_DEVINFO_DEVNAME,"china"},
.ntpinfo = {1,"cn.pool.ntp.org",8},
.dnsinfo = {1,ATSVR_WLAN_DEFAULT_DNS1,ATSVR_WLAN_DEFAULT_DNS2,ATSVR_WLAN_DEFAULT_DNS3},
.pskinfo ={5,ATSVR_PSKINFO_PSK,ATSVR_PSKINFO_HINT},
.ble_param={"BK_BLE",0,160,160,7,2,{0xff,0xee},2,{0x11,0x22},1,10,{0x09,0x09,0x37,0x32,0x33,0x38,0x5F,0x42,0x4C,0x45},6,{0x05,0x08,0x37,0x32,0x33,0x38},0,0,0,100,50,87,0,500},

};

ATSVR_STATUS g_atsvr_status = {

	.station_connect_status = 0,
	.softap_connect_status = 0,
	.network_transfer_mode = ATSVR_COMMON_MODE,
};
int get_atsvr_cmd_message_state(unsigned int timeout);



void set_at_uart_overflow(int enable)
{
	GLOBAL_INT_DECLARATION();

	GLOBAL_INT_DISABLE();
	atsvr_port.uart_overflow = (enable > 0)?1:0;
	GLOBAL_INT_RESTORE();
}

int get_at_uart_overflow(void)
{
	int flag;
	GLOBAL_INT_DECLARATION();

	GLOBAL_INT_DISABLE();
	flag = atsvr_port.uart_overflow;
	GLOBAL_INT_RESTORE();
	return flag;
}

int IN atsvr_input_char(unsigned char *buf)
{
	if (bk_uart_recv(AT_UART_PORT_CFG, buf, 1, BEKEN_NO_WAIT) == 0){
		return 1;
	}else{
		return 0;
	}
}

#define REG_READ(addr)          (*((volatile UINT32 *)(addr)))
#define REG_WRITE(addr, _data)  (*((volatile UINT32 *)(addr)) = (_data))
static void atsvr_send_byte(UINT8 data)
{
	UINT8 uport = AT_UART_PORT_CFG;
#if AT_UART_SEND_DATA_INTTRRUPT_PROTECT
	GLOBAL_INT_DECLARATION();
#endif
	if(UART1_PORT == uport)
		while(!UART1_TX_WRITE_READY);
	else
		while(!UART2_TX_WRITE_READY);
#if AT_UART_SEND_DATA_INTTRRUPT_PROTECT
	GLOBAL_INT_DISABLE();
#endif
	UART_WRITE_BYTE(uport, data);
#if AT_UART_SEND_DATA_INTTRRUPT_PROTECT
	GLOBAL_INT_RESTORE();
#endif
}

void at_msg_intf_clear(unsigned int dnum)
{
	unsigned char dat;
	unsigned int count = 0;
	int ret;

	do{
		if(count >= dnum){
			break;
		}
		ret = atsvr_input_char(&dat);
		if(ret == 1){
			count++;
		}
	}while(ret);
}

void atsvr_overflow_handler(void)
{
	at_msg_intf_clear(RX_RB_LENGTH);
	set_at_uart_overflow(0);
}

static void atsvr_rx_sema_callback(int uport, void *param)
{
	if(atsvr_port.at_rx_sema){
		rtos_set_semaphore(&atsvr_port.at_rx_sema);
	}
}

void atsvr_msg_intf_init(void)
{
	rtos_init_semaphore(&atsvr_port.at_rx_sema, 32);
	rtos_init_semaphore(&atsvr_port.at_tx_sema, 1);
	rtos_init_semaphore(&atsvr_port.resources_protection, 1);
	if(atsvr_port.at_tx_sema){
		rtos_set_semaphore(&atsvr_port.at_tx_sema);
	}
	if(atsvr_port.resources_protection){
		rtos_set_semaphore(&atsvr_port.resources_protection);
	}
	bk_uart_set_rx_callback(AT_UART_PORT_CFG, atsvr_rx_sema_callback,NULL);
}

static int resources_protection_func(int is_lock,unsigned int timeout)
{
	if(atsvr_port.resources_protection){
		if(is_lock){
			if(rtos_get_semaphore(&atsvr_port.resources_protection,timeout) != kNoErr){
				return -1;
			}
		}else{
			rtos_set_semaphore(&atsvr_port.resources_protection);
		}
	}
	return 0;
}

int OUT atsvr_command_msg_handle_result_output_u(char *msg,unsigned int len,unsigned int timeout)
{
	int i = 0;
	int sta = 0;

	if(atsvr_port.at_tx_sema) {
		sta = rtos_get_semaphore(&atsvr_port.at_tx_sema, timeout);
		if(kNoErr != sta ){
			sta = -1;
		} else{
			for(i = 0; i < len; i++) {
				atsvr_send_byte(msg[i]);
			}
			rtos_set_semaphore(&atsvr_port.at_tx_sema);
		}
	}
	else {
		sta = -1;
	}

	return sta;
}

static void atsvr_output_func(char *msg,unsigned int msg_len)
{
	atsvr_command_msg_handle_result_output_u(msg,msg_len,0xFFFFFFFFU);
}

static void atsvr_read_useless_data(uint16_t timeout)
{
	char data_tmp;
	while(1)
	{
		while(bk_uart_recv(AT_UART_PORT_CFG, &data_tmp, 1, BEKEN_NO_WAIT) == 0);

		if(get_atsvr_cmd_message_state(timeout) == kNoErr)
		{
			continue;
		}
		else
			return;
	}
}

static unsigned int atsvr_input_msg_get_func(char *data,unsigned int data_len)
{
	int read_len=0;
	if (data_len == 0)
	{
		return 0;
	}
	while(1)
	{
		if(get_atsvr_cmd_message_state(5) == kNoErr)
		{
			while (bk_uart_recv(AT_UART_PORT_CFG, data+read_len, 1, BEKEN_NO_WAIT) == 0)
			{
				if ((*(data+read_len) == ATSVR_RET_CHAR) || (*(data+read_len)  == ATSVR_END_CHAR))  /* end of input line */
				{
					if(read_len==0)
					{
						bk_printf("maybe had \"\r\n\" :%x\r\n",*(data+read_len));
					}
				}
				if(++read_len==data_len)
				{
					atsvr_read_useless_data(300);
					data[read_len] ='\0';
					return read_len;
				}
			}
		}
	}
}

static unsigned int atsvr_input_string(char *data,unsigned int data_len)
{
	int i=0;
	while(atsvr_input_char((unsigned char*) &data[i]))
	{
		i++;
	}
	data[i]='\0';
	return i;
}

int atsvr_port_send_msg_queue(atsvr_msg_t *sd_atsvrmsg,unsigned int timeout)
{
	if(atsvr_port.msg_queue != NULL){
		int ret = rtos_push_to_queue(&atsvr_port.msg_queue,sd_atsvrmsg,timeout);
		if(ret != kNoErr){
			ATSVRLOG("[ATSVR]msg_queue push error:%d\r\n",ret);
			return -1;
		}
		return 0;
	}else{
		ATSVRLOG("[ATSVR]atsvr_port.msg_queue is invaild\r\n");
		return -1;
	}
}

void atsvr_event_handler(int event)
{
	ATSVRLOG("[ATSVR]treated event(%x)\r\n",event);
	if(!wlan_event_handler(event)){
		return;
	}else{
		ATSVRLOG("[ATSVR]untreated event(%x)\r\n",event);
	}
}

int get_atsvr_cmd_message_state(unsigned int timeout)
{
	int32 sta = -1;

	if(atsvr_port.at_rx_sema){
		sta = rtos_get_semaphore(&atsvr_port.at_rx_sema, timeout);
	}else{
		ATSVRLOG("[%s]at_rx_sema error\r\n",__FUNCTION__);
	}
	return sta;
}

void atsvr_init_env_para()
{
	uint8 flag=0;

	read_env_from_flash(TAG_SYSSTORE_OFFSET,LEN_SYSSTORE_VALUE,&flag);
	if(flag ==0xFF)
	{
		write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(ENV_PARAM),(uint8*)&g_env_param);
		rtos_delay_milliseconds(10);
	}
	else{

		if(flag == 1)
		{
			read_env_from_flash(TAG_SYSSTORE_OFFSET,sizeof(ENV_PARAM),(uint8*)&g_env_param);
			int i = 0;
			for(i=0;i<16;i++)
			{
				ATSVRLOG("%d \r\n",((uint8*)&g_env_param)[i]);

			}

		}
	}

	ATSVRLOG("[%s]  read1 default param from flash,SYSSTORE flag:%d,READ SYSSTRORE:%d\r\n",__FUNCTION__,flag,g_env_param.sysstore);

	return ;
}

void atsvr_poweron_default_action()
{
	if(g_env_param.sysstore)
	{
		//reconnect ap
		if(g_env_param.wifimode.autoconnect && g_env_param.wifimode.mode & 0x01 && (strcmp(g_env_param.stainfo.con_ssid,ATSVR_WLAN_STA_SSID)!=0))
		{
			if(strcmp(g_env_param.stainfo.con_ssid,ATSVR_WLAN_STA_SSID_ERROR) == 0)
			{
				#if CFG_USE_DEFUALT_CMD
				extern int user_restore_func(void);
				user_restore_func();
				#endif
			}
			else
				wlan_start_station_connect(g_env_param.stainfo.con_ssid,g_env_param.stainfo.con_key);
		}

		if(g_env_param.wifimode.mode & 0x2){

			wlan_softap_start(g_env_param.apinfo.ap_ssid, g_env_param.apinfo.ap_key,g_env_param.apinfo.channel,g_env_param.apinfo.proto,g_env_param.apinfo.hidden) ;
		}

		if(g_env_param.uartinfo.baudrate != 115200){

		bk_uart_config_t uart_cfg;
		uart_cfg.baud_rate = g_env_param.uartinfo.baudrate;
		uart_cfg.data_width = g_env_param.uartinfo.databits;
		uart_cfg.parity = g_env_param.uartinfo.parity;
		uart_cfg.stop_bits = g_env_param.uartinfo.stopbits;
		uart_cfg.flow_control = g_env_param.uartinfo.flow_control;
		bk_uart_initialize(AT_UART_PORT_CFG,&uart_cfg,NULL);

		}
	}

}
#if CFG_USE_DISTRIBUTION_NETWORK
void atsvr_network_init();
#endif
#if CFG_USE_MQTT
void atsvr_mqtt_init();
#endif
int rx_lenth =  0;
void atsvr_set_size_rxbuf(int lenth)
{
	rx_lenth += lenth;
	return ;
}

void atsvr_clear_size_rxbuf()
{
	rx_lenth = 0;
}

int atsvr_get_size_rxbuf()
{
	return rx_lenth;
}

void atsvr_copy_lenth_rxbuf(char *buf,int lenth)
{
	for(int i=0;i<lenth;i++)
	{
		buf[i] = atsvr_port.rxbuf[i];
	}
	buf[lenth] = '\0';
	return ;
}
#if CFG_USE_DISTRIBUTION_NETWORK
extern void atsvr_airkiss_init();
#endif
extern int powerup_init();

static void atsvr_msg_main( UINT32 data )
{
	unsigned int timeout = BEKEN_NEVER_TIMEOUT;
	int state;
	log_output_state(FALSE);

	atsvr_init_env_para();
	#if CFG_USE_DEFUALT_CMD
	at_server_init();
	#endif
	atsvr_msg_intf_init();
	#if CFG_USE_NETWORKING
	atsvr_cmd_init();
	#endif
	#if CFG_USE_BLE
	at_ble_cmd_init();
	#endif
	#if CFG_USE_TCPUDP
	atsvr_network_init();
	#endif
	#if CFG_USE_MQTT
	atsvr_mqtt_init();
	#endif
	#if CFG_USE_HTTP
	atsvr_http_init();
	#endif
	#if CFG_USE_DISTRIBUTION_NETWORK
	atsvr_airkiss_init();
	#endif
	atsvr_register_resources_protection_func(resources_protection_func);
	atsvr_register_output_func(atsvr_output_func);
	atsvr_register_input_msg_func(atsvr_input_msg_get_func);
	rtos_delay_milliseconds(ATSVR_POWER_UP_READY_DELAY);
	at_msg_intf_clear(0xFFFFFFFF);
	atsvr_notice_ready();
	atsvr_poweron_default_action();
	//powerup_init();

	while (1)
	{
		state =  get_atsvr_cmd_message_state(timeout);
		if(state == kNoErr) {
			if(g_atsvr_status.network_transfer_mode == ATSVR_COMMON_MODE){
			con:
				state = atsvr_msg_get_input(&atsvr_port.rxbuf[0],&atsvr_port.bp);
				if( state == 1 ) {
					atsvr_msg_t msg;
					msg.type = ATSVR_MSG_STREAM;
					msg.sub_type = ATSVR_SUBMSG_NONE;
					msg.len = atsvr_port.bp;
					msg.msg_param = (void*)atsvr_port.rxbuf;
					atsvr_msg_handler(&msg);
					atsvr_port.bp = 0;
					goto con;

				}else if( state <= -1 ) {
					ATSVRLOG("[ATSVR]READ LINE ERROR(%d)\r\n",state);
					atsvr_cmd_analysis_notice_error();
					atsvr_port.bp = 0;
					goto con;
				}else if ( state != 0 ){
					atsvr_port.bp = 0;
				}
			}
			else if(g_atsvr_status.network_transfer_mode == ATSVR_PASSTHROUGH_MODE){   /*passthrough mode*/
				int len = atsvr_input_string((char*)&atsvr_port.rxbuf[0], ATSVR_INPUT_BUFF_MAX_SIZE);
				if(strcmp((char*)atsvr_port.rxbuf,"+++")==0)
				{
					g_atsvr_status.network_transfer_mode = ATSVR_COMMON_MODE;
					atsvr_output_msg("+QUIT\r\n",strlen("+QUIT\r\n"));
					continue;
				}

				if(len > 0)
				{
					#if CFG_USE_TCPUDP
					if(network_passthrough_send_msg(&atsvr_port.rxbuf[0],len) != len ) {
						ATSVRLOG("[ATSVR] passthrough data faill\r\n");
					}
					#endif
				}
			}
		}
		else {
			ATSVRLOG("[ATSVR]at msg receive timeout error\r\n");
		}
	}

	rtos_delete_thread(NULL);
}

static void atsvr_handler_main( UINT32 data )
{
	unsigned int timeout = BEKEN_NEVER_TIMEOUT;
	atsvr_msg_t atsvrmsg;
	int ret;

	while (1)
	{
		ret = rtos_pop_from_queue(&atsvr_port.msg_queue,&atsvrmsg,timeout);
		if(ret == kNoErr){
			atsvr_msg_handler(&atsvrmsg);
		}
	}

	rtos_delete_thread(NULL);
}


int atsvr_app_init(void)
{
	int ret;
	beken_thread_t msg_thread = NULL,handler_thread = NULL;
	if(atsvr_port.msg_queue == NULL){
		ret = rtos_init_queue(&atsvr_port.msg_queue,"atsvr-q",sizeof(atsvr_msg_t),ATSVR_QUEUE_MAX_NB);
		if(ret){
			ATSVRLOG("Error: Failed to create atsvr-q (ret:%d)\r\n",ret);
			goto init_general_err;
		}
	}
	ret = rtos_create_thread(&msg_thread,
							BEKEN_DEFAULT_WORKER_PRIORITY,
							"atsvr-m",
							(beken_thread_function_t)atsvr_msg_main,
							(1024 * 8),
							0);
	if (ret != kNoErr) {
		ATSVRLOG("Error: Failed to create cli thread: %d\r\n",ret);
		goto init_general_err;
	}

	ret = rtos_create_thread(&handler_thread,
							BEKEN_DEFAULT_WORKER_PRIORITY,
							"atsvr-h",
							(beken_thread_function_t)atsvr_handler_main,
							(1024 * 3),
							0);
	if (ret != kNoErr) {
		ATSVRLOG("Error: Failed to create cli thread: %d\r\n",ret);
		goto init_general_err;
	}

	return kNoErr;
init_general_err:
	if( msg_thread != NULL ) {
		rtos_delete_thread(&msg_thread);
	}
	if( handler_thread != NULL ) {
		rtos_delete_thread(&handler_thread);
	}
	if(atsvr_port.msg_queue != NULL){
		ret = rtos_deinit_queue( &atsvr_port.msg_queue );
		atsvr_port.msg_queue = NULL;
	}
	return kGeneralErr;
}

