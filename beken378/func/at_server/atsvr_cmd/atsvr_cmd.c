#include "atsvr_cmd.h"
#include "at_server.h"
#include "atsvr_wlan.h"
#include "atsvr_misc.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "typedef.h"
#include "atsvr_comm.h"
#include "wlan_defs_pub.h"
#include "common.h"
#include "wlan_ui_pub.h"
#include "lwip/ip_addr.h"
#if CFG_USE_NETWORKING

#define MAC_LEN_IN_BYTE  6

int copy_str2mac(unsigned char*mac,char*str)
{
	unsigned char hi_byte,lo_byte;
	char t_str; 
	for(int i = 0; i< MAC_LEN_IN_BYTE; i++)
	{
		t_str =  str[i*3];
		switch(t_str)
		{
		case '0'...'9':
			hi_byte = (unsigned char)atoi(&t_str);
			break;
		case 'a'...'f':
			hi_byte = 0xa + (t_str-'a');
			break;
		case 'A'...'F':
			hi_byte = 0xa + (t_str-'A');
			break;
		default:
			return -1;
		}
		
		t_str =  str[i*3+1];
		switch(t_str)
		{
		case '0'...'9':
			lo_byte = (unsigned char)atoi(&t_str);
			break;
		case 'a'...'f':
			lo_byte = 0xa + (t_str-'a');
			break;
		case 'A'...'F':
			lo_byte = 0xa + (t_str-'A');
			break;
		default:
			return -1;
		}

		mac[i] =(hi_byte << 4) | lo_byte; 
		
	}

	return 0;
}

extern int wifi_set_mac_address(char *mac);

static void atsvr_set_station_mac(int argc, char **argv)
{
	if(argc == 2){

	unsigned char mac[7];
	if(copy_str2mac(mac,argv[1])==-1)
	{
		ATSVRLOG("argv[1]:%s\r\n",argv[1]);
		atsvr_cmd_rsp_error();
		return;
	}
	ATSVRLOG("argv[1]:%s,mac:%02x:%02x:%02x:%02x:%02x:%02x\r\n",argv[1],mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
	wifi_set_mac_address((char*)mac);
	atsvr_cmd_rsp_ok();

	}
	else{
	atsvr_cmd_rsp_error();
	}
}

static void atsvr_get_station_mac(int argc, char **argv)
{
	if(argc == 1){
		unsigned char mac[MAC_LEN_IN_BYTE];
		int n = 0;
		char resultbuf[200];

		wlan_get_station_mac_address((char*)mac);

		n = snprintf(resultbuf,sizeof(resultbuf),"+WLMAC:%02x:%02x:%02x:%02x:%02x:%02x\r\n",mac[0],
				mac[1],mac[2],mac[3],mac[4],mac[5]);
		n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"%s",ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf,n);
		return;
	}
	else{
		atsvr_cmd_rsp_error();
	}
}

void atsvr_set_station_dhcp_query(int argc, char **argv)
{
	if(argc == 1){
		int n = 0;
		char resultbuf[200];
		n = snprintf(resultbuf,sizeof(resultbuf),ATSVR_CMDRSP_HEAD"+CWDHCP:%d\r\n",(g_env_param.stainfo.dhcp | (g_env_param.apinfo.dhcp<<1)));//(g_env_param.stainfo.dhcp & 0x01) | 0x2);
		n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"%s",ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf,n);
	}
	else{

		atsvr_cmd_rsp_error();
		return;
	}

	return;
}

void atsvr_set_station_dhcp(int argc, char **argv)
{
	int operate,mode;

	if(argc != 3){
		atsvr_cmd_rsp_error();
		return;
	}

	operate =  atoi(argv[1]);
	mode =  atoi(argv[2]);
	if(operate > 1)
	{
		operate = 1;
	}

	switch(mode)
	{
		case 0:
			atsvr_cmd_rsp_error();
		break;
		case 1:	//sta
			g_env_param.stainfo.dhcp = operate;
		break;
		case 2:	//ap
			g_env_param.apinfo.dhcp = operate;
		break;
		case 3:	//sta+ap
			g_env_param.stainfo.dhcp = operate;
			g_env_param.apinfo.dhcp = operate;
		break;
		default :
			g_env_param.stainfo.dhcp = operate;
			g_env_param.apinfo.dhcp = operate;
		break;
	}
	if(g_env_param.sysstore)
			write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param);
	atsvr_cmd_rsp_ok();
	return;
}

static void atsvr_station_ssid_pwd(int argc, char **argv)
{
	char *my_ssid=NULL;
	char *connect_key=NULL;
	int updateflag = 0;
	if(argc == 1){
		wlan_start_station_connect(g_env_param.stainfo.con_ssid,g_env_param.stainfo.con_key);
		atsvr_cmd_rsp_ok();
		return;
	}
	else if(argc == 2){
		my_ssid = argv[1];
		connect_key = "OPEN";
	}else if(argc == 3){
		my_ssid = argv[1];
		connect_key = argv[2];
	}else if(argc > 3){
		my_ssid = argv[1];
		connect_key = argv[2];
	}

	if(wlan_start_station_connect(my_ssid, connect_key) == 0){
		if(strcmp(g_env_param.stainfo.con_ssid, my_ssid)!=0){
		strcpy(g_env_param.stainfo.con_ssid,my_ssid);
		updateflag =1;
		}

	if(strcmp(g_env_param.stainfo.con_key, connect_key)!=0){

		strcpy(g_env_param.stainfo.con_key ,connect_key);
		updateflag =1;
	}

	if(updateflag)
	{
		write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param);
	}
	atsvr_cmd_rsp_ok();

	}else{
		atsvr_cmd_rsp_error();
	}

	return;
}

static void atsvr_station_stop(int argc, char **argv)
{
	if(argc != 1){
		atsvr_cmd_rsp_error();
		return;
	}

	if(wlan_stop_station() == 0){
		atsvr_cmd_rsp_ok();
	}else{
		atsvr_cmd_rsp_error();
	}

	return;
}

static void atsvr_station_static_ip(int argc, char **argv)
{
	if(argc != 4){
		atsvr_cmd_rsp_error();
		return;
	}
	char *ip = argv[1];
	char *gate = argv[2];
	char *mask = argv[3];

	if(judge_the_string_is_ipv4_string(ip) != 0){
		atsvr_cmd_rsp_error();
		ATSVRLOG("[atsvr]ip:%s error\r\n",ip);
		return;
	}
	if(judge_the_string_is_ipv4_string(gate) != 0){
		atsvr_cmd_rsp_error();
		ATSVRLOG("[atsvr]gate:%s error\r\n",gate);
		return;
	}
	if(judge_the_string_is_ipv4_string(mask) != 0){
		atsvr_cmd_rsp_error();
		ATSVRLOG("[atsvr]mask:%s error\r\n",mask);
		return;
	}

	if(wlan_set_station_static_ip(ip, mask,gate) == 0){
		strcpy(g_env_param.stainfo.sta_local_ip,ip);
		strcpy(g_env_param.stainfo.sta_mask,mask);
		strcpy(g_env_param.stainfo.sta_gate,gate);
		if(g_env_param.sysstore)
		{
			write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(g_env_param),(uint8*)&g_env_param);
		}

		IPStatusTypedef ipstatus;

		strcpy(ipstatus.ip,g_env_param.stainfo.sta_local_ip);
		strcpy(ipstatus.mask,g_env_param.stainfo.sta_mask);
		strcpy(ipstatus.gate,g_env_param.stainfo.sta_gate);
		strcpy(ipstatus.dns,g_env_param.dnsinfo.dns1);
		ipstatus.dhcp = g_env_param.stainfo.dhcp;
		bk_wlan_set_ip_status(&ipstatus,BK_STATION);

		atsvr_cmd_rsp_ok();
	}else{
		atsvr_cmd_rsp_error();
	}
}

void atsvr_get_station_status(int argc, char **argv)
{
	if(argc != 1){
		atsvr_cmd_rsp_error();
		return;
	}
	LinkStatusTypeDef stalinkstatus;
	memset(&stalinkstatus,0,sizeof(LinkStatusTypeDef));
	bk_wlan_get_link_status(&stalinkstatus);
	IPStatusTypedef ipStatus;
	bk_wlan_get_ip_status(&ipStatus, BK_STATION);

	int n = 0;
	char resultbuf[1024];
	n = snprintf(resultbuf,sizeof(resultbuf),"+STAST:%s,dhcp=%d, ip=%s, gate=%s, mask=%s,mac=" MACSTR ",RSSI:%d,ssid:%s,channel:%d,security:%d\r\n",stalinkstatus.conn_state? "connected":"disconnected",ipStatus.dhcp, ipStatus.ip, ipStatus.gate,
				ipStatus.mask, MAC2STR((unsigned char*)ipStatus.mac),stalinkstatus.wifi_strength,stalinkstatus.ssid,stalinkstatus.channel,stalinkstatus.security);
	n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"%s",ATSVR_CMD_RSP_SUCCEED);
	atsvr_output_msg(resultbuf,n);

	return;
}

void atsvr_get_information_ip(int argc, char **argv)
{
	if(argc != 1){
		atsvr_cmd_rsp_error();
		return;
	}
	LinkStatusTypeDef stalinkstatus;
	memset(&stalinkstatus,0,sizeof(LinkStatusTypeDef));
	bk_wlan_get_link_status(&stalinkstatus);
	IPStatusTypedef ipStatus;
	bk_wlan_get_ip_status(&ipStatus, BK_STATION);

	int n = 0;
	char resultbuf[1024];
	n = snprintf(resultbuf,sizeof(resultbuf),"+CIPSTA:ip:%s\r\n",ipStatus.ip);
	n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"+CIPSTA:gateway:%s\r\n",ipStatus.gate);
	n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"+CIPSTA:netmask:%s\r\n",ipStatus.mask);
	n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"%s",ATSVR_CMD_RSP_SUCCEED);
	atsvr_output_msg(resultbuf,n);

	return;
}

void atsvr_wlan_set_ap_proto_cmd(int argc, char **argv)
{
	if((argc != 1) && (argc != 2)){
		atsvr_cmd_rsp_error();
		return;
	}

	if(argc == 1)
	{
				char output[100];
				int n;
				n = snprintf(output,100,"+WLAPPROTO:%d\r\n",g_env_param.apinfo.proto);
				n += snprintf(output+n,100 - n,ATSVR_CMD_RSP_SUCCEED);
				atsvr_output_msg(output,n);
	}
	else if(argc == 2){

		g_env_param.apinfo.proto = atoi(argv[1]);

		wlan_softap_start(g_env_param.apinfo.ap_ssid, g_env_param.apinfo.ap_key,g_env_param.apinfo.channel,g_env_param.apinfo.proto,g_env_param.apinfo.hidden);

		if(g_env_param.sysstore)
			write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(ENV_PARAM),(uint8*)&g_env_param);
	}

	atsvr_cmd_rsp_ok();
	return;
}

void atsvr_wlan_scan_cmd(int argc, char **argv)
{
	if((argc != 1) && (argc != 2)){
		atsvr_cmd_rsp_error();
		return;
	}
	char *ssid = NULL;

	if(argc == 2){
		ssid = argv[1];
	}
	if(wlan_scan_start(ssid) != 0){
		atsvr_cmd_rsp_error();
		return;
	}
	set_atsvr_work_state(ATSVR_WK_DONE);
	atsvr_cmd_rsp_ok();
	return;
}

static void atsvr_softap_start(int argc, char **argv)
{
	if(argc < 5)
	{
		atsvr_cmd_rsp_error();
		return;
	}
	char *my_ssid;
	char *connect_key;
	int channel =0;
	int encryp=0;
	int hidden = 0 ;

	my_ssid = argv[1];
	connect_key = argv[2];
	channel= atoi(argv[3]);
	if(channel >14) {
		atsvr_cmd_rsp_error();
		return;
	}
	encryp=atoi(argv[4]);
	if(argc >= 6) {

	}

	if(argc >= 7) {

		hidden = atoi(argv[6]);
	}

	if(g_env_param.wifimode.mode < 2){

		atsvr_cmd_rsp_error();
		return;
	}

	if(wlan_softap_start(my_ssid, connect_key,channel,g_env_param.apinfo.proto,hidden) == 0){

			if(strcmp(g_env_param.apinfo.ap_ssid,my_ssid)!=0 || strcmp(g_env_param.apinfo.ap_key,connect_key)!=0 )
			{
				strcpy(g_env_param.apinfo.ap_ssid,my_ssid);
				strcpy(g_env_param.apinfo.ap_key,connect_key);
				g_env_param.apinfo.channel=channel;
				g_env_param.apinfo.ap_enc=encryp;
				g_env_param.apinfo.hidden=hidden;
				if(g_env_param.sysstore)
				write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(ENV_PARAM),(uint8*)&g_env_param);
			}
			atsvr_cmd_rsp_ok();
		}else{
			atsvr_cmd_rsp_error();
		}

	return;
}

static void atsvr_softap_stop(int argc, char **argv)
{
	if(argc != 1){
		atsvr_cmd_rsp_error();
		return;
	}

	if(wlan_stop_softap() == 0){
		set_atsvr_work_state(ATSVR_WK_DONE);
		atsvr_cmd_rsp_ok();
	}else{
		set_atsvr_work_state(ATSVR_WK_DONE);
		atsvr_cmd_rsp_error();
	}

	return;
}

static void atsvr_softap_query_static_ip(int argc, char **argv)
{
	if(argc ==1){
		int n = 0;
		char resultbuf[100];
		n = snprintf(resultbuf,sizeof(resultbuf),ATSVR_CMDRSP_HEAD"+CIPAP:%s,%s,%s\r\n",g_env_param.apinfo.ap_local_ip,g_env_param.apinfo.ap_mask,g_env_param.apinfo.ap_gate);
		n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"%s",ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf,n);
	}
	else{
		atsvr_cmd_rsp_error();
	}
	return;
}

static void atsvr_softap_static_ip(int argc, char **argv)
{
	if (argc < 4) {
			atsvr_cmd_rsp_error();
			return;
		}

		char *ip = argv[1];
		char *mask = argv[2];
		char *gate = argv[3];

		if(judge_the_string_is_ipv4_string(ip) != 0){
			atsvr_cmd_rsp_error();
			ATSVRLOG("[atsvr]ip:%s error\r\n",ip);
			return;
		}
		if(judge_the_string_is_ipv4_string(mask) != 0){
			atsvr_cmd_rsp_error();
			ATSVRLOG("[atsvr]mask:%s error\r\n",mask);
			return;
		}
		if(judge_the_string_is_ipv4_string(gate) != 0){
			atsvr_cmd_rsp_error();
			ATSVRLOG("[atsvr]gate:%s error\r\n",gate);
			return;
		}


		if(wlan_set_softap_static_ip(ip, mask,gate) == 0){
			strcpy(g_env_param.apinfo.ap_local_ip,ip);
			strcpy(g_env_param.apinfo.ap_mask,mask);
			strcpy(g_env_param.apinfo.ap_gate,gate);

			wlan_stop_softap();
			wlan_softap_start(g_env_param.apinfo.ap_ssid, g_env_param.apinfo.ap_key,g_env_param.apinfo.channel,g_env_param.apinfo.proto,g_env_param.apinfo.hidden);

			if(g_env_param.sysstore)
				write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(ENV_PARAM),(uint8*)&g_env_param);
			atsvr_cmd_rsp_ok();
		}else{
			atsvr_cmd_rsp_error();
		}
	return;
}

void atsvr_get_softap_status(int argc, char **argv)
{
	if(argc != 1){
		atsvr_cmd_rsp_error();
		return;
	}

	int n = 0;
	char resultbuf[200];
	wlan_ap_stas_t stas;
	memset(&stas,0,sizeof(wlan_ap_stas_t));

	wlan_ap_sta_info(&stas);

	bk_printf("connnected sta num:%d\r\n",stas.num);

	for(int index=0;index<stas.num;index++)
	{
		n += snprintf(resultbuf+n,sizeof(resultbuf)-n,"+CWLIF:index:%d,ip:%s,mac: "MACSTR", rssi:%d\r\n\r\n",index,ipaddr_ntoa((const ip_addr_t *)&stas.sta[index].ipaddr),MAC2STR((uint8_t*)&stas.sta[index].addr),stas.sta[index].rssi);
	}
	if(n>0)
	{
		atsvr_output_msg(resultbuf,n);
	}
	set_atsvr_work_state(ATSVR_WK_DONE);
	atsvr_cmd_rsp_ok();
	return;
}

void atsvr_wifimode_command(int argc, char **argv)
{
	uint8  mode;
	int n = 0;
	char resultbuf[200];
	uint8 autoconnect;
	if(argc == 1){
		n = snprintf(resultbuf,sizeof(resultbuf),"+WLMODE:%d,%d\r\n",g_env_param.wifimode.mode,g_env_param.wifimode.autoconnect);
		n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"%s",ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf,n);
		return;
	}
	else if(argc == 2){
		mode = atoi(argv[1]);
		g_env_param.wifimode.mode=mode;
		if(g_env_param.sysstore)
			write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(ENV_PARAM),(uint8*)&g_env_param);
		atsvr_cmd_rsp_ok();
	}
	else if(argc == 3)
	{
		mode = atoi(argv[1]);
		autoconnect = atoi(argv[2]);
		g_env_param.wifimode.mode =mode;
		g_env_param.wifimode.autoconnect = autoconnect;
		if(g_env_param.sysstore) {
			write_env_to_flash(TAG_SYSSTORE_OFFSET,sizeof(ENV_PARAM),(uint8*)&g_env_param);
		}
		atsvr_cmd_rsp_ok();
	}
	else{
		atsvr_cmd_rsp_error();
		return;
	}

	update_wifi_action(g_env_param.wifimode.mode,mode);
	return;
}

void atsvr_wifimode_query_command(int argc, char **argv)
{
	int n = 0;
	char resultbuf[200];
	if(argc == 1){
		n = snprintf(resultbuf,sizeof(resultbuf),"+WLMODE:%d,%d\r\n",g_env_param.wifimode.mode,g_env_param.wifimode.autoconnect);
		n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"%s",ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf,n);
		return;
	}

	return;
}

void atsvr_startsmartconfig_command(int argc, char **argv)
{
	atsvr_cmd_rsp_ok();
	return;
}

const struct _atsvr_command atsvc_cmds_table[] = {
	ATSVR_CMD_HADLER("AT+CWMODE","AT+CWMODE",atsvr_wifimode_command),
	ATSVR_CMD_HADLER("AT+CWMODE?","AT+CWMODE?",atsvr_wifimode_query_command),

	ATSVR_CMD_HADLER("AT+CIPSTAMAC?","AT+CIPSTAMAC?",atsvr_get_station_mac),
	ATSVR_CMD_HADLER("AT+CIPSTAMAC","AT+CIPSTAMAC=<\"mac\">",atsvr_set_station_mac),

	ATSVR_CMD_HADLER("AT+CWSAP","AT+CWSAP=<\"ssid\">,<\"pwd\">,<channel>,<ecn>,<max conn>,<hidden>",atsvr_softap_start),
	ATSVR_CMD_HADLER("AT+CWQIF","AT+CWQIF",atsvr_softap_stop),    //�Ͽ���AP������

	ATSVR_CMD_HADLER("AT+CIPAP?","AT+CIPAP?",atsvr_softap_query_static_ip),
	ATSVR_CMD_HADLER("AT+CIPAP","AT+CIPAP=<\"ip\">,<\"mask\">,<\"gate\">",atsvr_softap_static_ip),
	ATSVR_CMD_HADLER("AT+CWLIF","AT+CWLIF?",atsvr_get_softap_status),

	//ATSVR_CMD_HADLER("AT+WLAPPROTO","AT+WLAPPROTO",atsvr_wlan_set_ap_proto_cmd),
	ATSVR_CMD_HADLER("AT+WSCAN","AT+WSCAN",atsvr_wlan_scan_cmd),

	ATSVR_CMD_HADLER("AT+CWDHCP?","AT+CWDHCP?",atsvr_set_station_dhcp_query),
	ATSVR_CMD_HADLER("AT+CWDHCP","AT+CWDHCP=<enable>,<mode>",atsvr_set_station_dhcp),
	ATSVR_CMD_HADLER("AT+CWJAP","AT+CWJAP=[<SSID>],[<PWD>][,<bssid>][,<pci_en>][,<reconn_interval>][,<listen_interval>][,scan_mode][,jap_timeout][,<pmf>]",atsvr_station_ssid_pwd),
	ATSVR_CMD_HADLER("AT+CWJAP?","AT+CWJAP?",atsvr_station_ssid_pwd),
	ATSVR_CMD_HADLER("AT+CWQAP","AT+CWQAP",atsvr_station_stop),
	ATSVR_CMD_HADLER("AT+CIPSTA?","AT+CIPSTA?",atsvr_get_information_ip),
	ATSVR_CMD_HADLER("AT+CIPSTA","AT+CIPSTA=<\"ip\">,<\"mask\">,<\"gate\">",atsvr_station_static_ip),
	ATSVR_CMD_HADLER("AT+STAST?","AT+STAST?",atsvr_get_station_status),
};

void atsvr_cmd_init(void)
{
	atsvr_register_commands(atsvc_cmds_table, sizeof(atsvc_cmds_table) / sizeof(struct _atsvr_command));
	//atsvr_extern_cmd_init();

	atsvr_wlan_init();
}

void __weak__ atsvr_extern_cmd_init(void)
{

}
#endif
