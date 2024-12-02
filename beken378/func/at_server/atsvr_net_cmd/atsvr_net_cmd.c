
#include "lwip/netdb.h"

#include "_atsvr_func.h"
#include "_at_server.h"
#include "atsvr_comm.h"
#include "string.h"
#include "stdio.h"
#include "mem_pub.h"

#include "atsvr_misc.h"
#include "atsvr_port.h"
#include "at_server.h"
#include "atsvr_net_cmd.h"
#include "wlan_ui_pub.h"
#include "network_app.h"
#include "drv_model_pub.h"
#include "atsvr_wlan.h"
#include "dns.h"
#include "rtc.h"
#include "ntp.h"
#if CFG_USE_TCPUDP
static int g_conmode = 1; //0:单连接;1:多连接

extern int create_network_task();
extern int network_link_status();

static void _atsvr_BIPSTATUS_handle(int argc, char * *argv)
{
	if (argc != 1)
	{
		atsvr_cmd_rsp_error();
		return;
	}

	network_link_status();
}

char udp_remoteip[16];
int udp_port;

static void _atsvr_BIPSTART_handle(int argc, char * *argv)
{
	pm_socket_info_t rcvmsg;

	if (g_conmode)
	{
		if (argc < 5)
		{
			atsvr_cmd_rsp_error();
			return;
		}

		rcvmsg.linkid		= atoi(argv[1]);

		if ((strcmp(argv[2], "TCP") == 0) || (strcmp(argv[2], "tcp") == 0))
		{
			rcvmsg.type 		= NETWORK_TCP;
		}
		else if ((strcmp(argv[2], "UDP") == 0) || (strcmp(argv[2], "udp") == 0))
		{
			rcvmsg.type 		= NETWORK_UDP;
		}
		else if ((strcmp(argv[2], "TLS") == 0) || (strcmp(argv[2], "tls") == 0))
		{
			rcvmsg.type 		= NETWORK_TLS;
		}
		else if ((strcmp(argv[2], "DTLS") == 0) || (strcmp(argv[2], "dtls") == 0))
		{
			rcvmsg.type 		= NETWORK_DTLS;
		}
		else
		{
			ATSVRLOG("argc:2 error\r\n");
			atsvr_cmd_rsp_error();
			return;
		}

		if (judge_the_string_is_ipv4_string(argv[3]) == 0)
		{
			strcpy(rcvmsg.remoteip, argv[3]);
		}
		else
		{
			ATSVRLOG("argc:3 error\r\n");
			atsvr_cmd_rsp_error();
			return;
		}

		rcvmsg.port 		= atoi(argv[4]);
		strncpy(udp_remoteip, argv[3],sizeof(udp_remoteip));
		udp_port = atoi(argv[4]);
		if (argc >= 6)
		{
			switch (rcvmsg.type)
			{
				case NETWORK_TCP:
				case NETWORK_TLS:
					rcvmsg.keepalive = atoi(argv[5]);
					break;

				case NETWORK_UDP:
				case NETWORK_DTLS:
					rcvmsg.localport = atoi(argv[5]);
					break;
			}
		}

		if (argc >= 7)
		{
			if (judge_the_string_is_ipv4_string(argv[6]) == 0)
			{
				switch (rcvmsg.type)
				{
					case NETWORK_TCP:
					case NETWORK_TLS:
						strcpy(rcvmsg.localip, argv[6]);
						break;

					case NETWORK_UDP:
					case NETWORK_DTLS:
						rcvmsg.udpmode = atoi(argv[6]);
						break;
				}
			}
			else
			{
				ATSVRLOG("argc:7 error\r\n");
				atsvr_cmd_rsp_error();
				return;

			}
		}

		if (argc >= 8)
		{
			if (rcvmsg.type == NETWORK_UDP || rcvmsg.type == NETWORK_DTLS)
			{

				if (judge_the_string_is_ipv4_string(argv[7]) == 0)
				{
					strcpy(rcvmsg.localip, argv[7]);
				}
				else
				{
					ATSVRLOG("argc:9 error\r\n");
					atsvr_cmd_rsp_error();
					return;
				}
			}
			else
			{

				ATSVRLOG("argc:9 error\r\n");
				atsvr_cmd_rsp_error();
				return;
			}
		}

		if (rcvmsg.linkid > 5 || isconnected_network(rcvmsg.linkid))
		{
			ATSVRLOG("linkid error\r\n");
			atsvr_cmd_rsp_error();
			return;
		}
		else
		{
			rtos_push_to_queue(&network_msg_que, &rcvmsg, BEKEN_NO_WAIT);
		}
	}
	else
	{
		if (argc < 4)
		{
			atsvr_cmd_rsp_error();
			return;
		}

		if ((strcmp(argv[1], "TCP") == 0) || (strcmp(argv[1], "tcp") == 0))
		{
			rcvmsg.type 		= NETWORK_TCP;
		}
		else if ((strcmp(argv[1], "UDP") == 0) || (strcmp(argv[1], "udp") == 0))
		{
			rcvmsg.type 		= NETWORK_UDP;
		}
		else if ((strcmp(argv[1], "TLS") == 0) || (strcmp(argv[1], "tls") == 0))
		{
			rcvmsg.type 		= NETWORK_TLS;
		}
		else if ((strcmp(argv[1], "DTLS") == 0) || (strcmp(argv[1], "dtls") == 0))
		{
			rcvmsg.type 		= NETWORK_DTLS;
		}
		else
		{
			ATSVRLOG("\r\nargc:2 error\r\n");
			atsvr_cmd_rsp_error();
			return;
		}

		if (judge_the_string_is_ipv4_string(argv[2]) == 0)
		{
			strcpy(rcvmsg.remoteip, argv[2]);
		}
		else
		{
			ATSVRLOG("argc:3 error\r\n");
			atsvr_cmd_rsp_error();
			return;
		}

		rcvmsg.port 		= atoi(argv[3]);
		if (argc >= 5)
		{
			switch (rcvmsg.type)
			{
				case NETWORK_TCP:
				case NETWORK_TLS:
					rcvmsg.keepalive = atoi(argv[4]);
					break;

				case NETWORK_UDP:
				case NETWORK_DTLS:
					rcvmsg.localport = atoi(argv[4]);
					break;
			}
		}

		if (argc >= 6)
		{
			if (judge_the_string_is_ipv4_string(argv[5]) == 0)
			{
				switch (rcvmsg.type)
				{
					case NETWORK_TCP:
					case NETWORK_TLS:
						strcpy(rcvmsg.localip, argv[5]);
						break;

					case NETWORK_UDP:
					case NETWORK_DTLS:
						rcvmsg.udpmode = atoi(argv[5]);
						break;
				}
			}
			else
			{
				ATSVRLOG("argc:6 error\r\n");
				atsvr_cmd_rsp_error();
				return;

			}
		}

		if (argc >= 7)
		{
			if (rcvmsg.type == NETWORK_UDP || rcvmsg.type == NETWORK_DTLS)
			{
				if (judge_the_string_is_ipv4_string(argv[6]) == 0)
				{
					strcpy(rcvmsg.localip, argv[6]);
				}
				else
				{
					ATSVRLOG("argc:6 error\r\n");
					atsvr_cmd_rsp_error();
					return;
				}
			}
		}

		rcvmsg.linkid		= 0;

		if (rcvmsg.linkid > 5 || isconnected_network(rcvmsg.linkid))
		{
			ATSVRLOG("linkid error\r\n");
			atsvr_cmd_rsp_error();
			return;
		}
		else
		{
			rtos_push_to_queue(&network_msg_que, &rcvmsg, BEKEN_NO_WAIT);
		}
	}
	return;
}


static void _atsvr_BIPCLOSE_handle(int argc, char * *argv)
{
	int linkid=0;

	if(g_conmode){
		if (argc < 2)
		{
			atsvr_cmd_rsp_error();
			return;
		}

		linkid = atoi(argv[1]);

		if (linkid <= 6 && isconnected_network(linkid))
		{
			network_stop_thread(linkid);
			atsvr_cmd_rsp_ok();
		}
		else
		{
			atsvr_cmd_rsp_error();
		}
	}else{

		if (linkid <= 6 && isconnected_network(linkid))
		{
			network_stop_thread(linkid);
			atsvr_cmd_rsp_ok();
		}
		else
		{
			atsvr_cmd_rsp_error();
		}
	}

	return;
}

static void _atsvr_BIPMODE_handle(int argc, char * *argv)
{
	uint8			mode;
	int 			n	= 0;
	char			resultbuf[200];

	if (argc == 1)
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+CIPMODE:%d\r\n",
			g_env_param.net_transmission_mode);
		n					+= snprintf(resultbuf + n, sizeof(resultbuf) -n, "%s", ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf, n);
	}
	else if (argc == 2)
	{
		mode				= atoi(argv[1]);
		g_env_param.net_transmission_mode = mode ? ATSVR_PASSTHROUGH_MODE: ATSVR_COMMON_MODE;

		if (g_env_param.sysstore)
		{
			write_env_to_flash(TAG_SYSSTORE_OFFSET, sizeof(ENV_PARAM), (uint8 *) &g_env_param);
		}

		atsvr_cmd_rsp_ok();
	}
	else
	{
		atsvr_cmd_rsp_error();
		return;
	}

	return;
}

static void _atsvr_CIPMUX_Query_handle(int argc, char * *argv)
{
	int 			n	= 0;
	char			resultbuf[200];

	if (argc == 1)
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+CIPMUX:%d\r\n", g_conmode);
		n					+= snprintf(resultbuf + n, sizeof(resultbuf) -n, "%s", ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf, n);
	}
}


static void _atsvr_CIPMUX_handle(int argc, char * *argv)
{
	if (argc == 2)
	{
		g_conmode			= atoi(argv[1]);
		if (g_conmode == 0 || g_conmode == 1)
		{
			atsvr_cmd_rsp_ok();
			return;
		}
	}
	else
	{
		atsvr_cmd_rsp_error();
		return;
	}
	return;
}

static void _atsvr_BIPSEND_handle(int argc, char * *argv)
{
	AT_FUNC_ENTRY;

	char			revbuf[1024];
	char			resultbuf[200];
	int 			revlen = 0;
	int 			len = 0;
	int 			linkid = 0;
	int 			n	= 0;

	if (argc == 1) //enter passthrough mode
	{
		ATSVRLOG("1argc:%d\r\n", argc);

		if (network_connected_num() == 1 && g_env_param.net_transmission_mode == ATSVR_PASSTHROUGH_MODE)
		{
			g_atsvr_status.network_transfer_mode = ATSVR_PASSTHROUGH_MODE;
			atsvr_cmd_rsp_passthrough();
		}
		else
		{
			atsvr_cmd_rsp_error();
			return;
		}

	}
	else if (argc == 2)
	{

		if (g_conmode == 0)
		{ //单连接

			len 				= atoi(argv[1]);

			if (isconnected_network(0) && network_check_connect_type(0, NETWORK_TCP))
			{
				g_atsvr_status.network_transfer_mode = ATSVR_PASSTHROUGH_REV_MODE;
				atsvr_cmd_rsp_passthrough();

				revlen				= atsvr_input_msg_get(revbuf, len);
				g_atsvr_status.network_transfer_mode = ATSVR_COMMON_MODE;

				if (network_tcp_send_msg(0, (unsigned char *) &revbuf[0], len) != len)
				{
					atsvr_output_msg("SEND ERROR\r\n", strlen("SEND ERROR\r\n"));
					return;
				}
				else
				{
					atsvr_output_msg("SEND OK\r\n", strlen("SEND OK\r\n"));
				}

				if (revlen > len)
				{
					int 			n	= 0;
					char			resultbuf[100];

					n					= snprintf(resultbuf, sizeof(resultbuf), "%s\r\n", &revbuf[len]);
					atsvr_output_msg(resultbuf, n);
					atsvr_cmd_rsp_busy();
				}

				return;
			}
			else
			{
				atsvr_cmd_rsp_error();
				return;
			}
		}
		else
		{

			atsvr_cmd_rsp_error();
			return;
		}

	}
	else if (argc == 3)
	{
		linkid				= atoi(argv[1]);
		len 				= atoi(argv[2]);

		if (isconnected_network(linkid) && network_check_connect_type(linkid, NETWORK_TCP))
		{
			g_atsvr_status.network_transfer_mode = ATSVR_PASSTHROUGH_REV_MODE;
			atsvr_cmd_rsp_passthrough();

			// revlen = atsvr_input_msg_get_block(revbuf,len);
			revlen				= atsvr_input_msg_get(revbuf, len);
			g_atsvr_status.network_transfer_mode = ATSVR_COMMON_MODE;

			if (network_tcp_send_msg(linkid, (unsigned char *) &revbuf[0], len) != len)
			{
				//atsvr_cmd_rsp_error();
				atsvr_output_msg("SEND ERROR\r\n", strlen("SEND ERROR\r\n"));
				return;
			}
			else
			{
				atsvr_output_msg("SEND OK\r\n", strlen("SEND OK\r\n"));
			}

			if (revlen > len)
			{
				n					= snprintf(resultbuf, sizeof(resultbuf), "%s\r\n", &revbuf[len]);
				atsvr_output_msg(resultbuf, n);
				atsvr_cmd_rsp_busy();
			}

			return;
		}
		else
		{
			atsvr_cmd_rsp_error();
			return;
		}

	}
	else if (argc == 4)
	{

		if (g_conmode != 0)
		{
			atsvr_cmd_rsp_error();
			return;

		}

		len 				= atoi(argv[1]);
		char *			retmoteip = argv[2];
		int 			port = atoi(argv[3]);

		g_atsvr_status.network_transfer_mode = ATSVR_PASSTHROUGH_REV_MODE;
		atsvr_cmd_rsp_passthrough();

		revlen				= atsvr_input_msg_get(revbuf, len);
		g_atsvr_status.network_transfer_mode = ATSVR_COMMON_MODE;

		if (network_udp_send_msg(linkid, (unsigned char *) &revbuf[0], len, retmoteip, port) != len)
		{
			atsvr_output_msg("SEND ERROR\r\n", strlen("SEND ERROR\r\n"));
			return;
		}
		else
		{
			atsvr_output_msg("SEND OK\r\n", strlen("SEND OK\r\n"));
		}

		if (revlen > len)
		{

			n					= snprintf(resultbuf, sizeof(resultbuf), "%s\r\n", &revbuf[len]);
			atsvr_output_msg(resultbuf, n);
			atsvr_cmd_rsp_busy();
		}

	}
	else if (argc == 5)
	{
		if (g_conmode != 1)
		{
			atsvr_cmd_rsp_error();
			return;
		}

		linkid				= atoi(argv[1]);
		len 				= atoi(argv[2]);
		char *			retmoteip = argv[3];
		int 			port = atoi(argv[4]);

		g_atsvr_status.network_transfer_mode = ATSVR_PASSTHROUGH_REV_MODE;
		atsvr_cmd_rsp_passthrough();

		revlen				= atsvr_input_msg_get(revbuf, len);
		g_atsvr_status.network_transfer_mode = ATSVR_COMMON_MODE;

		if (network_udp_send_msg(linkid, (unsigned char *) &revbuf[0], len, retmoteip, port) != len)
		{
			atsvr_output_msg("SEND ERROR\r\n", strlen("SEND ERROR\r\n"));
			return;
		}
		else
		{
			atsvr_output_msg("SEND OK\r\n", strlen("SEND OK\r\n"));
		}

		if (revlen > len)
		{

			n					= snprintf(resultbuf, sizeof(resultbuf), "%s\r\n", &revbuf[len]);
			atsvr_output_msg(resultbuf, n);
			atsvr_cmd_rsp_busy();
		}

	}

	return;
}

extern int at_ping(char* target_name, uint32_t times, size_t size);
static void _atsvr_PING_handle(int argc, char * *argv)
{
	if (argc != 2)
	{
		atsvr_cmd_rsp_error();
		return;
	}

	int 			ret = at_ping(argv[1], 4, 0);

	if (ret == -1)
	{
		atsvr_cmd_rsp_error();
		return;
	}

	atsvr_cmd_rsp_ok();

	return;
}


void _atsvr_at_BIPPSK_Query_command(int argc, char * *argv)
{
	int 			n	= 0;
	char			resultbuf[200];

	if (argc == 1)
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+CIPSSLCPSK:%d,%s,%s\r\n",
			g_env_param.pskinfo.linkid, g_env_param.pskinfo.psk, g_env_param.pskinfo.hint);
		n					+= snprintf(resultbuf + n, sizeof(resultbuf) -n, "%s", ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf, n);
	}
	else
	{
		atsvr_cmd_rsp_error();
		return;
	}
}


void _atsvr_at_BIPPSK_command(int argc, char * *argv)
{
	ATSVRLOG("entr _atsvr_at_BIPPSK_command,argc:%d\r\n",argc);

	if (argc == 4)
	{
		int 			linkid = atoi(argv[1]);

		if (linkid > 5)
		{
			atsvr_cmd_rsp_error();
			return;
		}

		g_env_param.pskinfo.linkid = linkid;

		if (strlen(argv[2]) > 0)
		{
			memset(&g_env_param.pskinfo.psk,0,MAX_SIZE_OF_DEVICE_SECRET +1);
			strcpy(g_env_param.pskinfo.psk, argv[2]);
			ATSVRLOG("hint:%s\r\n", g_env_param.pskinfo.psk);
		}
		else
		{
			atsvr_cmd_rsp_error();
			return;
		}

		if (strlen(argv[3]) > 0)
		{
			memset(&g_env_param.pskinfo.hint,0,MAX_SIZE_OF_DEVICE_SECRET +1);
			strcpy(g_env_param.pskinfo.hint, argv[3]);
			ATSVRLOG("hint:%s\r\n", g_env_param.pskinfo.hint);
		}
		else
		{
			atsvr_cmd_rsp_error();
			return;
		}

		if (g_env_param.sysstore)
		{
			write_env_to_flash(TAG_SYSSTORE_OFFSET, sizeof(ENV_PARAM), (uint8 *) &g_env_param);
		}

		atsvr_cmd_rsp_ok();
	}
	else
	{
		atsvr_cmd_rsp_error();
		return;
	}
}

void _atsvr_at_BIPSNTPCFG_Query_command(int argc, char * *argv)
{
	int 			n	= 0;
	char			resultbuf[200];

	if (argc == 1)
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+CIPSNTPCFG:%d,%d,%s\r\n",
			g_env_param.ntpinfo.enable, g_env_param.ntpinfo.timezone, g_env_param.ntpinfo.hostname);
		n					+= snprintf(resultbuf + n, sizeof(resultbuf) -n, "%s", ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf, n);
	}
	else
	{

		atsvr_cmd_rsp_error();
		return;
	}
}

void _atsvr_at_BIPSNTPCFG_command(int argc, char * *argv)
{
	if (argc >= 4)
	{
		g_env_param.ntpinfo.enable = atoi(argv[1]);

		if (g_env_param.ntpinfo.enable == 0)
		{

			if (g_env_param.sysstore)
			{
				write_env_to_flash(TAG_SYSSTORE_OFFSET, sizeof(ENV_PARAM), (uint8 *) &g_env_param);
			}

			atsvr_cmd_rsp_ok();
			return;
		}

		g_env_param.ntpinfo.timezone = strtol(argv[2], NULL, 10);

		strcpy(g_env_param.ntpinfo.hostname, argv[3]);

		if (g_env_param.sysstore)
		{
			write_env_to_flash(TAG_SYSSTORE_OFFSET, sizeof(ENV_PARAM), (uint8 *) &g_env_param);
		}

		ATSVRLOG("eable:%d,timezone:%d,hostname:%s\r\n", g_env_param.ntpinfo.enable, g_env_param.ntpinfo.timezone,
			g_env_param.ntpinfo.hostname);

		ntp_info_update(g_env_param.ntpinfo.timezone, g_env_param.ntpinfo.hostname);

		atsvr_cmd_rsp_ok();
		return;

	}
	else
	{
		atsvr_cmd_rsp_error();
		return;
	}

}

void _atsvr_at_BIPSNTPTIME_command(int argc, char * *argv)
{
	if (argc == 1)
	{
		int 			n	= 0;
		char			resultbuf[200];
		time_t			now;

		struct tm * time_now;
		sddev_control(SOFT_RTC_DEVICE_NAME, RT_DEVICE_CTRL_RTC_GET_TIME, &now);
		time_now			= localtime(&now);

		n					= snprintf(resultbuf, sizeof(resultbuf), "+CIPSNTPTIME:%d-%d-%d %d-%d-%d\r\n",
			time_now->tm_year + 1900, time_now->tm_mon + 1, time_now->tm_mday, time_now->tm_hour, time_now->tm_min,
			time_now->tm_sec);
		n					+= snprintf(resultbuf + n, sizeof(resultbuf) -n, "%s", ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf, n);
		return;
	}
	else
	{

		atsvr_cmd_rsp_error();
		return;
	}
}

extern void *zalloc(size_t size);
int get_domainip(const char * hname, char * hostip)
{
	const size_t	hstbuflen = 1024;
	char *			tmphstbuf = NULL;

	if (hostip == NULL)
		return 0;

	tmphstbuf			= (char *) zalloc(sizeof(char) *hstbuflen);

	if (tmphstbuf == NULL)
		return 0;

	struct hostent * pent = lwip_gethostbyname(hname);

	if (pent == NULL || pent->h_addr == NULL)
	{
		free(tmphstbuf);
		return 0;
	}
	strcpy(hostip, inet_ntoa(* ((struct in_addr *) pent->h_addr)));
	free(tmphstbuf);
	return 1;
}

void _atsvr__atsvr_at_BIPDOMAIN_command(int argc, char * *argv)
{
	if (argc >= 2)
	{
		char			remoteip[16];

		if (get_domainip(argv[1], remoteip))
		{
			int 			n	= 0;
			char			resultbuf[200];

			if (judge_the_string_is_ipv4_string(remoteip) == -1)
			{
				atsvr_cmd_rsp_error();
				return;
			}

			n					= snprintf(resultbuf, sizeof(resultbuf), "+CIPDOMAIN:%s\r\n", remoteip);
			n					+= snprintf(resultbuf + n, sizeof(resultbuf) -n, "%s", ATSVR_CMD_RSP_SUCCEED);
			atsvr_output_msg(resultbuf, n);
			return;
		}
	}
	else
	{
		atsvr_cmd_rsp_error();
		return;
	}
}

void _atsvr__atsvr_at_BIPDNS_command(int argc, char * *argv)
{
	int 			enable = 0;

	if (argc == 2)
	{

		enable				= atoi(argv[1]);

		if (enable == 0)
		{
			g_env_param.dnsinfo.enable = 0;
			strcpy(g_env_param.dnsinfo.dns1, "208.67.222.222");
			atsvr_cmd_rsp_ok();
		}
		else
		{

			atsvr_cmd_rsp_error();
			return;
		}

	}
	else if (argc >= 3)
	{
		ip_addr_t		dns_array[3];
		int 			n	= 0;
		enable				= atoi(argv[1]);

		if (enable == 1)
		{
			if (judge_the_string_is_ipv4_string(argv[2]) == 0)
			{
				strcpy(g_env_param.dnsinfo.dns1, argv[2]);
				ipaddr_aton(g_env_param.dnsinfo.dns1, &dns_array[0]);
				dns_setserver(n, &dns_array[0]);
				n++;
			}
			else
			{
				atsvr_cmd_rsp_error();
				return;
			}
		}

		if (argc == 4)
		{

			if (judge_the_string_is_ipv4_string(argv[3]) == 0)
			{

				strcpy(g_env_param.dnsinfo.dns2, argv[3]);
				ipaddr_aton(g_env_param.dnsinfo.dns2, &dns_array[1]);
				dns_setserver(n, &dns_array[1]);
				n++;
			}
			else
			{
				atsvr_cmd_rsp_error();
				return;
			}
		}


		if (argc == 5)
		{

			if (judge_the_string_is_ipv4_string(argv[4]) == 0)
			{
				strcpy(g_env_param.dnsinfo.dns3, argv[4]);
				ipaddr_aton(g_env_param.dnsinfo.dns2, &dns_array[2]);
				dns_setserver(n, &dns_array[2]);
				n++;
			}
			else
			{
				atsvr_cmd_rsp_error();
				return;
			}
		}

		if (g_env_param.sysstore)
			write_env_to_flash(TAG_SYSSTORE_OFFSET, sizeof(ENV_PARAM), (uint8 *) &g_env_param);

		atsvr_cmd_rsp_ok();
	}
}

const struct _atsvr_command _atsvc_netcmds_table[] =
{
	_ATSVR_CMD_HADLER("AT+CIPSTATUS?", "AT+CIPSTATUS?", _atsvr_BIPSTATUS_handle),
	_ATSVR_CMD_HADLER("AT+CIPSTART", "AT+CIPSTART", _atsvr_BIPSTART_handle),
	_ATSVR_CMD_HADLER("AT+CIPCLOSE", "AT+CIPCLOSE", _atsvr_BIPCLOSE_handle),
	_ATSVR_CMD_HADLER("AT+CIPMODE", "AT+CIPMODE=<mode>", _atsvr_BIPMODE_handle),
	_ATSVR_CMD_HADLER("AT+CIPMUX?", "AT+CIPMUX?", _atsvr_CIPMUX_Query_handle),
	_ATSVR_CMD_HADLER("AT+CIPMUX", "AT+CIPMUX=<mode>", _atsvr_CIPMUX_handle),
	_ATSVR_CMD_HADLER("AT+CIPSEND", "AT+CIPSEND=[<linkid>,]<length>,[<remotehost>,<remoteport>]",
		_atsvr_BIPSEND_handle),
	_ATSVR_CMD_HADLER("AT+PING", "AT+PING=<host>", _atsvr_PING_handle),
	_ATSVR_CMD_HADLER("AT+CIPSSLCPSK", "AT+CIPSSLCPSK=<linkid>,<psk_key>,<hint>", _atsvr_at_BIPPSK_command),
	_ATSVR_CMD_HADLER("AT+CIPSSLCPSK?", "AT+CIPSSLCPSK?", _atsvr_at_BIPPSK_Query_command),
	_ATSVR_CMD_HADLER("AT+CIPSNTPCFG?", "AT+CIPSNTPCFG?", _atsvr_at_BIPSNTPCFG_Query_command),
	_ATSVR_CMD_HADLER("AT+CIPSNTPCFG", "AT+CIPSNTPCFG=<enable>,<timezone>,<SNTPserver1>,<SNTPserver2>,<SNTPserver3>,",
		_atsvr_at_BIPSNTPCFG_command),
	_ATSVR_CMD_HADLER("AT+CIPSNTPTIME?", "AT+CIPSNTPTIME?", _atsvr_at_BIPSNTPTIME_command),
	_ATSVR_CMD_HADLER("AT+CIPDOMAIN", "AT+CIPDOMAIN=<domain>[,<ip network>]", _atsvr__atsvr_at_BIPDOMAIN_command),
	_ATSVR_CMD_HADLER("AT+CIPDNS", "AT+CIPDNS=<enable>[,<dns1>,<dns2>,<dns3>]", _atsvr__atsvr_at_BIPDNS_command),
};

void atsvr_network_init()
{
	atsvr_register_commands(_atsvc_netcmds_table, sizeof(_atsvc_netcmds_table) / sizeof(struct _atsvr_command));
	create_network_task();
}
#endif
