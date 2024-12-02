
#include "_atsvr_func.h"
#include "_at_server.h"
#include "atsvr_comm.h"
#include "string.h"
#include "stdio.h"
#include "atsvr_misc.h"
#include "atsvr_port.h"
#include "uart_pub.h"
#include "at_server.h"


#include <stdlib.h>
#include "qcloud_iot_export_mqtt.h"
#include "qcloud_at_mqtt.h"
#include "atsvr_error.h"

#include "rtos_pub.h"
#if CFG_USE_MQTT
MQTT_CON_INFO	g_mqttmconcfg;


static void _atsvr_MQTTCONNSTATE_handle(int argc, char * *argv)
{
	char			resultbuf[200];
	int 			n	= 0;
	MQTTInitParams	conn_params;

	if (ATSVR_RET_SUCCESS == get_mqtt_conn_parameters(&conn_params))
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTCONNSTATE:%s,%u,%u,%u,%u\r\n",
			 get_mqtt_connect_state () > 0 ? "connected": "disconnect", (unsigned int) conn_params.command_timeout,
			(unsigned int) conn_params.keep_alive_interval_ms / 1000, conn_params.clean_session,
			 conn_params.auto_connect_enable);
		n					+= snprintf(resultbuf + n, sizeof(resultbuf) -n, "%s", ATSVR_CMD_RSP_SUCCEED);
		atsvr_output_msg(resultbuf, n);
	}
	else
	{
		atsvr_cmd_rsp_error();
	}
}

static void _atsvr_MQTTCONN_handle(int argc, char * *argv)
{
	if (argc < 5)
	{
		atsvr_cmd_rsp_error();
		return;
	}
	else if (argc == 5)
	{

		char			host[128];
		int 			linkid;
		int 			port;
		int 			reconnect;
		char			resultbuf[200];
		int 			n	= 0;

		//	int keepalive;
		linkid				= atoi(argv[1]);

		char *			tmp = argv[2];
		int 			len = strlen(tmp);

		if (len == 0 || len > 64)
		{
			atsvr_cmd_rsp_error();
			return;
		}
		else
		{
			strcpy(host, argv[2]);
		}


		port				= atoi(argv[3]);

		if (port > 65535 || port < 0)
		{
			atsvr_cmd_rsp_error();
			return;
		}

		reconnect			= atoi(argv[4]);

		if (reconnect > 1 || reconnect < 0)
		{
			atsvr_cmd_rsp_error();
			return;
		}


		atsvr_cmd_rsp_ok();

		ATSVRLOG("linkid:%d,host:%s,port:%d,reconnect:%d\r\n", linkid,host, port, reconnect);

		/* command execution */
		MQTTInitParams	conn_params = DEFAULT_MQTTINIT_PARAMS;

		//sDevInfo		 rdevinfo;
		conn_params.command_timeout = 5 * 1000;
		conn_params.keep_alive_interval_ms = g_mqttmconcfg.keepalive * 1000;
		conn_params.clean_session = g_mqttmconcfg.clean_session;
		conn_params.auto_connect_enable = reconnect;


		conn_params.product_id = g_env_param.deviceinfo.product_id;
		conn_params.device_name = g_env_param.deviceinfo.device_name;

#ifndef AUTH_MODE_CERT
		conn_params.device_secret = g_env_param.pskinfo.psk;
#endif

		conn_params.region	= g_env_param.deviceinfo.region;
		conn_params.conntype = g_mqttmconcfg.contype;
		conn_params.clientid = g_mqttmconcfg.clientid;

		memset(conn_params.host, 0, MAX_HOSTNAME_SIZE_LEN);
		strcpy(conn_params.host, host);
		conn_params.port	= port;

		int 			err_code = do_mqtt_connect(&conn_params);

		if (err_code)
		{
			n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTCONN:ERROR,%d\r\n", err_code);
			atsvr_output_msg(resultbuf, n);

		}
		else
		{
			n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTCONN:OK\r\n");
			atsvr_output_msg(resultbuf, n);
		}
	}
}


static void _atsvr_at_MQTTDISCONN_command(int argc, char * *argv)
{


	int 			err_code, n = 0;
	char			resultbuf[200];

	if (argc != 2)
	{

		atsvr_cmd_rsp_error();
		return;
	}

	//int 			linkid = atoi(argv[1]);

	err_code			= do_mqtt_disconnect();

	if (err_code != ATSVR_RET_SUCCESS)
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTDISCONN:%d\r\n", err_code);
		atsvr_output_msg(resultbuf, n);
	}
	else
		atsvr_cmd_rsp_ok();
}


static void _atsvr_MQTTPUB_handle(int argc, char * *argv)
{
	char			resultbuf[200];
	int 			n	= 0;

	/* check state */
	if (!get_mqtt_connect_state())
	{
		ATSVRLOG("MQTT is NOT connected\n");
		atsvr_cmd_rsp_error();
		return;
	}

	/* parameters parsing */
	int32_t 		qos;
	uint8_t *		topic = NULL;
	uint8_t *		payload = NULL;
	uint8			retain;

	if (argc != 6)
	{
		ATSVRLOG("argc is err %d", argc);
		atsvr_cmd_rsp_error();
		return;
	}

//	int 			linkid = atoi(argv[1]);

	topic				= (uint8_t *) argv[2];
	size_t			len = strlen((char *) topic);

	if (len == 0 || len > MAX_SIZE_OF_CLOUD_TOPIC)
	{
		ATSVRLOG("topic name oversize\n");
		atsvr_cmd_rsp_error();
		return;
	}

	payload 			= (uint8_t *) argv[3];
	len 				= strlen((char *) payload);

	if (len == 0)
	{

		ATSVRLOG("payload data length error\n");
		atsvr_cmd_rsp_error();
		return;
	}

	qos 				= atoi(argv[4]);

	if (qos < QOS0 || qos > QOS2)
	{
		ATSVRLOG("qos para error\n");
		atsvr_cmd_rsp_error();
		return;
	}

	retain				= atoi(argv[5]);

	if (retain < 0 || retain > 1)
	{
		ATSVRLOG("qos para error\n");
		atsvr_cmd_rsp_error();
		return;

		;

	}

	//atsvr_cmd_rsp_ok();
	int 			err_code = do_mqtt_pub_msg((char *) topic, qos, (char *) payload, len, retain);

	if (err_code)
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTPUB:ERROR,%d\r\n", err_code);
		atsvr_output_msg(resultbuf, n);
	}
	else
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTPUB:OK\r\n");
		atsvr_output_msg(resultbuf, n);
	}
}


static void _atsvr_MQTTPUBRAW_handle(int argc, char * *argv)
{

	char			resultbuf[200];
	int 			n	= 0;

	/* check state */
	if (!get_mqtt_connect_state())
	{
		ATSVRLOG("MQTT is NOT connected\n");
		atsvr_cmd_rsp_error();
		return;
	}

	int32_t 		qos, payload_len;
	uint8_t *		topic = NULL;
	int 			retain;

	if (argc != 6)
	{
		ATSVRLOG("argc	is err %d", argc);
		atsvr_cmd_rsp_error();
		return;
	}

	//int 			linkid = atoi(argv[1]);

	topic				= (uint8_t *) argv[2];
	size_t			len = strlen((char *) topic);

	if (len == 0 || len > MAX_SIZE_OF_CLOUD_TOPIC)
	{
		ATSVRLOG("topic name oversize\n");
		atsvr_cmd_rsp_error();
		return;
	}

	payload_len 		= atoi(argv[3]);

	if (payload_len <= 0 || payload_len > MAX_SIZE_OF_PUBL_PAYLOAD)
	{
		ATSVRLOG("topic payload oversize\n");
		atsvr_cmd_rsp_error();
		return;
	}

	qos 				= atoi(argv[4]);

	if (qos < QOS0 || qos > QOS2)
	{
		ATSVRLOG("Invalid QoS level %d", qos);
		atsvr_cmd_rsp_error();
		return;
	}

	retain				= atoi(argv[5]);

	if (retain < 0 || retain > 1)
	{
		ATSVRLOG("Invalid retain level %d", retain);
		atsvr_cmd_rsp_error();
		return;
	}

	char *			payload_buf = malloc(MAX_SIZE_OF_PUBL_PAYLOAD + 1);

	if (payload_buf == NULL)
	{
		ATSVRLOG("memory malloc failed\n");
		atsvr_cmd_rsp_error();
		return;
	}

	atsvr_cmd_rsp_ok();
	memset(payload_buf, 0, MAX_SIZE_OF_PUBL_PAYLOAD + 1);

	g_atsvr_status.network_transfer_mode = ATSVR_PASSTHROUGH_REV_MODE;
	atsvr_cmd_rsp_passthrough();

	//int revlen = atsvr_input_msg_get_block(payload_buf,payload_len);
	int 			revlen = atsvr_input_msg_get(payload_buf, payload_len);

	g_atsvr_status.network_transfer_mode = ATSVR_COMMON_MODE;

	if (revlen == payload_len)
	{

		ATSVRLOG("rev data len OK\n");
	//	atsvr_output_msg("+MQTTPUBRAW:OK\r\n", strlen("+MQTTPUBRAW:OK\r\n"));
	}
	else
	{
		ATSVRLOG("rev data len failed\n");
		atsvr_cmd_rsp_error();
		free(payload_buf);
		return;
	}

	/* command execution */
	int 			err_code = do_mqtt_pub_msg((char *) topic, qos, (char *) payload_buf, payload_len, retain);

	if (err_code)
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTPUBRAW:ERROR,%d\r\n", err_code);
		atsvr_output_msg(resultbuf, n);
	}
	else
	{
		n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTPUBRAW:OK\r\n");
		atsvr_output_msg(resultbuf, n);
	}

	free(payload_buf);
}


static void _atsvr_MQTTSUB_Query_handle(int argc, char * *argv)
{

	int 			ret = 0;

	ret 				= get_mqtt_sub_list();

	if (ret == ATSVR_ERR_INVAL)
	{
		atsvr_cmd_rsp_error();
		return;
	}

	atsvr_cmd_rsp_ok();
	return;
}


static void _atsvr_MQTTSUB_handle(int argc, char * *argv)
{
	/* check state */
	if (!get_mqtt_connect_state())
	{
		ATSVRLOG("MQTT is NOT connected\n");
		atsvr_cmd_rsp_error();
		return;
	}

	int32_t 		qos;
	uint8_t *		topic = NULL;

	if (argc != 4)
	{
		ATSVRLOG("argc is err，argc:%d\r\n", argc);

		atsvr_cmd_rsp_error();
		return;
	}

	//sscanf(argv[1],"\"%[^\"]\"",argv[1]);
	//int 			linkid = atoi(argv[1]);

	topic				= (uint8_t *) argv[2];
	size_t			len = strlen((char *) topic);

	if (len == 0 || len > MAX_SIZE_OF_CLOUD_TOPIC)
	{
		ATSVRLOG("topic name oversize\n");
		atsvr_cmd_rsp_error();
		return;
	}

	qos 				= atoi(argv[3]);

	if (qos < QOS0 || qos > QOS2)
	{
		ATSVRLOG("Invalid QoS level %d", qos);
		atsvr_cmd_rsp_error();
		return;
	}

	// atsvr_cmd_rsp_ok();

	/* command execution */
	int 			err_code = do_mqtt_sub_msg((char *) topic, qos);

	if (err_code)
	{
		atsvr_cmd_rsp_error();
	}
	else
	{
		atsvr_cmd_rsp_ok();
	}
}


static void _atsvr_at_MQTTUNSUB_command(int argc, char * *argv)
{
	/* check state */
	if (!get_mqtt_connect_state())
	{
		ATSVRLOG("MQTT is NOT connected\n");
		atsvr_cmd_rsp_error();
		return;
	}

	uint8_t *		topic = NULL;

	if (argc != 3)
	{
		ATSVRLOG("argc is err");
		atsvr_cmd_rsp_error();
		return;
	}

	//int 			linkid = atoi(argv[1]);

	topic				= (uint8_t *) argv[2];
	size_t			len = strlen((char *) topic);

	if (len == 0 || len > MAX_SIZE_OF_CLOUD_TOPIC)
	{
		ATSVRLOG("topic name oversize\n");
		atsvr_cmd_rsp_error();
		return;
	}


	/* command execution */
	int 			err_code = do_mqtt_unsub_msg((char *) topic);

	if (err_code)
	{
		atsvr_cmd_rsp_error();
		return;
	}
	else
	{
		os_printf("MQTTUNSUB_command ok\r\n");
		atsvr_cmd_rsp_ok();
		return;

	}
}


static void _atsvr_MQTTUSERCFG_handle(int argc, char * *argv)
{
	if (argc < 6)
	{
		ATSVRLOG("para number error\n");
		atsvr_cmd_rsp_error();
		return;

	}
	else if (argc >= 6)
	{


		g_mqttmconcfg.linkid = atoi(argv[1]);

		size_t			len;

		g_mqttmconcfg.contype = atoi(argv[2]);

		if (g_mqttmconcfg.contype > 1)
		{
			atsvr_cmd_rsp_error();
			return;
		}

		if (g_mqttmconcfg.contype == 1)
			g_mqttmconcfg.contype = 2; //TLS connect

		char *			tmp;

		tmp 				= (char *) argv[3];
		len 				= strlen((char *) tmp);
		ATSVRLOG("g_mqttmconcfg.linkid=%d\n",g_mqttmconcfg.linkid);
		if (len == 0 || len > MAX_SIZE_MQTT_CLIENTID)
		{
			ATSVRLOG("client_id oversize\n");
			atsvr_cmd_rsp_error();
		}
		else
		{
			strcpy(g_mqttmconcfg.clientid, argv[3]);
		}

		tmp 				= (char *) argv[4];
		len 				= strlen((char *) tmp);

		if (len == 0 || len > MAX_SIZE_MQTT_USER)
		{
			ATSVRLOG("username oversize\n");
			atsvr_cmd_rsp_error();
			return;
		}
		else
		{

			strcpy(g_mqttmconcfg.mquser, argv[4]);
		}

		tmp 				= (char *) argv[5];
		len 				= strlen((char *) tmp);

		if (len == 0 || len > MAX_SIZE_MQTT_USER)
		{
			ATSVRLOG("userpwd oversize\n");
			atsvr_cmd_rsp_error();
			return;
		}
		else
		{

			strcpy(g_mqttmconcfg.mqpwd, argv[5]);
		}

		if (argc >= 7)
		{
			g_mqttmconcfg.cert_keyid = atoi(argv[6]);
		}


		if (argc >= 8)
		{
			g_mqttmconcfg.CA_ID = atoi(argv[7]);
		}

		if (argc >= 9)
		{
			//if (strlen((char *)(argv[8]) >= 0))
			if(argv[8] != NULL)
			{
				strcpy(g_mqttmconcfg.path, argv[8]);
			}

		}

		//	ATSVRLOG("linkid:%d,contype:%s,clientid:%s,mquser:%s,mqpwd:%s,cert_keyid:%d,CA_ID:%d,path:%s\r\n",g_mqttmconcfg.linkid,g_mqttmconcfg.contype?"TLS":"TCP",g_mqttmconcfg.clientid,g_mqttmconcfg.mquser,g_mqttmconcfg.mqpwd,g_mqttmconcfg.cert_keyid,g_mqttmconcfg.CA_ID,g_mqttmconcfg.path);
		atsvr_cmd_rsp_ok();
		return;
	}

}


static void _atsvr_MQTTCONNCFG_handle(int argc, char * *argv)
{


	ATSVRLOG("_atsvr_MQTTCONNCFG_handle\n");

	if (argc < 8)
	{
		ATSVRLOG("para number error\n");
		atsvr_cmd_rsp_error();
		return;
	}

	g_mqttmconcfg.linkid = atoi(argv[1]);
	ATSVRLOG("g_mqttmconcfg.linkid=%d\n",g_mqttmconcfg.linkid);

	g_mqttmconcfg.keepalive = atoi(argv[2]);

	if (g_mqttmconcfg.keepalive == 0)
		g_mqttmconcfg.keepalive = 120;

	if (g_mqttmconcfg.keepalive > 7200)
	{

		ATSVRLOG("para keepalive error\n");
		atsvr_cmd_rsp_error();
		return;
	}

	g_mqttmconcfg.clean_session = atoi(argv[3]);

	if (g_mqttmconcfg.clean_session < 0 || g_mqttmconcfg.clean_session > 1)
	{

		ATSVRLOG("para number error\n");
		atsvr_cmd_rsp_error();
		return;
	}

	if (strlen(argv[4]) > 0 && strlen(argv[4]) < 128)
	{
		strcpy(g_mqttmconcfg.lwt_topic, argv[4]);
	}
	else
	{

		ATSVRLOG("para lwt_topic error\n");
		atsvr_cmd_rsp_error();
		return;

	}


	if (strlen(argv[5]) > 0 && strlen(argv[5]) < 128)
	{
		//g_mqttmconcfg.lwt_msg = atoi(argv[5]);
		strcpy(g_mqttmconcfg.lwt_msg, argv[5]);
	}
	else
	{

		ATSVRLOG("para lwt_topic error\n");
		atsvr_cmd_rsp_error();
		return;

	}

	g_mqttmconcfg.lwt_qos = atoi(argv[6]);

	if (g_mqttmconcfg.lwt_qos < 0 || g_mqttmconcfg.lwt_qos > 2)
	{

		ATSVRLOG("para lwt_qos error\n");
		g_mqttmconcfg.lwt_qos = 0;
		atsvr_cmd_rsp_error();
		return;
	}

	g_mqttmconcfg.lwt_retain = atoi(argv[7]);

	if (g_mqttmconcfg.lwt_retain < 0 || g_mqttmconcfg.lwt_retain > 1)
	{

		ATSVRLOG("para lwt_retain error\n");
		g_mqttmconcfg.lwt_retain = 0;
		atsvr_cmd_rsp_error();
		return;
	}

	atsvr_cmd_rsp_ok();
	return;
}

const struct _atsvr_command _atsvc_mqttcmds_table[] =
{
	_ATSVR_CMD_HADLER("AT+MQTTUSERCFG", "AT+MQTTUSERCFG=<linkid>,<contype>,<client_id>,<username>,<password>,<keepalive>,<cer_key_ID>,<CA_ID>,<path>",
		_atsvr_MQTTUSERCFG_handle),
	_ATSVR_CMD_HADLER("AT+MQTTCONNCFG", "AT+MQTTCONNCFG=<linkid>,<keepalive>,<disable_session>,<lwt_topic>,<lwt_msg>,<lwt_qos>,<lwt_retain>",
		_atsvr_MQTTCONNCFG_handle),
	_ATSVR_CMD_HADLER("AT+MQTTCONN", "AT+MQTTCONN=<linkid>,<host>,<port>,<reconnect>", _atsvr_MQTTCONN_handle),
	_ATSVR_CMD_HADLER("AT+MQTTCONN?", "AT+MQTTCONN?", _atsvr_MQTTCONNSTATE_handle),
	_ATSVR_CMD_HADLER("AT+MQTTPUB", "AT+MQTTPUB=<linkid>,<topic>,<data>,<qos>,<retain>", _atsvr_MQTTPUB_handle),
	_ATSVR_CMD_HADLER("AT+MQTTPUBRAW", "AT+MQTTPUBRAW=<topic>,<data>,<qos>,<retain>", _atsvr_MQTTPUBRAW_handle),
	_ATSVR_CMD_HADLER("AT+MQTTSUB", "AT+MQTTSUB=<linkid>,<topic>,<qos>", _atsvr_MQTTSUB_handle),
	_ATSVR_CMD_HADLER("AT+MQTTSUB?", "AT+MQTTSUB?", _atsvr_MQTTSUB_Query_handle),
	_ATSVR_CMD_HADLER("AT+MQTTUNSUB", "AT+MQTTUNSUB=<linkid>,<topic>", _atsvr_at_MQTTUNSUB_command),
	_ATSVR_CMD_HADLER("AT+MQTTCLEAN", "AT+MQTTCLEAN", _atsvr_at_MQTTDISCONN_command),
};

void atsvr_mqtt_init()
{
	atsvr_register_commands(_atsvc_mqttcmds_table, sizeof(_atsvc_mqttcmds_table) / sizeof(struct _atsvr_command));
}
#endif
