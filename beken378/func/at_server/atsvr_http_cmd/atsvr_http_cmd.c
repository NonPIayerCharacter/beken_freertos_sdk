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
#include "atsvr_http_cmd.h"
#include "utils_httpc.h"
#include "utils_timer.h"
#include "rtos_pub.h"
#include <stdlib.h>
#include "wlan_ui_pub.h"
#include "atsvr_net_cmd.h"
#include "network_app.h"
#include "network_interface.h"


#include "sl_tls.h"
#include "wlan_cli_pub.h"
#include "sockets.h"
#include "stdlib.h"

static MbedTLSSession *tls_demo_session = NULL;


#define HTTPCLIENT_MAX_HOST_LEN_CMD		64
#define HTTPSCLIENT_MAX_RECV_LEN		2048

int bk_https_get_response(unsigned char *buffer)
{
	if(buffer)
	{
		int http_response_code = 0;

		/* Parse HTTP response */
		if (sscanf(( char *)buffer, "HTTP/%*d.%*d %d %*[^\r\n]", &(http_response_code)) != 1) {
			/* Cannot match string, error */
			ATSVRLOG("Not a correct HTTP answer : %s\n", buffer);
			return ERROR_HTTP_UNRESOLVED_DNS;
		}

		if ((http_response_code < 200) || (http_response_code >= 400)) {
		/* Did not return a 2xx code; TODO fetch headers/(&data?) anyway and implement a mean of writing/reading headers */
			ATSVRLOG("Response code %d", http_response_code);
			return FAIL_RETURN;
		}
		return SUCCESS_RETURN;
	}

	return FAIL_RETURN;
}

int ssl_parse_data_length(unsigned char *msg)
{
	int ssl_data_len = 0;
	int response_ret = bk_https_get_response(msg);
	if(response_ret == SUCCESS_RETURN)
	{
		char *content = strstr((char *)msg, "Content-Length");
		if(content)
		{
			int response_content_len = 0;
			if(sscanf(content, "Content-Length: %d%*[^\r\n]", &response_content_len) == 1)
			{
				//ATSVRLOG("response_content_len=%d\r\n",response_content_len);
				ssl_data_len = response_content_len;
			}
		}
	}
	return ssl_data_len;
}

void http_output_size(int datalen,int type)
{
	char *resultbuf = NULL;
	int n = 0;
	resultbuf = (char *)malloc(64);

	if(resultbuf == NULL)
	{
		ATSVRLOG("resultbuf is error\r\n");
		atsvr_cmd_rsp_error();
		return;
	}

	switch(type)
	{
		case 1:
			n = snprintf(resultbuf,64,"+HTTPCLIENT:<%d>\r\n",datalen);
			atsvr_output_msg(resultbuf,n);
		break;
		case 2:
			n = snprintf(resultbuf,64,"+HTTPGETSIZE:<%d>\r\n",datalen);
			n += snprintf(resultbuf+n,sizeof(resultbuf) - n,"%s",ATSVR_CMD_RSP_SUCCEED);
			atsvr_output_msg(resultbuf,n);
		break;
		case 3:
		break;
	}

	free(resultbuf);
}

void http_output_data(int start_len,char *data,int type)
{
	char *resultbuf = NULL;
	int n = 0;
	resultbuf = (char *)malloc(HTTPSCLIENT_MAX_RECV_LEN);

	if(resultbuf == NULL)
	{
		ATSVRLOG("resultbuf is error\r\n");
		atsvr_cmd_rsp_error();
		return;
	}
	switch(type)
	{
		case 1:
			n = snprintf(resultbuf,HTTPSCLIENT_MAX_RECV_LEN,",<%s>\r\n",&data[start_len]);
			atsvr_output_msg(resultbuf,n);
		break;
		case 2:
		case 3:
		break;
	}
	free(resultbuf);
}

void http_output_ok()
{
	char resultbuf[32]={0};
	int n = 0;
	n = snprintf(resultbuf,32,"%s\r\n",ATSVR_CMD_RSP_SUCCEED);
	atsvr_output_msg(resultbuf,n);
}

int ssl_parse_data_start_position(unsigned char *msg)
{
	char *header_end = strstr((char *)msg,"\r\n\r\n");
	int header_len=0;
	if(header_end)
	{
		header_len = header_end-(char *)msg+2;
		ATSVRLOG("header_len=%d\r\n",header_len);
	}
	return (header_len);
}

static void _atsvr_HTTPCLIENT_handle(int argc, char **argv)
{
	if(argc < 6)
	{
		atsvr_cmd_rsp_error();
		return ;
	}
	http_is_ota =  0;
	httpclient_t httpclient;
	httpclient_data_t httpclient_data;
	char http_content[HTTP_RESP_CONTENT_LEN];

	os_memset(&httpclient, 0, sizeof(httpclient_t));
	os_memset(&httpclient_data, 0, sizeof(httpclient_data));
	os_memset(&http_content, 0, sizeof(HTTP_RESP_CONTENT_LEN));

	httpclient_data.response_buf = http_content;
	httpclient_data.response_buf_len = HTTP_RESP_CONTENT_LEN;
	httpclient_data.response_content_len = HTTP_RESP_CONTENT_LEN;

	int method;
	int content_type;
	int transport_type = 1;
	char *url;
	char host[HTTPCLIENT_MAX_HOST_LEN_CMD] = {0};
	char path[HTTPCLIENT_MAX_HOST_LEN_CMD] = {0};
	char *data=NULL;
	int port = 0;
	int ret;
	//char scheme[8] = { 0 };
	iotx_time_t timer;

	content_type = atoi(argv[2]);
	transport_type = atoi(argv[6]);

	ATSVRLOG("httpclient_parse_url content_type %d\r\n",content_type);
	ATSVRLOG("httpclient_parse_url transport_type %d\r\n",transport_type);
	if(argc >= 8)
	{
		data = argv[7];
	}
	url = argv[3];

	if(data != NULL)
	{
		httpclient_data.post_buf = data;
		ATSVRLOG("httpclient_parse_url data:%s\r\n",httpclient_data.post_buf);
	}

	switch(atoi(argv[1]))
	{
		case 1://HEAD
			method = HTTPCLIENT_HEAD;
		break;
		case 2://GET
			method = HTTPCLIENT_GET;
		break;
		case 3://POST
			method = HTTPCLIENT_POST;
		break;
		case 4://PUT
			method = HTTPCLIENT_PUT;
		break;
		case 5://DELETE
			method = HTTPCLIENT_DELETE;
		break;
		default:
			method = HTTPCLIENT_HEAD;
		break;
	}
	ATSVRLOG("httpclient_parse_url method %d\r\n",method);
	switch(content_type)
	{
		case 0:
			httpclient.header = "Accept: application/x-www-form-urlencoded\r\n";
		break;
		case 1:
			httpclient.header = "Accept: application/json\r\n";
		break;
		case 2:
			httpclient.header = "Accept: multipart/for-data\r\n";
		break;
		case 3:
			httpclient.header = "Accept: text/xml\r\n";
		break;
		default:
			httpclient.header = "Accept: text/xml\r\n";
		break;
	}

	if(url == NULL)
	{
		strncpy(host,argv[4],sizeof(host));
		strncpy(path,argv[5],sizeof(path));
	}
	else
	{
		ret = httpclient_parse_host(url, host, sizeof(host));
		if (0 != ret)
		{
			ATSVRLOG("httpclient_parse_url errno %d\r\n",ret);
			return ;
		}
		strncpy(path,argv[5],sizeof(path));
	}

	if(transport_type == HTTP_TRANSPORT_OVER_SSL)
	{
		MbedTLSSession *tls_session = NULL;
		int length;
		unsigned char *https_recbuf = NULL;
		char *resultbuf = NULL;
		int n = 0;
		char *httpsport = "443";
		int https_data_len = 0;
		int https_data_start_len = 0;
		ATSVRLOG("host: '%s',port: %d,path=%s\r\n", host, *httpsport,path);

		https_recbuf = (unsigned char *)malloc(HTTPSCLIENT_MAX_RECV_LEN);
		resultbuf = (char *)malloc(HTTPSCLIENT_MAX_RECV_LEN);
		if(https_recbuf == NULL)
		{
			ATSVRLOG("https_recbuf is error\r\n");
			atsvr_cmd_rsp_error();
			return;
		}
		if(resultbuf == NULL)
		{
			ATSVRLOG("resultbuf is error\r\n");
			free(https_recbuf);
			atsvr_cmd_rsp_error();
			return;
		}
		tls_session = ssl_create(host,httpsport);
		if(tls_session != NULL)
		{
			tls_demo_session = tls_session;
		}
		else
		{
			ATSVRLOG("https connect is error\r\n");
			atsvr_cmd_rsp_error();
			goto exit1;
		}

		char *meth = (method == HTTPCLIENT_GET) ? "GET" : (method == HTTPCLIENT_POST) ? "POST" :
				(method == HTTPCLIENT_PUT) ? "PUT" : (method == HTTPCLIENT_DELETE) ? "DELETE" :
				(method == HTTPCLIENT_HEAD) ? "HEAD" : "";

		snprintf(http_content, sizeof(http_content), "%s %s HTTP/1.1\r\nHost: %s\r\nUser-Agent: bekencorp/1.0 BK7231s\r\n\r\n", meth, url, host);
		length = strlen(http_content);
		ATSVRLOG("https length:%d\r\n",length);
		ATSVRLOG("https sendbuf:%s\r\n",http_content);
		if(ssl_txdat_sender(tls_session,length,http_content) > 0)
		{
			ATSVRLOG("[TLS]send hearder OK\r\n");
		}
		else
		{
			ATSVRLOG("[TLS]send hearder failed\r\n");
			atsvr_cmd_rsp_error();
			goto exit;
		}

		if (method == HTTPCLIENT_POST || method == HTTPCLIENT_PUT)
		{
			length = strlen(data);
			if(ssl_txdat_sender(tls_session,length,data) > 0)
			{
				ATSVRLOG("[TLS]send data OK\r\n");
			}
			else
			{
				ATSVRLOG("[TLS]send data failed\r\n");
				atsvr_cmd_rsp_error();
				goto exit;
			}
		}

		memset(https_recbuf,0,HTTPSCLIENT_MAX_RECV_LEN);
		length = ssl_read_data(tls_session,https_recbuf,HTTPSCLIENT_MAX_RECV_LEN,1000);
		if(length)
		{
			https_data_len = ssl_parse_data_length(https_recbuf);
			https_data_start_len = ssl_parse_data_start_position(https_recbuf);

			ATSVRLOG("[TLS]recv  %d--%d\r\n",https_data_len,https_data_start_len);

			n = snprintf(resultbuf,HTTPSCLIENT_MAX_RECV_LEN,"+HTTPCLIENT:<%d>,<%s>\r\n",https_data_len,&https_recbuf[https_data_start_len+2]);
			n += snprintf(resultbuf+n,HTTPSCLIENT_MAX_RECV_LEN - n,"%s",ATSVR_CMD_RSP_SUCCEED);
			atsvr_output_msg(resultbuf,n);

			ATSVRLOG("[TLS]recv  OK\r\n");
			goto exit;
		}
		else
		{
			ATSVRLOG("[TLS]recv  failed\r\n");
			atsvr_cmd_rsp_error();
			goto exit;
		}
		http_output_ok();
		exit:
		mbedtls_client_close(tls_session);
		exit1:
		free(resultbuf);
		free(https_recbuf);
		resultbuf = NULL;
		https_recbuf = NULL;
		return;
	}
	else
	{
		port = 80;
		iotx_net_init(&httpclient.net, host, port, NULL);

		ATSVRLOG("host: '%s',port: %d,path=%s\r\n", host, port,path);

		ret = httpclient_connect(&httpclient);
		if (0 != ret) {
			ATSVRLOG("httpclient_connect is error,ret = %d\r\n", ret);
			httpclient_close(&httpclient);
			return;
		}

		ret = httpclient_send_request(&httpclient, url, method, &httpclient_data);
		if (0 != ret) {
			ATSVRLOG("httpclient_send_request is error,ret = %d\r\n", ret);
			httpclient_close(&httpclient);
			return;
		}

		iotx_time_init(&timer);
		utils_time_countdown_ms(&timer, 5*1000);

		if ((NULL != httpclient_data.response_buf)
			|| (0 != httpclient_data.response_buf_len))
		{
			ret = user_httpclient_recv_response(&httpclient, iotx_time_left(&timer), &httpclient_data,1);
			if (ret < 0) {
				ATSVRLOG("httpclient_recv_response is error,ret = %d\r\n", ret);
				rtos_delay_milliseconds(50);
				httpclient_close(&httpclient);
				return;
			}
		}
		if (httpclient_data.is_more) {
			//Close the HTTP if no more data.
			ATSVRLOG("close http \r\n");
			httpclient_close(&httpclient);
		}
		http_output_ok();
	}
	return ;
}

static void _atsvr_HTTPGETSIZE_handle(int argc, char **argv)
{
	if(argc !=2)
	{
		atsvr_cmd_rsp_error();
		return;
	}
	httpclient_t httpclient;
	httpclient_data_t httpclient_data;
	char http_content[HTTP_RESP_CONTENT_LEN];

	os_memset(&httpclient, 0, sizeof(httpclient_t));
	os_memset(&httpclient_data, 0, sizeof(httpclient_data));
	os_memset(&http_content, 0, sizeof(HTTP_RESP_CONTENT_LEN));

	httpclient.header = "Accept: text/xml,text/html,\r\n";
	httpclient_data.response_buf = http_content;
	httpclient_data.response_buf_len = HTTP_RESP_CONTENT_LEN;
	httpclient_data.response_content_len = HTTP_RESP_CONTENT_LEN;

	char *url;
	char host[HTTPCLIENT_MAX_HOST_LEN_CMD] = {0};
	int port = 80;
	int ret;

	iotx_time_t timer;
	url = argv[1];

	ret = httpclient_parse_host(url, host, sizeof(host));
	if (ret != SUCCESS_RETURN)
	{
		ATSVRLOG("httpclient_parse_url errno %d\r\n",ret);
		return;
	}

	ATSVRLOG("host:%s,port:%d,\r\n", host, port);
	iotx_net_init(&httpclient.net, host, port, NULL);
	ret = httpclient_connect(&httpclient);
	if (0 != ret) {
		ATSVRLOG("httpclient_connect is error,ret = %d\r\n", ret);
		atsvr_cmd_rsp_error();
		return ;
	}

	ret = httpclient_send_request(&httpclient, url, HTTPCLIENT_GET, &httpclient_data);
	if (0 != ret) {
		ATSVRLOG("httpclient_send_request is error,ret = %d\r\n", ret);
		httpclient_close(&httpclient);
		atsvr_cmd_rsp_error();
		return;
	}
	iotx_time_init(&timer);
	utils_time_countdown_ms(&timer, 5*1000);

	if ((NULL != httpclient_data.response_buf)
	|| (0 != httpclient_data.response_buf_len))
	{
		ret = user_httpclient_recv_response(&httpclient, iotx_time_left(&timer), &httpclient_data,2);
		if (ret < 0) {
			ATSVRLOG("httpclient_recv_response is error,ret = %d\r\n", ret);
			rtos_delay_milliseconds(50);
			httpclient_close(&httpclient);
			atsvr_cmd_rsp_error();
			return;
		}
	}
	if (httpclient_data.is_more) {
		//Close the HTTP if no more data.
		ATSVRLOG("close http \r\n");
		httpclient_close(&httpclient);
	}
	return;
}
int  post_lenth=0;
httpclient_t httppostclient;
httpclient_data_t httpclientpost_data;
char http_postcontent[HTTP_RESP_CONTENT_LEN];
char posturl[HTTPCLIENT_MAX_HOST_LEN_CMD]={0};
char posthost[HTTPCLIENT_MAX_HOST_LEN_CMD] = {0};
static void httppost_task_thread( beken_thread_arg_t data )
{
	int port = 80;
	int ret;
	iotx_time_t timer;
	http_is_ota =  0;

	os_memset(&httppostclient, 0, sizeof(httpclient_t));
	os_memset(&httpclientpost_data, 0, sizeof(httpclientpost_data));
	os_memset(&http_postcontent, 0, sizeof(HTTP_RESP_CONTENT_LEN));

	httppostclient.header = "Accept: application/x-www-form-urlencoded\r\n";
	httpclientpost_data.post_buf = malloc(1024*2);
	if(httpclientpost_data.post_buf == NULL){
		goto exit;
	}
	httpclientpost_data.response_buf = http_postcontent;
	httpclientpost_data.response_buf_len = HTTP_RESP_CONTENT_LEN;

	ret = httpclient_parse_host(posturl, posthost, sizeof(posthost));
	if (ret != SUCCESS_RETURN)
	{
		ATSVRLOG("httpclient_parse_url errno %d\r\n",ret);
		goto exit;
	}

	http_output_ok();
	g_atsvr_status.network_transfer_mode = ATSVR_PASSTHROUGH_REV_MODE;
	atsvr_cmd_rsp_passthrough();
	atsvr_input_msg_get_block_one(httpclientpost_data.post_buf,post_lenth);

	g_atsvr_status.network_transfer_mode  = ATSVR_COMMON_MODE;
	atsvr_clear_size_rxbuf();

	iotx_net_init(&httppostclient.net, posthost, port, NULL);
	ret = httpclient_connect(&httppostclient);
	if (0 != ret) {
		ATSVRLOG("httpclient_connect is error,ret = %d\r\n", ret);
		goto exit;
	}

	ret = httpclient_send_request(&httppostclient, posturl, HTTPCLIENT_POST, &httpclientpost_data);
	if (0 != ret) {
		atsvr_output_msg("SEND FAIL\r\n",strlen("SEND FAIL\r\n"));
		httpclient_close(&httppostclient);
		goto exit;
	}
	atsvr_output_msg("SEND OK\r\n",strlen("SEND OK\r\n"));

	iotx_time_init(&timer);
	utils_time_countdown_ms(&timer, 5*1000);

	if ((NULL != httpclientpost_data.response_buf)
			|| (0 != httpclientpost_data.response_buf_len))
	{
		ret = user_httpclient_recv_response(&httppostclient, iotx_time_left(&timer), &httpclientpost_data,3);
		if (ret < 0) {
			ATSVRLOG("httpclient_recv_response is error,ret = %d\r\n", ret);
			rtos_delay_milliseconds(50);
			httpclient_close(&httppostclient);
			goto exit;
		}
	}

	if (httpclientpost_data.is_more) {
		//Close the HTTP if no more data.
		ATSVRLOG("close http\r\n");
		httpclient_close(&httppostclient);
	}
	exit:

	if(httpclientpost_data.post_buf !=NULL){
		free(httpclientpost_data.post_buf);
	}
	rtos_delete_thread( NULL );
}

static void _atsvr_HTTPCPOST_handle(int argc, char **argv)
{
	strncpy(posturl,argv[1],sizeof(posturl));
	post_lenth = atoi(argv[2]);
	rtos_create_thread(NULL,
				   THD_APPLICATION_PRIORITY,
				   "http_task",
				   (beken_thread_function_t)httppost_task_thread,
				   4096,
				   (beken_thread_arg_t)0);
}

static void _atsvr_OTA_handle(int argc, char **argv)
{
	int ret;
	char *url;
	http_is_ota = 1;
	httpclient_t httpclient;
	httpclient_data_t httpclient_data;
	char http_content[HTTP_RESP_CONTENT_LEN];

	os_memset(&httpclient, 0, sizeof(httpclient_t));
	os_memset(&httpclient_data, 0, sizeof(httpclient_data));
	os_memset(&http_content, 0, sizeof(HTTP_RESP_CONTENT_LEN));
	httpclient.header = "Accept: text/xml,text/html,\r\n";
	httpclient_data.response_buf = http_content;
	httpclient_data.response_buf_len = HTTP_RESP_CONTENT_LEN;
	httpclient_data.response_content_len = HTTP_RESP_CONTENT_LEN;
	http_output_ok();
	url = argv[1];
	ret = httpclient_common(&httpclient,
							url,
							80,/*port*/
							NULL,
							HTTPCLIENT_GET,
							50000,
							&httpclient_data);

	if (0 != ret) {
		os_printf("request epoch time from remote server failed.\r\n");
		http_is_ota =  0;
	} else {
		os_printf("sucess.\r\n");
		http_is_ota =  0;
		bk_reboot();
	}

	return ;
}

/*
write server certificate
*/
char *user_certificate_data = NULL;
#define certificate_data_len        2048
char user_write_certificate_data_fg = 0;
void _atsvr_https_write_certificate_handle(int argc, char **argv)
{
	ATSVRLOG("argc :%d\r\n",argc);

	if(argc < 2)
	{
		atsvr_cmd_rsp_error();
		return ;
	}
	char recvtype;
	int recvdatalen;
	if(user_certificate_data != NULL)
	{
		free(user_certificate_data);
		user_certificate_data = NULL;
	}
	user_certificate_data = ( char *)malloc(certificate_data_len);
	if(user_certificate_data == NULL)
	{
		ATSVRLOG("malloc faild\r\n");
	}

	recvtype = atoi(argv[1]);
	recvdatalen = atoi(argv[2]);
	ATSVRLOG("recvtype=%d -- recvdatalen=%d\r\n",recvtype,recvdatalen);

	if(recvtype == 1)
	{
		atsvr_cmd_rsp_passthrough();
		g_atsvr_status.network_transfer_mode = ATSVR_PASSTHROUGH_REV_MODE;
		atsvr_input_msg_get_block_one(user_certificate_data,recvdatalen);

		g_atsvr_status.network_transfer_mode = ATSVR_COMMON_MODE;
	}
	else
	{
		int datalen=0;
		atsvr_cmd_rsp_passthrough();
		user_write_certificate_data_fg = 1;
		g_atsvr_status.network_transfer_mode = ATSVR_PASSTHROUGH_REV_MODE;
		datalen = atsvr_input_msg_get_block_one(user_certificate_data,certificate_data_len);
		user_write_certificate_data_fg = 0;
		ATSVRLOG("datalen=%d\r\n",datalen);
		g_atsvr_status.network_transfer_mode = ATSVR_COMMON_MODE;
	}

	http_output_ok();
}

const struct _atsvr_command _atsvc_httpcmds_table[] = {
	_ATSVR_CMD_HADLER("AT+HTTPCLIENT","AT+HTTPCLIENT=<opt>,<content-type>,<url>,<host>,<path>,<transport_type>,<data>,<http_req_header>",_atsvr_HTTPCLIENT_handle),
	_ATSVR_CMD_HADLER("AT+HTTPGETSIZE","AT+HTTPGETSIZE=<url>",_atsvr_HTTPGETSIZE_handle),
	_ATSVR_CMD_HADLER("AT+HTTPCPOST","AT+HTTPCPOST=<url>,<length>,<http_req_head_cnt>,<http_req_header>",_atsvr_HTTPCPOST_handle),
	_ATSVR_CMD_HADLER("AT+OTA","AT+OTA=<url>",_atsvr_OTA_handle),
	_ATSVR_CMD_HADLER("AT+HTTPSCERT","AT+HTTPSCERT=<type>,<lenth>",_atsvr_https_write_certificate_handle),
};

void atsvr_http_init()
{
	atsvr_register_commands(_atsvc_httpcmds_table,sizeof(_atsvc_httpcmds_table) / sizeof(struct _atsvr_command));
}
