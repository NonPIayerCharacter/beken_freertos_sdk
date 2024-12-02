

/*
 * Tencent is pleased to support the open source community by making IoT Hub
 available.
 * Copyright (C) 2018-2020 THL A29 Limited, a Tencent company. All rights
 reserved.

 * Licensed under the MIT License (the "License"); you may not use this file
 except in
 * compliance with the License. You may obtain a copy of the License at
 * http://opensource.org/licenses/MIT

 * Unless required by applicable law or agreed to in writing, software
 distributed under the License is
 * distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 KIND,
 * either express or implied. See the License for the specific language
 governing permissions and
 * limitations under the License.
 *
 */
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef __cplusplus
extern "C"
{
#endif

	/*
	#include "lite-utils.h"
	#include "log_upload.h"
	#include "mqtt_client.h"
	#include "qcloud_iot_ca.h"
	#include "qcloud_iot_common.h"
	#include "qcloud_iot_device.h"
	#include "qcloud_iot_export.h"
	#include "qcloud_iot_import.h"
	#include "utils_base64.h"
	#include "utils_list.h"
	#include "sys_config.h"
	*/
#include "qcloud_iot_export_mqtt.h"
#include "mqtt_client.h"
#include "generic.h"
#include "atsvr_param_check.h"
#include "atsvr_error.h"
#include "network_interface.h"
#include "atsvr_comm.h"
#include "utils_list.h"
#include "utils_base64.h"
#include "qcloud_iot_common.h"

	extern void atsvr_output_msg(char * msg, unsigned int msg_len);

	static uint16_t _get_random_start_packet_id(void)
	{
		srand((unsigned) HAL_GetTimeMs());
		return rand() % 65536 + 1;
	}

	// currently return a constant value
	int IOT_MQTT_GetErrCode(void)
	{
		return ATSVR_ERR_FAILURE;
	}

	int IOT_STRING_DECODE_ARRAY(unsigned char * dec, char * src)
	{
		POINTER_SANITY_CHECK(dec, ATSVR_ERR_INVAL);
		POINTER_SANITY_CHECK(src, ATSVR_ERR_INVAL);

		int 			i;
		int 			count = 0;
		char			temp[3] =
		{
			0
		};

		for (i = 0; i < strlen(src); i += 2)
		{
			memset(temp, 0, sizeof(temp));
			strncpy(temp, &src[i], 2);
			dec[count++]		= strtol(temp, NULL, 16); //atol(temp);
		}

		return count;
	}

	void * IOT_MQTT_Construct(MQTTInitParams * pParams)
	{
		POINTER_SANITY_CHECK(pParams, NULL);
		STRING_PTR_SANITY_CHECK(pParams->product_id, NULL);
		STRING_PTR_SANITY_CHECK(pParams->device_name, NULL);

		Qcloud_IoT_Client * mqtt_client = NULL;
		char *			client_id = NULL;

		// create and init MQTTClient
		if ((mqtt_client = (Qcloud_IoT_Client *) at_malloc(sizeof(Qcloud_IoT_Client))) == NULL)
		{
			ATSVRLOG("malloc MQTTClient failed");
			return NULL;
		}

		int 			rc	= qcloud_iot_mqtt_init(mqtt_client, pParams);

		if (rc != ATSVR_RET_SUCCESS)
		{
			ATSVRLOG("mqtt init failed: %d", rc);
			at_free(mqtt_client);
			return NULL;
		}

		MQTTConnectParams connect_params = DEFAULT_MQTTCONNECT_PARAMS;

		client_id			= at_malloc(MAX_SIZE_OF_CLIENT_ID + 1);

		if (client_id == NULL)
		{
			ATSVRLOG("malloc client_id failed");
			at_free(mqtt_client);
			return NULL;
		}

		memset(client_id, 0, MAX_SIZE_OF_CLIENT_ID + 1);

		//	snprintf(client_id, MAX_SIZE_OF_CLIENT_ID, "%s%s", pParams->product_id, pParams->device_name);
		snprintf(client_id, MAX_SIZE_OF_CLIENT_ID, "%s", pParams->clientid);


		ATSVRLOG("IOT_MQTT_Construct client_id:%s\r\n", client_id);

		connect_params.client_id = client_id;

		// Upper limit of keep alive interval is (11.5 * 60) seconds
		connect_params.keep_alive_interval = _MIN(pParams->keep_alive_interval_ms / 1000, 690);
		connect_params.clean_session = pParams->clean_session;
		connect_params.auto_connect_enable = pParams->auto_connect_enable;


		//#if defined(AUTH_WITH_NOTLS)
		if (pParams->device_secret == NULL)
		{
			ATSVRLOG("Device secret is null!");
			qcloud_iot_mqtt_fini(mqtt_client);
			at_free(mqtt_client);
			at_free(client_id);
			return NULL;
		}

		// size_t src_len = strlen(pParams->device_secret);
		size_t			len;

		memset(mqtt_client->psk_decode, 0x00, DECODE_PSK_LENGTH);

		/*rc							   = qcloud_iot_utils_base64decode(mqtt_client->psk_decode, DECODE_PSK_LENGTH,
			 &len,
										   (unsigned char *)pParams->device_secret, src_len);*/
		// not using qqiot server
		//strcpy(mqtt_client->psk_decode,"2fe2b750a610dab52fa24374c0cb90a2e52b4c02f947107f83f4c11d81f74681");
		//len = IOT_STRING_DECODE_ARRAY(mqtt_client->psk_decode,"2fe2b750a610dab52fa24374c0cb90a2e52b4c02f947107f83f4c11d81f74681");
		len 				= IOT_STRING_DECODE_ARRAY(mqtt_client->psk_decode, g_env_param.pskinfo.psk);
		connect_params.device_secret = (char *) mqtt_client->psk_decode;
		connect_params.device_secret_len = len;

		if (rc != ATSVR_RET_SUCCESS)
		{
			ATSVRLOG("Device secret decode err, secret:%s", pParams->device_secret);
			qcloud_iot_mqtt_fini(mqtt_client);
			at_free(mqtt_client);
			at_free(client_id);
			return NULL;
		}

		//#endif
		rc					= qcloud_iot_mqtt_connect(mqtt_client, &connect_params);

		if (rc != ATSVR_RET_SUCCESS)
		{
			ATSVRLOG("mqtt connect with id: %s failed: %d\r\n", mqtt_client->options.conn_id, rc);
			qcloud_iot_mqtt_fini(mqtt_client);
			at_free(mqtt_client);
			at_free(client_id);
			return NULL;
		}
		else
		{
			ATSVRLOG("mqtt connect with id: %s success\r\n", mqtt_client->options.conn_id);
		}

		return mqtt_client;
	}

	int IOT_MQTT_Destroy(void * *pClient)
	{
		POINTER_SANITY_CHECK(*pClient, ATSVR_ERR_INVAL);

		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) (*pClient);

		int 			rc	= qcloud_iot_mqtt_disconnect(mqtt_client);

		// disconnect network stack by force
		if (rc != ATSVR_RET_SUCCESS)
		{
			mqtt_client->network_stack.disconnect(& (mqtt_client->network_stack));
			set_client_conn_state(mqtt_client, NOTCONNECTED);
		}

		int 			i	= 0;

		for (i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
		{
			/* notify this event to topic subscriber */
			if (NULL != mqtt_client->sub_handles[i].topic_filter &&
				 NULL != mqtt_client->sub_handles[i].sub_event_handler)
				mqtt_client->sub_handles[i].sub_event_handler(mqtt_client, MQTT_EVENT_CLIENT_DESTROY,
					mqtt_client->sub_handles[i].handler_user_data);

			if (NULL != mqtt_client->sub_handles[i].topic_filter)
			{
				at_free((void *) mqtt_client->sub_handles[i].topic_filter);
				mqtt_client->sub_handles[i].topic_filter = NULL;
			}
		}

#ifdef MQTT_RMDUP_MSG_ENABLED
		reset_repeat_packet_id_buffer(mqtt_client);
#endif

		HAL_MutexDestroy(mqtt_client->lock_generic);
		HAL_MutexDestroy(mqtt_client->lock_write_buf);

		HAL_MutexDestroy(mqtt_client->lock_list_sub);
		HAL_MutexDestroy(mqtt_client->lock_list_pub);

		list_destroy(mqtt_client->list_pub_wait_ack);
		list_destroy(mqtt_client->list_sub_wait_ack);
		at_free(mqtt_client->options.client_id);

		at_free(*pClient);
		*pClient			= NULL;
		ATSVRLOG("mqtt release!");

		return rc;
	}

	int IOT_MQTT_Yield(void * pClient, uint32_t timeout_ms)
	{
		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;

#ifdef MULTITHREAD_ENABLED

		/* only one instance of yield is allowed in running state*/
		if (mqtt_client->yield_thread_running)
		{
			rtos_delay_milliseconds(timeout_ms);
			return ATSVR_RET_SUCCESS;
		}

#endif

		int 			rc	= qcloud_iot_mqtt_yield(mqtt_client, timeout_ms);

		return rc;
	}

	int IOT_MQTT_Publish(void * pClient, char * topicName, PublishParams * pParams)
	{
		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;

		return qcloud_iot_mqtt_publish(mqtt_client, topicName, pParams);
	}

	int IOT_MQTT_Subscribe(void * pClient, char * topicFilter, SubscribeParams * pParams)
	{
		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;

		return qcloud_iot_mqtt_subscribe(mqtt_client, topicFilter, pParams);
	}

	int IOT_MQTT_Unsubscribe(void * pClient, char * topicFilter)
	{
		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;

		return qcloud_iot_mqtt_unsubscribe(mqtt_client, topicFilter);
	}

	bool IOT_MQTT_IsSubReady(void * pClient, char * topicFilter)
	{
		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;

		return qcloud_iot_mqtt_is_sub_ready(mqtt_client, topicFilter);
	}

	bool IOT_MQTT_IsConnected(void * pClient)
	{
		AT_FUNC_ENTRY;

		POINTER_SANITY_CHECK(pClient, ATSVR_ERR_INVAL);

		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;

		AT_FUNC_EXIT_RC(get_client_conn_state(mqtt_client) == 1)
	}




	static void _mqtt_yield_thread(void * ptr)
	{
		int 			rc	= ATSVR_RET_SUCCESS;

#if ATSVR_AT_CFG
		int 			n;
		char			resultbuf[200];

#endif

		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) ptr;

		ATSVRLOG("start mqtt_yield_thread...");

		while (mqtt_client->yield_thread_running)
		{
			rc					= qcloud_iot_mqtt_yield(mqtt_client, 200);


			if (rc == ATSVR_ERR_MQTT_ATTEMPTING_RECONNECT)
			{
#if ATSVR_AT_CFG
				n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTRECONNECTING\r\n");
				atsvr_output_msg(resultbuf, n);
#endif

				rtos_delay_milliseconds(500);
				continue;
			}
			else if (rc == ATSVR_RET_MQTT_MANUALLY_DISCONNECTED || rc == ATSVR_ERR_MQTT_RECONNECT_TIMEOUT)
			{
				ATSVRLOG("MQTT Yield thread exit with error: %d", rc);

#if ATSVR_AT_CFG
				n					= snprintf(resultbuf, sizeof(resultbuf), "+MQTTDISCON,%d\r\n", rc);
				atsvr_output_msg(resultbuf, n);
#endif

				break;
			}
			else if (rc != ATSVR_RET_SUCCESS && rc != ATSVR_RET_MQTT_RECONNECTED)
			{
				ATSVRLOG("MQTT Yield thread error: %d", rc);
			}

			rtos_delay_milliseconds(200);
		}

		mqtt_client->yield_thread_running = false;
		mqtt_client->yield_thread_exit_code = rc;

	}

	int IOT_MQTT_StartLoop(void * pClient)
	{
		POINTER_SANITY_CHECK(pClient, ATSVR_ERR_INVAL);

		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;
		ThreadParams	thread_params =
		{
			0
		};
		thread_params.thread_func = _mqtt_yield_thread;

		// thread_params.thread_name		 = "mqtt_yield_thread";
		strcpy(thread_params.thread_name, "mqtt_yield_thread");
		thread_params.user_arg = pClient;
		thread_params.stack_size = 4096;
		thread_params.priority = 1;
		mqtt_client->yield_thread_running = true;

		int 			rc	= HAL_ThreadCreate(&thread_params);

		if (rc)
		{
			ATSVRLOG("create mqtt yield thread fail: %d", rc);
			return ATSVR_ERR_FAILURE;
		}

		rtos_delay_milliseconds(500);
		return ATSVR_RET_SUCCESS;
	}

	void IOT_MQTT_StopLoop(void * pClient)
	{
		POINTER_SANITY_CHECK_RTN(pClient);

		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;

		mqtt_client->yield_thread_running = false;
		rtos_delay_milliseconds(1000);
		return;
	}

	bool IOT_MQTT_GetLoopStatus(void * pClient, int * exit_code)
	{
		POINTER_SANITY_CHECK(pClient, false);
		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;

		*exit_code			= mqtt_client->yield_thread_exit_code;
		return mqtt_client->yield_thread_running;
	}

	void IOT_MQTT_SetLoopStatus(void * pClient, bool loop_status)
	{
		POINTER_SANITY_CHECK_RTN(pClient);
		Qcloud_IoT_Client * mqtt_client = (Qcloud_IoT_Client *) pClient;

		mqtt_client->yield_thread_running = loop_status;
	}


	int qcloud_iot_mqtt_init(Qcloud_IoT_Client * pClient, MQTTInitParams * pParams)
	{
		AT_FUNC_ENTRY;

		POINTER_SANITY_CHECK(pClient, ATSVR_ERR_INVAL);
		POINTER_SANITY_CHECK(pParams, ATSVR_ERR_INVAL);

		memset(pClient, 0x0, sizeof(Qcloud_IoT_Client));

		/*	int size =
		  snprintf(pClient->host_addr, HOST_STR_LENGTH, "%s.%s", STRING_PTR_PRINT_SANITY_CHECK(pParams->product_id),
						   STRING_PTR_PRINT_SANITY_CHECK(iot_get_mqtt_domain(pParams->region)));*/
		int 			size = snprintf(pClient->host_addr, HOST_STR_LENGTH, "%s.%s",
			STRING_PTR_PRINT_SANITY_CHECK(pParams->product_id),
			STRING_PTR_PRINT_SANITY_CHECK(pParams->region));

		if (size < 0 || size > HOST_STR_LENGTH - 1)
		{
			AT_FUNC_EXIT_RC(ATSVR_ERR_FAILURE);
		}

		int 			i	= 0;

		for (i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
		{
			pClient->sub_handles[i].topic_filter = NULL;
			pClient->sub_handles[i].message_handler = NULL;
			pClient->sub_handles[i].sub_event_handler = NULL;
			pClient->sub_handles[i].qos = QOS0;
			pClient->sub_handles[i].handler_user_data = NULL;
		}

		if (pParams->command_timeout < MIN_COMMAND_TIMEOUT)
			pParams->command_timeout = MIN_COMMAND_TIMEOUT;

		if (pParams->command_timeout > MAX_COMMAND_TIMEOUT)
			pParams->command_timeout = MAX_COMMAND_TIMEOUT;

		pClient->command_timeout_ms = pParams->command_timeout;

		// packet id, random from [1 - 65536]
		pClient->next_packet_id = _get_random_start_packet_id();
		pClient->write_buf_size = ATSVR_IOT_MQTT_TX_BUF_LEN;
		pClient->read_buf_size = ATSVR_IOT_MQTT_RX_BUF_LEN;
		pClient->is_ping_outstanding = 0;
		pClient->was_manually_disconnected = 0;
		pClient->counter_network_disconnected = 0;

#ifdef MULTITHREAD_ENABLED
		pClient->yield_thread_running = false;
#endif

		pClient->event_handle = pParams->event_handle;

		pClient->lock_generic = HAL_MutexCreate();

		if (NULL == pClient->lock_generic)
		{
			AT_FUNC_EXIT_RC(ATSVR_ERR_FAILURE);
		}

		set_client_conn_state(pClient, NOTCONNECTED);

		if ((pClient->lock_write_buf = HAL_MutexCreate()) == NULL)
		{
			ATSVRLOG("create write buf lock failed.");
			goto error;
		}

		if ((pClient->lock_list_sub = HAL_MutexCreate()) == NULL)
		{
			ATSVRLOG("create sub list lock failed.");
			goto error;
		}

		if ((pClient->lock_list_pub = HAL_MutexCreate()) == NULL)
		{
			ATSVRLOG("create pub list lock failed.");
			goto error;
		}

		if ((pClient->list_pub_wait_ack = list_new()) == NULL)
		{
			ATSVRLOG("create pub wait list failed.");
			goto error;
		}

		pClient->list_pub_wait_ack->free = at_free;

		if ((pClient->list_sub_wait_ack = list_new()) == NULL)
		{
			ATSVRLOG("create sub wait list failed.");
			goto error;
		}

		pClient->list_sub_wait_ack->free = at_free;


		if (pParams->device_secret != NULL)
		{
			// size_t src_len = strlen(pParams->device_secret);
			size_t			len;

			memset(pClient->psk_decode, 0x00, DECODE_PSK_LENGTH);

			/* qcloud_iot_utils_base64decode(pClient->psk_decode, DECODE_PSK_LENGTH, &len,
										   (unsigned char *)pParams->device_secret, src_len);*/
			// len = IOT_STRING_DECODE_ARRAY(pClient->psk_decode,"2fe2b750a610dab52fa24374c0cb90a2e52b4c02f947107f83f4c11d81f74681");
			len 				= IOT_STRING_DECODE_ARRAY(pClient->psk_decode, g_env_param.pskinfo.psk);
			pClient->network_stack.ssl_connect_params.psk = (char *) pClient->psk_decode;
			pClient->network_stack.ssl_connect_params.psk_length = len;

			//ATSVRLOG("device_secret :%s\r\n", pParams->device_secret);
			ATSVRLOG("psk len:%d,%s\r\n", len, pClient->network_stack.ssl_connect_params.psk);

		}
		else
		{
			ATSVRLOG("psk is empty!");
			AT_FUNC_EXIT_RC(ATSVR_ERR_INVAL);
		}

		memset(pClient->network_stack.ssl_connect_params.psk_id, 0, MAX_SIZE_OF_CLIENT_ID);

		/* snprintf(pClient->network_stack.ssl_connect_params.psk_id, MAX_SIZE_OF_CLIENT_ID, "%s%s",
					  STRING_PTR_PRINT_SANITY_CHECK(pParams->product_id),
					  STRING_PTR_PRINT_SANITY_CHECK(pParams->device_name));
		*/
		strcpy(pClient->network_stack.ssl_connect_params.psk_id, g_env_param.pskinfo.hint);


		ATSVRLOG("psk_id :%s\r\n", pClient->network_stack.ssl_connect_params.psk_id);



		pClient->network_stack.ssl_connect_params.ca_crt = NULL;
		pClient->network_stack.ssl_connect_params.ca_crt_len = 0;

		strcpy(pClient->network_stack.host, pParams->host);
		pClient->network_stack.port = pParams->port;
		pClient->network_stack.ssl_connect_params.timeout_ms = (pClient->command_timeout_ms >
			 ATSEVER_TLS_HANDSHAKE_TIMEOUT) ? pClient->command_timeout_ms: ATSEVER_TLS_HANDSHAKE_TIMEOUT;

		pClient->network_stack.type = pParams->conntype;


		// init network stack
		qcloud_iot_mqtt_network_init(& (pClient->network_stack));

		// ping timer and reconnect delay timer
		HAL_Timer_init(& (pClient->ping_timer));
		HAL_Timer_init(& (pClient->reconnect_delay_timer));

		AT_FUNC_EXIT_RC(ATSVR_RET_SUCCESS);




error:

		if (pClient->list_pub_wait_ack)
		{
			pClient->list_pub_wait_ack->free(pClient->list_pub_wait_ack);
			pClient->list_pub_wait_ack = NULL;
		}

		if (pClient->list_sub_wait_ack)
		{
			pClient->list_sub_wait_ack->free(pClient->list_sub_wait_ack);
			pClient->list_sub_wait_ack = NULL;
		}

		if (pClient->lock_generic)
		{
			HAL_MutexDestroy(pClient->lock_generic);
			pClient->lock_generic = NULL;
		}

		if (pClient->lock_list_sub)
		{
			HAL_MutexDestroy(pClient->lock_list_sub);
			pClient->lock_list_sub = NULL;
		}

		if (pClient->lock_list_pub)
		{
			HAL_MutexDestroy(pClient->lock_list_pub);
			pClient->lock_list_pub = NULL;
		}

		if (pClient->lock_write_buf)
		{
			HAL_MutexDestroy(pClient->lock_write_buf);
			pClient->lock_write_buf = NULL;
		}

		AT_FUNC_EXIT_RC(ATSVR_ERR_FAILURE)
	}

	int qcloud_iot_mqtt_fini(Qcloud_IoT_Client * mqtt_client)
	{
		AT_FUNC_ENTRY;

		POINTER_SANITY_CHECK(mqtt_client, ATSVR_ERR_INVAL);

		HAL_MutexDestroy(mqtt_client->lock_generic);
		HAL_MutexDestroy(mqtt_client->lock_write_buf);

		HAL_MutexDestroy(mqtt_client->lock_list_sub);
		HAL_MutexDestroy(mqtt_client->lock_list_pub);

		list_destroy(mqtt_client->list_pub_wait_ack);
		list_destroy(mqtt_client->list_sub_wait_ack);

		ATSVRLOG("release mqtt client resources");

		AT_FUNC_EXIT_RC(ATSVR_RET_SUCCESS);
	}

	int qcloud_iot_mqtt_set_autoreconnect(Qcloud_IoT_Client * pClient, bool value)
	{
		AT_FUNC_ENTRY;

		POINTER_SANITY_CHECK(pClient, ATSVR_ERR_INVAL);

		pClient->options.auto_connect_enable = (uint8_t) value;

		AT_FUNC_EXIT_RC(ATSVR_RET_SUCCESS);
	}

	bool qcloud_iot_mqtt_is_autoreconnect_enabled(Qcloud_IoT_Client * pClient)
	{
		AT_FUNC_ENTRY;

		POINTER_SANITY_CHECK(pClient, ATSVR_ERR_INVAL);

		bool			is_enabled = false;

		if (pClient->options.auto_connect_enable == 1)
		{
			is_enabled			= true;
		}

		AT_FUNC_EXIT_RC(is_enabled);
	}

	int qcloud_iot_mqtt_get_network_disconnected_count(Qcloud_IoT_Client * pClient)
	{
		AT_FUNC_ENTRY;

		POINTER_SANITY_CHECK(pClient, ATSVR_ERR_INVAL);

		AT_FUNC_EXIT_RC(pClient->counter_network_disconnected);
	}

	int qcloud_iot_mqtt_reset_network_disconnected_count(Qcloud_IoT_Client * pClient)
	{
		AT_FUNC_ENTRY;

		POINTER_SANITY_CHECK(pClient, ATSVR_ERR_INVAL);

		pClient->counter_network_disconnected = 0;

		AT_FUNC_EXIT_RC(ATSVR_RET_SUCCESS);
	}

#ifdef __cplusplus
}


#endif

