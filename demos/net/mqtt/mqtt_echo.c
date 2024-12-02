/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "MQTTFreeRTOS.h"
#include "MQTTClient.h"
#include "mqtt_test.h"
#include "rw_pub.h"
#include "wlan_ui_pub.h"

#if (defined(MQTT_CLIENT_DEMO) && MQTT_ECHO_TEST)

uint32_t g_echo_wifi_flag = 0;

extern void user_connected_callback(FUNCPTR fn);

void echo_wifi_connect_cb(void)
{
	g_echo_wifi_flag = 1;
}

void echo_wifi_disconnect_cb(enum rw_evt_type evt_type, void *data)
{
	g_echo_wifi_flag = 0;
}

uint32_t echo_is_wifi_connected(void)
{
	return (1 == g_echo_wifi_flag);
}

void echo_waiting_for_wifi_connected(void)
{
	while(0 == echo_is_wifi_connected())
	{
		os_printf("reposing......\r\n");
		rtos_delay_milliseconds(ECHO_REPOSE_COUNT);
	}
}

void echo_msg_arrived_cb(MessageData* data)
{
	os_printf("Message arrived on topic %.*s: %.*s\n", data->topicName->lenstring.len, data->topicName->lenstring.data,
		data->message->payloadlen, data->message->payload);
}

static void wifi_cb_init(void)
{
	user_callback_register();
	
	user_connected_callback(echo_wifi_connect_cb);
    rw_evt_set_callback(RW_EVT_STA_DISCONNECTED, (void *)echo_wifi_disconnect_cb);
    rw_evt_set_callback(RW_EVT_STA_CONNECT_FAILED, (void *)echo_wifi_disconnect_cb);
}

static void echo_task_handler(void *pvParameters)
{
	/* connect to m2m.eclipse.org, subscribe to a topic, send and receive 
	 * messages regularly every 1 second
	   mqtt server:
	 			"test.mosquitto.org"  "test.mosca.io"
	 			"iot.eclipse.org" "broker.hivemq.com"
	 */
	Network network;
	MQTTClient client;
	uint32_t fail_cnt;
	int rc = 0, count = 0;
	char* link_name = "broker.mqttdashboard.com";
	unsigned char sendbuf[80], readbuf[80];
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

	wifi_cb_init();
	echo_waiting_for_wifi_connected();
	
	pvParameters = 0;
	
	NetworkInit(&network);	
	os_printf("[MQTT]mqtt_client_init\r\n");
	mqtt_client_init(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

	os_printf("[MQTT]NetworkConnect\r\n");
	if ((rc = NetworkConnect(&network, link_name, 1883)) != 0)
	{
		os_printf("[MQTT]NetworkConnect failed:%d\n", rc);
		goto failed_exit;
	}

#if MQTT_TASK
	os_printf("[MQTT]StartTask\r\n");
	if ((rc = mqtt_start_task(&client)) != pdPASS)
		os_printf("[MQTT]Return code from start tasks is %d\n", rc);
#endif

	connectData.MQTTVersion = 3;
	connectData.clientID.cstring = "FreeRTOS_sample";

	os_printf("[MQTT]Connecting\r\n");
	if ((rc = mqtt_connect(&client, &connectData)) != 0)
	{
		os_printf("[MQTT]Return code from connect is %d\n", rc);
		goto failed_exit;
	}
	else
		os_printf("[MQTT]Connected\n");

	os_printf("[MQTT]Subscribe\r\n");
	if ((rc = mqtt_subscribe(&client, "FreeRTOS/sample/#", 2, echo_msg_arrived_cb)) != 0)
		os_printf("[MQTT]Return code from subscribe is %d\n", rc);

	fail_cnt = 0;
	while (++ count)
	{
		MQTTMessage message;
		char payload[30];

		message.qos = 1;
		message.retained = 0;
		message.payload = payload;
		sprintf(payload, "message number %d", count);
		message.payloadlen = strlen(payload);
		
		os_printf("[MQTT]Publish\r\n");
		if ((rc = mqtt_publish(&client, "FreeRTOS/sample/a", &message)) != 0)
		{
			os_printf("[MQTT]Return code from publish is %d\n", rc);
			fail_cnt ++;

			if(fail_cnt > 20)
			{
				break;
			}
		}
		
#if !defined(MQTT_TASK)
		if ((rc = mqtt_yield(&client, 1000)) != 0)
			os_printf("[MQTT]Return code from yield is %d\n", rc);
#endif
	}

failed_exit:
	os_printf("[MQTT]exit\r\n\n");

	vTaskDelete( NULL );
	/* do not return */
}


void echo_start_tasks(uint16_t usTaskStackSize, UBaseType_t uxTaskPriority)
{
	BaseType_t x = 0L;

	xTaskCreate(echo_task_handler,	/* The function that implements the task. */
			"MQTTEcho0",			/* Just a text name for the task to aid debugging. */
			usTaskStackSize,		/* The stack size is defined in FreeRTOSIPConfig.h. */
			(void *)x,		    	/* The task parameter, not used in this case. */
			uxTaskPriority,			/* The priority assigned to the task is defined in FreeRTOSConfig.h. */
			NULL);					/* The task handle is not used. */
}

int demo_start(void)
{
	echo_start_tasks(ECHO_STACK_SIZE, ECHO_THD_PRORITY);
	
	return 0;
}

#endif
// eof

