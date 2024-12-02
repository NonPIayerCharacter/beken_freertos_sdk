/*
* Tencent Cloud IoT AT library
* Copyright (C) 2016 THL A29 Limited, a Tencent company. All rights reserved.

* Licensed under the MIT License (the "License"); you may not use this file except in
* compliance with the License. You may obtain a copy of the License at
* http://opensource.org/licenses/MIT

* Unless required by applicable law or agreed to in writing, software distributed under the License is
* distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
* either express or implied. See the License for the specific language governing permissions and
* limitations under the License.
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <stdbool.h>
#include <string.h>

#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"


//#include "qcloud_iot_export.h"
//#include "qcloud_iot_import.h"
//#include "gateway_common.h"


#include "qcloud_at_mqtt.h"
//#include "at_qcloud.h"
#include "rtos_pub.h"
#include "mem_pub.h"
#include "atsvr_comm.h"
#include "atsvr_error.h"


#define MQTT_CLIENT_TASK_NAME        "mqtt_client_task"
#define MQTT_CLIENT_TASK_STACK_BYTES 4096
#define MQTT_CLIENT_TASK_PRIO           7

static void *         g_mqtt_client = NULL;
TaskHandle_t          g_mqtt_task;
static MQTTInitParams g_init_params = DEFAULT_MQTTINIT_PARAMS;

static bool g_mqtt_task_running = false;
static bool g_cmd_processing    = false;
static bool sg_pub_ack = false;

static int  g_ret_code                             = 0;
static char g_mqtt_test_server_ip[HOST_STR_LENGTH] = {0};
//static void *sg_gw_client = NULL;

extern void atsvr_output_msg(char *msg,unsigned int msg_len);

bool get_sg_pub_ack_flag(void)
{
    return sg_pub_ack;
}

bool set_sg_pub_ack_flag(bool pub_ack)
{
    return sg_pub_ack=pub_ack;
}

// support both raw IP and host URL
int set_mqtt_test_server_ip(char *server_ip)
{
    if (server_ip != NULL && strlen(server_ip) > 0) {
        memset(g_mqtt_test_server_ip, 0, sizeof(g_mqtt_test_server_ip));
        strncpy(g_mqtt_test_server_ip, server_ip, sizeof(g_mqtt_test_server_ip) - 1);
//       Log_w("MQTT server has been set to %s", g_mqtt_test_server_ip);
        return 0;
    }

    return 1;
}

char *get_mqtt_test_server_ip(void)
{
    if (strlen(g_mqtt_test_server_ip))
        return g_mqtt_test_server_ip;
    else
        return NULL;
}

static void _on_message_callback(void *pClient, MQTTMessage *message, void *userData)
{
    int n;
    char resultbuf[MAX_SIZE_OF_PUBL_PAYLOAD]={0};
    char *tmp=os_zalloc(message->payload_len+1);

    if (message == NULL) {
        ATSVRLOG("msg null");
        return;
    }

    if (message->topic_len == 0 && message->payload_len == 0) {
        ATSVRLOG("length zero");
        return;
    }

    strncpy(tmp, (char *)message->payload, message->payload_len);
    tmp[message->payload_len]='\0';
    ATSVRLOG("recv msg topic: %s, len: %u payload:%s\r\n", message->ptopic, message->payload_len,tmp);
    n = snprintf(resultbuf,sizeof(resultbuf), "+MQTTSUBREV:\"%s\",%d,\"%s\"\r\n", message->ptopic, message->payload_len,tmp);
    atsvr_output_msg(resultbuf,n);
#undef AT_MQTTPUB_FIX_LEN

    os_free(tmp);
    return;
}

static void _mqtt_event_handler(void *pclient, void *handle_context, MQTTEventMsg *msg)
{
    MQTTMessage *mqtt_messge = NULL;
    uintptr_t    packet_id   = (uintptr_t)msg->msg;
    int n;
    char resultbuf[200];

    switch (msg->event_type) {
        case MQTT_EVENT_UNDEF:
            ATSVRLOG("undefined event occur.");
            break;

        case MQTT_EVENT_DISCONNECT:
            ATSVRLOG("MQTT disconnect.");
        //   n = snprintf(resultbuf,sizeof(resultbuf),"+TCMQTTDISCON,%d\r\n", ATSVR_ERR_MQTT_NO_CONN);
        //   atsvr_output_msg(resultbuf,n);
        // set_wifi_led_state(LED_OFF);
            break;

        case MQTT_EVENT_RECONNECT:
            ATSVRLOG("MQTT reconnect.");
            n = snprintf(resultbuf,sizeof(resultbuf),"+TCMQTTRECONNECTED\r\n");
            atsvr_output_msg(resultbuf,n);
        // set_wifi_led_state(LED_ON);
            break;

        case MQTT_EVENT_PUBLISH_RECVEIVED:
            mqtt_messge = (MQTTMessage *)msg->msg;
            ATSVRLOG("topic message arrived without any handler: %s", mqtt_messge->ptopic);
            break;

        case MQTT_EVENT_SUBCRIBE_SUCCESS:
            g_cmd_processing = false;
            g_ret_code       = ATSVR_RET_SUCCESS;
            ATSVRLOG("subscribe success, packet-id=%u", (unsigned int)packet_id);
            break;

        case MQTT_EVENT_SUBCRIBE_TIMEOUT:
            g_cmd_processing = false;
            g_ret_code       = ATSVR_ERR_MQTT_REQUEST_TIMEOUT;
            ATSVRLOG("subscribe wait ack timeout, packet-id=%u", (unsigned int)packet_id);
            break;

        case MQTT_EVENT_SUBCRIBE_NACK:
            g_cmd_processing = false;
            g_ret_code       = ATSVR_ERR_MQTT_SUB;
            ATSVRLOG("subscribe nack, packet-id=%u", (unsigned int)packet_id);
            break;

        case MQTT_EVENT_UNSUBCRIBE_SUCCESS:
            g_cmd_processing = false;
            g_ret_code       = ATSVR_RET_SUCCESS;
            ATSVRLOG("unsubscribe success, packet-id=%u", (unsigned int)packet_id);
            break;

        case MQTT_EVENT_UNSUBCRIBE_TIMEOUT:
            g_cmd_processing = false;
            g_ret_code       = ATSVR_ERR_MQTT_REQUEST_TIMEOUT;
            ATSVRLOG("unsubscribe timeout, packet-id=%u", (unsigned int)packet_id);
            break;

        case MQTT_EVENT_UNSUBCRIBE_NACK:
            g_cmd_processing = false;
            g_ret_code       = ATSVR_ERR_MQTT_UNSUB_FAIL;
            ATSVRLOG("unsubscribe nack, packet-id=%u", (unsigned int)packet_id);
            break;

        case MQTT_EVENT_PUBLISH_SUCCESS:

            g_cmd_processing = false;
            sg_pub_ack =true;
            g_ret_code       = ATSVR_RET_SUCCESS;
            ATSVRLOG("publish success, packet-id=%u", (unsigned int)packet_id);
            break;

        case MQTT_EVENT_PUBLISH_TIMEOUT:
            g_cmd_processing = false;
            g_ret_code       = ATSVR_ERR_MQTT_REQUEST_TIMEOUT;
            ATSVRLOG("publish timeout, packet-id=%u", (unsigned int)packet_id);
            break;

        case MQTT_EVENT_PUBLISH_NACK:
            g_cmd_processing = false;
            g_ret_code       = ATSVR_ERR_FAILURE;
            ATSVRLOG("publish nack, packet-id=%u", (unsigned int)packet_id);
            break;

        default:
            ATSVRLOG("Should NOT arrive here.");
            break;
    }
}

static void _mqtt_client_task(void *pvParameters)
{
    int             n,rc;
    char        resultbuf[200];
    MQTTInitParams *mqtt_conn_param = (MQTTInitParams *)pvParameters;

    if (mqtt_conn_param == NULL) {
        ATSVRLOG("mqtt_conn_param is null!");
        goto end_of_task;
    }

    ATSVRLOG("task start");

    mqtt_conn_param->event_handle.h_fp    = _mqtt_event_handler;
    mqtt_conn_param->event_handle.context = NULL;
#if 0
    if (strlen(g_mqtt_test_server_ip))
        mqtt_conn_param->mqtt_test_server_ip = g_mqtt_test_server_ip;
    else
        mqtt_conn_param->mqtt_test_server_ip = NULL;
#endif
    g_mqtt_client = IOT_MQTT_Construct(mqtt_conn_param);
    if (g_mqtt_client != NULL) {
        ATSVRLOG("Cloud Device Construct Success");
        g_ret_code       = ATSVR_RET_SUCCESS;
        g_cmd_processing = false;
        //set_wifi_led_state(LED_ON);
    } else {
        ATSVRLOG("Cloud Device Construct Failed");
        g_ret_code       = IOT_MQTT_GetErrCode();
        g_cmd_processing = false;
        goto end_of_task;
    }

    /* the parameters might be changed after construct */
    g_init_params.command_timeout        = mqtt_conn_param->command_timeout;
    g_init_params.keep_alive_interval_ms = mqtt_conn_param->keep_alive_interval_ms;
    g_init_params.clean_session          = mqtt_conn_param->clean_session;
    g_init_params.auto_connect_enable    = mqtt_conn_param->auto_connect_enable;

//#ifdef MULTITHREAD_ENABLED
    IOT_MQTT_SetLoopStatus(g_mqtt_client, true);
//#endif

    do {
        if (!g_mqtt_task_running) {
            ATSVRLOG("MQTT Disconnect by user!");
        //  n = snprintf(resultbuf,sizeof(resultbuf),"+TCMQTTDISCON,%d\r\n", ATSVR_ERR_MQTT_NO_CONN);
        //    atsvr_output_msg(resultbuf,n);
            //set_wifi_led_state(LED_OFF);
            break;
        }

        rc = qcloud_iot_mqtt_yield(g_mqtt_client, g_init_params.command_timeout);

        if (rc == ATSVR_ERR_MQTT_ATTEMPTING_RECONNECT) {
            n = snprintf(resultbuf,sizeof(resultbuf),"+TCMQTTRECONNECTING\r\n");
            atsvr_output_msg(resultbuf,n);
            rtos_delay_milliseconds(1000);
            continue;
        } else if (rc == ATSVR_RET_MQTT_MANUALLY_DISCONNECTED || rc == ATSVR_ERR_MQTT_NO_CONN ||
                rc == ATSVR_ERR_MQTT_RECONNECT_TIMEOUT) {
            // wait for OTA finished
        /* if (is_fw_downloading()) {
                ATSVRLOG("qcloud_iot_mqtt_yield error: %d but OTA is going on!", rc);
                rtos_delay_milliseconds(1000);
                continue;
            }*/

            ATSVRLOG("task exit with error: %d", rc);
        //   n = snprintf(resultbuf,sizeof(resultbuf),"+TCMQTTDISCON,%d\r\n", rc);
        //  atsvr_output_msg(resultbuf,n);
            break;
        } else if (rc != ATSVR_RET_SUCCESS && rc != ATSVR_RET_MQTT_RECONNECTED) {
            ATSVRLOG("IOT_MQTT_Yield return with error: %d", rc);
        }

        rtos_delay_milliseconds(200);

    } while (g_mqtt_client != NULL);

end_of_task:

    if (g_mqtt_client != NULL) {

    // do_fw_ota_update(false, NULL);

        IOT_MQTT_Destroy(&g_mqtt_client);

        g_mqtt_client = NULL;
    }

    g_mqtt_task_running = false;
    ATSVRLOG("task going to be deleted");

    vTaskDelete(NULL);

    return;
}

int do_mqtt_connect(MQTTInitParams *mqtt_conn_param)
{
    if (g_mqtt_client != NULL) {
        ATSVRLOG("MQTT connected already!");
        return ATSVR_ERR_INVAL;
    }

    if (mqtt_conn_param == NULL) {
        ATSVRLOG("mqtt_conn_param is null!");
        return ATSVR_ERR_INVAL;
    }

    g_mqtt_task_running = true;

    rtos_create_thread(&g_mqtt_task,
                MQTT_CLIENT_TASK_PRIO,
                MQTT_CLIENT_TASK_NAME,
                (beken_thread_function_t)_mqtt_client_task,
                MQTT_CLIENT_TASK_STACK_BYTES,
                (beken_thread_arg_t)mqtt_conn_param);

    g_cmd_processing = true;
    Timer timer;
    HAL_Timer_init(&timer);
    HAL_Timer_countdown_ms(&timer, mqtt_conn_param->command_timeout);

    do {
        if (g_cmd_processing == false)
            return g_ret_code;

        rtos_delay_milliseconds(200);
    } while (!HAL_Timer_expired(&timer));

    return ATSVR_ERR_INVAL;
}

int do_mqtt_disconnect()
{
    if (g_mqtt_client == NULL) {
        ATSVRLOG("MQTT NOT connected yet!");
        return ATSVR_ERR_INVAL;
    }

    g_mqtt_task_running = false;
    Timer timer;
    HAL_Timer_init(&timer);
    HAL_Timer_countdown_ms(&timer, g_init_params.command_timeout);

    do {
        if (g_mqtt_client == NULL)
            return ATSVR_RET_SUCCESS;

        rtos_delay_milliseconds(200);

    } while (!HAL_Timer_expired(&timer));

    return ATSVR_ERR_INVAL;
}

int do_mqtt_pub_msg(char *topic_name, int QoS, char *topic_payload, size_t payload_len,int retain)
{
    if (g_mqtt_client == NULL) {
        ATSVRLOG("MQTT NOT connected yet!");
        return ATSVR_ERR_INVAL;
    }

    PublishParams pub_params = DEFAULT_PUB_PARAMS;
    pub_params.qos           = QoS;
    pub_params.payload       = topic_payload;
    pub_params.payload_len   = payload_len;
    pub_params.retained    = retain;

    int rc = IOT_MQTT_Publish(g_mqtt_client, topic_name, &pub_params);
    if (rc < 0) {
        ATSVRLOG("MQTT publish failed %d", rc);
        return rc;
    }

    if (QoS == QOS0)
        return 0;

    /* wait for puback */
    g_cmd_processing = true;
    Timer timer;
    HAL_Timer_init(&timer);
    HAL_Timer_countdown_ms(&timer, g_init_params.command_timeout);

    do {
        if (g_cmd_processing == false)
            return g_ret_code;

        rtos_delay_milliseconds(200);

    } while (!HAL_Timer_expired(&timer));

    return ATSVR_RET_SUCCESS;
}

int do_mqtt_sub_msg(char *topic_name, int QoS)
{
    if (g_mqtt_client == NULL) {
        ATSVRLOG("MQTT NOT connected yet!");
        return ATSVR_ERR_INVAL;
    }

    SubscribeParams sub_params    = DEFAULT_SUB_PARAMS;
    sub_params.on_message_handler = _on_message_callback;
    sub_params.qos                = QoS;

    int rc = IOT_MQTT_Subscribe(g_mqtt_client, topic_name, &sub_params);
    if (rc < 0) {
        ATSVRLOG("MQTT subscribe failed %d", rc);
        return rc;
    }

    /* wait for suback */
    g_cmd_processing = true;
    Timer timer;
    HAL_Timer_init(&timer);
    HAL_Timer_countdown_ms(&timer, g_init_params.command_timeout);

    do {
        if (g_cmd_processing == false)
            return g_ret_code;

        rtos_delay_milliseconds(200);

    } while (!HAL_Timer_expired(&timer));

    return ATSVR_RET_SUCCESS;
}

int do_mqtt_unsub_msg(char *topic_name)
{
    if (g_mqtt_client == NULL) {
        ATSVRLOG("MQTT NOT connected yet!");
        return ATSVR_ERR_INVAL;
    }

    int rc = IOT_MQTT_Unsubscribe(g_mqtt_client, topic_name);
    if (rc < 0) {
        ATSVRLOG("MQTT unsubscribe failed %d", rc);
        return rc;
    }

    /* wait for unsuback */
    g_cmd_processing = true;
    Timer timer;
    HAL_Timer_init(&timer);
    HAL_Timer_countdown_ms(&timer, g_init_params.command_timeout);

    do {
        if (g_cmd_processing == false)
            return g_ret_code;

        rtos_delay_milliseconds(200);

    } while (!HAL_Timer_expired(&timer));

    return ATSVR_RET_SUCCESS;
}

int get_mqtt_conn_parameters(MQTTInitParams *mqtt_conn_param)
{
    if (mqtt_conn_param == NULL) {
        ATSVRLOG("Null pointer");
        return ATSVR_ERR_INVAL;
    }

    memcpy(mqtt_conn_param, &g_init_params, sizeof(MQTTInitParams));

    return ATSVR_RET_SUCCESS;
}

int get_mqtt_sub_list()
{
    char resultbuf[200];
    int n = 0;
    if (g_mqtt_client == NULL) {
        ATSVRLOG("MQTT NOT connected yet!");
        return ATSVR_ERR_INVAL;
    }

    Qcloud_IoT_Client *pClient = (Qcloud_IoT_Client *)g_mqtt_client;
    char *             topic   = NULL;
    int                i;
    for (i = 0; i < MAX_MESSAGE_HANDLERS; i++) {
        topic = (char *)pClient->sub_handles[i].topic_filter;
        if (topic == NULL) {
            continue;
        }
        n = snprintf(resultbuf,sizeof(resultbuf),"+MQTTSUB:\"%s\",%d\r\n", topic, pClient->sub_handles[i].qos);
        atsvr_output_msg(resultbuf,n);
    }

    return 0;
}

int get_mqtt_connect_state()
{
    if (g_mqtt_client == NULL) {
        return 0;
    }

    if (IOT_MQTT_IsConnected(g_mqtt_client))
        return 1;

    return 0;
}

bool is_mqtt_task_running()
{
    return g_mqtt_task_running;
}

void *get_mqtt_client()
{
    return g_mqtt_client;
}
//#endif
