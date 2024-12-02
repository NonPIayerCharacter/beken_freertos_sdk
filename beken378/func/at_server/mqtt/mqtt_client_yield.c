/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Allan Stockdill-Mander/Ian Craggs - initial API and implementation and/or initial documentation
 *******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

//#include "log_upload.h"
#include "mqtt_client.h"
//#include "qcloud_iot_import.h"
#include "atsvr_error.h"
#include "atsvr_comm.h"


static uint32_t _get_random_interval(void)
{
    srand((unsigned)HAL_GetTimeMs());
    /* range: 1000 - 2000 ms, in 10ms unit */
    return (rand() % 100 + 100) * 10;
}

static void _iot_disconnect_callback(Qcloud_IoT_Client *pClient)
{
    if (NULL != pClient->event_handle.h_fp) {
        MQTTEventMsg msg;
        msg.event_type = MQTT_EVENT_DISCONNECT;
        msg.msg        = NULL;

        pClient->event_handle.h_fp(pClient, pClient->event_handle.context, &msg);
    }
}

static void _reconnect_callback(Qcloud_IoT_Client *pClient)
{
    if (NULL != pClient->event_handle.h_fp) {
        MQTTEventMsg msg;
        msg.event_type = MQTT_EVENT_RECONNECT;
        msg.msg        = NULL;

        pClient->event_handle.h_fp(pClient, pClient->event_handle.context, &msg);
    }
}

/**
 * @brief handle exceptional disconnection
 *
 * @param pClient
 * @return
 */
static int _handle_disconnect(Qcloud_IoT_Client *pClient)
{
    AT_FUNC_ENTRY;
    int rc;

    if (0 == get_client_conn_state(pClient)) {
        AT_FUNC_EXIT_RC(ATSVR_ERR_MQTT_NO_CONN);
    }

    rc = qcloud_iot_mqtt_disconnect(pClient);
    // disconnect network stack by force
    if (rc != ATSVR_RET_SUCCESS) {
        pClient->network_stack.disconnect(&(pClient->network_stack));
        set_client_conn_state(pClient, NOTCONNECTED);
    }

    ATSVRLOG("disconnect MQTT for some reasons..");

    _iot_disconnect_callback(pClient);

    // exceptional disconnection
    pClient->was_manually_disconnected = 0;
    AT_FUNC_EXIT_RC(ATSVR_ERR_MQTT_NO_CONN);
}

/**
 * @brief handle reconnect
 *
 * @param pClient
 * @return
 */
static int _handle_reconnect(Qcloud_IoT_Client *pClient)
{
    AT_FUNC_ENTRY;

    int8_t isPhysicalLayerConnected = 1;
    int    rc                       = ATSVR_RET_MQTT_RECONNECTED;

    // reconnect control by delay timer (increase interval exponentially )
    if (!HAL_Timer_expired(&(pClient->reconnect_delay_timer))) {
        AT_FUNC_EXIT_RC(ATSVR_ERR_MQTT_ATTEMPTING_RECONNECT);
    }

    if (NULL != pClient->network_stack.is_connected) {
        isPhysicalLayerConnected =
            (int8_t)pClient->network_stack.is_connected(&(pClient->network_stack));  // always return 1
    }

    if (isPhysicalLayerConnected) {
        rc = qcloud_iot_mqtt_attempt_reconnect(pClient);
        if (rc == ATSVR_RET_MQTT_RECONNECTED) {
            ATSVRLOG("attempt to reconnect success.");
            _reconnect_callback(pClient);
            AT_FUNC_EXIT_RC(rc);
        } else {
            ATSVRLOG("attempt to reconnect failed, errCode: %d", rc);
            rc = ATSVR_ERR_MQTT_ATTEMPTING_RECONNECT;
        }
    }

    pClient->current_reconnect_wait_interval *= 2;

    if (MAX_RECONNECT_WAIT_INTERVAL < pClient->current_reconnect_wait_interval) {
        AT_FUNC_EXIT_RC(ATSVR_ERR_MQTT_RECONNECT_TIMEOUT);
    }
    HAL_Timer_countdown_ms(&(pClient->reconnect_delay_timer), pClient->current_reconnect_wait_interval);

    AT_FUNC_EXIT_RC(rc);
}

/**
 * @brief handle MQTT keep alive (hearbeat with server)
 *
 * @param pClient
 * @return
 */
static int _mqtt_keep_alive(Qcloud_IoT_Client *pClient)
{
#define MQTT_PING_RETRY_TIMES 2

    AT_FUNC_ENTRY;

    int      rc;
    Timer    timer;
    uint32_t serialized_len = 0;
	//ATSVRLOG("yuan 1 keep_alive_interval:%d\r\n",pClient->options.keep_alive_interval);

    if (0 == pClient->options.keep_alive_interval) {
        AT_FUNC_EXIT_RC(ATSVR_RET_SUCCESS);
    }

    if (!HAL_Timer_expired(&pClient->ping_timer)) {
        AT_FUNC_EXIT_RC(ATSVR_RET_SUCCESS);
    }

    if (pClient->is_ping_outstanding >= MQTT_PING_RETRY_TIMES) {
        // Reaching here means we haven't received any MQTT packet for a long time (keep_alive_interval)
        ATSVRLOG("Fail to recv MQTT msg. Something wrong with the connection.");
        rc = _handle_disconnect(pClient);
        AT_FUNC_EXIT_RC(rc);
    }

// ATSVRLOG("yuan 2 keep_alive_interval:%d\r\n",pClient->options.keep_alive_interval);

    /* there is no ping outstanding - send one */
    HAL_MutexLock(pClient->lock_write_buf);
    rc = serialize_packet_with_zero_payload(pClient->write_buf, pClient->write_buf_size, PINGREQ, &serialized_len);
    if (ATSVR_RET_SUCCESS != rc) {
        HAL_MutexUnlock(pClient->lock_write_buf);
        AT_FUNC_EXIT_RC(rc);
    }

    /* send the ping packet */
    int i = 0;
    HAL_Timer_init(&timer);
    do {
        HAL_Timer_countdown_ms(&timer, pClient->command_timeout_ms);
        rc = send_mqtt_packet(pClient, serialized_len, &timer);
    } while (ATSVR_RET_SUCCESS != rc && (i++ < 3));

    if (ATSVR_RET_SUCCESS != rc) {
        HAL_MutexUnlock(pClient->lock_write_buf);
        // If sending a PING fails, propably the connection is not OK and we decide to disconnect and begin reconnection
        // attempts
        ATSVRLOG("Fail to send PING request. Something wrong with the connection.");
        rc = _handle_disconnect(pClient);
        AT_FUNC_EXIT_RC(rc);
    }
    HAL_MutexUnlock(pClient->lock_write_buf);

    HAL_MutexLock(pClient->lock_generic);
    pClient->is_ping_outstanding++;
    /* start a timer to wait for PINGRESP from server */
    HAL_Timer_countdown(&pClient->ping_timer, _MIN(5, pClient->options.keep_alive_interval / 2));
    HAL_MutexUnlock(pClient->lock_generic);
    ATSVRLOG("PING request %u has been sent...", pClient->is_ping_outstanding);

    AT_FUNC_EXIT_RC(ATSVR_RET_SUCCESS);
}

/**
 * @brief Check connection and keep alive state, read/handle MQTT message in synchronized way
 *
 * @param pClient    handle to MQTT client
 * @param timeout_ms timeout value (unit: ms) for this operation
 *
 * @return ATSVR_RET_SUCCESS when success, ATSVR_ERR_MQTT_ATTEMPTING_RECONNECT when try reconnecing, or err code for
 * failure
 */
int qcloud_iot_mqtt_yield(Qcloud_IoT_Client *pClient, uint32_t timeout_ms)
{
    AT_FUNC_ENTRY;

    int     rc = ATSVR_RET_SUCCESS;
    Timer   timer;
    uint8_t packet_type;

    POINTER_SANITY_CHECK(pClient, ATSVR_ERR_INVAL);
    NUMBERIC_SANITY_CHECK(timeout_ms, ATSVR_ERR_INVAL);

    // 1. check if manually disconnect
    if (!get_client_conn_state(pClient) && pClient->was_manually_disconnected == 1) {
        AT_FUNC_EXIT_RC(ATSVR_RET_MQTT_MANUALLY_DISCONNECTED);
    }

    // 2. check connection state and if auto reconnect is enabled
    if (!get_client_conn_state(pClient) && pClient->options.auto_connect_enable == 0) {
        AT_FUNC_EXIT_RC(ATSVR_ERR_MQTT_NO_CONN);
    }

    HAL_Timer_init(&timer);
    HAL_Timer_countdown_ms(&timer, timeout_ms);

    // 3. main loop for packet reading/handling and keep alive maintainance
    while (!HAL_Timer_expired(&timer)) {
        if (!get_client_conn_state(pClient)) {
            if (pClient->current_reconnect_wait_interval > MAX_RECONNECT_WAIT_INTERVAL) {
                rc = ATSVR_ERR_MQTT_RECONNECT_TIMEOUT;
                break;
            }
            rc = _handle_reconnect(pClient);

            continue;
        }
        rc = cycle_for_read(pClient, &timer, &packet_type, QOS0);
        if (rc == ATSVR_RET_SUCCESS) {
            /* check list of wait publish ACK to remove node that is ACKED or timeout */
            qcloud_iot_mqtt_pub_info_proc(pClient);

            /* check list of wait subscribe(or unsubscribe) ACK to remove node that is ACKED or timeout */
            qcloud_iot_mqtt_sub_info_proc(pClient);

        rc = _mqtt_keep_alive(pClient);
        } else if (rc == ATSVR_ERR_SSL_READ_TIMEOUT || rc == ATSVR_ERR_SSL_READ ||
                   rc == ATSVR_ERR_TCP_PEER_SHUTDOWN || rc == ATSVR_ERR_TCP_READ_FAIL) {
            ATSVRLOG("network read failed, rc: %d. MQTT Disconnect.", rc);
            rc = _handle_disconnect(pClient);
        }

        if (rc == ATSVR_ERR_MQTT_NO_CONN) {
            pClient->counter_network_disconnected++;

            if (pClient->options.auto_connect_enable == 1) {
                pClient->current_reconnect_wait_interval = _get_random_interval();
                HAL_Timer_countdown_ms(&(pClient->reconnect_delay_timer), pClient->current_reconnect_wait_interval);

                // reconnect timeout
                rc = ATSVR_ERR_MQTT_ATTEMPTING_RECONNECT;
            } else {
                break;
            }
        } else if (rc != ATSVR_RET_SUCCESS) {
            break;
        }
    }

    AT_FUNC_EXIT_RC(rc);
}



// workaround wrapper for qcloud_iot_mqtt_yield for multi-thread mode
int qcloud_iot_mqtt_yield_mt(Qcloud_IoT_Client *mqtt_client, uint32_t timeout_ms)
{
    POINTER_SANITY_CHECK(mqtt_client, ATSVR_ERR_INVAL);
    NUMBERIC_SANITY_CHECK(timeout_ms, ATSVR_ERR_INVAL);

#ifdef MULTITHREAD_ENABLED
    /* only one instance of yield is allowed in running state*/
    if (mqtt_client->yield_thread_running) {
        rtos_delay_milliseconds(timeout_ms);
        return ATSVR_RET_SUCCESS;
    }
#endif

    return qcloud_iot_mqtt_yield(mqtt_client, timeout_ms);
}



/**
 * @brief puback waiting timeout process
 *
 * @param pClient reference to MQTTClient
 *
 */
int qcloud_iot_mqtt_pub_info_proc(Qcloud_IoT_Client *pClient)
{
    AT_FUNC_ENTRY;

    POINTER_SANITY_CHECK(pClient, ATSVR_ERR_INVAL);

    HAL_MutexLock(pClient->lock_list_pub);
    do {
        if (0 == pClient->list_pub_wait_ack->len) {
            break;
        }

        ListIterator *iter;
        ListNode *    node      = NULL;
        ListNode *    temp_node = NULL;

        if (NULL == (iter = list_iterator_new(pClient->list_pub_wait_ack, LIST_TAIL))) {
            ATSVRLOG("new list failed");
            break;
        }

        for (;;) {
            node = list_iterator_next(iter);

            if (NULL != temp_node) {
                list_remove(pClient->list_pub_wait_ack, temp_node);
                temp_node = NULL;
            }

            if (NULL == node) {
                break; /* end of list */
            }

            QcloudIotPubInfo *repubInfo = (QcloudIotPubInfo *)node->val;
            if (NULL == repubInfo) {
                ATSVRLOG("node's value is invalid!");
                temp_node = node;
                continue;
            }

            /* remove invalid node */
            if (MQTT_NODE_STATE_INVALID == repubInfo->node_state) {
                temp_node = node;
                continue;
            }

            if (!pClient->is_connected) {
                continue;
            }

            /* check the request if timeout or not */
            if (HAL_Timer_remain(&repubInfo->pub_start_time) > 0) {
                continue;
            }

            HAL_MutexUnlock(pClient->lock_list_pub);
            /* If wait ACK timeout, remove the node from list */
            /* It is up to user to do republishing or not */
            temp_node = node;

            HAL_Timer_countdown_ms(&repubInfo->pub_start_time, pClient->command_timeout_ms);
            HAL_MutexLock(pClient->lock_list_pub);

            /* notify timeout event */
            if (NULL != pClient->event_handle.h_fp) {
                MQTTEventMsg msg;
                msg.event_type = MQTT_EVENT_PUBLISH_TIMEOUT;
                msg.msg        = (void *)(uintptr_t)repubInfo->msg_id;
                pClient->event_handle.h_fp(pClient, pClient->event_handle.context, &msg);
            }
        }

        list_iterator_destroy(iter);

    } while (0);

    HAL_MutexUnlock(pClient->lock_list_pub);

    AT_FUNC_EXIT_RC(ATSVR_RET_SUCCESS);
}

/**
 * @brief suback waiting timeout process
 *
 * @param pClient reference to MQTTClient
 *
 */
int qcloud_iot_mqtt_sub_info_proc(Qcloud_IoT_Client *pClient)
{
    AT_FUNC_ENTRY;
    int rc = ATSVR_RET_SUCCESS;

    if (!pClient) {
        AT_FUNC_EXIT_RC(ATSVR_ERR_INVAL);
    }

    HAL_MutexLock(pClient->lock_list_sub);
    do {
        if (0 == pClient->list_sub_wait_ack->len) {
            break;
        }

        ListIterator *iter;
        ListNode *    node      = NULL;
        ListNode *    temp_node = NULL;
        uint16_t      packet_id = 0;
        MessageTypes  msg_type;

        if (NULL == (iter = list_iterator_new(pClient->list_sub_wait_ack, LIST_TAIL))) {
            ATSVRLOG("new list failed");
            HAL_MutexUnlock(pClient->lock_list_sub);
            AT_FUNC_EXIT_RC(ATSVR_RET_SUCCESS);
        }

        for (;;) {
            node = list_iterator_next(iter);

            if (NULL != temp_node) {
                list_remove(pClient->list_sub_wait_ack, temp_node);
                temp_node = NULL;
            }

            if (NULL == node) {
                break; /* end of list */
            }

            QcloudIotSubInfo *sub_info = (QcloudIotSubInfo *)node->val;
            if (NULL == sub_info) {
                ATSVRLOG("node's value is invalid!");
                temp_node = node;
                continue;
            }

            /* remove invalid node */
            if (MQTT_NODE_STATE_INVALID == sub_info->node_state) {
                temp_node = node;
                continue;
            }

            if (pClient->is_connected <= 0) {
                continue;
            }

            /* check the request if timeout or not */
            if (HAL_Timer_remain(&sub_info->sub_start_time) > 0) {
                continue;
            }

            /* When arrive here, it means timeout to wait ACK */
            packet_id = sub_info->msg_id;
            msg_type  = sub_info->type;

            /* Wait MQTT SUBSCRIBE ACK timeout */
            if (NULL != pClient->event_handle.h_fp) {
                MQTTEventMsg msg;

                if (SUBSCRIBE == msg_type) {
                    /* subscribe timeout */
                    msg.event_type = MQTT_EVENT_SUBCRIBE_TIMEOUT;
                    msg.msg        = (void *)(uintptr_t)packet_id;

                    /* notify this event to topic subscriber */
                    if (NULL != sub_info->handler.sub_event_handler)
                        sub_info->handler.sub_event_handler(pClient, MQTT_EVENT_SUBCRIBE_TIMEOUT,
                                                            sub_info->handler.handler_user_data);

                } else {
                    /* unsubscribe timeout */
                    msg.event_type = MQTT_EVENT_UNSUBCRIBE_TIMEOUT;
                    msg.msg        = (void *)(uintptr_t)packet_id;
                }

                pClient->event_handle.h_fp(pClient, pClient->event_handle.context, &msg);
            }

            if (NULL != sub_info->handler.topic_filter)
                at_free((void *)(sub_info->handler.topic_filter));

            temp_node = node;
        }

        list_iterator_destroy(iter);

    } while (0);

    HAL_MutexUnlock(pClient->lock_list_sub);

    AT_FUNC_EXIT_RC(rc);
}

#ifdef __cplusplus
}
#endif
