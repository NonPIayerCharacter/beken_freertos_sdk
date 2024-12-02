/*
 * Tencent is pleased to support the open source community by making IoT Hub available.
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

#ifndef QCLOUD_AT_MQTT_H_
#define QCLOUD_AT_MQTT_H_

//#include "qcloud_iot_export.h"
#include "mqtt_client.h"

void *get_mqtt_client();

int get_mqtt_connect_state();

bool is_mqtt_task_running();

int get_mqtt_conn_parameters(MQTTInitParams *mqtt_conn_param);

int do_mqtt_connect(MQTTInitParams *mqtt_conn_param);

int do_mqtt_disconnect();

int do_mqtt_pub_msg(char *topic_name, int QoS, char *topic_payload, size_t payload_len,int retain);


int do_mqtt_sub_msg(char *topic_name, int QoS);

int do_mqtt_unsub_msg(char *topic_name);

int get_mqtt_sub_list();

int do_fw_ota_update(bool ota_enable_t, char *version);

bool is_fw_downloading();

bool is_flash_erasing();

int set_mqtt_test_server_ip(char *server_ip);

char *get_mqtt_test_server_ip(void);

#ifdef DEV_DYN_REG_ENABLED
int do_dyn_reg_dev(void *pDevInfo);
#endif
int do_gw_bind_subdev(int mode, char *product_id, char *device_name, char *device_secret);

int do_gw_online_subdev(int mode, char *product_id, char *device_name);

int get_online_subdev_list(void);

#endif  // QCLOUD_AT_MQTT_H_
