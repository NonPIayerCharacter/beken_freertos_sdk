// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#ifndef _THROUGHPUT_TEST_H_
#define _THROUGHPUT_TEST_H_

#include "include.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define TP_ERR_BLE_SUCCESS               0
#define TP_ERR_BLE_FAIL                  1

typedef struct {
    uint16_t msg_id;
    uint8_t *data;
} ble_tp_msg_t;

struct tp_env_tag {
    uint16_t pkt_num_1s;
    uint32_t pkt_num_last_total;
    uint32_t pkt_tatoal;
    uint16_t pkt_lost_num_1s;
    uint32_t pkt_lost_num;
    uint16_t test_time; //s

    uint8_t sending;
    uint8_t receiving;
    uint8_t state;
    uint8_t actv_idx;
};

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
typedef enum {
    TP_START_ADV,
    TP_START_INIT,
    TP_SLAVE_CONFIG,
    TP_MASTER_CONFIG,
    TP_SET_DURATION,
    TP_DISCONNECTED,

    TP_NTF_SEND_START,
    TP_NTF_SEND_STOP,
    TP_NTF_RECV_SATRT,
    TP_NTF_RECV_PKT,
    TP_NTF_RECV_STOP,

    TP_WC_SEND_START,
    TP_WC_SEND_STOP,
    TP_WC_RECV_SATRT,
    TP_WC_RECV_PKT,
    TP_WC_RECV_STOP,
} tp_msg_id;

enum {
    IDLE,
    ADV,
    INIT,
    SLAVE_CONNECTION,
    MASTER_CONNECTION,
};

typedef enum {
    RECEIVE_READY,
    RECEIVE_ONGOING,
    RECEIVE_STOPPED,
} receive_state_t;

typedef enum {
    SEND_READY,
    SEND_ONGOING,
} send_state_t;

typedef enum {
    RECV_STOP_CODE_DISCONNECT,
    RECV_STOP_CODE_TX_STOP,
    RECV_STOP_CODE_BY_SELF,
} recv_stop_reason_code_t;

void ble_tp_cli_cmd(int argc, char **argv);

#endif
