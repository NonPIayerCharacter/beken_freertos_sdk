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


#ifndef __BLE_H_
#define __BLE_H_

#define BLE_CONN_IDX_MAX             (0x01)
#define BLE_CHAR_DATA_LEN            (128)

#define BLE_MSG_QUEUE_COUNT          (20)
#define BLE_STACK_SIZE               (4096)

// ble dut uart port
#define PORT_UART1                   (1)
#define PORT_UART2                   (2)
#define BLE_DUT_UART_PORT            PORT_UART2  // PORT_UART2

enum
{
    BLE_MSG_POLL = 0,
    BLE_MSG_DUT,
    BLE_DUT_START,
    BLE_DUT_EXIT,
    BLE_MSG_SLEEP,
    BLE_MSG_NULL,
    BLE_MSG_TO_HOST_HCI,
    BLE_THREAD_EXIT,
};

enum system_run_mode {
    NORMAL_MODE = 0,
    DUT_FCC_MODE = (0x01 << 0),
};

typedef struct ble_message {
    uint32_t data;
} BLE_MSG_T;

uint8_t ble_get_sys_mode(void);
void ble_send_msg(UINT32 data);
UINT32 ble_ctrl( UINT32 cmd, void *param );
void ble_switch_rf_to_wifi(void);
void ble_entry(void);
void ble_thread_exit(void);
bool ble_thread_is_up(void);
bool ble_thread_is_busy(void);
bool ble_stack_is_ready(void);
void ble_set_ext_wkup(uint8_t enable);
void ble_coex_set_pta(bool enable);
bool ble_coex_pta_is_on(void);
#endif

