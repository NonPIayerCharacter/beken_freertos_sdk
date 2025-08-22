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

#ifndef __FORCE_PS_PUB_H__
#define __FORCE_PS_PUB_H__

#include "uart_pub.h"

#define FS_PRT                   os_null_printf
#define FS_WARN                  os_null_printf
#define FS_FATAL                 bk_printf

typedef struct mac_sleep_ctrl {
    uint8_t off_mac;
    uint8_t off_modem;
    uint16_t wakeup_mcu;
} MAC_S_CFG_ST, *MAC_S_CFG_PTR;

typedef struct mcu_sleep_ctrl {
    uint8_t sleep_mode;
    uint8_t off_26M;
    uint16_t off_ble;
    uint32_t wakeup_sig;
    uint64_t sleep_us;
} MCU_S_CFG_ST, *MCU_S_CFG_PTR;

typedef enum mcu_sleep_mode {
    MCU_NORMAL_SLEEP            =  0,
    MCU_LOW_VOLTAGE_SLEEP       =  1,
} MCU_S_MODE;

#endif // #ifndef __FORCE_PS_PUB_H__

