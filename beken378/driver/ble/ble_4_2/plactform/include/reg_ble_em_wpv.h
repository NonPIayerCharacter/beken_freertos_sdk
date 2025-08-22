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

#ifndef _REG_BLE_EM_WPV_H_
#define _REG_BLE_EM_WPV_H_

#include <stdint.h>
#include "_reg_ble_em_wpv.h"
#include "ble_compiler.h"
#include "architect.h"
#include "em_map.h"
#include "ble_reg_access.h"

#define REG_BLE_EM_WPV_COUNT 1

#define REG_BLE_EM_WPV_DECODING_MASK 0x00000000

#define REG_BLE_EM_WPV_ADDR_GET(idx) (EM_BLE_WPV_OFFSET + (idx) * REG_BLE_EM_WPV_SIZE)

/**
 * @brief WLPRV register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                WLPRV   0x0
 * </pre>
 */
#define BLE_WLPRV_ADDR   (0x00814000 + EM_BLE_WPV_OFFSET)
#define BLE_WLPRV_INDEX  0x00000000
#define BLE_WLPRV_RESET  0x00000000
#define BLE_WLPRV_COUNT  3

__INLINE uint16_t ble_wlprv_get(int elt_idx, int reg_idx)
{
    BLE_ASSERT_ERR(reg_idx <= 2);
    return EM_BLE_RD(BLE_WLPRV_ADDR + elt_idx * REG_BLE_EM_WPV_SIZE + reg_idx * 2);
}

__INLINE void ble_wlprv_set(int elt_idx, int reg_idx, uint16_t value)
{
    BLE_ASSERT_ERR(reg_idx <= 2);
    EM_BLE_WR(BLE_WLPRV_ADDR + elt_idx * REG_BLE_EM_WPV_SIZE + reg_idx * 2, value);
}

// field definitions
#define BLE_WLPRV_MASK   ((uint16_t)0x0000FFFF)
#define BLE_WLPRV_LSB    0
#define BLE_WLPRV_WIDTH  ((uint16_t)0x00000010)

#define BLE_WLPRV_RST    0x0

__INLINE uint16_t ble_wlprv_getf(int elt_idx, int reg_idx)
{
    BLE_ASSERT_ERR(reg_idx <= 2);
    uint16_t localVal = EM_BLE_RD(BLE_WLPRV_ADDR + elt_idx * REG_BLE_EM_WPV_SIZE + reg_idx * 2);
    BLE_ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_wlprv_setf(int elt_idx, int reg_idx, uint16_t wlprv)
{
    BLE_ASSERT_ERR(reg_idx <= 2);
    BLE_ASSERT_ERR((((uint16_t)wlprv << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(BLE_WLPRV_ADDR + elt_idx * REG_BLE_EM_WPV_SIZE + reg_idx * 2, (uint16_t)wlprv << 0);
}


#endif // _REG_BLE_EM_WPV_H_

