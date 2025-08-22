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

#include "rw_platf_pub.h"
#include "sys_config.h"
#include "reg_mac_core.h"
#include "reg_mac_pl.h"
#if CFG_USE_MCU_PS && CFG_USE_TICK_CAL && (0 == CFG_LOW_VOLTAGE_PS)
#include "mcu_ps_pub.h"
#endif

extern void rwnxl_violence_reset_patch(void);
UINT32 mcu_ps_machw_reset(void);

void rwxl_reset_patch(void)
{
    rwnxl_violence_reset_patch();
}

void hal_machw_init_diagnostic_ports(void)
{
    // Initialize diagnostic ports
    #if (CFG_SOC_NAME == SOC_BK7231)
    nxmac_debug_port_sel_pack(0x1C, 0x25);
    #else
    nxmac_debug_port_sel_pack(0x01, 0x07);

    //bypass buf fix
    REG_PL_WR(REG_MAC_CORE_BASE_ADDR + 0x00000700, 0x08);
    #endif // (CFG_SOC_NAME == SOC_BK7231)
}

void hal_machw_before_reset_patch(void)
{
    #if CFG_USE_MCU_PS && CFG_USE_TICK_CAL && (0 == CFG_LOW_VOLTAGE_PS)
    mcu_ps_machw_cal();
    mcu_ps_machw_reset();
    #endif
}

void hal_machw_after_reset_patch(void)
{
    #if CFG_USE_MCU_PS && CFG_USE_TICK_CAL && (0 == CFG_LOW_VOLTAGE_PS)
    mcu_ps_machw_init();
    #endif
}

