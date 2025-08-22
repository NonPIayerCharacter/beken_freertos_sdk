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

#include "include.h"
#include "include.h"

#include "arm_arch.h"
#include "drv_model_pub.h"
#include "ipchksum.h"
#include "ipchksum_pub.h"
#include "target_util_pub.h"

#include "intc_pub.h"
#include "icu_pub.h"
#include "gpio_pub.h"
#include "sys_ctrl_pub.h"
#include "bk_timer_pub.h"
#include "rtos_pub.h"

#if (CFG_SOC_NAME == SOC_BK7252N)

__maybe_unused static SDD_OPERATIONS ipchksum_op = {
    ipchksum_ctrl
};

__maybe_unused static void ipchksum_active(bool enable)
{
    REG_WRITE(IPCHKSUM_REG0X2_ADDR, enable);
}

__maybe_unused static void ipchksum_set_base_addr(UINT32 val)
{
    REG_WRITE(IPCHKSUM_CHECKSUM_BASE_ADDR, val);
}

__maybe_unused static void ipchksum_set_length(UINT16 val)
{
    REG_WRITE(IPCHKSUM_REG0X1_ADDR, val);
}

__maybe_unused static bool ipchksum_get_valid_flag(void)
{
    return (REG_READ(IPCHKSUM_REG0X3_ADDR) & 1);
}

__maybe_unused static UINT16 ipchksum_get_data(void)
{
    return (REG_READ(IPCHKSUM_REG0X4_ADDR) & IPCHKSUM_IP_CHECKSUM_MASK);
}

__maybe_unused static void ipchksum_clr_int(bool enable)
{
    if (enable)
        REG_WRITE(IPCHKSUM_REG0X5_ADDR, 1);
}

__maybe_unused static void ipchksum_start(UINT32 addr, UINT16 len)
{
    ipchksum_set_base_addr(addr);
    ipchksum_set_length(len);

    ipchksum_active(1);
}

/*
 * calculate ip checksum.
 * @addr: buf address, aligned address allowed
 * @len:  buf lenght
 *
 * return ip checksum. Mutex may be used to avoid concurrent access.
 */
UINT16 ipchksum_get_result(UINT32 addr, UINT16 len)
{
    ipchksum_start(addr, len);
    while (!ipchksum_get_valid_flag())
    {
    }
    ipchksum_clr_int(1);
    return ipchksum_get_data();
}

void ipchksum_init(void)
{
    #if CFG_IPCHKSUM_INT_ENABLE
    UINT32 param;
    #endif

    ipchksum_set_base_addr(0);
    ipchksum_set_length(0);
    ipchksum_active(0);
    ipchksum_clr_int(1);

    #if CFG_IPCHKSUM_INT_ENABLE
    intc_service_register(IRQ_IPCHKSUM, PRI_IRQ_IPCHKSUM, ipchksum_isr);
    // sddev_register_dev(IPCHKSUM_DEV_NAME, &ipchksum_op);

    param = IRQ_IPCHKSUM_BIT;
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);
    #endif
}

void ipchksum_exit(void)
{
    sddev_unregister_dev(IPCHKSUM_DEV_NAME);
}

UINT32 ipchksum_ctrl(UINT32 cmd, void *param)
{
    switch (cmd) {
    default:
        break;
    }

    return 0;
}

void ipchksum_isr(void)
{
    #if CFG_IPCHKSUM_INT_ENABLE
    ipchksum_clr_int(1);
    #endif
}

#if IPCHKSUM_UNIT_TEST
static uint8_t csum_buf[4096];
UINT16 ipchksum_get_result(UINT32 addr, UINT16 len);
UINT16 lwip_standard_chksum(const void *dataptr, int len);

// return true if unit test passed, else false.
bool ipchksum_unit_test()
{
    UINT16 hw_csum, sw_csum;

    // init csum buf
    for (int i = 0; i < sizeof(csum_buf); i++) {
        csum_buf[i] = i & 0xFF;
    }

    // caculate csum by hw
    REG_WRITE(0x00802800 + 0x36*4, 2);  // D6, P38
    hw_csum = ipchksum_get_result((UINT32)csum_buf, 4096);
    REG_WRITE(0x00802800 + 0x36*4, 0);

    // caculate csum by sw
    REG_WRITE(0x00802800 + 0x37*4, 2); // D7, P39
    sw_csum = lwip_standard_chksum(csum_buf, 4096);
    REG_WRITE(0x00802800 + 0x37*4, 0);
    os_printf("1: csum: %x/%x\n", hw_csum, sw_csum);
    if (hw_csum != sw_csum)
        return false;

    // caculate csum by hw, addr not aligned
    REG_WRITE(0x00802800 + 0x36*4, 2);
    hw_csum = ipchksum_get_result((UINT32)&csum_buf[1], 4095);
    REG_WRITE(0x00802800 + 0x36*4, 0);

    // caculate csum by sw, addr not aligned
    REG_WRITE(0x00802800 + 0x37*4, 2);
    sw_csum = lwip_standard_chksum(&csum_buf[1], 4095);
    REG_WRITE(0x00802800 + 0x37*4, 0);
    os_printf("2: csum: %x/%x\n", hw_csum, sw_csum);
    if (hw_csum != sw_csum)
        return false;

    // caculate csum by hw, addr not aligned, network length
    REG_WRITE(0x00802800 + 0x36*4, 2);
    hw_csum = ipchksum_get_result((UINT32)&csum_buf[1], 1460);
    REG_WRITE(0x00802800 + 0x36*4, 0);

    // caculate csum by sw, addr not aligned, network length
    REG_WRITE(0x00802800 + 0x37*4, 2);
    sw_csum = lwip_standard_chksum(&csum_buf[1], 1460);
    REG_WRITE(0x00802800 + 0x37*4, 0);
    os_printf("3: csum: %x/%x\n", hw_csum, sw_csum);

    if (hw_csum != sw_csum)
        return false;

    return true;
}
#endif

#endif

// eof

