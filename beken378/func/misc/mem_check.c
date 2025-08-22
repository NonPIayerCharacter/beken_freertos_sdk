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
#include <stdarg.h>
#include <string.h>
#include "sys_ctrl_pub.h"
#include "sys_ctrl.h"
#include "uart_pub.h"
#include "wdt_pub.h"
#include "start_type_pub.h"
#include "reg_agc.h"
#include "reg_mdm_cfg.h"

#if CFG_MEM_CHECK_ENABLE
#define _SVC_STACK_SIZE_                  0x3F0
#define ITCM_TCM_BEGIN_ADDR               0x3F0000

#if (CFG_SOC_NAME == SOC_BK7231N)
#define ITCM_TCM_END_ADDR                 0x400000
#elif (CFG_SOC_NAME == SOC_BK7238)
#define ITCM_TCM_END_ADDR                 0x3F4000
#else
#define ITCM_TCM_END_ADDR                 0x3F4000
#endif

#define RAM_BEGIN_ADDR                    0x400100

#if (CFG_SOC_NAME == SOC_BK7231N)
#define RAM_END_ADDR                      0x430000
#elif (CFG_SOC_NAME == SOC_BK7238)
#define RAM_END_ADDR                      0x440000
#else
#define RAM_END_ADDR                      0x440000
#endif

extern char _itcmcode_ram_begin, _itcmcode_ram_end;
extern char svc_stack_start;
extern void bk_reboot_with_type(RESET_SOURCE_STATUS type);
static void mem_printf(const char *fmt, ...)
{
    va_list ap;
    char string[32];

    va_start(ap, fmt);
    vsnprintf(string, sizeof(string) - 1, fmt, ap);
    string[31] = 0;
    bk_send_string(UART1_PORT, string);
    va_end(ap);
}

static int memcheck(uint32_t *aligned4_ptr, uint32_t bytes_length, uint32_t magic)
{
    int ret = 0;
    uint32_t *aligned4_end = aligned4_ptr + (bytes_length >> 2);
    memset((void *)aligned4_ptr, (uint8_t)magic, bytes_length);
    for (; aligned4_ptr < aligned4_end; aligned4_ptr++)
    {
        if (*aligned4_ptr != magic)
        {
            mem_printf("%x\n", aligned4_ptr);
            ret = -1;
        }
    }

    return ret;
}

int cmd_do_memcheck(void)
{
    UINT32 param;
    UINT32 start_type;
    // FUNCPTR reinit = NULL;
    UINT32 ret = 0;

    #if CFG_SYS_START_TIME
    REG_WRITE(0x0802800 + 23*4, 0x02);
    #endif

    start_type = *((volatile UINT32 *)SCTRL_SW_RETENTION);
    if ((start_type & 0xFFFF) != 0xAA55)
    {
        return 0;
    }
    #if (CFG_SOC_NAME == SOC_BK7238)
    sctrl_init();
    #endif

    *((volatile uint32_t *)SCTRL_SW_RETENTION) = RESET_SOURCE_MEM_CHECK;
    // uart_init();  // entry_main() has fast init uart
    if (memcheck((uint32_t *)ITCM_TCM_BEGIN_ADDR, (uint32_t)&_itcmcode_ram_begin - ITCM_TCM_BEGIN_ADDR, 0xAAAAAAAA))
    {
        mem_printf("%s\n", "[MEMCHECK] tcm error!");
        ret = -1;
    }
    if (memcheck((uint32_t *)ITCM_TCM_BEGIN_ADDR, (uint32_t)&_itcmcode_ram_begin - ITCM_TCM_BEGIN_ADDR, 0x55555555))
    {
        mem_printf("%s\n", "[MEMCHECK] tcm error!");
        ret = -1;
    }

    if (memcheck((uint32_t *)((uint32_t)&_itcmcode_ram_end), ITCM_TCM_END_ADDR - (uint32_t)&_itcmcode_ram_end, 0xAAAAAAAA))
    {
        mem_printf("%s\n", "[MEMCHECK] itcm error!");
        ret = -1;
    }
    if (memcheck((uint32_t *)((uint32_t)&_itcmcode_ram_end), ITCM_TCM_END_ADDR - (uint32_t)&_itcmcode_ram_end, 0x55555555))
    {
        mem_printf("%s\n", "[MEMCHECK] itcm error!");
        ret = -1;
    }

    if (memcheck((uint32_t *)RAM_BEGIN_ADDR, (uint32_t)&svc_stack_start - RAM_BEGIN_ADDR, 0xAAAAAAAA))
    {
        mem_printf("%s\n", "[MEMCHECK] ram error!");
        ret = -1;
    }
    if (memcheck((uint32_t *)((uint32_t)&svc_stack_start + _SVC_STACK_SIZE_), RAM_END_ADDR - ((uint32_t)&svc_stack_start + _SVC_STACK_SIZE_), 0xAAAAAAAA))
    {
        mem_printf("%s\n", "[MEMCHECK] ram error!");
        ret = -1;
    }
    #if (CFG_SOC_NAME != SOC_BK7238)
    sctrl_ctrl(CMD_SCTRL_BLE_POWERUP, NULL);
    param = PWD_BLE_CLK_BIT;
    icu_ctrl(CMD_TL410_CLK_PWR_UP, &param);
    #endif
    //for AGC
    param = agc_rwnxagccntl_get();
    param |= AGC_AGCFSMRESET_BIT;
    agc_rwnxagccntl_set(param);

    param = mdm_memclkctrl0_get();
    param &= ~MDM_AGCMEMCLKCTRL_BIT;
    mdm_memclkctrl0_set(param);
    if (memcheck((uint32_t *)RAM_BEGIN_ADDR, (uint32_t)&svc_stack_start - RAM_BEGIN_ADDR, 0x55555555))
    {
        mem_printf("%s\n", "[MEMCHECK] ram error!");
        ret = -1;
    }
    if (memcheck((uint32_t *)((uint32_t)&svc_stack_start + _SVC_STACK_SIZE_), RAM_END_ADDR - ((uint32_t)&svc_stack_start + _SVC_STACK_SIZE_), 0x55555555))
    {
        mem_printf("%s\n", "[MEMCHECK] ram error!");
        ret = -1;
    }
    #if (CFG_SOC_NAME != SOC_BK7238)
    if (memcheck((uint32_t *)0x910000, 0x4000, 0xAAAAAAAA))
    {
        mem_printf("%s\n", "[MEMCHECK] ble ram error!");
        ret = -1;
    }
    if (memcheck((uint32_t *)0x910000, 0x4000, 0x55555555))
    {
        mem_printf("%s\n", "[MEMCHECK] ble ram error!");
        ret = -1;
    }

    param = PWD_BLE_CLK_BIT;
    icu_ctrl(CMD_TL410_CLK_PWR_DOWN, &param);
    sctrl_ctrl(CMD_SCTRL_BLE_POWERDOWN, NULL);
    #endif
    param = agc_rwnxagccntl_get();
    param &= ~AGC_AGCFSMRESET_BIT;
    agc_rwnxagccntl_set(param);

    param = mdm_memclkctrl0_get();
    param |= MDM_AGCMEMCLKCTRL_BIT;
    mdm_memclkctrl0_set(param);
    if (ret == 0)
    {
        mem_printf("%s\n", "[MEMCHECK] PASS!");
    }
    else
    {
        mem_printf("%s\n", "[MEMCHECK] FAIL!");
        goto failed;
    }

    #if CFG_SYS_START_TIME
    REG_WRITE(0x0802800 + 23*4, 0x00);
    #endif
    // reinit();
    while(1);
    return 1;

failed:
    param = PWD_BLE_CLK_BIT;
    icu_ctrl(CMD_TL410_CLK_PWR_DOWN, &param);
    sctrl_ctrl(CMD_SCTRL_BLE_POWERDOWN, NULL);
    #if CFG_SYS_START_TIME
    REG_WRITE(0x0802800 + 23*4, 0x00);
    #endif
    // reinit();
    while(1);
    return -1;
}

void cmd_start_memcheck(void)
{
    UINT32 param;

    #if CFG_SYS_START_TIME
    REG_WRITE(0x0802800 + 23*4, 0x00);
    REG_WRITE(0x0802800 + 23*4, 0x02);
    #endif
    *((volatile uint32_t *)SCTRL_SW_RETENTION) = 0xAA55;
    param = PWD_ARM_WATCHDOG_CLK_BIT;
    icu_ctrl(CMD_CLK_PWR_UP, (void *)&param);
    param = 5;
    extern UINT32 wdt_ctrl(UINT32 cmd, void *param);
    wdt_ctrl(WCMD_SET_PERIOD, (void *)&param);
    #if CFG_SYS_START_TIME
    REG_WRITE(0x0802800 + 23*4, 0x00);
    #endif
    while(1);
}
#endif