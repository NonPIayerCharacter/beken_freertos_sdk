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
#include "driver_pub.h"
#include "func_pub.h"
#include "app.h"
#include "ate_app.h"
#include "start_type_pub.h"
#include "bk7011_cal_pub.h"
#include "power_save_pub.h"
#include "reg_mdm_cfg.h"

#if CFG_SUPPORT_LITEOS
#include "los_context.h"
#include "los_task.h"
#endif
#include "sys_ctrl_pub.h"
#include "wdt_pub.h"

beken_semaphore_t extended_app_sema = NULL;
uint32_t  extended_app_stack_size = 2048;

extern void user_main_entry(void);
extern void arm9_enable_alignfault(void);
extern void rwnx_cal_dis_rx_filter_offset(void);
extern int bk7011_reduce_vdddig_for_rx(int reduce);
extern void uart_fast_init(void);
extern OSStatus bk_wdg_finalize( void );
extern UINT32 wdt_ctrl(UINT32 cmd, void *param);

void extended_app_launch_over(void)
{
    OSStatus ret;
    ret = rtos_set_semaphore(&extended_app_sema);

    (void)ret;
}

void extended_app_waiting_for_launch(void)
{
    OSStatus ret;

    ret = rtos_get_semaphore(&extended_app_sema, BEKEN_WAIT_FOREVER);
    ASSERT(kNoErr == ret);

    (void)ret;
}

void improve_rx_sensitivity(void)
{
    #if (CFG_SOC_NAME == SOC_BK7231N)
    /* set TRX_REG12<8:7>=2 for rx */
    rwnx_cal_set_reg_adda_ldo(2);
    /* set MDM_REG202<23>=1 for rx */
    mdm_cpemode_setf(1);
    /* set MDM_REG206<17:16>=3 for band20 */
    mdm_cfgsmooth_setf(3);
    rwnx_cal_dis_rx_filter_offset();
    bk7011_reduce_vdddig_for_rx(1);
    #elif (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
    rwnx_cal_en_rx_filter_offset();
    #endif
}

static void extended_app_task_handler(void *arg)
{
    /* step 0: function layer initialization*/
    func_init_extended();

    /* step 1: startup application layer*/
    #if CFG_TX_EVM_TEST || CFG_RX_SENSITIVITY_TEST
    if(get_ate_mode_state())
    {
        ate_start();
        improve_rx_sensitivity();
    }
    else
    #endif
    {
        #if (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
        power_save_rf_hold_bit_set(RF_HOLD_RF_SLEEP_BIT);
        improve_rx_sensitivity();
        power_save_rf_hold_bit_clear(RF_HOLD_RF_SLEEP_BIT);
        #endif
        app_start();
    }

    extended_app_launch_over();

    rtos_delete_thread( NULL );
}

void extended_app_init(void)
{
    OSStatus ret;

    ret = rtos_init_semaphore(&extended_app_sema, 1);
    ASSERT(kNoErr == ret);
}

void extended_app_uninit(void)
{
    OSStatus ret;

    ret = rtos_deinit_semaphore(&extended_app_sema);
    ASSERT(kNoErr == ret);
}

void extended_app_launch(void)
{
    OSStatus ret;

    ret = rtos_create_thread(NULL,
                             THD_EXTENDED_APP_PRIORITY,
                             "extended_app",
                             (beken_thread_function_t)extended_app_task_handler,
                             extended_app_stack_size,
                             (beken_thread_arg_t)0);
    ASSERT(kNoErr == ret);
}

#ifdef CFG_SUPPORT_OHOS
void __attribute__((weak)) OHOS_SystemInit(void)
{
    printf("@NOOS@");
    return;
}
#endif

void entry_main(void)
{
    #if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
    uart_fast_init();
    #endif

    #if CFG_MEM_CHECK_ENABLE
    extern int cmd_do_memcheck(void);
    cmd_do_memcheck();
    #endif

    arm9_enable_alignfault();
    #if CFG_SUPPORT_LITEOS
    LOS_KernelInit();
    #endif

    #if CFG_TX_EVM_TEST || CFG_RX_SENSITIVITY_TEST
    ate_app_init();
    #endif
#ifdef SOC_BK7231T
    * (volatile uint32_t*)0x00400000 = 1;
#endif
    #if CFG_USE_DEEP_PS
    bk_init_deep_wakeup_gpio_status();
    #endif
    bk_misc_init_start_type();
    bk_misc_check_start_type();
#if (CFG_SOC_NAME == SOC_BK7231N)
    {
        INT32 param = PWD_ARM_WATCHDOG_CLK_BIT;
        icu_ctrl(CMD_CLK_PWR_UP, (void *)&param);
        param = 2000;
        wdt_ctrl(WCMD_SET_PERIOD, &param);
    }
#endif

    /* step 1: driver layer initialization*/
    driver_init();
    func_init_basic();
#if (CFG_SOC_NAME == SOC_BK7231N)
    {
        INT32 param = CFG_INT_WDG_PERIOD_MS; //UPDATE PERIOD TO default
        wdt_ctrl(WCMD_SET_PERIOD, &param);
    }
#endif

    extended_app_init();
    /* step 2: user entry
       attention: if the user wants to increase boot time and light up the board, you may invoke the
       			  routine:user_main_entry before invoking extended_app_launch; and then user MUST wait
       			  for the init of most other devices, such as:wlan and so on;

       			  if you do not care the boot time, the function: extended_app_launch shall be invoked
       			  BEFORE user_main_entry;*/
    user_main_entry();

    /* step 3: init of the most of devices*/
    extended_app_launch();

    #ifdef CFG_SUPPORT_OHOS
    extern void OHOS_SystemInit(void);
    OHOS_SystemInit();
    #endif

    #if (CFG_OS_FREERTOS)
    vTaskStartScheduler();
    #endif

    #if CFG_SUPPORT_LITEOS
    LOS_Start();
    #endif
}
// eof

