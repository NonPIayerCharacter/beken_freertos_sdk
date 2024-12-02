#include "include.h"
#include "fake_clock_pub.h"
#include "icu_pub.h"
#include "drv_model_pub.h"
#include "uart_pub.h"
#include "wdt_pub.h"
#include "target_util_pub.h"
#include "sys_rtos.h"
#include "arm_arch.h"
#include "sys_ctrl.h"
#include "sys_ctrl_pub.h"
#include "start_type_pub.h"
#include "jpeg_encoder_pub.h"
#if((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7231U))
void bk_misc_crash_xat0_reboot(void)
{
    extern UINT32 wdt_ctrl(UINT32 cmd, void *param);

    UINT32 wdt_val = 5, parameter;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();

    parameter = PWD_ARM_WATCHDOG_CLK_BIT;
    icu_ctrl(CMD_CLK_PWR_DOWN, (void *)&parameter);

    delay(1000); // 10ms
    wdt_ctrl(WCMD_SET_PERIOD, &wdt_val);

    parameter = PWD_ARM_WATCHDOG_CLK_BIT;
    icu_ctrl(CMD_CLK_PWR_UP, (void *)&parameter);

    while(1);
    GLOBAL_INT_RESTORE();
}
#else
void bk_misc_crash_xat0_reboot(void)
{
    UINT32 wdt_val = 5;

    os_printf("xat0_reboot\r\n");

    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();

    sddev_control(WDT_DEV_NAME, WCMD_POWER_DOWN, NULL);
    delay_ms(100);
    sddev_control(WDT_DEV_NAME, WCMD_SET_PERIOD, &wdt_val);
    sddev_control(WDT_DEV_NAME, WCMD_POWER_UP, NULL);
    while(1);
    GLOBAL_INT_RESTORE();
}
#endif
static RESET_SOURCE_STATUS start_type;
RESET_SOURCE_STATUS bk_misc_get_start_type(void)
{
    return start_type;
}

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
extern UINT32 sctrl_ctrl(UINT32 cmd, void *param);
//only can be do once
 RESET_SOURCE_STATUS bk_misc_init_start_type(void)
{
    uint32_t misc_value;
    sctrl_ctrl(CMD_GET_SCTRL_RETETION, &misc_value);

#if CFG_USE_DEEP_PS
    if((start_type = sctrl_get_deep_sleep_wake_soure()) == 0)
#else
    if(1)
#endif
    {
        if(0 == (misc_value & SW_RETENTION_WDT_FLAG))
        {
            if((uint32_t)CRASH_XAT0_VALUE ==
                *((volatile uint32_t *)(START_TYPE_DMEMORY_ADDR)))
            {
                start_type = RESET_SOURCE_CRASH_PER_XAT0;
            }
            else
            {
                start_type = RESET_SOURCE_POWERON;
            }
        }
        else
        {
            switch(misc_value & SW_RETENTION_VAL_MASK)
            {
                case (RESET_SOURCE_REBOOT & SW_RETENTION_VAL_MASK):
                    start_type = RESET_SOURCE_REBOOT;
                    break;
                case (CRASH_UNDEFINED_VALUE & SW_RETENTION_VAL_MASK):
                    start_type = RESET_SOURCE_CRASH_UNDEFINED;
                    break;
                case (CRASH_PREFETCH_ABORT_VALUE & SW_RETENTION_VAL_MASK):
                    start_type = RESET_SOURCE_CRASH_PREFETCH_ABORT;
                    break;
                case (CRASH_DATA_ABORT_VALUE & SW_RETENTION_VAL_MASK):
                    start_type = RESET_SOURCE_CRASH_DATA_ABORT;
                    break;
                case (CRASH_UNUSED_VALUE & SW_RETENTION_VAL_MASK):
                    start_type = RESET_SOURCE_CRASH_UNUSED;
                    break;
                case (CRASH_2ND_XAT0_VALUE & SW_RETENTION_VAL_MASK):
                    start_type = RESET_SOURCE_CRASH_XAT0;
                    break;
                case (RESET_SOURCE_WATCHDOG & SW_RETENTION_VAL_MASK):
                    start_type = RESET_SOURCE_WATCHDOG;
                    break;
                case (RESET_SOURCE_FORCE_ATE & SW_RETENTION_VAL_MASK):
                    start_type = RESET_SOURCE_FORCE_ATE;
                    break;
                default:
                start_type = RESET_SOURCE_WATCHDOG;
                break;
            }
        }
    }
    else if ((RESET_SOURCE_DEEPPS_RTC == start_type) && (RESET_SOURCE_REBOOT == misc_value))
    {
        /* adjust for deepsleep reboot */
        start_type = RESET_SOURCE_REBOOT;
    }
    //clear
    sctrl_ctrl(CMD_SET_SCTRL_RETETION, &misc_value);

    *((volatile uint32_t *)(START_TYPE_DMEMORY_ADDR)) = (uint32_t)CRASH_XAT0_VALUE;

    os_printf("bk_misc_init_start_type %x %x\r\n",start_type,misc_value);
    return start_type;

}
void bk_misc_update_set_type(RESET_SOURCE_STATUS type)
{
    uint32_t misc_value = type & SW_RETENTION_VAL_MASK;
    sctrl_ctrl(CMD_SET_SCTRL_RETETION, &misc_value);
}

void bk_misc_check_start_type(void)
{
    if (RESET_SOURCE_CRASH_PER_XAT0 == start_type) {
        uint32_t misc_value = CRASH_2ND_XAT0_VALUE & SW_RETENTION_VAL_MASK;
        sctrl_ctrl(CMD_SET_SCTRL_RETETION, &misc_value);

#if (0 == CFG_JTAG_ENABLE)
            bk_misc_crash_xat0_reboot();
#endif
    }
}
#else

void bk_misc_update_set_type(RESET_SOURCE_STATUS type)
{
#if((START_TYPE_ADDR == 0x0080a080) && (CFG_USE_CAMERA_INTF == 1))// jpeg_quant_table addr base
    if(ejpeg_is_on())
    {
        if(type == RESET_SOURCE_WATCHDOG)
        {
            // can't write to the jpeg_quant_table addr base, for jepeg is on.
            // but the addr is wroten with the value that quant_table[0], we use this value as the key words
            return;
        }
        else
        {
            // if not for watch reason, it must be need reboot or crash.
            // so jpeg is not useful, we can close it, for write type successfully.
            ejpeg_off();
        }
    }
#endif

    *((volatile uint32_t *)(START_TYPE_ADDR)) = (uint32_t)type;
}

//only can be do once
 RESET_SOURCE_STATUS bk_misc_init_start_type(void)
{
    uint32_t misc_value = *((volatile uint32_t *)(START_TYPE_ADDR));

#if((START_TYPE_ADDR == 0x0080a080) && (CFG_USE_CAMERA_INTF == 1)) // jpeg_quant_table addr base
    if (misc_value == ejpeg_get_quant_base_value())
    {
        // it must happend in: before reset, jpeg was working, while watchdog cause reboot.
        misc_value = RESET_SOURCE_WATCHDOG;
    }
#endif

    if((start_type = sctrl_get_deep_sleep_wake_soure()) == 0)
    {
#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7231U))
        if((misc_value & CRASH_DEEP_PS_REBOOT_MASK_H) == CRASH_DEEP_PS_REBOOT_VALUE_H)
        {
            uint32_t type = 0, wakeup_gpio_num = 0;
            type = DPS_GET_TYPEK(misc_value);

            if(type == RESET_SOURCE_DEEPPS_GPIO)
            {
                wakeup_gpio_num = DPS_GET_PIN(misc_value);
                bk_save_deep_set_wakeup_gpio_status(wakeup_gpio_num);
            }
            start_type = type;
        }
        else
#endif
        {
            switch(misc_value)
            {
                case RESET_SOURCE_REBOOT:
                case RESET_SOURCE_FORCE_ATE:
                    start_type = misc_value;
                    break;
                case CRASH_UNDEFINED_VALUE:
                    start_type = RESET_SOURCE_CRASH_UNDEFINED;
                    break;
                case CRASH_PREFETCH_ABORT_VALUE:
                    start_type = RESET_SOURCE_CRASH_PREFETCH_ABORT;
                    break;
                case CRASH_DATA_ABORT_VALUE:
                    start_type = RESET_SOURCE_CRASH_DATA_ABORT;
                    break;
                case CRASH_UNUSED_VALUE:
                    start_type = RESET_SOURCE_CRASH_UNUSED;
                    break;
                case CRASH_XAT0_VALUE:
                    start_type = RESET_SOURCE_CRASH_PER_XAT0;
                    break;
                case CRASH_2ND_XAT0_VALUE:
                    start_type = RESET_SOURCE_CRASH_XAT0;
                    break;
                case RESET_SOURCE_WATCHDOG:
                    if((uint32_t)CRASH_XAT0_VALUE ==
                        *((volatile uint32_t *)(START_TYPE_DMEMORY_ADDR)))
                    {
                        start_type = RESET_SOURCE_CRASH_PER_XAT0;
                    }
                    else
                    {
                        start_type = misc_value;
                    }
                    break;

                default:
                    start_type = RESET_SOURCE_POWERON;
                    break;
            }
        }
    }
#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7231U))
    else 
    {
        uint32_t value = 0, wakeup_gpio_num = 0;
        if(start_type == RESET_SOURCE_DEEPPS_GPIO)
        {
            wakeup_gpio_num = bk_save_deep_get_wakeup_gpio_status();
        }
        value = CRASH_DEEP_PS_REBOOT_VALUE(wakeup_gpio_num, start_type);
        *((volatile uint32_t *)(START_TYPE_ADDR)) = (uint32_t)value;
        bk_misc_crash_xat0_reboot();
    }
#endif
    *((volatile uint32_t *)(START_TYPE_DMEMORY_ADDR)) = (uint32_t)CRASH_XAT0_VALUE;
    bk_misc_update_set_type((RESET_SOURCE_STATUS)CRASH_XAT0_VALUE);

    return start_type;
}

void bk_misc_check_start_type(void)
{
    if (RESET_SOURCE_CRASH_PER_XAT0 == start_type) {
        bk_misc_update_set_type((RESET_SOURCE_STATUS)CRASH_2ND_XAT0_VALUE);

        #if (0 == CFG_JTAG_ENABLE)
        bk_misc_crash_xat0_reboot();
        #endif
    }
}
#endif
