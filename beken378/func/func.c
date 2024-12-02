#include "include.h"
#include "func_pub.h"
#include "intc.h"
#include "rwnx.h"
#include "uart_pub.h"
#include "lwip_intf.h"
#include "param_config.h"
#include "saradc_pub.h"
#include "sys_ctrl_pub.h"
#include "drv_model_pub.h"
#include "ate_app.h"
#include "BkDriverWdg.h"
#include "sys_config.h"

#if CFG_ROLE_LAUNCH
#include "role_launch.h"
#endif

#if CFG_SUPPORT_CALIBRATION
#include "bk7011_cal_pub.h"
#endif

#if CFG_UART_DEBUG 
#include "uart_debug_pub.h"
#endif

#if CFG_SDIO
#include "sdio_intf_pub.h"
#endif

#if CFG_USB
#include "fusb_pub.h"
#endif

#include "start_type_pub.h"

#if CFG_ENABLE_BUTTON
#include "key/key_main.h"
#endif

#if CFG_EASY_FLASH && (!CFG_SUPPORT_RTT)
#include "easyflash.h"
#include "bk_ef.h"
#endif

#include "BkDriverFlash.h"

#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7238))
#include "flash_bypass.h"
#endif

extern void rwnx_cal_initial_calibration(void);

UINT32 func_init_extended(void)
{
    char temp_mac[6];

	cfg_param_init();
    // load mac, init mac first
    wifi_get_mac_address(temp_mac, CONFIG_ROLE_NULL);
	
#if (CFG_SOC_NAME == SOC_BK7231N)
    manual_cal_load_bandgap_calm();
#endif
    FUNC_PRT("[FUNC]rwnxl_init\r\n");
    rwnxl_init();

#if CFG_UART_DEBUG
	#ifndef KEIL_SIMULATOR
    FUNC_PRT("[FUNC]uart_debug_init\r\n");   
    uart_debug_init();
	#endif
#endif

#if (!CFG_SUPPORT_RTT)
    FUNC_PRT("[FUNC]intc_init\r\n");
    intc_init();
#endif

#if CFG_SUPPORT_CALIBRATION
	UINT32 is_tab_inflash = 0;
    FUNC_PRT("[FUNC]calibration_main\r\n");
    calibration_main();
    #if CFG_SUPPORT_MANUAL_CALI
	is_tab_inflash = manual_cal_load_txpwr_tab_flash();
    manual_cal_load_default_txpwr_tab(is_tab_inflash);
    #endif
    #if CFG_SARADC_CALIBRATE
    manual_cal_load_adc_cali_flash();
    #endif
    #if CFG_USE_TEMPERATURE_DETECT
    manual_cal_load_temp_tag_flash();
    #endif
	
    #if (CFG_SOC_NAME != SOC_BK7231)
    manual_cal_load_lpf_iq_tag_flash();
    manual_cal_load_xtal_tag_flash();
    #endif // (CFG_SOC_NAME != SOC_BK7231)

    rwnx_cal_initial_calibration();

	#if CFG_SUPPORT_MANUAL_CALI
	if (0) //(is_tab_inflash == 0)
	{
		manual_cal_fitting_txpwr_tab();
		manual_cal_save_chipinfo_tab_to_flash();
		manual_cal_save_txpwr_tab_to_flash();
	}
	#endif // CFG_SUPPORT_MANUAL_CALI
#endif    

#if CFG_SDIO
    FUNC_PRT("[FUNC]sdio_intf_init\r\n");
    sdio_intf_init();
#endif

#if CFG_SDIO_TRANS
    FUNC_PRT("[FUNC]sdio_intf_trans_init\r\n");
    sdio_trans_init();
#endif


#if CFG_USB
    FUNC_PRT("[FUNC]fusb_init\r\n");
    fusb_init();
#endif

#if  CFG_USE_STA_PS
    FUNC_PRT("[FUNC]ps_init\r\n");
#endif

#if CFG_ROLE_LAUNCH
    rl_init();
#endif

	#if CFG_ENABLE_BUTTON
	key_initialization();
	#endif

#if (CFG_SOC_NAME == SOC_BK7221U)
	#if CFG_USE_USB_CHARGE
    extern void usb_plug_func_open(void);
    usb_plug_func_open();
	#endif
#endif

#if (CFG_OS_FREERTOS)
#if CFG_INT_WDG_ENABLED
	FUNC_PRT("int watchdog enabled, period=%u\r\n", CFG_INT_WDG_PERIOD_MS);
	bk_wdg_initialize(CFG_INT_WDG_PERIOD_MS);
        bk_wdg_reload();
#endif //CFG_INT_WDG_ENABLED

#if CFG_TASK_WDG_ENABLED
	FUNC_PRT("task watchdog enabled, period=%u\r\n", CFG_TASK_WDG_PERIOD_MS);
#endif
#endif //CFG_OS_FREERTOS

    FUNC_PRT("[FUNC]func_init_extended OVER!!!\r\n\r\n");
    os_printf("start_type:%d\r\n",bk_misc_get_start_type());
#if (1 == CFG_USE_FORCE_LOWVOL_PS)
    if(RESET_SOURCE_DEEPPS_GPIO == bk_misc_get_start_type())
        os_printf("deep sleep waked by GPIO%d\r\n",bk_misc_wakeup_get_gpio_num());
#endif
    return 0;
}

UINT32 func_init_basic(void)
{
#if (!CFG_SUPPORT_RTT)
    intc_init();
#endif
    hal_flash_init();

#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7238))
	flash_bypass_operate_sr_init();
#endif

#if (CFG_OS_FREERTOS) || (CFG_SUPPORT_RTT)
    os_printf("SDK Rev: %s %s\r\n", BEKEN_SDK_REV, SDK_COMMIT_ID);
#else
    os_printf("SDK Rev: %s\r\n", BEKEN_SDK_REV);
#endif

    return 0;
}

// eof
