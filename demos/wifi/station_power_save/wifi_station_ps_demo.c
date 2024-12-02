#include "wifi_station_ps_demo.h"
#include "include.h"

#if WIFI_STATION_PS_DEMO
#include "rtos_pub.h"
#include "error.h"
#include "wlan_ui_pub.h"
#include "uart_pub.h"
#include "mem_pub.h"
#include "rw_pub.h"
#include "param_config.h"
#include "str_pub.h"

static beken_semaphore_t station_ps_scan_handle = NULL;
beken_timer_t station_ps_timer_handle, station_ps_stop_timer_handle;


void wifi_station_ps_scan_ap_cb(void *ctxt, uint8_t param)
{
    if(station_ps_scan_handle)
    {
        rtos_set_semaphore( &station_ps_scan_handle );
    }
}

int wifi_station_ps_scan_result(void)
{
    struct sta_scan_res *scan_rst_table;
    char scan_rst_ap_num = 0;       /**< The number of access points found in scanning. */
    int i;

    scan_rst_ap_num = bk_wlan_get_scan_ap_result_numbers();
    if(scan_rst_ap_num == 0)
    {
        return -1;
    }

    return 1;
}

int wifi_station_ps_scan_ap(unsigned char *ssid)
{
    int status = -1;
    OSStatus err = kNoErr;

    bk_wlan_scan_ap_reg_cb(wifi_station_ps_scan_ap_cb);
    bk_wlan_start_assign_scan(&ssid, 1);

    err = rtos_get_semaphore(&station_ps_scan_handle, BEKEN_WAIT_FOREVER);
    if(err == kNoErr)
    {
        status = wifi_station_ps_scan_result();
    }

    return status;
}

int wifi_station_ps_connect_wifi(IN CONST char *ssid, IN CONST char *passwd)
{
    network_InitTypeDef_st wNetConfig;

    os_memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_st));

    os_strcpy((char *)wNetConfig.wifi_ssid, ssid);
    os_strcpy((char *)wNetConfig.wifi_key, passwd);

    wNetConfig.wifi_mode = STATION;
    wNetConfig.dhcp_mode = DHCP_CLIENT;
    wNetConfig.wifi_retry_interval = 100;

    bk_wlan_start(&wNetConfig);

    return 0;
}

int wifi_station_ps_connect_check(void)
{
    msg_sta_states Sta_sts;

    Sta_sts = mhdr_get_station_status();
    if(Sta_sts == MSG_GOT_IP)
    {
        return 1;
    }

    return 0;
}

void station_ps_stop_timer_alarm( void *arg )
{
    OSStatus err = kNoErr;

    if(!wifi_station_ps_connect_check())
    {
        rtos_stop_timer(&station_ps_stop_timer_handle);
        os_printf("\r\nps dtim stop!\r\n");
        bk_wlan_ps_dtim_disable();

        err = rtos_start_timer(&station_ps_timer_handle);
        if(kNoErr != err)
        {
            os_printf("ps_timer start failed!\r\n");
        }
    }
}

void station_ps_timer_alarm( void *arg )
{
    OSStatus err = kNoErr;

    if(wifi_station_ps_connect_check())
    {
        rtos_stop_timer(&station_ps_timer_handle);
        os_printf("\r\nps dtim start!\r\n");
        bk_wlan_ps_dtim_enable(DATA_WAKEUP_TIME , UART_WAKEUP_TIME);

        err = rtos_start_timer(&station_ps_stop_timer_handle);
        if(kNoErr != err)
        {
            os_printf("ps_stop_timer start failed!\r\n");
        }
    }
}

void wifi_station_ps_start(void)
{
    OSStatus err = kNoErr;

    err = rtos_init_timer(&station_ps_timer_handle, 1000, station_ps_timer_alarm, 0);
    if(kNoErr != err)
    {
        os_printf("ps_timer init failed!\r\n");
        return;
    }

    err = rtos_init_timer(&station_ps_stop_timer_handle, 1000, station_ps_stop_timer_alarm, 0);
    if(kNoErr != err)
    {
        os_printf("ps_timer stop init failed!\r\n");
        return;
    }

    err = rtos_start_timer(&station_ps_timer_handle);
    if(kNoErr != err)
    {
        os_printf("ps_timer start failed!\r\n");
    }
}

void wifi_station_ps_main( beken_thread_arg_t arg )
{
    (void) arg;
    int status;
    char *ap_ssid = "bk7231_sta";
    char *ap_key = "12345678";
    OSStatus err = kNoErr;

    err = rtos_init_semaphore( &station_ps_scan_handle, 1 );
    if(err != kNoErr)
    {
        goto exit;
    }

    status = wifi_station_ps_scan_ap((unsigned char *)ap_ssid);
    if(status < 0)
    {
        os_printf("The ssid(%s) is nothingness\r\n", ap_ssid);
        goto exit;
    }

    os_printf("\r\nconnect wifi_SSID:%s,wifi_Password:%s\r\n", ap_ssid, ap_key);

    wifi_station_ps_connect_wifi(ap_ssid, ap_key);

    wifi_station_ps_start();

exit:

    rtos_delete_thread( NULL );
}

int demo_start(void)
{
    OSStatus err = kNoErr;

    err = rtos_create_thread( NULL,
                              BEKEN_APPLICATION_PRIORITY,
                              "sta_ps",
                              (beken_thread_function_t)wifi_station_ps_main,
                              0x800,
                              (beken_thread_arg_t)0 );

    return err;
}

#endif /*WIFI_STATION_PS_DEMO*/
// EOF
