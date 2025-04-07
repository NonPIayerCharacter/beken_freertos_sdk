/**
 ******************************************************************************
 * @file    wifi_scan.c
 * @author
 * @version V1.0.0
 * @date
 * @brief   scan wifi hot spots demo
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2017 Beken Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 */
#include "include.h"
#include "wifi_scan.h"

#if WIFI_SCAN_DEMO
#include "include.h"
#include "rtos_pub.h"
#include "error.h"
#include "wlan_ui_pub.h"
#include "uart_pub.h"
#include "mem_pub.h"
#include "rw_pub.h"
#include "param_config.h"

static beken_semaphore_t scan_handle = NULL;

void scan_ap_cb(void *ctxt, uint8_t param)
{
    if(scan_handle)
    {
        rtos_set_semaphore( &scan_handle );
    }
}

void show_scan_ap_result(void)
{
#if !CFG_WPA_CTRL_IFACE
    struct scanu_rst_upload *scan_rst;
    ScanResult apList;
    int i;
    GLOBAL_INT_DECLARATION();

    apList.ApList = NULL;

    GLOBAL_INT_DISABLE();
    scan_rst = sr_get_scan_results();
    if (scan_rst == NULL) {
        GLOBAL_INT_RESTORE();
        apList.ApNum = 0;
        return;
    } else {
        apList.ApNum = scan_rst->scanu_num;
    }
    if (apList.ApNum > 0) {
        apList.ApList = (void *)os_zalloc(sizeof(*apList.ApList) * apList.ApNum);
        if(apList.ApList == NULL){
            GLOBAL_INT_RESTORE();
            bk_printf("Got ap count: %d,but malloc failed\r\n", apList.ApNum);
            return;
        }
        for (i = 0; i < scan_rst->scanu_num; i++) {
            os_memcpy(apList.ApList[i].ssid, scan_rst->res[i]->ssid, 32);
            apList.ApList[i].ApPower = scan_rst->res[i]->level;
        }
    }
    GLOBAL_INT_RESTORE();

    if (apList.ApList == NULL)
        apList.ApNum = 0;

    bk_printf("Got ap count: %d\r\n", apList.ApNum);
    for (i = 0; i < apList.ApNum; i++) {
        apList.ApList[i].ssid[32] = '\0';
        bk_printf("    %s, RSSI=%d\r\n", apList.ApList[i].ssid, apList.ApList[i].ApPower);
    }
    bk_printf("Get ap end.......\r\n\r\n");

    if (apList.ApList != NULL) {
        os_free(apList.ApList);
        apList.ApList = NULL;
    }

#if CFG_ROLE_LAUNCH
    rl_pre_sta_set_status(RL_STATUS_STA_LAUNCHED);
#endif

    sr_release_scan_results(scan_rst);
#else    /* CFG_WPA_CTRL_IFACE */
    int ret;
    ScanResult_adv apList;
    extern int hostapd_scan_started;

    if (bk_wlan_ap_is_up() > 0 || hostapd_scan_started)
        ret = wlan_ap_scan_result(&apList);
    else
        ret = wlan_sta_scan_result(&apList);

    if (!ret) {
        int ap_num = apList.ApNum;
        int i;

        os_printf("\r\nscan ap count:%d\r\n", ap_num);
        for( i = 0; i < ap_num; i++ )
        {
            apList.ApList[i].ssid[32] = '\0';
            os_printf("%d: %s, ", i + 1, apList.ApList[i].ssid);
            os_printf("Channal:%d, ", apList.ApList[i].channel);
            switch(apList.ApList[i].security)
            {
            case BK_SECURITY_TYPE_NONE:
                os_printf(" %s, ", "Open");
                break;
            case BK_SECURITY_TYPE_WEP:
                os_printf(" %s, ", "CIPHER_WEP");
                break;
            case BK_SECURITY_TYPE_WPA_TKIP:
                os_printf(" %s, ", "CIPHER_WPA_TKIP");
                break;
            case BK_SECURITY_TYPE_WPA_AES:
                os_printf(" %s, ", "CIPHER_WPA_AES");
                break;
            case BK_SECURITY_TYPE_WPA2_TKIP:
                os_printf(" %s, ", "CIPHER_WPA2_TKIP");
                break;
            case BK_SECURITY_TYPE_WPA2_AES:
                os_printf(" %s, ", "CIPHER_WPA2_AES");
                break;
            case BK_SECURITY_TYPE_WPA2_MIXED:
                os_printf(" %s, ", "CIPHER_WPA2_MIXED");
                break;
            case BK_SECURITY_TYPE_AUTO:
                os_printf(" %s, ", "CIPHER_AUTO");
                break;
            default:
                os_printf(" %s(%d), ", "unknown", apList.ApList[i].security);
                break;
            }
            os_printf("RSSI=%d \r\n", apList.ApList[i].ApPower);
        }
    }

    if (apList.ApList != NULL) {
        os_free(apList.ApList);
        apList.ApList = NULL;
    }
#endif /* CFG_WPA_CTRL_IFACE */
}

void wifi_scan_thread( beken_thread_arg_t arg )
{
    (void) arg;
    OSStatus err = kNoErr;

    os_printf("start scanning..............\r\n");

    err = rtos_init_semaphore( &scan_handle, 1 );
    if(err == kNoErr)
    {
        bk_wlan_scan_ap_reg_cb(scan_ap_cb);
        bk_wlan_start_scan();

        err = rtos_get_semaphore(&scan_handle, BEKEN_WAIT_FOREVER);
        if(err == kNoErr)
        {
            show_scan_ap_result();
        }

        if(scan_handle)
        {
            rtos_deinit_semaphore(&scan_handle);
        }
    }
    else
    {
        os_printf("scan_handle init failed!\r\n");
    }

    rtos_delete_thread( NULL );
}

int demo_start( void )
{
    OSStatus err = kNoErr;

    os_printf("\r\n\r\nwifi scan demo............\r\n\r\n" );
    err = rtos_create_thread( NULL, BEKEN_APPLICATION_PRIORITY,
                              "wifiscan",
                              (beken_thread_function_t)wifi_scan_thread,
                              0x800,
                              (beken_thread_arg_t)0 );

    return err;
}


#endif /**WIFI_SCAN_DEMO**/

