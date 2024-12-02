#include "include.h"
#include "arm_arch.h"
#include "target_util_pub.h"
#include "mem_pub.h"
#include "uart_pub.h"
#include "sys_rtos.h"
#include "rtos_pub.h"
#include "error.h"
#include "fake_clock_pub.h"
#include "wlan_ui_pub.h"
#include "monitor.h"
#include "mac_ie.h"
#include "mac_frame.h"

static beken_timer_t mtr_chan_timer;
static beken_thread_t mtr_thread_handle = NULL;
static beken_semaphore_t mtr_semaphore = NULL;
volatile u8 g_mtr_exit = 0;
static monitor_channel_t g_mtr_channels;
/*Check whether softap receive station's auth frame*/
static int g_has_softap_rx_auth = 0;
/*Record data frame count during mtr test*/
static u32 g_mtr_data_count = 0;
#if CFG_AP_MONITOR_COEXIST_TBTT
/*Next mtr scan channel time*/
static u32 g_next_mtr_channel_time_ms = 0;
/*Mtr scan channel at ttbt*/
static u16 g_mtr_channel_at_tbtt;
#endif

/**
 *******************************************************************************************************
 * @brief  Callback called upon rx frame
 *******************************************************************************************************
 */
static void monitor_frame_callback(uint8_t *data, int len, wifi_link_info_t *info)
{
    struct mac_hdr *fmac_hdr;

    if(!data || !len)
        return;

    fmac_hdr = (struct mac_hdr *)data;

    if(MAC_FCTRL_DATA_T == (fmac_hdr->fctl & MAC_FCTRL_TYPE_MASK)) {
        g_mtr_data_count++;
    } else if (MAC_FCTRL_AUTHENT == (fmac_hdr->fctl & MAC_FCTRL_TYPESUBTYPE_MASK)) {
        uint8_t softap_mac[ETH_ALEN] = {0}, bssid_mac[ETH_ALEN] = {0};
        bk_wifi_get_softap_mac_address((char*)softap_mac);
        os_memcpy(bssid_mac, (u8 *)&fmac_hdr->addr1, ETH_ALEN);
        if (os_memcmp(softap_mac, bssid_mac, ETH_ALEN) == 0) {
            g_has_softap_rx_auth = 1;
        }
    }

    if(mtr_semaphore) {
        rtos_set_semaphore(&mtr_semaphore);
    }
}

#if CFG_AP_MONITOR_COEXIST_TBTT
/**
 *******************************************************************************************************
 * @brief calculate next mtr scan channel time which is used in tbtt and tbtt duration callback
 *******************************************************************************************************
 */
static void monitor_calc_time_for_next_monitor_channel(void)
{
    uint32_t tick_now = 0;

    beken_time_get_time(&tick_now);
    g_next_mtr_channel_time_ms = tick_now + MONITOR_SWITCH_TIMER;
}

/**
 *******************************************************************************************************
 * @brief Callback called upon the time after ttbt duration
 *******************************************************************************************************
 */
static void monitor_tbtt_dur_callback(void)
{
    uint32_t tick_now = 0;
    u16 cur_channel = g_mtr_channels.channel_list[g_mtr_channels.cur_channel_idx];

    beken_time_get_time(&tick_now);

    /*check switch to cur_channel(tbtt duration callback channel) conditions:
    1: g_mtr_channel_at_tbtt
    if same, fit the condition, if different, the channel has switched by mtr_chan_timer.
    2: softap channel
    if different, fit the condition. if same, already in the softap channel.
    3: MONITOR_CHANNEL_SWITCH_TIMER_MARGIN
    ensure switch channel has sufficient time*/
    if (cur_channel == g_mtr_channel_at_tbtt&&
        cur_channel != g_mtr_channels.softap_channel &&
        tick_now + MONITOR_CHANNEL_SWITCH_TIMER_MARGIN < g_next_mtr_channel_time_ms) {
        bk_wlan_set_channel_sync(cur_channel);
    }
}

/**
 *******************************************************************************************************
 * @brief Callback called upon tbtt is coming
 *******************************************************************************************************
 */
static void monitor_tbtt_callback(void)
{
    int ret;
    uint32_t tick_now = 0;

    beken_time_get_time(&tick_now);
    g_mtr_channel_at_tbtt = g_mtr_channels.channel_list[g_mtr_channels.cur_channel_idx];

    if (tick_now + MONITOR_TBTT_DUR_TIMER > g_next_mtr_channel_time_ms) {
        /*in this condition, change next mtr scan channel time until tbtt_monitor_tbtt_dur timer end*/
        if (rtos_is_timer_running(&mtr_chan_timer)) {
            ret = rtos_change_period(&mtr_chan_timer,
                                  tick_now + MONITOR_TBTT_DUR_TIMER - g_next_mtr_channel_time_ms);
            ASSERT(kNoErr == ret);
        }
    }
}
#endif

/**
 *******************************************************************************************************
 * @brief mtr channel software timer is uesd to switch channel
 *******************************************************************************************************
 */
static void monitor_switch_channel_callback(void *data)
{
    int ret;
    u16 channel = 0;

    //1: update channel info
    g_mtr_channels.cur_channel_idx++;

    if(g_mtr_channels.cur_channel_idx >= g_mtr_channels.all_channel_nums)
        g_mtr_channels.cur_channel_idx = 0;

    channel = g_mtr_channels.channel_list[g_mtr_channels.cur_channel_idx];
    //2: switch to next channel
    MONITOR_INFO("start scan ch:%02d/%02d\r\n", g_mtr_channels.cur_channel_idx, channel);
    bk_wlan_set_channel_sync(channel);

    if (!g_mtr_exit) {
        ret = rtos_change_period(&mtr_chan_timer, MONITOR_SWITCH_TIMER);
        ASSERT(kNoErr == ret);
#if CFG_AP_MONITOR_COEXIST_TBTT
        monitor_calc_time_for_next_monitor_channel();
#endif
    }
}

static void monitor_init_scan_channels(void)
{
    int i;

    g_mtr_channels.all_channel_nums = MONITOR_MAX_CHANNELS;
    for(i = 0; i < g_mtr_channels.all_channel_nums; i++) {
        g_mtr_channels.channel_list[i] = i + 1;
    }
    g_mtr_channels.cur_channel_idx = 0;

    g_mtr_channels.softap_channel = bk_wlan_ap_get_channel_config();
}

static void monitor_register_cb(void)
{
    bk_wlan_register_monitor_cb(NULL);
    bk_wlan_register_monitor_cb(monitor_frame_callback);
#if CFG_AP_MONITOR_COEXIST_TBTT
    bk_wlan_register_tbtt_cb(NULL);
    bk_wlan_register_tbtt_cb(monitor_tbtt_callback);
    bk_wlan_register_transmitted_bcn_cb(NULL);
    bk_wlan_register_transmitted_bcn_cb(monitor_tbtt_dur_callback);
#endif
}

static void monitor_unregister_cb(void)
{
    bk_wlan_register_monitor_cb(NULL);
#if CFG_AP_MONITOR_COEXIST_TBTT
    bk_wlan_register_tbtt_cb(NULL);
    bk_wlan_register_transmitted_bcn_cb(NULL);
#endif
}

static void monitor_init(void)
{
    int result;

    result = rtos_init_timer(&mtr_chan_timer,
                         MONITOR_SWITCH_TIMER,
                         monitor_switch_channel_callback,
                         (void *)0);
    ASSERT(kNoErr == result);

    monitor_init_scan_channels();
}

static void monitor_deinit(void)
{
    int result;

    result = rtos_stop_timer(&mtr_chan_timer);
    ASSERT(kNoErr == result);
    result = rtos_deinit_timer_block(&mtr_chan_timer);
    ASSERT(kNoErr == result);

    rtos_deinit_semaphore(&mtr_semaphore);
    mtr_semaphore = NULL;

    mtr_thread_handle = NULL;
    rtos_delete_thread(NULL);
}

static void monitor_scan_start(void)
{
    int result;

    bk_wlan_stop_monitor();
    monitor_register_cb();
    bk_wlan_start_monitor();
#if CFG_AP_MONITOR_COEXIST_TBTT
    bk_wlan_ap_monitor_coexist_tbtt_duration(MONITOR_TBTT_DUR_TIMER);
    bk_wlan_ap_monitor_coexist_tbtt_enable();
#endif
// start from first channel
    bk_wlan_set_channel_sync(g_mtr_channels.channel_list[g_mtr_channels.cur_channel_idx]);

    result = rtos_start_timer(&mtr_chan_timer);
    ASSERT(kNoErr == result);

#if CFG_AP_MONITOR_COEXIST_TBTT
    monitor_calc_time_for_next_monitor_channel();
#endif
}

static void monitor_scan_end(void)
{
#if CFG_AP_MONITOR_COEXIST_TBTT
    bk_wlan_ap_monitor_coexist_tbtt_disable();
#endif
    bk_wlan_stop_monitor();
    monitor_unregister_cb();

    MONITOR_PRT("monitor statistics: receive data frame count is %u\r\n", g_mtr_data_count);
    g_mtr_data_count = 0;
    g_has_softap_rx_auth = 0;
//restore to softap channel
    bk_wlan_set_channel_sync(g_mtr_channels.softap_channel);
}

static void monitor_main( void *arg )
{
    int result;

    monitor_init();
    monitor_scan_start();

    g_mtr_exit = 0;
    while (g_mtr_exit == 0) {
        result = rtos_get_semaphore(&mtr_semaphore, BEKEN_WAIT_FOREVER);
        if (result == kNoErr) {
            if (g_has_softap_rx_auth == 1) {
                break;
            }
        }
    }

    monitor_scan_end();
    monitor_deinit();
}

static uint32_t monitor_start(void)
{
    uint32_t ret = kNoErr;

    if(NULL == mtr_semaphore) {
        ret = rtos_init_semaphore(&mtr_semaphore, 1);
        ASSERT(kNoErr == ret);
    }

    if (NULL == mtr_thread_handle) {
        ret = rtos_create_thread(&mtr_thread_handle,
                               BEKEN_DEFAULT_WORKER_PRIORITY,
                               "monitor",
                               (beken_thread_function_t)monitor_main,
                               4096,
                               (beken_thread_arg_t)0);
        if (ret != kNoErr) {
            MONITOR_FATAL("Error: monitor_start_process: %d\r\n", ret);
            ret = kGeneralErr;
        }
    }

    return ret;
}

static uint32_t monitor_stop(void)
{
    GLOBAL_INT_DECLARATION();

    if (mtr_thread_handle && mtr_semaphore) {
        GLOBAL_INT_DISABLE();
        g_mtr_exit = 1;
        GLOBAL_INT_RESTORE();
    }

    return kNoErr;
}

u32 monitor_process(u8 start)
{
    int ret;

    MONITOR_PRT("monitor_process:%d\r\n", start);

    if (start)
        ret = monitor_start();
    else
        ret = monitor_stop();

    return ret;
}
//eof
