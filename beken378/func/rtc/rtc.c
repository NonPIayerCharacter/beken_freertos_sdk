/*
 * File      : rtc.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-01-29     aozima       first version.
 * 2012-04-12     aozima       optimization: find rtc device only first.
 * 2012-04-16     aozima       add scheduler lock for set_date and set_time.
 * 2018-02-16     armink       add auto sync time by NTP
 */

#include "rtc_time.h"
#include <string.h>
#include "rtos_pub.h"
#include "rtc.h"
#include "fake_clock_pub.h"
#include "drv_model_pub.h"

/* NTP first sync delay time for network connect, unit: second */
#ifndef RTC_NTP_FIRST_SYNC_DELAY
#define RTC_NTP_FIRST_SYNC_DELAY                 (30)
#endif
/* NTP sync period, unit: second */
#ifndef RTC_NTP_SYNC_PERIOD
#define RTC_NTP_SYNC_PERIOD                      (10L*60L)/*(1L*60L*60L)*/
#endif



/**
 * Set system date(time not modify).
 *
 * @param rt_uint32_t year  e.g: 2012.
 * @param rt_uint32_t month e.g: 12 (1~12).
 * @param rt_uint32_t day   e.g: 31.
 *
 * @return rt_err_t if set success, return RT_EOK.
 *
 */
UINT32 set_date(UINT32 year, UINT32 month, UINT32 day)
{
    time_t_at now;
    struct tm_at *p_tm;
    struct tm_at tm_new;
    UINT32 ret = -1;

    /* get current time */
    now = time_at(NULL);

    /* lock scheduler. */
    /* converts calendar time time into local time. */
    p_tm = localtime_at(&now);
    /* copy the statically located variable */
    memcpy(&tm_new, p_tm, sizeof(struct tm_at));
    /* unlock scheduler. */

    /* update date. */
    tm_new.tm_year = year - 1900;
    tm_new.tm_mon  = month - 1; /* tm_mon: 0~11 */
    tm_new.tm_mday = day;

    /* converts the local time in time to calendar time. */
    now = mktime_at(&tm_new);

	ret = sddev_control(SOFT_RTC_DEVICE_NAME, RT_DEVICE_CTRL_RTC_SET_TIME, &now);

    return ret;
}

/**
 * Set system time(date not modify).
 *
 * @param rt_uint32_t hour   e.g: 0~23.
 * @param rt_uint32_t minute e.g: 0~59.
 * @param rt_uint32_t second e.g: 0~59.
 *
 * @return rt_err_t if set success, return RT_EOK.
 *
 */
UINT32 set_time(UINT32 hour, UINT32 minute, UINT32 second)
{
    time_t_at now;
    struct tm_at *p_tm;
    struct tm_at tm_new;
    UINT32 ret = -1;

    /* get current time */
    now = time_at(NULL);

    /* lock scheduler. */
    /* converts calendar time time into local time. */
    p_tm = localtime_at(&now);
    /* copy the statically located variable */
    memcpy(&tm_new, p_tm, sizeof(struct tm_at));
    /* unlock scheduler. */


    /* update time. */
    tm_new.tm_hour = hour;
    tm_new.tm_min  = minute;
    tm_new.tm_sec  = second;

    /* converts the local time in time to calendar time. */
    now = mktime_at(&tm_new);

    ret = sddev_control(SOFT_RTC_DEVICE_NAME, RT_DEVICE_CTRL_RTC_SET_TIME, &now);

    return ret;
}

static long ntp_time=0;
static char ntp_time_need_updata_fg = 0;//0:no 1:need
void user_set_ntp_time(long time)
{
    //bk_printf("time=%d\r\n",time);
    ntp_time = time;
}
long  user_get_ntp_time()
{
    //	bk_printf("ntp_time=%d\r\n",ntp_time);
    return ntp_time;
}
void ntp_time_need_updata_set_fg(char fg)
{
    ntp_time_need_updata_fg = fg;
}
char ntp_time_need_updata_get_fg()
{
    return ntp_time_need_updata_fg;
}

static void ntp_sync_thread_enrty(void *param)
{
    time_t_at now;
    struct tm_at *time_now;
    int retry_cnt;
    extern time_t_at ntp_sync_to_rtc(void);

    while (1)
    {
        retry_cnt = 4;
        while(--retry_cnt)
        {
            if(ntp_sync_to_rtc()!= 0)
            {
                bk_printf("get time ok\r\n");
                break;
            }
            else
            {
                bk_printf("get time error\r\n");
                rtos_delay_milliseconds(50);
            }
        }
        sddev_control(SOFT_RTC_DEVICE_NAME, RT_DEVICE_CTRL_RTC_GET_TIME, &now);
        //		bk_printf("now=%d\r\n",now);
        ntp_time_need_updata_set_fg(1);
        user_set_ntp_time(now);
        time_now = localtime_at(&now);
        bk_printf("[NPT TIME]%d-%d-%d %d-%d-%d\r\n", time_now->tm_year + 1900, time_now->tm_mon + 1, time_now->tm_mday, time_now->tm_hour, time_now->tm_min, time_now->tm_sec);
        rtos_delay_milliseconds(RTC_NTP_SYNC_PERIOD * TICK_PER_SECOND);
    }
}

void rt_rtc_ntp_sync_init(void)
{
    static int init_ok = 0;
    if(init_ok == 0)
    {
        if(0 == rtos_create_thread(NULL, BEKEN_APPLICATION_PRIORITY, "ntp_sync", ntp_sync_thread_enrty, 1536, NULL))
            init_ok = 1;
        else
            bk_printf("[%s] thread create failed\n", __func__);
    }
    return;
}
