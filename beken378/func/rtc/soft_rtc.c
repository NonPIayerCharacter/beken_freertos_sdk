/*
* File      : soft_rtc.c
* This file is part of RT-Thread RTOS
* COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
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
* 2018-01-30     armink       the first version
*/
#include "include.h"
#if (CFG_USE_SOFT_RTC)
#include "rtc_time.h"
#include <string.h>

#include <rtc.h>
#include "drv_model_pub.h"
#include "fake_clock_pub.h"

/* 2018-01-30 14:44:50 = RTC_TIME_INIT(2018, 1, 30, 14, 44, 50)  */
#define RTC_TIME_INIT(year, month, day, hour, minute, second)        \
    {.tm_year = year - 1900, .tm_mon = month - 1, .tm_mday = day, .tm_hour = hour, .tm_min = minute, .tm_sec = second}

#ifndef SOFT_RTC_TIME_DEFAULT
#define SOFT_RTC_TIME_DEFAULT        RTC_TIME_INIT(DEFAULT_YEAR, DEFAULT_MONTH, DEFAULT_DAY, DEFAULT_HOUR, DEFAULT_MIN ,DEFAULT_SEC)
#endif

static UINT64 init_tick;
static time_t_at init_time;

static UINT32 soft_rtc_control(UINT32 cmd, void *args)
{
    time_t_at *time = (time_t_at *) args;
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        *time = init_time + (fclk_get_tick() - init_tick) / TICK_PER_SECOND;
        break;
    case RT_DEVICE_CTRL_RTC_GET_MS:
        *time = (fclk_get_tick() - init_tick) * FCLK_DURATION_MS;
        break;
    case RT_DEVICE_CTRL_RTC_SET_TIME:
        init_time = *time - (fclk_get_tick() - init_tick) / TICK_PER_SECOND;
        break;
    }

    return 0;
}

static SDD_OPERATIONS soft_rtc_ops = {
    soft_rtc_control
};

void rt_soft_rtc_init(void)
{
    struct tm_at time_new = SOFT_RTC_TIME_DEFAULT;
    init_tick = fclk_get_tick();
    init_time = mktime_at(&time_new);

    sddev_register_dev(SOFT_RTC_DEVICE_NAME, &soft_rtc_ops);
}

#endif  // CFG_USE_SOFT_RTC
