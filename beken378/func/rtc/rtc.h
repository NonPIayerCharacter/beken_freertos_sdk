/*
 * File      : rtc.h
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
 * 2012-10-10     aozima       first version.
 */

#ifndef __RTC_H__
#define __RTC_H__
#include "typedef.h"

#define SOFT_RTC_DEVICE_NAME			"soft_rtc"

#define RT_DEVICE_CTRL_RTC_GET_TIME     0x10            /**< get time */
#define RT_DEVICE_CTRL_RTC_SET_TIME     0x11            /**< set time */
#define RT_DEVICE_CTRL_RTC_GET_MS       0x12

#define DEFAULT_YEAR 2021
#define DEFAULT_MONTH 1
#define DEFAULT_DAY 1
#define DEFAULT_HOUR 0
#define DEFAULT_MIN	0
#define DEFAULT_SEC 0

UINT32 set_date(UINT32 year, UINT32 month, UINT32 day);
UINT32 set_time(UINT32 hour, UINT32 minute, UINT32 second);

void rt_soft_rtc_init(void);
void rt_rtc_ntp_sync_init(void);

#endif /* __RTC_H__ */
