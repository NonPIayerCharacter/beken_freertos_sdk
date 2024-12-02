/*
 * Tencent is pleased to support the open source community by making IoT Hub
 available.
 * Copyright (C) 2018-2020 THL A29 Limited, a Tencent company. All rights
 reserved.

 * Licensed under the MIT License (the "License"); you may not use this file
 except in
 * compliance with the License. You may obtain a copy of the License at
 * http://opensource.org/licenses/MIT

 * Unless required by applicable law or agreed to in writing, software
 distributed under the License is
 * distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 KIND,
 * either express or implied. See the License for the specific language
 governing permissions and
 * limitations under the License.
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "network_interface.h"

#define PLATFORM_HAS_TIME_FUNCS
//#define PLATFORM_HAS_CMSIS

#ifdef PLATFORM_HAS_TIME_FUNCS
#include <sys/time.h>
#include <time.h>
#endif

#ifdef PLATFORM_HAS_CMSIS
#include "cmsis_os.h"
#include "stm32l4xx_hal.h"
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "atsvr_error.h"

extern int gettimeofday(struct timeval *tp, void *ignore);
extern int gettimeofday_ms_at(void);

uint32_t HAL_GetTimeMs(void)
{
	#if defined PLATFORM_HAS_TIME_FUNCS
	uint32_t ret;
	ret = (uint32_t)gettimeofday_ms_at();
	return ret;
	#elif defined PLATFORM_HAS_CMSIS
	return HAL_GetTick();
	#endif
}

/*Get timestamp*/
long HAL_Timer_current_sec(void)
{
	return HAL_GetTimeMs() / 1000;
}

char *HAL_Timer_current(char *time_str)
{
	#if defined PLATFORM_HAS_TIME_FUNCS
	struct timeval tv;
	gettimeofday(&tv, NULL);
	time_t    now_time = tv.tv_sec;
	struct tm tm_tmp   = *localtime(&now_time);
	strftime(time_str, TIME_FORMAT_STR_LEN, "%F %T", &tm_tmp);
	return time_str;
	#else
	long time_sec;
	time_sec = HAL_Timer_current_sec();
	memset(time_str, 0, TIME_FORMAT_STR_LEN);
	snprintf(time_str, TIME_FORMAT_STR_LEN, "%ld", time_sec);
	return time_str;
	#endif
}

bool HAL_Timer_expired(Timer *timer)
{
	uint32_t now_ts;
	now_ts = HAL_GetTimeMs();
	return (now_ts > timer->end_time) ? true : false;
}

void HAL_Timer_countdown_ms(Timer *timer, unsigned int timeout_ms)
{
	timer->end_time = HAL_GetTimeMs();
	timer->end_time += timeout_ms;
}

void HAL_Timer_countdown(Timer *timer, unsigned int timeout)
{
	timer->end_time = HAL_GetTimeMs();
	timer->end_time += timeout * 1000;
}

int HAL_Timer_remain(Timer *timer)
{
	return (int)(timer->end_time - HAL_GetTimeMs());
}

void HAL_Timer_init(Timer *timer)
{
	timer->end_time = 0;
}

void *HAL_MutexCreate(void)
{
	SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
	if (NULL == mutex) {
		ATSVRLOG("%s: xSemaphoreCreateMutex failed\n", __FUNCTION__);
		return NULL;
	}

	return mutex;
}

void HAL_MutexDestroy(void *mutex)
{
	if (xSemaphoreTake(mutex, 0) != pdTRUE) {
		ATSVRLOG("%s: xSemaphoreTake failed\n", __FUNCTION__);
	}

	vSemaphoreDelete(mutex);
}

void HAL_MutexLock(void *mutex)
{
	if (!mutex) {
		ATSVRLOG("%s: invalid mutex\n", __FUNCTION__);
		return;
	}

	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
		ATSVRLOG("%s: xSemaphoreTake failed\n", __FUNCTION__);
		return;
	}
}

int HAL_MutexTryLock( void *mutex)
{

	if (!mutex) {
		ATSVRLOG("%s: invalid mutex\n", __FUNCTION__);
		return -1;
	}

	if (xSemaphoreTake(mutex, 0) != pdTRUE) {
		ATSVRLOG("%s: xSemaphoreTake failed\n", __FUNCTION__);
		return -1;
	}

	return 0;
}

void HAL_MutexUnlock(void *mutex)
{
	if (!mutex) {
		ATSVRLOG("%s: invalid mutex\n", __FUNCTION__);
		return;
	}

	if (xSemaphoreGive(mutex) != pdTRUE) {
		ATSVRLOG("%s: xSemaphoreGive failed\n", __FUNCTION__);
		return;
	}
	return;
}

// platform-dependant thread routine/entry function
static void _HAL_thread_func_wrapper_(void *ptr)
{
	ThreadParams *params = (ThreadParams *)ptr;

	params->thread_func(params->user_arg);

	vTaskDelete(NULL);
}

// platform-dependant thread create function
int HAL_ThreadCreate(ThreadParams *params)
{
	if (params == NULL)
		return ATSVR_ERR_INVAL;

	if (params->thread_name == NULL) {
		ATSVRLOG("thread name is required for FreeRTOS platform!\n");
		return ATSVR_ERR_INVAL;
	}

	int ret = xTaskCreate(_HAL_thread_func_wrapper_, params->thread_name, params->stack_size, (void *)params,
						  params->priority, (void *)&params->thread_id);
	if (ret != pdPASS) {
		ATSVRLOG("%s: xTaskCreate failed: %d\n", __FUNCTION__, ret);
		return ATSVR_ERR_FAILURE;
	}

	return ATSVR_RET_SUCCESS;
}

int HAL_ThreadDestroy(void *thread_t)
{
	vTaskDelete(thread_t);
	return 0;
}

/*
void *HAL_SemaphoreCreate(void)
{
	return (void *)osSemaphoreCreate(NULL, 1);
}

void HAL_SemaphoreDestroy(void *sem)
{
	osStatus ret;

	ret = osSemaphoreDelete((osSemaphoreId)sem);
	if (osOK != ret) {
		ATSVRLOG("HAL_SemaphoreDestroy err, err:%d\n\r", ret);
	}
}

void HAL_SemaphorePost(void *sem)
{
	osStatus ret;

	ret = osSemaphoreRelease((osSemaphoreId)sem);

	if (osOK != ret) {
		ATSVRLOG("HAL_SemaphorePost err, err:%d\n\r", ret);
	}
}

int HAL_SemaphoreWait(void *sem, uint32_t timeout_ms)
{
	return osSemaphoreWait((osSemaphoreId)sem, timeout_ms);
}*/

#ifdef __cplusplus
}
#endif
