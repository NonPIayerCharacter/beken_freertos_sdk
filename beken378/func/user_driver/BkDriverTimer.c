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
#include "rtos_pub.h"
#include "drv_model_pub.h"
#include "bk_timer_pub.h"

/**@brief Initialises a timer and start
 *
 * @note  user can user timer0 timer1 timer2 timer4 timer5
 *
 * @param timer_id      : the timer id
 * @param time_ms       : time value(unit:ms)
 * @param callback      : callback
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_timer_initialize(uint8_t timer_id, uint32_t time_ms, void *callback)
{
    UINT32 ret;
    timer_param_t param;

    param.channel = timer_id;
    param.div = 1;              //timer0 timer1 timer2 26M // timer4 timer5 32K (n+1) division
    param.period = time_ms;
    param.t_Int_Handler= callback;

    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_INIT_PARAM, &param);
    ASSERT(BK_TIMER_SUCCESS == ret);

    UINT32 timer_channel;
    timer_channel = param.channel;
    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_ENABLE, &timer_channel);
    ASSERT(BK_TIMER_SUCCESS == ret);

    return kNoErr;
}


OSStatus bk_timer_initialize_us(uint8_t timer_id, uint32_t time_us, void *callback)
{
    UINT32 ret;
    timer_param_t param;

    param.channel = timer_id;
    param.div = 1;              //timer0 timer1 timer2 26M // timer4 timer5 32K (n+1) division
    param.period = time_us;
    param.t_Int_Handler= callback;

    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_INIT_PARAM_US, &param);
    ASSERT(BK_TIMER_SUCCESS == ret);

    UINT32 timer_channel;
    timer_channel = param.channel;
    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_ENABLE, &timer_channel);
    ASSERT(BK_TIMER_SUCCESS == ret);

    return kNoErr;
}

UINT32 bk_get_timer_cnt(uint8_t timer_id)
{
    timer_param_t param;

    param.channel = timer_id;

    if(sddev_control(TIMER_DEV_NAME, CMD_TIMER_READ_CNT, &param) != BK_TIMER_SUCCESS) {
        return 0xFFFFFFFFU;
    }

    return param.period;
}

/**@brief stop timer
 *
 * @note  user can user timer0 timer1 timer2 timer4 timer5
 *
 * @param timer_id      : the timer id
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_timer_stop(uint8_t timer_id)
{
    UINT32 ret;
    UINT32 timer_channel;

    timer_channel = timer_id;
    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_DISABLE, &timer_channel);
    ASSERT(BK_TIMER_SUCCESS == ret);
    return kNoErr;
}

#if BKDRIVERTIMRE_TEST_DEMO
static void bk_timer_test_isr_cb(UINT8 arg)
{
    bk_printf("%s %d rtos-time: %d mS\r\n",__FUNCTION__,__LINE__,rtos_get_time());
}

void bk_timer_test_start(void)
{
    bk_timer_initialize(BKTIMER5,1000,bk_timer_test_isr_cb);
}

void user_main(void)
{
    bk_printf("%s %s\r\n",__FILE__,__FUNCTION__);
    bk_timer_test_start();
}

#endif
// eof

