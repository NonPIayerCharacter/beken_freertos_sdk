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
#include "fake_clock_pub.h"
#include "pwm_pub.h"
#include "icu_pub.h"
#include "drv_model_pub.h"
#include "uart_pub.h"

#include <rtthread.h>
#include "bk_timer_pub.h"
#include "power_save_pub.h"
#include "bk_timer.h"

#if CFG_USE_MCU_PS
#include "mcu_ps_pub.h"
#endif

#if !(CFG_SOC_NAME == SOC_BK7252N)
#include "calendar_pub.h"
#else
#include "rtc_reg_pub.h"
#endif
static volatile UINT32 current_clock = 0;
static volatile UINT32 current_seconds = 0;
static UINT32 second_countdown = FCLK_SECOND;
static BK_HW_TIMER_INDEX fclk_id = BK_PWM_TIMER_ID0;

extern void mcu_ps_increase_clr(void);
#if (CFG_SOC_NAME != SOC_BK7231) && (CFG_SOC_NAME != SOC_BK7271)
static CAL_TICK_T cal_tick_save;
#endif
UINT32 use_cal_net = 0;
__maybe_unused static UINT32 fclk_freertos_update_tick(UINT32 tick);

static void fclk_hdl(UINT8 param)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();

    current_clock ++;

    rt_tick_increase();
    GLOBAL_INT_RESTORE();

    if (--second_countdown == 0)
    {
        current_seconds ++;
        second_countdown = FCLK_SECOND;

        #if defined(RT_USING_ALARM)
        rt_alarm_update(NULL, 0);
        #endif
    }
}

static UINT32 fclk_freertos_update_tick(UINT32 tick)
{
    current_clock += tick;

    while(tick >= FCLK_SECOND)
    {
        current_seconds ++;
        tick -= FCLK_SECOND;
    }

    if(second_countdown <= tick)
    {
        current_seconds ++;
        second_countdown = FCLK_SECOND - (tick - second_countdown);
    }
    else
    {
        second_countdown -= tick;
    }

    return 0;
}

#if (CFG_SUPPORT_RTT)
UINT32 rtt_update_tick(UINT32 tick)
{
    if (tick)
    {
        rt_enter_critical();

        /* adjust OS tick */
        rt_tick_set(rt_tick_get() + tick);
        /* check system timer */
        rt_timer_check();
        rt_exit_critical();
    }
    return 0;
}
#endif

UINT32 fclk_update_tick(UINT32 tick)
{
    rtt_update_tick(tick);
    return 0;
}

UINT32 fclk_freertos_get_tick(void)
{
    return current_clock;
}

UINT64 fclk_get_tick(void)
{
    UINT64 fclk;
    fclk = (UINT64)rt_tick_get();
    return fclk;
}

UINT32 fclk_get_second(void)
{
    return (rt_tick_get()/FCLK_SECOND);
}

UINT32 fclk_from_sec_to_tick(UINT32 sec)
{
    return sec * FCLK_SECOND;
}

void fclk_reset_count(void)
{
    current_clock = 0;
    current_seconds = 0;
}

#if CFG_USE_TICK_CAL
/// save last tick for runtime tick calibration
static CAL_TICK_T cal_tick_save;
/// indicate whether use tsf to calibrate tick
//UINT32 use_cal_net = 0;

/* Forward Declaration */
#if (0 == CFG_LOW_VOLTAGE_PS)
void cal_timer_set(void);
void cal_timer_deset(void);
#else
void fclk_cal_init(void);
#endif

/**
 * Init os tick calibration.
 * when normal sleep    use 26M & mac & tsf
 * when low voltage     use 32k
 * @param setting indicates whether use tsf to calibrate tick
*/
UINT32 bk_cal_init(UINT32 setting)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();

    if(1 == setting)
    {
        use_cal_net = 1;
        #if (0 == CFG_LOW_VOLTAGE_PS)
        cal_timer_deset();
        mcu_ps_machw_init();
        #else
        fclk_cal_init();
        #endif
        #if (CFG_OS_FREERTOS)
        os_printf("decset:%d %d %d %d\r\n",use_cal_net, fclk_freertos_get_tick32(), fclk_get_second(), xTaskGetTickCount());
        #endif
    }
    else
    {
        use_cal_net = 0;
        #if (0 == CFG_LOW_VOLTAGE_PS)
        mcu_ps_machw_cal();
        cal_timer_set();
        mcu_ps_machw_reset();
        #else
        fclk_cal_init();
        #endif
        #if (CFG_OS_FREERTOS)
        os_printf("cset:%d %d %d %d\r\n",use_cal_net, fclk_freertos_get_tick32(), fclk_get_second(), xTaskGetTickCount());
        #endif
    }
    GLOBAL_INT_RESTORE();

    return 0;
}

#if (0 == CFG_LOW_VOLTAGE_PS)
/**
 * use 26M timer to calibrate tick
 * timer period setted to 15s
*/
UINT32 timer_cal_init(void)
{
    UINT32 fclk;

    fclk = BK_TICKS_TO_MS(fclk_get_tick());

    cal_tick_save.fclk_tick = fclk;
    cal_tick_save.tmp1 = 0;
    return 0;
}

extern int increase_tick;
UINT32 timer_cal_tick(void)
{
    UINT32 fclk, tmp2;
    UINT32 machw = 0;
    INT32 lost;
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    fclk = BK_TICKS_TO_MS(fclk_get_tick());
    cal_tick_save.tmp1 += ONE_CAL_TIME;

    tmp2 = fclk;

    lost = (INT32)(cal_tick_save.tmp1  - (UINT32)tmp2);

    if((lost >= (2*FCLK_DURATION_MS)))
    {
        if(lost > 200)
        {
            //os_printf("m cal_:%x %x\r\n", lost, machw);
        }

        lost -= FCLK_DURATION_MS;
        fclk_update_tick(BK_MS_TO_TICKS(lost));
        increase_tick = 0;
    }
    #if 1
    else
    {
        if(lost <= (-(2*FCLK_DURATION_MS)))
        {
            if(lost < (-50000))
            {
                os_printf("m reset:%x %x\r\n", lost, machw);
            }
            increase_tick = lost + FCLK_DURATION_MS;
        }
    }
    #endif
    //os_printf("tc:%d\r\n",lost);

    #if CFG_USE_MCU_PS
    mcu_ps_machw_init();
    #endif
    GLOBAL_INT_RESTORE();
    return 0 ;
}


void cal_timer_hdl(UINT8 param)
{
    #if CFG_USE_MCU_PS
    timer_cal_tick();
    #endif
}

void cal_timer_set(void)
{
    timer_param_t param;
    UINT32 ret;
    UINT32 timer_channel;

    timer_cal_init();

    #if (CFG_SOC_NAME == SOC_BK7231)
    #else
    param.channel = CAL_TIMER_ID;
    param.div = 1;
    param.period = ONE_CAL_TIME;
    param.t_Int_Handler= cal_timer_hdl;

    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_INIT_PARAM, &param);
    ASSERT(BK_TIMER_SUCCESS == ret);
    timer_channel = param.channel;
    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_ENABLE, &timer_channel);
    ASSERT(BK_TIMER_SUCCESS == ret);
    #endif
}

void cal_timer_deset(void)
{
    UINT32 ret;
    UINT32 timer_channel;

    #if (CFG_SOC_NAME == SOC_BK7231)
    #else
    timer_channel = CAL_TIMER_ID;
    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_DISABLE, &timer_channel);
    ASSERT(BK_TIMER_SUCCESS == ret);
    #endif
    timer_cal_init();
}
#else
/**
 * Use 32K time_us to calibrate tick by calculate
 * the lost tick, please notice that there are two
 * places (fclk_hdl & lv_ps_sleep_check) we do tick cal.
*/
void fclk_cal_init(void)
{
    UINT64 fclk, time_us;

    fclk = BK_TICKS_TO_MS(fclk_get_tick());
    #if !(CFG_SOC_NAME == SOC_BK7252N)
    time_us = cal_get_time_us();
    #else
    time_us = rtc_reg_get_time_us();
    #endif

    cal_tick_save.fclk_tick = fclk;
    cal_tick_save.time_us = time_us;
}

void fclk_cal_tick(void)
{
    UINT64 delta_fclk, delta_time;
    INT32 lost;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();

    delta_fclk = fclk_get_tick() - cal_tick_save.fclk_tick;
    #if !(CFG_SOC_NAME == SOC_BK7252N)
    delta_time = cal_get_time_us() - cal_tick_save.time_us;
    #else
    delta_time = rtc_reg_get_time_us() - cal_tick_save.time_us;
    #endif

    lost = (INT32)(delta_time/1000 - BK_TICKS_TO_MS(delta_fclk));
    os_null_printf("tick lost:%d\r\n", lost);

    if( lost >= (2*FCLK_DURATION_MS) )
    {
        lost -= FCLK_DURATION_MS;
        fclk_update_tick(BK_MS_TO_TICKS(lost));
    }
    else if( lost < 0 )
    {
        os_null_printf("tick go fast:%d\r\n", lost);
    }

    GLOBAL_INT_RESTORE();
}
#endif
#endif

UINT32 fclk_cal_endvalue(UINT32 mode)
{
    UINT32 value = 1;

    if(PWM_CLK_32K == mode)
    {
        /*32k clock*/
        value = FCLK_DURATION_MS * 32;
    }
    else if(PWM_CLK_26M == mode)
    {
        /*26m clock*/
        value = CFG_XTAL_FREQUENCE / RT_TICK_PER_SECOND;
    }

    return value;
}

BK_HW_TIMER_INDEX fclk_get_tick_id(void)
{
    return fclk_id;
}

/*timer_id:BK_PWM_TIMER_ID0 or BK_TIMER_ID3*/
void fclk_timer_hw_init(BK_HW_TIMER_INDEX timer_id)
{
    UINT32 ret;

    #if (CFG_SOC_NAME == SOC_BK7231)
    ASSERT(timer_id>= BK_PWM_TIMER_ID0);
    #endif

    fclk_id = timer_id;

    if(fclk_id >= BK_PWM_TIMER_ID0)
    {   //pwm timer
        pwm_param_t param;

        /*init pwm*/
        param.channel         = (fclk_id - PWM0);
        param.cfg.bits.en     = PWM_ENABLE;
        param.cfg.bits.int_en = PWM_INT_EN;
        param.cfg.bits.mode   = PWM_TIMER_MODE;

        #if(CFG_RUNNING_PLATFORM == FPGA_PLATFORM)  // FPGA:PWM0-2-32kCLK, pwm3-5-24CLK
        param.cfg.bits.clk    = PWM_CLK_32K;
        #else
        param.cfg.bits.clk    = PWM_CLK_26M;
        #endif

        param.p_Int_Handler   = fclk_hdl;
        #if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
        param.duty_cycle1     = 0;
        #else
        param.duty_cycle      = 0;
        #endif
        param.end_value       = fclk_cal_endvalue((UINT32)param.cfg.bits.clk);

        ret = sddev_control(PWM_DEV_NAME, CMD_PWM_INIT_PARAM, &param);
        ASSERT(PWM_SUCCESS == ret);
    }
    else
    {   //timer
        timer_param_t param;
        param.channel = fclk_id;
        param.div = 1;
        param.period = FCLK_DURATION_MS;
        param.t_Int_Handler= fclk_hdl;

        ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_INIT_PARAM, &param);
        ASSERT(BK_TIMER_SUCCESS == ret);
        UINT32 timer_channel;
        timer_channel = param.channel;
        ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_ENABLE, &timer_channel);
        ASSERT(BK_TIMER_SUCCESS == ret);
    }
}

void os_clk_init(void)
{
    #if (CFG_SOC_NAME == SOC_BK7231)
    fclk_timer_hw_init(BK_PWM_TIMER_ID0);
    #else
    fclk_timer_hw_init(BK_TIMER_ID3);
    #endif

    #if CFG_USE_TICK_CAL
    bk_cal_init(0);
    #endif
}
// eof

