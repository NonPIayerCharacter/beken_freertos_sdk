#include "sys_rtos.h"
#include "fake_clock_pub.h"
#include "pwm_pub.h"
#include "bk_timer_pub.h"
#include "icu_pub.h"
#include "drv_model_pub.h"
#include "uart_pub.h"
#include "rtos_pub.h"
#include "power_save_pub.h"
#include "mcu_ps_pub.h"
#include "BkDriverWdg.h"

#if (CFG_SUPPORT_ALIOS)
#include "k_api.h"
#endif

#if CFG_SUPPORT_LITEOS
#include "bk_los_timer.h"
#endif
#include "calendar_pub.h"

#if(CFG_OS_FREERTOS && CFG_TASK_WDG_ENABLED)
#include "BkDriverTimer.h"
#include "bk_err.h"
#include "tasks.h"
#endif

static volatile UINT64 current_clock = 0;
static volatile UINT32 current_seconds = 0;
#if (CFG_OS_FREERTOS)
static UINT32 second_countdown = FCLK_SECOND;
#endif
static BK_HW_TIMER_INDEX fclk_id = BK_PWM_TIMER_ID0;

static inline UINT64 fclk_freertos_get_tick64(void);

#if (CFG_OS_FREERTOS)
#define INT_WDG_FEED_PERIOD_TICK ((BK_MS_TO_TICKS(CFG_INT_WDG_PERIOD_MS)) >> 4)
#define TASK_WDG_PERIOD_TICK (BK_MS_TO_TICKS(CFG_TASK_WDG_PERIOD_MS))

#if (CFG_INT_WDG_ENABLED)
void int_watchdog_feed(void)
{
        static UINT64 s_last_int_wdg_feed = 0;
        UINT64 current_tick = fclk_freertos_get_tick64();

        if ( (current_tick - s_last_int_wdg_feed) >= INT_WDG_FEED_PERIOD_TICK) {
                bk_wdg_reload();
                s_last_int_wdg_feed = current_tick;
                //os_printf("feed interrupt watchdog\n");
        }
}
#endif

#if (CFG_OS_FREERTOS && CFG_TASK_WDG_ENABLED)

#define WDT_BARK_TIME_MS    2000

static uint64_t s_last_task_wdt_feed_tick = 0;
static uint64_t s_last_task_wdt_log_tick = 0;

static bool s_wdt_driver_is_init =false;

void bk_task_wdt_feed()
{
	s_last_task_wdt_feed_tick = fclk_freertos_get_tick64();// fclk_get_tick();
}

void bk_task_wdt_timeout_check()
{
	if (s_last_task_wdt_feed_tick) {
		uint64_t current_tick = fclk_freertos_get_tick64();//fclk_get_tick();
		if ((current_tick - s_last_task_wdt_feed_tick) > TASK_WDG_PERIOD_TICK) {
			if ((current_tick - s_last_task_wdt_log_tick) > TASK_WDG_PERIOD_TICK) {
				bk_printf("task watchdog tiggered\n");
				s_last_task_wdt_log_tick = current_tick;
#if DUMP_THREAD_WHEN_TASK_WDG_TIGGERED
				rtos_dump_task_list();
#endif
#if DUMP_STACK_WHEN_TASK_WDG_TIGGERED
				rtos_dump_backtrace();
#endif
			}
		}
	}
}

void bk_wdt_feed_handle(void)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	bk_task_wdt_timeout_check();

	GLOBAL_INT_RESTORE();
}

bk_err_t bk_wdt_driver_init(void)
{
	if (s_wdt_driver_is_init) {
		return BK_OK;
	}

	bk_timer_initialize(BKTIMER4,WDT_BARK_TIME_MS,(void*)bk_wdt_feed_handle);

	s_wdt_driver_is_init = true;

	return BK_OK;
}
#endif
#endif //CFG_OS_FREERTOS

void fclk_hdl(UINT8 param)
{
#if CFG_USE_TICK_CAL && (0 == CFG_LOW_VOLTAGE_PS)
	if(!mcu_ps_need_pstick()) {
		return;
	}
#endif

#if (CFG_OS_FREERTOS)
        GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
	current_clock ++;

    if (--second_countdown == 0)
    {
        current_seconds ++;
        second_countdown = FCLK_SECOND;

#if CFG_USE_TICK_CAL && (1 == CFG_LOW_VOLTAGE_PS)
        if( current_seconds % 15 == 0) {
            fclk_cal_tick();
        }
#endif
    }

#if (CFG_INT_WDG_ENABLED)
        int_watchdog_feed();
#endif

#if (CFG_OS_FREERTOS && CFG_TASK_WDG_ENABLED)
        bk_wdt_driver_init();
#endif
	if (xTaskIncrementTick() != pdFALSE) {
		vTaskSwitchContext();
	}
	GLOBAL_INT_RESTORE();
#endif

#if (CFG_SUPPORT_ALIOS)
	krhino_tick_proc();
#endif
}

#if !(CFG_SUPPORT_RTT || CFG_SUPPORT_ALIOS || CFG_SUPPORT_LITEOS)
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
#endif

UINT32 fclk_update_tick(UINT32 tick)
{
#if (CFG_SUPPORT_RTT)
    rtt_update_tick(tick);
#elif (CFG_SUPPORT_ALIOS)
    krhino_update_sys_tick((UINT64)tick);
#elif CFG_OS_FREERTOS
    GLOBAL_INT_DECLARATION();

    if(tick == 0)
        return 0;

    GLOBAL_INT_DISABLE();
#if CFG_USE_TICK_CAL && (0 == CFG_LOW_VOLTAGE_PS)
    mcu_ps_increase_clr();
#endif
    fclk_freertos_update_tick(tick);
    vTaskStepTick( tick );
    GLOBAL_INT_RESTORE();
#endif
    return 0;
}

static inline UINT32 fclk_freertos_get_tick32(void)
{
    return (UINT32)current_clock;
}

static inline UINT64 fclk_freertos_get_tick64(void)
{
	return current_clock;
}

UINT64 fclk_get_tick(void)
{
    UINT64 fclk;
#if (CFG_SUPPORT_RTT)
    fclk = (UINT64)rt_tick_get();
#elif (CFG_SUPPORT_ALIOS)
    fclk = krhino_sys_tick_get();
#elif CFG_SUPPORT_LITEOS
	return rtos_get_tick_count();
#else
    fclk = fclk_freertos_get_tick32();
#endif
    return fclk;
}

UINT32 fclk_get_second(void)
{
#if (CFG_SUPPORT_RTT)
    return (rt_tick_get()/FCLK_SECOND);
#elif (CFG_SUPPORT_ALIOS)
    return (krhino_sys_tick_get()/FCLK_SECOND);
#elif CFG_SUPPORT_LITEOS
	return OsGetCurrSecond();
#else
    return fclk_freertos_get_tick32()/FCLK_SECOND;
#endif
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
UINT32 use_cal_net = 0;

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
    INT32 lost;
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    
    fclk = BK_TICKS_TO_MS(fclk_get_tick());
    cal_tick_save.tmp1 += ONE_CAL_TIME;

    tmp2 = fclk;

    lost = (INT32)(cal_tick_save.tmp1  - (UINT32)tmp2);

    if((lost >= (2*FCLK_DURATION_MS)))
    {
        lost -= FCLK_DURATION_MS;
        fclk_update_tick(BK_MS_TO_TICKS(lost));
        increase_tick = 0;
    }
    else
    {
        if(lost <= (-(2*FCLK_DURATION_MS)))
        {
            increase_tick = lost + FCLK_DURATION_MS;
            if(lost < (-50000))
            {
                os_printf("m reset:%d %d\r\n", lost, increase_tick);
            }
        }
    }
    
    mcu_ps_machw_init();
    GLOBAL_INT_RESTORE();
    return 0 ;
}


void cal_timer_hdl(UINT8 param)
{
    timer_cal_tick();
}

void cal_timer_set(void)
{
    timer_param_t param;
    UINT32 ret;
    UINT32 timer_channel;

    timer_cal_init();

    param.channel = CAL_TIMER_ID;
    param.div = 1;
    param.period = ONE_CAL_TIME;
    param.t_Int_Handler= cal_timer_hdl;

    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_INIT_PARAM, &param);
    ASSERT(BK_TIMER_SUCCESS == ret);
    timer_channel = param.channel;
    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_ENABLE, &timer_channel);
    ASSERT(BK_TIMER_SUCCESS == ret);
}

void cal_timer_deset(void)
{
    UINT32 ret;
    UINT32 timer_channel;

    timer_channel = CAL_TIMER_ID;
    ret = sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_DISABLE, &timer_channel);
    ASSERT(BK_TIMER_SUCCESS == ret);
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
    time_us = cal_get_time_us();

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
    delta_time = cal_get_time_us() - cal_tick_save.time_us;

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
        value = FCLK_DURATION_MS * 26000;
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
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
        param.duty_cycle1     = 0;
#else
        param.duty_cycle      = 0;
#endif
        param.end_value       = fclk_cal_endvalue((UINT32)param.cfg.bits.clk);

        sddev_control(PWM_DEV_NAME, CMD_PWM_INIT_PARAM, &param);
        
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
        sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_ENABLE, &timer_channel);
        
    }
}

void fclk_init(void)
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

