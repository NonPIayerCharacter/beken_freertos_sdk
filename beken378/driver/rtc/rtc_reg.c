#include "include.h"
#include "arm_arch.h"
#include "drv_model_pub.h"
#include "rtc_reg_pub.h"
#include "rtc_reg.h"
#include "target_util_pub.h"

#if (CFG_SOC_NAME == SOC_BK7252N)

#define AON_RTC_OPS_SAFE_DELAY_US (10)

static bool rtc_is_start;
static SDD_OPERATIONS rtc_reg_op = {
    rtc_reg_ctrl
};
/*
 * RTC uses 32k clock,and 3 cycles later can be clock sync.
 * CPU clock is more faster then RTC, so software operates RTC register
 * will be effect after 125 us(RTC 3+1 clock cycles).
 * So if operate the same register in 125 us, the second operation will be failed.
 */
static void rtc_delay_to_guarantee_ops_safe(void)
{
    delay(AON_RTC_OPS_SAFE_DELAY_US);
}

void rtc_reg_cfg_init_value(uint32_t val)
{
    REG_WRITE(RTC_REG0X0, val);
}

static void rtc_enable(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_CLK_EN);
}

static void rtc_disable(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_CLK_DIS);
}

__maybe_unused static bool rtc_is_enable(void)
{
    return RTC_REG0X0_CLK_EN_GET;
}

__maybe_unused static void rtc_stop_counter(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_CNT_STOP);
}

__maybe_unused static void rtc_start_counter(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_CNT_START);
}

__maybe_unused static bool rtc_is_counter_stop(void)
{
    return RTC_REG0X0_CNT_STOP_GET;
}

static void rtc_reset_counter(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_CNT_RST);
}

static void rtc_clear_reset_counter(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_CNT_RST_CLR);
}

__maybe_unused static bool rtc_is_counter_reset(void)
{
    return RTC_REG0X0_CNT_RST_GET;
}

static void rtc_clear_ctrl(void)
{
    REG_WRITE(RTC_REG0X0, 0);
}

static void rtc_set_tick_val_l(uint32_t tick_val_l)
{
    RTC_REG0X2_RTC_TICK_VAL_L_SET(tick_val_l);
}

static void rtc_set_tick_val_h(uint32_t tick_val_h)
{
    RTC_REG0X7_RTC_TICK_VAL_H_SET(tick_val_h);
}

void rtc_set_tick_val(uint64_t tick_val)
{
    rtc_set_tick_val_h(tick_val >> 32);
    rtc_set_tick_val_l((uint32_t)tick_val);
}

static uint32_t rtc_get_tick_val_l(void)
{
    return RTC_REG0X2_RTC_TICK_VAL_L_GET;
}

static uint32_t rtc_get_tick_val_h(void)
{
    return RTC_REG0X7_RTC_TICK_VAL_H_GET;
}

uint64_t rtc_get_tick_val(void)
{
    return (((uint64_t)rtc_get_tick_val_h() << 32) + rtc_get_tick_val_l());
}

__maybe_unused static void rtc_enable_tick_int(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_TICK_INT_EN);
}

__maybe_unused static void rtc_disable_tick_int(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_TICK_INT_DIS);
}

__maybe_unused static bool rtc_is_tick_int_enable(void)
{
    return RTC_REG0X0_TICK_INT_EN_GET;
}

__maybe_unused static bool rtc_get_tick_int_status(void)
{
    return RTC_REG0X0_TICK_INT_GET;
}

//write 1 to clear interrupt
static void rtc_clear_tick_int_status(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_TICK_INT_CLR);
}

static void rtc_set_upper_val_l(uint32_t up_val_l)
{
    RTC_REG0X1_RTC_UP_VAL_L_SET(up_val_l);
}

static void rtc_set_upper_val_h(uint32_t up_val_h)
{
    RTC_REG0X6_RTC_UP_VAL_H_SET(up_val_h);
}

static void rtc_set_upper_val(uint64_t up_val)
{
    rtc_set_upper_val_h(up_val >> 32);
    rtc_set_upper_val_l((uint32_t)up_val);
}

static void rtc_set_upper_val_max(void)
{
    rtc_set_upper_val_h(RTC_UPPER_VAL_MAX);
    rtc_set_upper_val_l(RTC_UPPER_VAL_MAX);
}

static uint32_t rtc_get_upper_val_l(void)
{
    return RTC_REG0X1_RTC_UP_VAL_L_GET;
}

static uint32_t rtc_get_upper_val_h(void)
{
    return RTC_REG0X6_RTC_UP_VAL_H_GET;
}

uint64_t rtc_get_upper_val(void)
{
    return (((uint64_t)rtc_get_upper_val_h() << 32) + rtc_get_upper_val_l());
}

__maybe_unused static void rtc_enable_upper_int(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_AON_INT_EN);
}

__maybe_unused static void rtc_disable_upper_int(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_AON_INT_DIS);
}

__maybe_unused static bool rtc_is_upper_int_enable(void)
{
    return RTC_REG0X0_AON_INT_EN_GET;
}

__maybe_unused static bool rtc_get_upper_int_status(void)
{
    return RTC_REG0X0_AON_INT_GET;
}

//write 1 to clear interrupt
static void rtc_clear_upper_int_status(void)
{
    REG_WRITE(RTC_REG0X0, RTC_REG0X0_AON_INT_CLR);
}

static uint32_t rtc_get_counter_val_l(void)
{
    return RTC_REG0X3_RTC_CNT_VAL_L_GET;
}

static uint32_t rtc_get_counter_val_h(void)
{
    return RTC_REG0XA_RTC_CNT_VAL_H_GET;
}

uint64_t rtc_get_counter_val(void)
{
    volatile uint32_t val_l = rtc_get_counter_val_l();
    volatile uint32_t val_h = rtc_get_counter_val_h();

    while (rtc_get_counter_val_l() != val_l || rtc_get_counter_val_h() != val_h)
    {
        val_l = rtc_get_counter_val_l();
        val_h = rtc_get_counter_val_h();
    }

    return (((uint64_t)(val_h) << 32) + val_l);
}

static uint32_t rtc_get_upper_val_lpo_l(void)
{
    return RTC_REG0X4_RTC_UP_VAL_LPO_L_GET;
}

static uint32_t rtc_get_upper_val_lpo_h(void)
{
    return RTC_REG0X8_RTC_UP_VAL_LPO_H_GET;
}

uint64_t rtc_get_upper_val_lpo(void)
{
    return (((uint64_t)rtc_get_upper_val_lpo_h() << 32) + rtc_get_upper_val_lpo_l());
}

static uint32_t rtc_get_tick_val_lpo_l(void)
{
    return RTC_REG0X5_RTC_TICK_VAL_LPO_L_GET;
}

static uint32_t rtc_get_tick_val_lpo_h(void)
{
    return RTC_REG0X9_RTC_TICK_VAL_LPO_H_GET;
}

uint64_t rtc_get_tick_val_lpo(void)
{
    return (((uint64_t)rtc_get_tick_val_lpo_h() << 32) + rtc_get_tick_val_lpo_l());
}

void rtc_reg_init(void)
{
    //enter deepsleep/reboot,before init,the tick is still on-going
    rtc_reset_counter();
    rtc_delay_to_guarantee_ops_safe();
    rtc_clear_reset_counter();

    rtc_clear_ctrl();
    rtc_set_tick_val(0);
    rtc_set_upper_val(0);

    // rtc_is_start = 0;

    rtc_reg_start();

    sddev_register_dev(RTC_REG_DEV_NAME, &rtc_reg_op);
}

void rtc_reg_start(void)
{
    rtc_set_upper_val_max();
    rtc_enable();

    rtc_is_start = 1;
}


uint64_t rtc_reg_get_time_us(void)
{
    uint64_t time_us;
    uint64_t cnt;

    cnt = rtc_get_counter_val();
    time_us = cnt * RTC_CLK_PER_CNT_EQL_31_25_US;

    return time_us;
}

void rtc_reg_tick_prog(UINT32 period_tick)
{
    UINT64 value = rtc_get_counter_val();
    UINT32 incr = period_tick;

    value += incr;

    rtc_set_tick_val(value);
    rtc_enable_tick_int();
}

void rtc_reg_tick_clear()
{
    rtc_disable_tick_int();
    rtc_clear_tick_int_status();
    rtc_set_tick_val(0);
}

void rtc_reg_exit(void)
{
    rtc_reset_counter();
    rtc_clear_tick_int_status();
    rtc_clear_upper_int_status();
    rtc_set_tick_val(0);
    rtc_set_upper_val(0);
    rtc_clear_ctrl();
    rtc_disable();
    rtc_is_start = 0;
}

UINT32 rtc_reg_ctrl(UINT32 cmd, void *param)
{
    UINT32 ret = 0;

    switch (cmd) {
    case CMD_RTC_TMR_PROG:
        if (!rtc_is_start)
        {
            rtc_reg_start();
        }
        rtc_reg_tick_prog(*(UINT32 *)param);
        break;

    case CMD_RTC_TMR_CLEAR:
        if (!rtc_is_start)
        {
            ret = -1;
        }
        rtc_reg_tick_clear();
    default:
        break;
    }

    return ret;
}

#endif

// eof
