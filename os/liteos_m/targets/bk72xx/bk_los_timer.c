#include "include.h"
#include "bk_los_timer.h"
#include "bk_timer.h"
#include "los_context.h"
#include "rtos_pub.h"
#include "arm_arch.h"
#include "los_interrupt.h"

static OS_TICK_HANDLER g_tickHandler = NULL;

void HalTickHandler(u8 param)
{
    if (g_tickHandler != NULL) {
        g_tickHandler();
    }
}

unsigned int HalTickStart(OS_TICK_HANDLER handler)
{
	unsigned int intSave;
	UINT32 timer_channel;

	intSave = LOS_IntLock();
	timer_param_t param;
	param.channel = LOS_TICK_TIMER_ID;
	param.div = 1;
	param.period = LOS_TICK_MS;
	param.t_Int_Handler= HalTickHandler;
	g_tickHandler = handler;

	bk_timer_ctrl( CMD_TIMER_INIT_PARAM, &param);
	timer_channel = param.channel;
	bk_timer_ctrl( CMD_TIMER_UNIT_ENABLE, &timer_channel);
	LOS_IntRestore(intSave);

    return 0;
}

void HalTickLock(void)
{
    unsigned int timer_channel = LOS_TICK_TIMER_ID;
    (void)bk_timer_ctrl(CMD_TIMER_UNIT_DISABLE, &timer_channel);
}

void HalTickUnlock(void)
{
    unsigned int timer_channel = LOS_TICK_TIMER_ID;
    (void)bk_timer_ctrl(CMD_TIMER_UNIT_ENABLE, &timer_channel);
}

static void HalClockIrqClear(void)
{
    unsigned int status1 = 0x7 << TIMERCTLB_INT_POSI;

    do {
        REG_WRITE(LOS_TICK_CTRL_REG, (REG_READ(LOS_TICK_CTRL_REG) & (~(0x7 << TIMERCTLB_INT_POSI))) | status1);
    } while (REG_READ(LOS_TICK_CTRL_REG) & status1 & (0x7 << TIMERCTLB_INT_POSI));
}

void HalSysTickReload(unsigned long long nextResponseTime)
{
	UINT32 inSave;

	inSave = HalIntLock();
    HalTickLock();
	
	HalClockIrqClear();
    REG_WRITE(REG_TIMERCTL_PERIOD_ADDR(LOS_TICK_TIMER_GROUP, LOS_TICK_TIMER_ID), nextResponseTime);
    HalTickUnlock();
	HalIntRestore(inSave);
}

unsigned long long HalGetTickCycle(unsigned int *period)
{
    unsigned int val = 0;
    unsigned int i_time_out = 0;

    *period = REG_READ(REG_TIMERCTL_PERIOD_ADDR(LOS_TICK_TIMER_GROUP, LOS_TICK_TIMER_ID));

    //get val
    REG_WRITE(LOS_TICK_RD_CTRL_REG, (LOS_TICK_TIMER_RD_ID << 2) | 1);
    while (REG_READ(LOS_TICK_RD_CTRL_REG) & 1) {
         i_time_out++;
         if (i_time_out > (120 * 1000)) {
             break;
         }
    }

    if (i_time_out <= (120 * 1000)) {
        val = REG_READ(LOS_TICK_RD_VAL_REG);
    }

    return (unsigned long long)val;
}

UINT64 OsGetCurrSecond(VOID)
{
	return rtos_get_tick_count() * LOS_TICK_MS / 1000;
}
// eof

