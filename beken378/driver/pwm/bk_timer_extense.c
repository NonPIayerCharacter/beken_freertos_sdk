#include "sys_config.h"

#if (1 == CFG_USE_FORCE_LOWVOL_PS)
#include "include.h"
#include "arm_arch.h"
#include "bk_timer_pub.h"
#include "bk_timer.h"
#include "drv_model_pub.h"
#include "intc_pub.h"
#include "icu_pub.h"
#include "uart_pub.h"

#if (CFG_SOC_NAME != SOC_BK7231)
UINT32 bk_timer_disable(UINT32 channel)
{
	GLOBAL_INT_DECLARATION();

	UINT32 ucChannel = channel, value;
	if (ucChannel > BKTIMER5)
		return BK_TIMER_FAILURE;

	GLOBAL_INT_DISABLE();
	if (ucChannel < BKTIMER3) {
		value = REG_READ(TIMER0_2_CTL);
		value &= ~(0x7 << TIMERCTLA_INT_POSI);
		value &= ~(1 << ucChannel);
		value |= (0x1 << (TIMERCTLA_INT_POSI + ucChannel));
		REG_WRITE(TIMER0_2_CTL, value);
	} else {
		value = REG_READ(TIMER3_5_CTL);
		value &= ~(0x7 << TIMERCTLB_INT_POSI);
		value &= ~(1 << (ucChannel - 3));
		value |= (0x1 << (TIMERCTLB_INT_POSI + (ucChannel - 3)));
		REG_WRITE(TIMER3_5_CTL, value);
	}
	GLOBAL_INT_RESTORE();

	return BK_TIMER_SUCCESS;
}

UINT32 bk_timer_enable(UINT32 channel, UINT32 delay_ms)
{
	GLOBAL_INT_DECLARATION();

	UINT32 ucChannel = channel, value;
	if (ucChannel > BKTIMER5)
		return BK_TIMER_FAILURE;

	GLOBAL_INT_DISABLE();
	if (ucChannel < BKTIMER3) {
		value = delay_ms * 26000;
		REG_WRITE(REG_TIMERCTLA_PERIOD_ADDR(channel), value);

		value = REG_READ(TIMER0_2_CTL);
		value &= ~(0x7 << TIMERCTLA_INT_POSI);
		value |= (1 << ucChannel);
		value |= (0x1 << (TIMERCTLA_INT_POSI + ucChannel));
		REG_WRITE(TIMER0_2_CTL, value);
	} else {
		value = delay_ms * 32;
		REG_WRITE(REG_TIMERCTLB_PERIOD_ADDR(channel), value);

		value = REG_READ(TIMER3_5_CTL);
		value &= ~(0x7 << TIMERCTLB_INT_POSI);
		value |= (1 << (ucChannel - 3));
		value |= (0x1 << (TIMERCTLB_INT_POSI + (ucChannel - 3)));
		REG_WRITE(TIMER3_5_CTL, value);
	}
	GLOBAL_INT_RESTORE();

	return BK_TIMER_SUCCESS;
}

UINT32 bk_timer_get_end_time(UINT32 channel, UINT32 *count_ms)
{
	GLOBAL_INT_DECLARATION();

	UINT32 ucChannel = channel, value;
	if (ucChannel > BKTIMER5)
		return BK_TIMER_FAILURE;

	GLOBAL_INT_DISABLE();
	if (ucChannel < BKTIMER3) {
		value = REG_READ(REG_TIMERCTLA_PERIOD_ADDR(channel));
		value /= 26000;
	} else {
		value = REG_READ(REG_TIMERCTLB_PERIOD_ADDR(channel));
		value /= 32;
	}

	if (count_ms)
		*count_ms = value;

	GLOBAL_INT_RESTORE();

	return BK_TIMER_SUCCESS;
}

UINT32 bk_timer_get_int_status(UINT32 channel, UINT32 *status)
{
	GLOBAL_INT_DECLARATION();

	UINT32 ucChannel = channel, value;
	if (ucChannel > BKTIMER5)
		return BK_TIMER_FAILURE;

	GLOBAL_INT_DISABLE();
	if (ucChannel < BKTIMER3) {
		value = REG_READ(TIMER0_2_CTL);
		if (value & (0x1 << (TIMERCTLA_INT_POSI + ucChannel))) {
			if (status)
				*status = 1;
		}
	} else {
		value = REG_READ(TIMER3_5_CTL);
		if (value & (0x1 << (TIMERCTLB_INT_POSI + (ucChannel - 3)))) {
			if (status)
				*status = 1;
		}
	}
	GLOBAL_INT_RESTORE();

	return BK_TIMER_SUCCESS;
}

UINT32 bk_timer_pre_measure(UINT32 channel)
{
	GLOBAL_INT_DECLARATION();

	UINT32 ucChannel = channel, value;
	if (ucChannel > BKTIMER5)
		return BK_TIMER_FAILURE;

	GLOBAL_INT_DISABLE();
	if (ucChannel < BKTIMER3) {
		value = ((channel << TIMER0_2_READ_INDEX_POSI) | TIMER0_2_READ_OP_BIT);
		REG_WRITE(TIMER0_2_READ_CTL, value);
	} else {
		value = (((channel - BKTIMER3) << TIMER3_5_READ_INDEX_POSI) | TIMER3_5_READ_OP_BIT);
		REG_WRITE(TIMER3_5_READ_CTL, value);
	}
	GLOBAL_INT_RESTORE();

	return BK_TIMER_SUCCESS;
}

UINT32 bk_timer_measure(UINT32 channel, UINT32 *count_ms)
{
	GLOBAL_INT_DECLARATION();

	UINT32 ucChannel = channel, value, ret = BK_TIMER_SUCCESS;
	if (ucChannel > BKTIMER5)
		return BK_TIMER_FAILURE;

	GLOBAL_INT_DISABLE();
	value = 0;
	if (ucChannel < BKTIMER3) {
		while (REG_READ(TIMER0_2_READ_CTL) & TIMER0_2_READ_OP_BIT) {
			value++;
			if (value > (120 * 1000)) {
				ret = BK_TIMER_FAILURE;
				break;
			}
		}

		if (value <= (120 * 1000)) {
			value = REG_READ(TIMER0_2_READ_VALUE);
			value /= 26000;
		}
	} else {
		while (REG_READ(TIMER3_5_READ_CTL) & TIMER3_5_READ_OP_BIT) {
			value++;
			if (value > (120 * 1000)) {
				ret = BK_TIMER_FAILURE;
				break;
			}
		}

		if (value <= (120 * 1000)) {
			value = REG_READ(TIMER3_5_READ_VALUE);
			value /= 32;
		}
	}

	if ((count_ms) && (ret == BK_TIMER_SUCCESS))
		*count_ms = value;

	GLOBAL_INT_RESTORE();

	return ret;
}

#endif
#endif

