#include "include.h"
#include "arm_arch.h"
#include <stdlib.h>

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7236) || (CFG_SOC_NAME == SOC_BK7238)
#include "pwm_pub.h"
#include "drv_model_pub.h"
#include "intc_pub.h"
#include "icu_pub.h"
#include "gpio_pub.h"
#include "uart_pub.h"
#include "pwm_bk7231n.h"
#include "mem_pub.h"

#define DRV_USED_PWM_CW_GROUP           1
#define DRV_USED_PWM_CW                 1
#define DRV_USED_PWM                    1
#define DRV_USED_PWM_CAP                1

typedef struct
{
	UINT32 is_active;
	UINT8 init_level;
	UINT8 another_pwm_idx;   //
	UINT8 status;
	UINT8 is_p0;   // p0 is the pwm chan that high first
	UINT32 t1;
	UINT32 t2;
	UINT32 t3;
	UINT32 t4;
} drv_pwm_cw_param_t;

static SDD_OPERATIONS pwm_op = { pwm_ctrl };
const UINT8 pwm_ch_to_group[PWM_GROUP_NUM * PWM_CHAN_IN_GROUP] = {0, 0, 1, 1, 2, 2};

// 0-->1-->2<-->3
#define PWM_STATUS_UNINIT               0
#define PWM_STATUS_INITED               1
#define PWM_STATUS_RUNNING              2
#define PWM_STATUS_LOADING              3

#define PWM_CHAN_TO_GROUP_NUM(ch)       (pwm_ch_to_group[(ch)])
#define PWM_CHAN_TO_CHAN_IN_GROUP(ch)   ((ch) % 2)
#define GET_P0_IDX_FROM_IDXS(idxs)      ((idxs) & 0xf)
#define GET_P1_IDX_FROM_IDXS(idxs)      (((idxs) >> 4) & 0xf)

// PWM MODE
#define PWM_NOT_USED                    0
#define PWM_USED_CW_GROUP               1
#define PWM_USED_CW                     2
#define PWM_USED_PWM                    3
#define PWM_USED_PWM_CAP                4

#if DRV_USED_PWM_CW_GROUP
pwm_cw_group_param_t g_cw_group_pwm_param[PWM_GROUP_NUM];
#endif
#if DRV_USED_PWM_CW
drv_pwm_cw_param_t g_cw_pwm_param[PWM_GROUP_NUM * PWM_CHAN_IN_GROUP];
#endif
#if DRV_USED_PWM
pwm_param_st g_pwm_param[PWM_GROUP_NUM * PWM_CHAN_IN_GROUP];
#endif
#if DRV_USED_PWM_CAP
pwm_cap_param_st g_pwm_cap_param[PWM_GROUP_NUM * PWM_CHAN_IN_GROUP];
#endif

void pwm_gpio_configuration(UINT8 chan, UINT8 enable)
{
	UINT32 ret;
	UINT32 param;

	switch (chan)
	{
	case PWM0:
		param = GFUNC_MODE_PWM0;
		break;

	case PWM1:
		param = GFUNC_MODE_PWM1;
		break;

	case PWM2:
		param = GFUNC_MODE_PWM2;
		break;

	case PWM3:
		param = GFUNC_MODE_PWM3;
		break;

	case PWM4:
		param = GFUNC_MODE_PWM4;
		break;

	case PWM5:
		param = GFUNC_MODE_PWM5;
		break;

	default:
		break;
	}

	if (enable)
	{
		ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &param);
	}
	else
	{
		param = GPIO_CFG_PARAM(param, GMODE_INPUT);
		ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
	}
	ASSERT(GPIO_SUCCESS == ret);
}

void pwm_icu_configuration(UINT8 channel, UINT8 clk_mode, UINT8 enable, UINT8 isr_enable)
{
	UINT32 ret;
	UINT32 prm;

	/* set clock power down of icu module*/
	switch (channel)
	{
	case PWM0:
		prm = PWD_PWM0_CLK_BIT;
		break;

	case PWM1:
		prm = PWD_PWM0_CLK_BIT;
		break;

	case PWM2:
		prm = PWD_PWM2_CLK_BIT;
		break;

	case PWM3:
		prm = PWD_PWM2_CLK_BIT;
		break;

	case PWM4:
		prm = PWD_PWM4_CLK_BIT;
		break;

	case PWM5:
		prm = PWD_PWM4_CLK_BIT;
		break;

	default:
		PWM_WARN("pwm_iconfig_fail\r\n");
		goto exit_icu;
	}

	if (enable)
	{
		ret = sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, (void *)&prm);
		ASSERT(ICU_SUCCESS == ret);

		if (PWM_CLK_32K == clk_mode)
		{
			prm = channel;
			ret = sddev_control(ICU_DEV_NAME, CMD_CONF_PWM_LPOCLK, (void *)&prm);
		}
		else if (PWM_CLK_26M == clk_mode)
		{
			prm = PCLK_POSI_PWMS;
			ret = sddev_control(ICU_DEV_NAME, CMD_CONF_PCLK_26M, (void *)&prm);

			prm = channel;
			ret = sddev_control(ICU_DEV_NAME, CMD_CONF_PWM_PCLK, (void *)&prm);
		}
		else
		{
			prm = PCLK_POSI_PWMS;
			ret = sddev_control(ICU_DEV_NAME, CMD_CONF_PCLK_DCO, (void *)&prm);

			prm = channel;
			ret = sddev_control(ICU_DEV_NAME, CMD_CONF_PWM_PCLK, (void *)&prm);
		}
		ASSERT(ICU_SUCCESS == ret);
	}
	else
	{
		ret = sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_DOWN, (void *)&prm);
		ASSERT(ICU_SUCCESS == ret);
	}

	if (PWM_INT_EN == isr_enable)
	{
		prm = IRQ_PWM_BIT;
		ret = sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, (void *)&prm);
	}

exit_icu:

	return;
}

void pwm_init(void)
{
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(0), 0x0);
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(1), 0x0);
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(2), 0x0);

#if DRV_USED_PWM_CW_GROUP
	os_memset(g_cw_group_pwm_param, 0, PWM_GROUP_NUM * sizeof(pwm_cw_group_param_t));
#endif
#if DRV_USED_PWM_CW
	os_memset(g_cw_pwm_param, 0, PWM_GROUP_NUM * PWM_CHAN_IN_GROUP * sizeof(drv_pwm_cw_param_t));
#endif
#if DRV_USED_PWM
	os_memset(g_pwm_param, 0, PWM_GROUP_NUM * PWM_CHAN_IN_GROUP * sizeof(pwm_param_st));
#endif
#if DRV_USED_PWM_CAP
	os_memset(g_pwm_cap_param, 0, PWM_GROUP_NUM * PWM_CHAN_IN_GROUP * sizeof(pwm_cap_param_st));
#endif

	intc_service_register(IRQ_PWM, PRI_IRQ_PWM, pwm_isr);

	sddev_register_dev(PWM_DEV_NAME, &pwm_op);
}

void pwm_exit(void)
{
	sddev_unregister_dev(PWM_DEV_NAME);
}

UINT32 pwm_ctrl(UINT32 cmd, void *param)
{
	UINT32 ret = PWM_SUCCESS;

	switch (cmd)
	{
	default:
		ret = PWM_FAILURE;
		break;
	}

	return ret;
}

UINT8 pwm_cw_group_check(UINT8 pwm1, UINT8 pwm2)
{
	UINT8 pwm1_group = PWM_CHAN_TO_GROUP_NUM(pwm1);
	UINT8 pwm2_group = PWM_CHAN_TO_GROUP_NUM(pwm2);

	if (pwm1_group == pwm2_group)
	{
		return pwm1_group;
	}
	else
	{
		return PWM_GROUP_NUM;
	}
}

// return   0: pwm not used.
// return > 0: pwm is used

//            2: used for cw group mode
//            3: used for cw mode
//            4: used for pwm mode
UINT8 pwm_check_is_used(UINT8 chan)
{
	UINT8 group = PWM_CHAN_TO_GROUP_NUM(chan);

	#if DRV_USED_PWM_CW_GROUP
	pwm_cw_group_param_t *group_param_ptr = &g_cw_group_pwm_param[group];
	#endif
	#if DRV_USED_PWM_CW
	drv_pwm_cw_param_t *cw_param_ptr = &g_cw_pwm_param[chan];
	#endif
	#if DRV_USED_PWM
	pwm_param_st *pwm_param_ptr = &g_pwm_param[chan];
	#endif
	#if DRV_USED_PWM_CAP
	pwm_cap_param_st *pwm_cap_param_ptr = &g_pwm_cap_param[chan];
	#endif

	#if DRV_USED_PWM_CW_GROUP
	if(group_param_ptr->is_active)
	{
		return PWM_USED_CW_GROUP;
	} else
	#endif // DRV_USED_PWM_CW_GROUP

	#if DRV_USED_PWM_CW
	if(cw_param_ptr->is_active)
	{
		return PWM_USED_CW;
	} else
	#endif // DRV_USED_PWM_CW

	#if DRV_USED_PWM
	if(pwm_param_ptr->is_active)
	{
		return PWM_USED_PWM;
	} else
	#endif // DRV_USED_PWM

	#if DRV_USED_PWM_CAP
	if(pwm_cap_param_ptr->is_active)
	{
		return PWM_USED_PWM_CAP;
	}
	#endif // DRV_USED_PWM_CAP

	group = group; // fix warning

	return PWM_NOT_USED;
}

#if DRV_USED_PWM_CW_GROUP
static UINT32 pwm_cw_group_updata_regs(pwm_cw_group_param_t *pwm_param, UINT32 pwm_cfg_reg)
{
	UINT8 group;

	group = pwm_param->group;

	REG_WRITE(REG_GROUP_PWM0_T1_ADDR(group), pwm_param->p0_t1);
	REG_WRITE(REG_GROUP_PWM0_T2_ADDR(group), 0);
	REG_WRITE(REG_GROUP_PWM0_T3_ADDR(group), 0);
	REG_WRITE(REG_GROUP_PWM0_T4_ADDR(group), pwm_param->p_t4);

	REG_WRITE(REG_GROUP_PWM1_T1_ADDR(group), pwm_param->p1_t1);
	REG_WRITE(REG_GROUP_PWM1_T2_ADDR(group), pwm_param->p1_t2);
	REG_WRITE(REG_GROUP_PWM1_T3_ADDR(group), 0);
	REG_WRITE(REG_GROUP_PWM1_T4_ADDR(group), pwm_param->p_t4);

	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(1) | PWM_GROUP_PWM_INT_LEVL_MASK(0));
	pwm_cfg_reg |= PWM_GROUP_PWM_CFG_UPDATA_MASK(1) | PWM_GROUP_PWM_CFG_UPDATA_MASK(0);
	pwm_cfg_reg |= (pwm_param->p1_init_level << PWM_GROUP_PWM_INT_LEVL_BIT(1)) | (pwm_param->p0_init_level << PWM_GROUP_PWM_INT_LEVL_BIT(0));

	return pwm_cfg_reg;
}

UINT8 pwm_cw_group_init_param(pwm_cw_group_param_t *pwm_param)
{
	UINT32 pwm_cfg_reg;
	pwm_cw_group_param_t *local_param_ptr;
	UINT8 p0_chan, p1_chan, group;
	GLOBAL_INT_DECLARATION();

	if ((pwm_param == NULL)
		|| (pwm_param->group >= PWM_GROUP_NUM))
	{
		return 1;
	}

	group = pwm_param->group;
	p0_chan = group * PWM_CHAN_IN_GROUP;
	p1_chan = group * PWM_CHAN_IN_GROUP + 1;
	if((pwm_check_is_used(p0_chan) != PWM_NOT_USED)
		|| (pwm_check_is_used(p1_chan) != PWM_NOT_USED))
	{
		return 3;
	}
	local_param_ptr = &g_cw_group_pwm_param[pwm_param->group];
	os_memset(local_param_ptr, 0, sizeof(pwm_cw_group_param_t));

	GLOBAL_INT_DISABLE();
	os_memcpy(local_param_ptr, pwm_param, sizeof(pwm_cw_group_param_t));
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(1) | PWM_GROUP_PWM_INT_LEVL_MASK(0));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(1) | PWM_GROUP_PWM_CFG_UPDATA_MASK(0));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(1) | PWM_GROUP_PWM_STOP_MASK(0));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(1) | PWM_GROUP_PWM_INT_ENABLE_MASK(0));
	pwm_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(1) | PWM_GROUP_MODE_SET_MASK(0));

	// setting reg
	pwm_cfg_reg |= (PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(1)) | (PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(0));
	// pwm_cfg_reg |= PWM_GROUP_PWM_INT_ENABLE_MASK(0); // only enable p0 interrupt
	pwm_cfg_reg = pwm_cw_group_updata_regs(local_param_ptr, pwm_cfg_reg);
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	pwm_gpio_configuration(p0_chan, 1);
	pwm_gpio_configuration(p1_chan, 1);
	pwm_icu_configuration(p0_chan, PWM_CLK_26M, 1, 1);
	pwm_icu_configuration(p1_chan, PWM_CLK_26M, 1, 0);

	// clear int status
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_cfg_reg |= (PWM_GROUP_PWM_INT_STAT_CLEAR(1) | PWM_GROUP_PWM_INT_STAT_CLEAR(0));
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	local_param_ptr->status = PWM_STATUS_INITED;
	local_param_ptr->is_active = 1;

	PWM_PRT("pwm cw group inited:0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_cw_group_start(UINT8 group)
{
	pwm_cw_group_param_t *local_param_ptr = NULL;
	UINT32 pwm_cfg_reg;
	UINT8 p0_chan, p1_chan, ret;
	GLOBAL_INT_DECLARATION();

	if (group >= PWM_GROUP_NUM)
	{
		return 1;
	}

	p0_chan = group * PWM_CHAN_IN_GROUP;
	p1_chan = group * PWM_CHAN_IN_GROUP + 1;
	ret = pwm_check_is_used(p0_chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_CW_GROUP))
	{
		return 3;
	}
	ret = pwm_check_is_used(p1_chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_CW_GROUP))
	{
		return 3;
	}

	// enable
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	if ((pwm_cfg_reg & (PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK | PWM_GROUP_PWM_ENABLE_MASK(1) | PWM_GROUP_PWM_ENABLE_MASK(0)))
		== (PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK | PWM_GROUP_PWM_ENABLE_MASK(1) | PWM_GROUP_PWM_ENABLE_MASK(0)))
	{
		return 0;
	}

	GLOBAL_INT_DISABLE();

	local_param_ptr = &g_cw_group_pwm_param[group];
	//local_param_ptr->status = PWM_STATUS_RUNNING; // not set beacause it may called after update_param

	// init
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	// setting reg
	pwm_cfg_reg |= (PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(1)) | (PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(0));
	// pwm_cfg_reg |= PWM_GROUP_PWM_INT_ENABLE_MASK(0); // only enable p0 interrupt
	pwm_cfg_reg = pwm_cw_group_updata_regs(local_param_ptr, pwm_cfg_reg);
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	// start
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0)); // not clear int status
	pwm_cfg_reg |= (PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
	pwm_cfg_reg |= (PWM_GROUP_PWM_ENABLE_MASK(1) | PWM_GROUP_PWM_ENABLE_MASK(0));
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	local_param_ptr->is_active = 1;

	PWM_PRT("pwm cw group started:0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_cw_group_update_param(pwm_cw_group_param_t *pwm_param)
{
	pwm_cw_group_param_t *local_param_ptr = NULL;
	UINT32 pwm_cfg_reg;
	UINT8 group;
	UINT8 p0_chan, p1_chan, ret;
	GLOBAL_INT_DECLARATION();

	if ((pwm_param == NULL)
		|| (pwm_param->group >= PWM_GROUP_NUM))
	{
		return 1;
	}

	group = pwm_param->group;
	p0_chan = group * PWM_CHAN_IN_GROUP;
	p1_chan = group * PWM_CHAN_IN_GROUP + 1;
	ret = pwm_check_is_used(p0_chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_CW_GROUP))
	{
		return 3;
	}
	ret = pwm_check_is_used(p1_chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_CW_GROUP))
	{
		return 3;
	}

	local_param_ptr = &g_cw_group_pwm_param[group];
	// don't care about status an active
	// if(local_param_ptr->status == PWM_STATUS_LOADING)
	{
		// return 2;
	}
	GLOBAL_INT_DISABLE();

	// only copy them, and enable p0 isr, updata to registers in isr
	os_memcpy(local_param_ptr, pwm_param, sizeof(pwm_cw_group_param_t));
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));

	if ((pwm_cfg_reg & PWM_GROUP_PWM_INT_ENABLE_MASK(0)) == 0)
	{
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0)); // not clear int status
		pwm_cfg_reg |= PWM_GROUP_PWM_INT_ENABLE_MASK(0);								   // only enable p0 interrupt
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);
	}

	local_param_ptr->status = PWM_STATUS_LOADING;
	local_param_ptr->is_active = 1;

	PWM_PRT("pwm cw group started:0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));

	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_cw_group_stop(UINT8 group)
{
	pwm_cw_group_param_t *local_param_ptr = NULL;
	UINT32 pwm_cfg_reg;
	UINT8 p0_chan, p1_chan, ret;
	GLOBAL_INT_DECLARATION();

	if (group >= PWM_GROUP_NUM)
	{
		return 1;
	}

	p0_chan = group * PWM_CHAN_IN_GROUP;
	p1_chan = group * PWM_CHAN_IN_GROUP + 1;
	ret = pwm_check_is_used(p0_chan);
	if(ret != PWM_USED_CW_GROUP)
	{
		return 3;
	}
	ret = pwm_check_is_used(p1_chan);
	if(ret != PWM_USED_CW_GROUP)
	{
		return 3;
	}

	// enable
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	if ((pwm_cfg_reg & (PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK | PWM_GROUP_PWM_ENABLE_MASK(1) | PWM_GROUP_PWM_ENABLE_MASK(0)))
		== (PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK | PWM_GROUP_PWM_ENABLE_MASK(1) | PWM_GROUP_PWM_ENABLE_MASK(0)))
	{
		GLOBAL_INT_DISABLE();
		pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK | PWM_GROUP_PWM_ENABLE_MASK(1) | PWM_GROUP_PWM_ENABLE_MASK(0)); // not clear int status
		pwm_cfg_reg |= (PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0));																	  // clear int status
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

		// close ?
		pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(1) | PWM_GROUP_PWM_INT_LEVL_MASK(0));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(1) | PWM_GROUP_PWM_CFG_UPDATA_MASK(0));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(1) | PWM_GROUP_PWM_STOP_MASK(0));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(1) | PWM_GROUP_PWM_INT_ENABLE_MASK(0));
		pwm_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(1) | PWM_GROUP_MODE_SET_MASK(0));
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

		local_param_ptr = &g_cw_group_pwm_param[group];
		local_param_ptr->status = PWM_STATUS_INITED;
		local_param_ptr->is_active = 0;

		PWM_PRT("pwm cw group stop:0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
		GLOBAL_INT_RESTORE();
	}

	return 0;
}
#else
UINT8 pwm_cw_group_init_param(pwm_cw_group_param_t *pwm_param)
{
	return 1;
}

UINT8 pwm_cw_group_update_param(pwm_cw_group_param_t *pwm_param)
{
	return 1;
}

UINT8 pwm_cw_group_start(UINT8 group)
{
	return 1;
}

UINT8 pwm_cw_group_stop(UINT8 group)
{
	return 1;
}
#endif  // DRV_USED_PWM_CW_GROUP

#if DRV_USED_PWM_CW
static inline void pwm_cw_updata_regs(drv_pwm_cw_param_t *p1_param_ptr, drv_pwm_cw_param_t *p0_param_ptr, UINT32 *p1_cfg_reg_ptr, UINT32 *p0_cfg_reg_ptr)
{
	UINT32 p0_cfg_reg, p1_cfg_reg;
	UINT8 p0_chan, p1_chan;
	UINT8 p0_group, p1_group;
	UINT8 p0_post, p1_post;   // 0: in low bits, 1: in high bits

	p1_chan = p0_param_ptr->another_pwm_idx;
	p0_chan = p1_param_ptr->another_pwm_idx;
	p0_cfg_reg = *p0_cfg_reg_ptr;
	p1_cfg_reg = *p1_cfg_reg_ptr;
	p0_group = PWM_CHAN_TO_GROUP_NUM(p0_chan);
	p1_group = PWM_CHAN_TO_GROUP_NUM(p1_chan);
	p0_post = PWM_CHAN_TO_CHAN_IN_GROUP(p0_chan);
	p1_post = PWM_CHAN_TO_CHAN_IN_GROUP(p1_chan);

	// setting reg
	if(p0_post == 0)
	{
		REG_WRITE(REG_GROUP_PWM0_T1_ADDR(p0_group), p0_param_ptr->t1);
		REG_WRITE(REG_GROUP_PWM0_T2_ADDR(p0_group), p0_param_ptr->t2);
		REG_WRITE(REG_GROUP_PWM0_T3_ADDR(p0_group), p0_param_ptr->t3);
		REG_WRITE(REG_GROUP_PWM0_T4_ADDR(p0_group), p0_param_ptr->t4);
	}
	else
	{
		REG_WRITE(REG_GROUP_PWM1_T1_ADDR(p0_group), p0_param_ptr->t1);
		REG_WRITE(REG_GROUP_PWM1_T2_ADDR(p0_group), p0_param_ptr->t2);
		REG_WRITE(REG_GROUP_PWM1_T3_ADDR(p0_group), p0_param_ptr->t3);
		REG_WRITE(REG_GROUP_PWM1_T4_ADDR(p0_group), p0_param_ptr->t4);
	}
	p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(p0_post));
	p0_cfg_reg |= PWM_GROUP_PWM_CFG_UPDATA_MASK(p0_post);
	p0_cfg_reg |= (p0_param_ptr->init_level << PWM_GROUP_PWM_INT_LEVL_BIT(p0_post));
	p0_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(p0_post);  // clear init status

	// p1
	if(p1_post == 0)
	{
		REG_WRITE(REG_GROUP_PWM0_T1_ADDR(p1_group), p1_param_ptr->t1);
		REG_WRITE(REG_GROUP_PWM0_T2_ADDR(p1_group), p1_param_ptr->t2);
		REG_WRITE(REG_GROUP_PWM0_T3_ADDR(p1_group), p1_param_ptr->t3);
		REG_WRITE(REG_GROUP_PWM0_T4_ADDR(p1_group), p1_param_ptr->t4);
	}
	else
	{
		REG_WRITE(REG_GROUP_PWM1_T1_ADDR(p1_group), p1_param_ptr->t1);
		REG_WRITE(REG_GROUP_PWM1_T2_ADDR(p1_group), p1_param_ptr->t2);
		REG_WRITE(REG_GROUP_PWM1_T3_ADDR(p1_group), p1_param_ptr->t3);
		REG_WRITE(REG_GROUP_PWM1_T4_ADDR(p1_group), p1_param_ptr->t4);
	}
	p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(p1_post));
	p1_cfg_reg |= PWM_GROUP_PWM_CFG_UPDATA_MASK(p1_post);
	p1_cfg_reg |= (p1_param_ptr->init_level << PWM_GROUP_PWM_INT_LEVL_BIT(p1_post));
	p1_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(p1_post);  // clear init status

	*p0_cfg_reg_ptr = p0_cfg_reg;
	*p1_cfg_reg_ptr = p1_cfg_reg;
}

UINT8 pwm_cw_init_param(pwm_cw_group_param_t *pwm_param)
{
	UINT32 p0_cfg_reg, p1_cfg_reg;
	drv_pwm_cw_param_t *p0_param_ptr, *p1_param_ptr;
	UINT8 p0_chan, p1_chan;
	UINT8 p0_group, p1_group;
	UINT8 p0_post, p1_post;   // 0: in low bits, 1: in high bits
	GLOBAL_INT_DECLARATION();

	if (pwm_param == NULL)
	{
		return 1;
	}

	p0_chan = GET_P0_IDX_FROM_IDXS(pwm_param->group);
	p1_chan = GET_P1_IDX_FROM_IDXS(pwm_param->group);
	if((p0_chan >= PWM_COUNT)
		|| (p1_chan >= PWM_COUNT))
	{
		return 1;
	}

	p0_group = PWM_CHAN_TO_GROUP_NUM(p0_chan);
	p1_group = PWM_CHAN_TO_GROUP_NUM(p1_chan);
	p0_post = PWM_CHAN_TO_CHAN_IN_GROUP(p0_chan);
	p1_post = PWM_CHAN_TO_CHAN_IN_GROUP(p1_chan);

	if(p0_group == p1_group)
	{
		return 1;
	}

	// check p0 p1 used,  in cw-group-pwm and only-pwm
	p0_param_ptr = &g_cw_pwm_param[p0_chan];
	p1_param_ptr = &g_cw_pwm_param[p1_chan];
	if ((p0_param_ptr->is_active != 0)
		||(p1_param_ptr->is_active != 0))
	{
		return 2;
	}

	if((pwm_check_is_used(p0_chan) != PWM_NOT_USED)
		|| (pwm_check_is_used(p1_chan) != PWM_NOT_USED))
	{
		return 3;
	}

	GLOBAL_INT_DISABLE();
	os_memset(p0_param_ptr, 0, sizeof(drv_pwm_cw_param_t));
	p0_param_ptr->init_level = pwm_param->p0_init_level;
	p0_param_ptr->is_p0 = 1;   // set this px to p0, p0 will open pwm isr, p1 not
	p0_param_ptr->another_pwm_idx = p1_chan;   // record another chan id
	p0_param_ptr->status = PWM_STATUS_UNINIT;
	p0_param_ptr->t1 = pwm_param->p0_t1;
	p0_param_ptr->t2 = 0;
	p0_param_ptr->t3 = 0;
	p0_param_ptr->t4 = pwm_param->p_t4;
	p0_param_ptr->is_active = 1;

	os_memset(p1_param_ptr, 0, sizeof(drv_pwm_cw_param_t));
	p1_param_ptr->init_level = pwm_param->p1_init_level;
	p1_param_ptr->is_p0 = 0;
	p1_param_ptr->another_pwm_idx = p0_chan;  // record another chan id
	p1_param_ptr->status = PWM_STATUS_UNINIT;
	p1_param_ptr->t1 = pwm_param->p1_t1;
	p1_param_ptr->t2 = pwm_param->p1_t2;
	p1_param_ptr->t3 = 0;
	p1_param_ptr->t4 = pwm_param->p_t4;
	p1_param_ptr->is_active = 1;

	// p0
	p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
	p0_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
	p0_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
	p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(p0_post));
	p0_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(p0_post));
	p0_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(p0_post));
	p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(p0_post));
	p0_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(p0_post));
	p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK((!p0_post))); // not clear other chan int status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p0_group), p0_cfg_reg);

	// p1
	p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
	p1_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
	p1_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
	p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(p1_post));
	p1_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(p1_post));
	p1_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(p1_post));
	p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(p1_post));
	p1_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(p1_post));
	p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK((!p1_post))); // not clear other chan int status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p1_group), p1_cfg_reg);

	p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
	p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
	pwm_cw_updata_regs(p1_param_ptr, p0_param_ptr, &p1_cfg_reg, &p0_cfg_reg);

	p0_cfg_reg |= PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(p0_post);
	p0_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(p0_post);  // clear init status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p0_group), p0_cfg_reg);

	p1_cfg_reg |= PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(p1_post);
	p1_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(p1_post);  // clear init status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p1_group), p1_cfg_reg);

	pwm_gpio_configuration(p0_chan, 1);
	pwm_gpio_configuration(p1_chan, 1);
	pwm_icu_configuration(p0_chan, PWM_CLK_26M, 1, 1);
	pwm_icu_configuration(p1_chan, PWM_CLK_26M, 1, 0);

	// clear int status, but no enable here, but enable in start or updata
	PWM_PRT("pwm cw inited:0x%lx,0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group)),  REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group)));
	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_cw_start(UINT8 pwm1, UINT8 pwm2)
{
	UINT32 p0_cfg_reg, p1_cfg_reg;
	drv_pwm_cw_param_t *p0_param_ptr, *p1_param_ptr;
	UINT8 p0_chan, p1_chan, ret;
	UINT8 p0_group, p1_group;
	UINT8 p0_post, p1_post;   // 0: in low bits, 1: in high bits

	GLOBAL_INT_DECLARATION();

	if((pwm1 >= PWM_COUNT)
		|| (pwm2 >= PWM_COUNT))
	{
		return 1;
	}

	p0_chan = pwm2;
	p1_chan = pwm1;
	p0_param_ptr = &g_cw_pwm_param[p0_chan];
	p1_param_ptr = &g_cw_pwm_param[p1_chan];
	if(p0_param_ptr->is_p0 == p1_param_ptr->is_p0)
	{
		// only one chann is p0
		return 1;
	}

	// if this px is not ture p0, another px must be p0
	if(p0_param_ptr->is_p0 == 0)
	{
		p0_chan = pwm1;
		p1_chan = pwm2;
	}

	p0_param_ptr = &g_cw_pwm_param[p0_chan];
	p1_param_ptr = &g_cw_pwm_param[p1_chan];
	p0_group = PWM_CHAN_TO_GROUP_NUM(p0_chan);
	p1_group = PWM_CHAN_TO_GROUP_NUM(p1_chan);
	p0_post = PWM_CHAN_TO_CHAN_IN_GROUP(p0_chan);
	p1_post = PWM_CHAN_TO_CHAN_IN_GROUP(p1_chan);

	if(p0_group == p1_group)
	{
		return 1;
	}

	ret = pwm_check_is_used(p0_chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_CW))
	{
		return 3;
	}
	ret = pwm_check_is_used(p1_chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_CW))
	{
		return 3;
	}

	// enable
	p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
	p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
	if (((p0_cfg_reg & PWM_GROUP_PWM_ENABLE_MASK(p0_post)) == PWM_GROUP_PWM_ENABLE_MASK(p0_post))
		&& ((p1_cfg_reg & PWM_GROUP_PWM_ENABLE_MASK(p1_post)) == PWM_GROUP_PWM_ENABLE_MASK(p1_post)))
	{
		if (((p0_cfg_reg & PWM_GROUP_MODE_SET_MASK(p0_post)) == PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(p0_post))
			&& ((p1_cfg_reg & PWM_GROUP_MODE_SET_MASK(p1_post)) == PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(p1_post)))
		{
			return 0;
		}
	}

	GLOBAL_INT_DISABLE();
	p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
	p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
	pwm_cw_updata_regs(p1_param_ptr, p0_param_ptr, &p1_cfg_reg, &p0_cfg_reg);

	p0_cfg_reg |= PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(p0_post);
	p0_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(p0_post);  // clear init status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p0_group), p0_cfg_reg);

	p1_cfg_reg |= PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(p1_post);
	p1_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(p1_post);  // clear init status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p1_group), p1_cfg_reg);

	// start
	p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
	p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0)); // not clear int status
	p0_cfg_reg |= PWM_GROUP_PWM_ENABLE_MASK(p0_post);
	p0_cfg_reg |= PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(p0_post);

	p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
	p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0)); // not clear int status
	p1_cfg_reg |= PWM_GROUP_PWM_ENABLE_MASK(p1_post);
	p1_cfg_reg |= PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(p1_post);

	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p0_group), p0_cfg_reg);
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p1_group), p1_cfg_reg);
	p0_param_ptr->is_active = 1;
	p1_param_ptr->is_active = 1;

	PWM_PRT("pwm cw started:0x%lx,0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group)),  REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group)));
	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_cw_update_param(pwm_cw_group_param_t *pwm_param)
{
	UINT32 p0_cfg_reg, p1_cfg_reg;
	drv_pwm_cw_param_t *p0_param_ptr, *p1_param_ptr;
	UINT8 p0_chan, p1_chan, ret;
	UINT8 p0_group, p1_group;
	UINT8 p0_post, p1_post;   // 0: in low bits, 1: in high bits

	GLOBAL_INT_DECLARATION();

	if (pwm_param == NULL)
	{
		return 1;
	}

	p0_chan = GET_P0_IDX_FROM_IDXS(pwm_param->group);
	p1_chan = GET_P1_IDX_FROM_IDXS(pwm_param->group);
	if((p0_chan >= PWM_COUNT)
		|| (p1_chan >= PWM_COUNT))
	{
		return 1;
	}

	p0_param_ptr = &g_cw_pwm_param[p0_chan];
	p1_param_ptr = &g_cw_pwm_param[p1_chan];

	if(p0_param_ptr->is_p0 == p1_param_ptr->is_p0)
	{
		// only one chann is p0
		return 1;
	}

	// if this px is not ture p0, another px must be p0
	if(p0_param_ptr->is_p0 == 0)
	{
		p0_chan = GET_P1_IDX_FROM_IDXS(pwm_param->group);
		p1_chan = GET_P0_IDX_FROM_IDXS(pwm_param->group);;
	}

	p0_param_ptr = &g_cw_pwm_param[p0_chan];
	p1_param_ptr = &g_cw_pwm_param[p1_chan];
	p0_group = PWM_CHAN_TO_GROUP_NUM(p0_chan);
	p1_group = PWM_CHAN_TO_GROUP_NUM(p1_chan);
	p0_post = PWM_CHAN_TO_CHAN_IN_GROUP(p0_chan);
	p1_post = PWM_CHAN_TO_CHAN_IN_GROUP(p1_chan);
	p1_post = p1_post;

	if(p0_group == p1_group)
	{
		return 1;
	}

	ret = pwm_check_is_used(p0_chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_CW))
	{
		return 3;
	}
	ret = pwm_check_is_used(p1_chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_CW))
	{
		return 3;
	}


	GLOBAL_INT_DISABLE();
	// only copy them, and enable p0 isr, updata to registers in isr
	p0_param_ptr->init_level = pwm_param->p0_init_level;
	p0_param_ptr->is_p0 = 1;
	p0_param_ptr->another_pwm_idx = p1_chan;   // record another chan id
	p0_param_ptr->status = PWM_STATUS_LOADING;
	p0_param_ptr->t1 = pwm_param->p0_t1;
	p0_param_ptr->t2 = 0;
	p0_param_ptr->t3 = 0;
	p0_param_ptr->t4 = pwm_param->p_t4;

	p1_param_ptr->init_level = pwm_param->p1_init_level;
	p1_param_ptr->is_p0 = 0;
	p1_param_ptr->another_pwm_idx = p0_chan;  // record another chan id
	p1_param_ptr->status = PWM_STATUS_LOADING;
	p1_param_ptr->t1 = pwm_param->p1_t1;
	p1_param_ptr->t2 = pwm_param->p1_t2;
	p1_param_ptr->t3 = 0;
	p1_param_ptr->t4 = pwm_param->p_t4;

	p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
	p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
	p1_cfg_reg = p1_cfg_reg;
	if ((p0_cfg_reg & PWM_GROUP_PWM_INT_ENABLE_MASK(p0_post)) == 0)
	{
		p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0)); // not clear int status
		p0_cfg_reg |= PWM_GROUP_PWM_INT_ENABLE_MASK(p0_post);							  // only enable p0 interrupt
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p0_group), p0_cfg_reg);
	}

	p0_param_ptr->status = PWM_STATUS_LOADING;
	p1_param_ptr->status = PWM_STATUS_LOADING;
	p0_param_ptr->is_active = 1;
	p1_param_ptr->is_active = 1;

	PWM_PRT("pwm cw updataed:0x%08x,0x%08x\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group)),  REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group)));

	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_cw_stop(UINT8 pwm1, UINT8 pwm2)
{
	UINT32 p0_cfg_reg, p1_cfg_reg;
	drv_pwm_cw_param_t *p0_param_ptr, *p1_param_ptr;
	UINT8 p0_chan, p1_chan, ret;
	UINT8 p0_group, p1_group;
	UINT8 p0_post, p1_post;   // 0: in low bits, 1: in high bits

	GLOBAL_INT_DECLARATION();

	if((pwm1 >= PWM_COUNT)
		|| (pwm2 >= PWM_COUNT))
	{
		return 1;
	}

	p0_chan = pwm2;
	p1_chan = pwm1;
	p0_param_ptr = &g_cw_pwm_param[p0_chan];
	p1_param_ptr = &g_cw_pwm_param[p1_chan];
	if((p0_param_ptr->is_active == 0)
		|| (p0_param_ptr->is_active == 0))
	{
		return 1;
	}

	if(p0_param_ptr->is_p0 == p1_param_ptr->is_p0)
	{
		// only one chann is p0
		return 1;
	}

	// if this px is not ture p0, another px must be p0
	if(p0_param_ptr->is_p0 == 0)
	{
		p0_chan = pwm1;
		p1_chan = pwm2;
	}

	p0_param_ptr = &g_cw_pwm_param[p0_chan];
	p1_param_ptr = &g_cw_pwm_param[p1_chan];
	p0_group = PWM_CHAN_TO_GROUP_NUM(p0_chan);
	p1_group = PWM_CHAN_TO_GROUP_NUM(p1_chan);
	p0_post = PWM_CHAN_TO_CHAN_IN_GROUP(p0_chan);
	p1_post = PWM_CHAN_TO_CHAN_IN_GROUP(p1_chan);

	if(p0_group == p1_group)
	{
		return 1;
	}

	ret = pwm_check_is_used(p0_chan);
	if(ret != PWM_USED_CW)
	{
		return 3;
	}
	ret = pwm_check_is_used(p1_chan);
	if(ret != PWM_USED_CW)
	{
		return 3;
	}

	// enable
	p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
	p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
	if ((p0_cfg_reg & PWM_GROUP_PWM_ENABLE_MASK(p0_post)) == PWM_GROUP_PWM_ENABLE_MASK(p0_post)
		&& (p1_cfg_reg & PWM_GROUP_PWM_ENABLE_MASK(p1_post)) == PWM_GROUP_PWM_ENABLE_MASK(p1_post))
	{
		GLOBAL_INT_DISABLE();
		p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
		p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(!p0_post)); // not clear another int status
		p0_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(p0_post); 	// clear int status
		p0_cfg_reg &= ~(PWM_GROUP_PWM_ENABLE_MASK(p0_post));

		p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
		p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0)); // not clear int status
		p1_cfg_reg &= ~(PWM_GROUP_PWM_ENABLE_MASK(p1_post));

		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p0_group), p0_cfg_reg);
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p1_group), p1_cfg_reg);

		// init
		// p0
		p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
		p0_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
		p0_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
		p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(p0_post));
		p0_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(p0_post));
		p0_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(p0_post));
		p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(p0_post));
		p0_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(p0_post));
		p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK((!p0_post))); // not clear other chan int status
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p0_group), p0_cfg_reg);

		// p1
		p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
		p1_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
		p1_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
		p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(p1_post));
		p1_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(p1_post));
		p1_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(p1_post));
		p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(p1_post));
		p1_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(p1_post));
		p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK((!p1_post))); // not clear other chan int status
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p1_group), p1_cfg_reg);

		p0_param_ptr->status = PWM_STATUS_INITED;
		p1_param_ptr->status = PWM_STATUS_INITED;
		p0_param_ptr->is_active = 0;
		p1_param_ptr->is_active = 0;

		PWM_PRT("pwm cw stop:0x%08x,0x%08x\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group)),  REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group)));
		GLOBAL_INT_RESTORE();
	}

	return 0;
}
#else
UINT8 pwm_cw_init_param(pwm_cw_group_param_t *pwm_param)
{
	return 1;
}

UINT8 pwm_cw_update_param(pwm_cw_group_param_t *pwm_param)
{
	return 1;
}

UINT8 pwm_cw_stop(UINT8 pwm1, UINT8 pwm2)
{
	return 1;
}

UINT8 pwm_cw_start(UINT8 pwm1, UINT8 pwm2)
{
	return 1;
}
#endif // DRV_USED_PWM_CW

#if DRV_USED_PWM
static inline void pwm_updata_regs(pwm_param_st *pwm_param_ptr, UINT32 *pwm_cfg_reg_ptr)
{
	UINT32 pwm_cfg_reg;
	UINT8 chan;
	UINT8 group;
	UINT8 post;   // 0: in low bits, 1: in high bits

	chan = pwm_param_ptr->chan;
	pwm_cfg_reg = *pwm_cfg_reg_ptr;

	group = PWM_CHAN_TO_GROUP_NUM(chan);
	post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);
	// setting reg
	if(post == 0)
	{
		REG_WRITE(REG_GROUP_PWM0_T1_ADDR(group), pwm_param_ptr->t1);
		REG_WRITE(REG_GROUP_PWM0_T2_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM0_T3_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM0_T4_ADDR(group), pwm_param_ptr->t4);
	}
	else
	{
		REG_WRITE(REG_GROUP_PWM1_T1_ADDR(group), pwm_param_ptr->t1);
		REG_WRITE(REG_GROUP_PWM1_T2_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM1_T3_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM1_T4_ADDR(group), pwm_param_ptr->t4);
	}
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(post));
	pwm_cfg_reg |= PWM_GROUP_PWM_CFG_UPDATA_MASK(post);
	pwm_cfg_reg |= (pwm_param_ptr->init_level << PWM_GROUP_PWM_INT_LEVL_BIT(post));
	pwm_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(post);  // clear init status

	*pwm_cfg_reg_ptr = pwm_cfg_reg;
}

UINT8 pwm_init_param(pwm_param_st *pwm_param)
{
	UINT32 pwm_cfg_reg;
	pwm_param_st *pwm_param_ptr;
	UINT8 chan;
	UINT8 group;
	UINT8 post;   // 0: in low bits, 1: in high bits
	GLOBAL_INT_DECLARATION();

	if (pwm_param == NULL)
	{
		return 1;
	}

	chan = pwm_param->chan;
	if(chan >= PWM_COUNT)
	{
		return 1;
	}

	if(pwm_check_is_used(chan) != PWM_NOT_USED)
	{
		return 3;
	}

	group = PWM_CHAN_TO_GROUP_NUM(chan);
	post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);

	// check p0 p1 used,  in cw-group-pwm and only-pwm
	// check this chan no used in cw-pwm

	pwm_param_ptr = &g_pwm_param[chan];

	GLOBAL_INT_DISABLE();
	os_memset(pwm_param_ptr, 0, sizeof(pwm_param_st));
	pwm_param_ptr->init_level = pwm_param->init_level;
	pwm_param_ptr->chan = chan;   // record another chan id
	pwm_param_ptr->status = PWM_STATUS_UNINIT;
	pwm_param_ptr->t1 = pwm_param->t1;
	pwm_param_ptr->t4 = pwm_param->t4;

	// p0
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK((!post))); // not clear other chan int status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_updata_regs(pwm_param_ptr, &pwm_cfg_reg);
	pwm_cfg_reg |= PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(post);
	pwm_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(post);  // clear init status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	pwm_gpio_configuration(chan, 1);
	pwm_icu_configuration(chan, PWM_CLK_26M, 1, 1);

	pwm_param_ptr->is_active = 1;

	// clear int status, but no enable here, but enable in start or updata
	PWM_PRT("pwm inited:0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_start(UINT8 chan)
{
	UINT32 pwm_cfg_reg;
	pwm_param_st *pwm_param_ptr;
	UINT8 group, ret;
	UINT8 post;   // 0: in low bits, 1: in high bits
	GLOBAL_INT_DECLARATION();

	if(chan >= PWM_COUNT)
	{
		return 1;
	}

	ret = pwm_check_is_used(chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_PWM))
	{
		return 3;
	}

	group = PWM_CHAN_TO_GROUP_NUM(chan);
	post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);
	pwm_param_ptr = &g_pwm_param[chan];

	// enable
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	if ((pwm_cfg_reg & PWM_GROUP_PWM_ENABLE_MASK(post)) == PWM_GROUP_PWM_ENABLE_MASK(post))
	{
		if ((pwm_cfg_reg & PWM_GROUP_MODE_SET_MASK(post)) == PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(post))
		{
			return 0;
		}
	}

	GLOBAL_INT_DISABLE();
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_updata_regs(pwm_param_ptr, &pwm_cfg_reg);
	pwm_cfg_reg |= PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(post);
	pwm_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(post);  // clear init status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	// start
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0)); // not clear int status
	pwm_cfg_reg |= PWM_GROUP_PWM_ENABLE_MASK(post);
	pwm_cfg_reg |= PWM_PWM_MODE << PWM_GROUP_MODE_SET_BIT(post);

	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);
	//local_param_ptr->status = PWM_STATUS_RUNNING;
	pwm_param_ptr->is_active = 1;

	PWM_PRT("pwm started:0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_update_param(pwm_param_st *pwm_param)
{
	UINT32 pwm_cfg_reg;
	pwm_param_st *pwm_param_ptr;
	UINT8 chan;
	UINT8 group, ret;
	UINT8 post;   // 0: in low bits, 1: in high bits
	GLOBAL_INT_DECLARATION();

	if (pwm_param == NULL)
	{
		return 1;
	}

	chan = pwm_param->chan;
	if(chan >= PWM_COUNT)
	{
		return 1;
	}

	ret = pwm_check_is_used(chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_PWM))
	{
		return 3;
	}

	group = PWM_CHAN_TO_GROUP_NUM(chan);
	post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);
	pwm_param_ptr = &g_pwm_param[chan];

	GLOBAL_INT_DISABLE();
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	if ((pwm_cfg_reg & PWM_GROUP_PWM_INT_ENABLE_MASK(post)) == 0)
	{
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0)); // not clear int status
		pwm_cfg_reg |= PWM_GROUP_PWM_INT_ENABLE_MASK(post);							  // only enable p0 interrupt
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);
	}
	// only copy them, and enable p0 isr, updata to registers in isr
	pwm_param_ptr->init_level = pwm_param->init_level;
	pwm_param_ptr->chan = chan;   // record another chan id
	pwm_param_ptr->t1 = pwm_param->t1;
	pwm_param_ptr->t4 = pwm_param->t4;
	pwm_param_ptr->status = PWM_STATUS_LOADING;
	pwm_param_ptr->is_active = 1;
	PWM_PRT("pwm updataed:0x%08x\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_stop(UINT8 chan)
{
	UINT32 pwm_cfg_reg;
	pwm_param_st *pwm_param_ptr;
	UINT8 group, ret;
	UINT8 post;   // 0: in low bits, 1: in high bits
	GLOBAL_INT_DECLARATION();

	if(chan >= PWM_COUNT)
	{
		return 1;
	}

	ret = pwm_check_is_used(chan);
	if(ret != PWM_USED_PWM)
	{
		return 3;
	}

	group = PWM_CHAN_TO_GROUP_NUM(chan);
	post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);
	pwm_param_ptr = &g_pwm_param[chan];
	// enable
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	if ((pwm_cfg_reg & PWM_GROUP_PWM_ENABLE_MASK(post)) == PWM_GROUP_PWM_ENABLE_MASK(post))
	{
		GLOBAL_INT_DISABLE();
		pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(!post)); // not clear another int status
		pwm_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(post); 	// clear int status
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_ENABLE_MASK(post));

		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

		// init
		// p0
		pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(post));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(post));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(post));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(post));
		pwm_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(post));
		pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK((!post))); // not clear other chan int status
		REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

		pwm_param_ptr->status = PWM_STATUS_INITED;
		pwm_param_ptr->is_active = 0;

		PWM_PRT("pwm stop:0x%08x\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
		GLOBAL_INT_RESTORE();
	}

	return 0;
}
#else
UINT8 pwm_init_param(pwm_param_st *pwm_param)
{
	return 1;
}

UINT8 pwm_update_param(pwm_param_st *pwm_param)
{
	return 1;
}

UINT8 pwm_start(UINT8 chan)
{
	return 1;
}

UINT8 pwm_stop(UINT8 chan)
{
	return 1;
}
#endif // DRV_USED_PWM

#if DRV_USED_PWM_CAP
UINT32 pwm_capture_value_get(UINT8 chan)
{
	UINT32 value;
	UINT8 group, post;   // 0: in low bits, 1: in high bits

	if(chan >= PWM_COUNT)
	{
		return 0;
	}

	group = PWM_CHAN_TO_GROUP_NUM(chan);
	post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);

	value = REG_READ(REG_GROUP_PWM_CPU_ADDR(group));
	value |=  1 << post;
	REG_WRITE(REG_GROUP_PWM_CPU_ADDR(group), value);

	value = REG_READ(REG_GROUP_PWM_CPU_ADDR(group));
	while ((value & (1 << post)) != 0)
	{
		value = REG_READ(REG_GROUP_PWM_CPU_ADDR(group));
	}

	if (post == 1)
	{
		value = REG_READ(REG_GROUP_PWM1_RD_DATA_ADDR(group));
	}
	else
	{
		value = REG_READ(REG_GROUP_PWM0_RD_DATA_ADDR(group));
	}

	return value;
}

UINT8 pwm_capture_init_param(pwm_cap_param_st *pwm_param)
{
	UINT32 pwm_cfg_reg;
	pwm_cap_param_st *pwm_cap_param_ptr = NULL;
	UINT8 chan;
	UINT8 group;
	UINT8 post;   // 0: in low bits, 1: in high bits
	GLOBAL_INT_DECLARATION();

	if (pwm_param == NULL)
	{
		return 1;
	}

	chan = pwm_param->chan;
	if(chan >= PWM_COUNT)
	{
		return 1;
	}

	if(pwm_check_is_used(chan) != PWM_NOT_USED)
	{
		return 3;
	}

	if(pwm_param->mode < PWM_CAP_POS_MODE)
	{
		return 2;
	}

	group = PWM_CHAN_TO_GROUP_NUM(chan);
	post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);
	pwm_cap_param_ptr = &g_pwm_cap_param[chan];

	GLOBAL_INT_DISABLE();
	os_memset(pwm_cap_param_ptr, 0, sizeof(pwm_param_st));
	pwm_cap_param_ptr->mode = pwm_param->mode;
	pwm_cap_param_ptr->chan = chan;   // record another chan id
	pwm_cap_param_ptr->status = PWM_STATUS_INITED;
	pwm_cap_param_ptr->p_Int_Handler = pwm_param->p_Int_Handler;

	// p0
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK((!post))); // not clear other chan int status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	if(post == 0)
	{
		REG_WRITE(REG_GROUP_PWM0_T1_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM0_T2_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM0_T3_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM0_T4_ADDR(group), 0);
	}
	else
	{
		REG_WRITE(REG_GROUP_PWM1_T1_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM1_T2_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM1_T3_ADDR(group), 0);
		REG_WRITE(REG_GROUP_PWM1_T4_ADDR(group), 0);
	}
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(post));
	pwm_cfg_reg |= PWM_GROUP_PWM_CFG_UPDATA_MASK(post);
	pwm_cfg_reg |= (0 << PWM_GROUP_PWM_INT_LEVL_BIT(post));
	pwm_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(post);  // clear init status
	pwm_cfg_reg |= pwm_cap_param_ptr->mode << PWM_GROUP_MODE_SET_BIT(post);
	pwm_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(post);  // clear init status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	pwm_gpio_configuration(chan, 1);
	pwm_icu_configuration(chan, PWM_CLK_26M, 1, 1);

	pwm_cap_param_ptr->is_active = 1;

	// clear int status, but no enable here, but enable in start or updata
	PWM_PRT("pwm cap inited:0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_capture_start(UINT8 chan)
{
	UINT32 pwm_cfg_reg;
	pwm_cap_param_st *pwm_cap_param_ptr = NULL;
	UINT8 group, ret;
	UINT8 post;   // 0: in low bits, 1: in high bits
	GLOBAL_INT_DECLARATION();

	if(chan >= PWM_COUNT)
	{
		return 1;
	}

	ret = pwm_check_is_used(chan);
	if((ret != PWM_NOT_USED) && (ret != PWM_USED_PWM_CAP))
	{
		return 3;
	}

	group = PWM_CHAN_TO_GROUP_NUM(chan);
	post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);
	pwm_cap_param_ptr = &g_pwm_cap_param[chan];

	// enable
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	if ((pwm_cfg_reg & PWM_GROUP_PWM_ENABLE_MASK(post)) == PWM_GROUP_PWM_ENABLE_MASK(post))
	{
		if ((pwm_cfg_reg & PWM_GROUP_MODE_SET_MASK(post)) == (pwm_cap_param_ptr->mode << PWM_GROUP_MODE_SET_BIT(post)))
		{
			return 0;
		}
	}

	GLOBAL_INT_DISABLE();
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_cfg_reg |= pwm_cap_param_ptr->mode << PWM_GROUP_MODE_SET_BIT(post);
	pwm_cfg_reg |= PWM_GROUP_PWM_INT_STAT_MASK(post);  // clear init status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	// start
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0)); // not clear int status
	pwm_cfg_reg |= PWM_GROUP_PWM_ENABLE_MASK(post);

	if(pwm_cap_param_ptr->p_Int_Handler)
		pwm_cfg_reg |= PWM_GROUP_PWM_INT_ENABLE_MASK(post);

	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);
	pwm_cap_param_ptr->status = PWM_STATUS_RUNNING;
	pwm_cap_param_ptr->is_active = 1;

	PWM_PRT("pwm cap started:0x%lx\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
	GLOBAL_INT_RESTORE();

	return 0;
}

UINT8 pwm_capture_stop(UINT8 chan)
{
	UINT32 pwm_cfg_reg;
	pwm_cap_param_st *pwm_cap_param_ptr = NULL;
	UINT8 group, post;
	UINT8 ret;   // 0: in low bits, 1: in high bits
	GLOBAL_INT_DECLARATION();

	if(chan >= PWM_COUNT)
	{
		return 1;
	}

	ret = pwm_check_is_used(chan);
	if(ret != PWM_USED_PWM)
	{
		return 3;
	}

	group = PWM_CHAN_TO_GROUP_NUM(chan);
	post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);
	pwm_cap_param_ptr = &g_pwm_cap_param[chan];

	GLOBAL_INT_DISABLE();
	// init
	// p0
	pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_GROUP_MODE_ENABLE_MASK | PWM_GROUP_PWM_GROUP_MODE_MASK);
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_PRE_DIV_MASK);
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_LEVL_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_CFG_UPDATA_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_STOP_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_MODE_SET_MASK(post));
	pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK((!post))); // not clear other chan int status
	REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);

	pwm_cap_param_ptr->status = PWM_STATUS_INITED;
	pwm_cap_param_ptr->is_active = 0;

	PWM_PRT("pwm cap deinit:0x%08x\r\n", REG_READ(REG_PWM_GROUP_CTRL_ADDR(group)));
	GLOBAL_INT_RESTORE();

	return 0;
}
#else
UINT8 pwm_capture_init_param(pwm_cap_param_st *pwm_param)
{
	return 1;
}

UINT8 pwm_capture_start(UINT8 chan)
{
	return 1;
}

UINT8 pwm_capture_stop(UINT8 chan)
{
	return 1;
}

UINT32 pwm_capture_value_get(UINT8 chan)
{
	return 0;
}
#endif // DRV_USED_PWM_CAP

void pwm_isr(void)
{
	int group, idx;
	UINT32 status = 0;

	for (group = 0; group < PWM_GROUP_NUM; group++)
	{
		for (idx = 0; idx < PWM_CHAN_IN_GROUP; idx++)
		{
			status = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
			if (status & PWM_GROUP_PWM_INT_STAT_MASK(idx))
			{
				UINT8 chan = group * PWM_CHAN_IN_GROUP + idx;

				#if DRV_USED_PWM_CW_GROUP
				pwm_cw_group_param_t *group_param_ptr = &g_cw_group_pwm_param[group];
				#endif

				#if DRV_USED_PWM_CW
				drv_pwm_cw_param_t *cw_param_ptr = &g_cw_pwm_param[chan];
				#endif

				#if DRV_USED_PWM
				pwm_param_st *pwm_param_ptr = &g_pwm_param[chan];
				#endif

				#if DRV_USED_PWM_CAP
				pwm_cap_param_st *pwm_cap_param_ptr = &g_pwm_cap_param[chan];
				#endif

				// clear isr status first
				status &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0));
				status |= PWM_GROUP_PWM_INT_STAT_MASK(idx);
				REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), status);

				#if DRV_USED_PWM_CW_GROUP
				if (group_param_ptr->is_active)
				{
					if (group_param_ptr->status == PWM_STATUS_LOADING) // idx need to only 0
					{
						status = pwm_cw_group_updata_regs(group_param_ptr, status);

						status &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0));
						status &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(0));	 // disable p0 interrupt after updata
						REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), status);
						group_param_ptr->status = PWM_STATUS_RUNNING;
					}
				} else
				#endif // DRV_USED_PWM_CW_GROUP
				#if DRV_USED_PWM_CW
				if(cw_param_ptr->is_active)
				{
					drv_pwm_cw_param_t *p0_param_ptr, *p1_param_ptr;
					UINT8 p0_chan, p1_chan;
					p0_chan = chan;
					p0_param_ptr = &g_cw_pwm_param[p0_chan];
					p1_chan = p0_param_ptr->another_pwm_idx;
					p1_param_ptr = &g_cw_pwm_param[p1_chan];
					if((p0_param_ptr->is_active) && (p0_param_ptr->is_p0)
						&& (p1_param_ptr->is_active) && (p0_param_ptr->status == PWM_STATUS_LOADING))
					{
						UINT8 p0_group, p1_group;
						UINT32 p0_cfg_reg, p1_cfg_reg;
						UINT8 p0_post;

						p0_group = PWM_CHAN_TO_GROUP_NUM(p0_chan);
						p1_group = PWM_CHAN_TO_GROUP_NUM(p1_chan);
						p0_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p0_group));
						p1_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(p1_group));
						p0_post = PWM_CHAN_TO_CHAN_IN_GROUP(p0_chan);

						pwm_cw_updata_regs(p1_param_ptr, p0_param_ptr, &p1_cfg_reg, &p0_cfg_reg);

						p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0));
						p0_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(p0_post));	 // disable p0 interrupt after updata
						p1_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0));
						REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p0_group), p0_cfg_reg);
						REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(p1_group), p1_cfg_reg);
						status = p0_cfg_reg;
						p0_param_ptr->status = PWM_STATUS_RUNNING;
						p1_param_ptr->status = PWM_STATUS_RUNNING;
					}
				} else
				#endif // DRV_USED_PWM_CW
				#if DRV_USED_PWM
				if(pwm_param_ptr->is_active)
				{
					if(pwm_param_ptr->status == PWM_STATUS_LOADING)
					{
						UINT32 pwm_cfg_reg = REG_READ(REG_PWM_GROUP_CTRL_ADDR(group));
						UINT8 post = PWM_CHAN_TO_CHAN_IN_GROUP(chan);

						pwm_updata_regs(pwm_param_ptr, &pwm_cfg_reg);
						pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_STAT_MASK(1) | PWM_GROUP_PWM_INT_STAT_MASK(0));
						pwm_cfg_reg &= ~(PWM_GROUP_PWM_INT_ENABLE_MASK(post));	 // disable p0 interrupt after updata
						REG_WRITE(REG_PWM_GROUP_CTRL_ADDR(group), pwm_cfg_reg);
						status = pwm_cfg_reg;
						pwm_param_ptr->status = PWM_STATUS_RUNNING;
					}
				} else
				#endif // DRV_USED_PWM
				#if DRV_USED_PWM_CAP
				if(pwm_cap_param_ptr->is_active)
				{
					UINT32 cap_value = pwm_capture_value_get(chan);
					if(pwm_cap_param_ptr->p_Int_Handler)
						pwm_cap_param_ptr->p_Int_Handler(chan, cap_value);
				}
				#endif //DRV_USED_PWM
				{
					chan = chan; // fix warning
				}
			}
		}
	}
}
#endif
