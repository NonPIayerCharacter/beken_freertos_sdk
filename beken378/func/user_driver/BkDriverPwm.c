/**
 ******************************************************************************
 * @file    BkDriverPwm.h
 * @brief   This file provides all the headers of PWM operation functions.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2017 BEKEN Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */
#include "include.h"
#include "rtos_pub.h"
#include "BkDriverPwm.h"
#include "drv_model_pub.h"
#include "error.h"
#include "pwm_pub.h"


#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
#include "bk_err.h"
#include "pwm_bk7271.h"

OSStatus bk_pwm_initialize(bk_pwm_t pwm, uint32_t frequency, uint32_t duty_cycle)
{
	UINT32 ret;
	pwm_param_t param;

	/*init pwm*/
	param.channel			= (uint8_t)pwm;
	param.cfg.bits.en		= PWM_INT_DIS;
	param.cfg.bits.int_en	= PWM_INT_DIS;
	param.cfg.bits.mode		= PWM_PWM_MODE;
	param.cfg.bits.clk		= PWM_CLK_26M;
	param.p_Int_Handler		= 0;
	param.duty_cycle		= duty_cycle;
	param.end_value			= frequency;

	ret = sddev_control(PWM_DEV_NAME, CMD_PWM_INIT_PARAM, &param);
	ASSERT(PWM_SUCCESS == ret);

	return ret;

}

OSStatus bk_pwm_capture_initialize(bk_pwm_t pwm, uint8_t cap_mode)
{

	uint8_t ret;
	pwm_param_t param;

	/*init pwm*/
	param.channel			= (uint8_t)pwm;
	param.cfg.bits.en		= PWM_INT_EN;
	param.cfg.bits.int_en	= PWM_INT_EN;
	param.cfg.bits.clk		= PWM_CLK_26M;
	param.p_Int_Handler		= 0;
	param.duty_cycle		= 0;
	param.end_value			= 0;

	if (cap_mode == 0x01)
		param.cfg.bits.mode   = PWM_CAP_POS_MODE;
	else if (cap_mode == 0x02)
		param.cfg.bits.mode   = PWM_CAP_NEG_MODE;
	else
		param.cfg.bits.mode   = PWM_PWM_MODE;

	ret = sddev_control(PWM_DEV_NAME, CMD_PWM_INIT_PARAM, &param);
	ASSERT(PWM_SUCCESS == ret);

	return kNoErr;
}

OSStatus bk_pwm_start(bk_pwm_t pwm)
{
    UINT32 ret;
    UINT32 param;

    param = pwm;

    ret = sddev_control(PWM_DEV_NAME, CMD_PWM_UNIT_ENABLE, &param);
    ASSERT(PWM_SUCCESS == ret);

    return kNoErr;
}

OSStatus bk_pwm_stop(bk_pwm_t pwm)
{
    UINT32 ret;
    UINT32 param;

    param = pwm;
    ret = sddev_control(PWM_DEV_NAME, CMD_PWM_UNIT_DISABLE, &param);
    ASSERT(PWM_SUCCESS == ret);

    return kNoErr;
}

bk_err_t bk_pwm_update_param(bk_pwm_t pwm, uint32_t frequency, uint32_t duty_cycle)
{
	pwm_param_t param;
	UINT32 ret;

	param.channel         = (uint8_t)pwm;
	param.duty_cycle	  = duty_cycle;
	param.end_value       = frequency;
	ret = sddev_control(PWM_DEV_NAME, CMD_PWM_UPDATE_PARAM, &param);

	return ret;
}

#else
#include "mem_pub.h"
#include "pwm_bk7231n.h"

UINT32 bk_pwm_get_capvalue(bk_pwm_t pwm)
{
	UINT32 ret;

	ret = pwm_capture_value_get(pwm);

	return ret;
}

static void pwm_cap_int_callback(UINT8 chan, UINT32 value)
{
	PWM_LOGI(TAG, "pwm %d cap value:%d \r\n", chan, value);
}

OSStatus bk_pwm_capture_initialize(bk_pwm_t pwm, UINT8 cap_mode)
{
	UINT32 ret;
	pwm_cap_param_st param;

	/*init pwm*/
	param.chan			= (uint8_t)pwm;
	param.mode			= cap_mode;  //
	param.status		= 0;
	param.is_active		= 0;
	param.p_Int_Handler = pwm_cap_int_callback;

	ret = pwm_capture_init_param(&param);
	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}

OSStatus bk_pwm_capture_start(bk_pwm_t pwm)
{
	UINT32 ret;

	ret = pwm_capture_start((uint8_t)pwm);
	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}

OSStatus bk_pwm_capture_stop(bk_pwm_t pwm)
{
	UINT32 ret;

	ret = pwm_capture_stop((uint8_t)pwm);
	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}

OSStatus bk_pwm_start(bk_pwm_t pwm)
{
	UINT32 ret;

	ret =  pwm_start(pwm);
	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}

OSStatus bk_pwm_stop(bk_pwm_t pwm)
{
	UINT32 ret;

	ret =  pwm_stop(pwm);
	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}


OSStatus bk_pwm_initialize(bk_pwm_t pwm, uint32_t frequency, uint32_t duty_cycle)
{
	UINT8 ret;
	pwm_param_st param;

	if (frequency < duty_cycle)
	{
		bk_printf("bk_pwm_initialize error:freq:%x ,cycle1:%x\r\n", frequency, duty_cycle);
		return kGeneralErr;
	}
	os_memset(&param, 0, sizeof(pwm_param_st));

	param.chan = pwm;
	param.t4 = frequency;

	param.t1 = duty_cycle;
	param.init_level = 1;  // if duty cycle not 0, high level pwm pin first
	if(param.t1 == 0)
	{
		param.init_level = 0;
	}

	ret = pwm_init_param(&param);
	bk_printf("bk pwm initial:duty_cycle1 = %x, ret:%d\r\n", duty_cycle, ret);
	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}

OSStatus bk_pwm_update_param(bk_pwm_t pwm, uint32_t frequency, uint32_t duty_cycle1)
{
	UINT8 ret;
	pwm_param_st param;

	if (frequency < duty_cycle1)
	{
		bk_printf("bk_pwm_update_param error:freq:%x ,cycle1:%x\r\n", frequency, duty_cycle1);
		return kGeneralErr;
	}
	os_memset(&param, 0, sizeof(pwm_param_st));

	param.chan = pwm;
	param.t4 = frequency;

	param.t1 = duty_cycle1;
	param.init_level = 1;  // if duty cycle not 0, high level pwm pin first
	if(param.t1 == 0)
	{
		param.init_level = 0;
	}

	ret = pwm_update_param(&param);
	bk_printf("bk_pwm_update_param:duty_cycle1 = %d, ret:%d\r\n", duty_cycle1, ret);
	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}

OSStatus bk_pwm_cw_initialize(bk_pwm_t pwm1, bk_pwm_t pwm2, uint32_t frequency, uint32_t duty_cycle1, uint32_t duty_cycle2, uint32_t dead_band)
{
	UINT8 group;

	//bk_printf("[zjw] init pwm:%d-%d, f:%d, c1:%d, c2:%d, db:%d\r\n", pwm1, pwm2, frequency, duty_cycle1, duty_cycle2, dead_band);

	if (frequency < (duty_cycle1 + duty_cycle2 + 2 * dead_band))
	{
		bk_printf("pwm param set error:freq:%x ,cycle1:%x,cycle2:%x,dead_band:%x\r\n", frequency, duty_cycle1, duty_cycle2, dead_band);
		return kGeneralErr;
	}

	group = pwm_cw_group_check(pwm1, pwm2);
	pwm_cw_group_param_t param;
	UINT8 ret;

	os_memset(&param, 0, sizeof(pwm_cw_group_param_t));

	if(group < PWM_GROUP_NUM)
		param.group = group;
	else
		param.group = (pwm1 << 4) | pwm2;
	param.p_t4 = frequency;

	// pwm2
	param.p0_t1 = duty_cycle2;
	param.p0_init_level = 1;  // if duty cycle not 0, high level pwm pin first
	if(duty_cycle2 == 0)
	{
		param.p0_init_level = 0;
	}

	// pwm1
	if(duty_cycle1 == 0)
	{
		param.p1_t1 = 0;
		param.p1_t2 = 0;
		param.p1_init_level = 0;
	}
	else if(duty_cycle1 == frequency)
	{
		param.p1_t1 = 0;
		param.p1_t2 = 0;
		param.p1_init_level = 1;
	}
	else
	{
		param.p1_t1 = frequency - duty_cycle1 - dead_band;
		param.p1_t2 = frequency - dead_band;
		param.p1_init_level = 0;
	}

	if(group < PWM_GROUP_NUM)
		ret = pwm_cw_group_init_param(&param);
	else
		ret = pwm_cw_init_param(&param);
	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}

OSStatus bk_pwm_cw_start(bk_pwm_t pwm1, bk_pwm_t pwm2)
{
	UINT8 group, ret = 0;

	group = pwm_cw_group_check(pwm1, pwm2);
	if(group < PWM_GROUP_NUM)
	{
		ret = pwm_cw_group_start(group);
	}
	else
	{
		ret = pwm_cw_start(pwm1, pwm2);
	}

	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}

OSStatus bk_pwm_cw_stop(bk_pwm_t pwm1, bk_pwm_t pwm2)
{
	UINT8 group, ret = 0;

	group = pwm_cw_group_check(pwm1, pwm2);
	if(group < PWM_GROUP_NUM)
	{
		ret = pwm_cw_group_stop(group);
	}
	else
	{
		ret = pwm_cw_stop(pwm1, pwm2);
	}

	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}

OSStatus bk_pwm_cw_update_param(bk_pwm_t pwm1, bk_pwm_t pwm2, uint32_t frequency, uint32_t duty_cycle1, uint32_t duty_cycle2, uint32_t dead_band)
{
	UINT8 group;

	//bk_printf("[zjw] up pwm:%d-%d, f:%d, c1:%d, c2:%d, db:%d\r\n", pwm1, pwm2, frequency, duty_cycle1, duty_cycle2, dead_band);

	if (frequency < (duty_cycle1 + duty_cycle2 + 2 * dead_band))
	{
		bk_printf("pwm param set error:freq:%x ,cycle1:%x,cycle2:%x,dead_band:%x\r\n", frequency, duty_cycle1, duty_cycle2, dead_band);
		return kGeneralErr;
	}

	group = pwm_cw_group_check(pwm1, pwm2);

	pwm_cw_group_param_t param;
	UINT8 ret = 0;

	os_memset(&param, 0, sizeof(pwm_cw_group_param_t));

	if(group < PWM_GROUP_NUM)
		param.group = group;
	else
		param.group = (pwm1 << 4) | pwm2;

	param.p_t4 = frequency;

	// pwm2
	param.p0_t1 = duty_cycle2;
	param.p0_init_level = 1;  // if duty cycle not 0, high level pwm pin first
	if(duty_cycle2 == 0)
	{
		param.p0_init_level = 0;
	}

	// pwm1
	if(duty_cycle1 == 0)
	{
		param.p1_t1 = 0;
		param.p1_t2 = 0;
		param.p1_init_level = 0;
	}
	else if(duty_cycle1 == frequency)
	{
		param.p1_t1 = 0;
		param.p1_t2 = 0;
		param.p1_init_level = 1;
	}
	else
	{
		param.p1_t1 = frequency - duty_cycle1 - dead_band;
		param.p1_t2 = frequency - dead_band;
		param.p1_init_level = 0;
	}

	if(group < PWM_GROUP_NUM)
		ret = pwm_cw_group_update_param(&param);
	else
		ret = pwm_cw_update_param(&param);

	if(ret != 0)
		return kGeneralErr;
	else
		return kNoErr;
}
#endif
// eof
