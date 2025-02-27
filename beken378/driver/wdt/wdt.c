#include "include.h"
#include "arm_arch.h"

#include "wdt_pub.h"
#include "wdt.h"
#include "icu_pub.h"
#include "uart_pub.h"

#include "drv_model_pub.h"
#include "start_type_pub.h"

static SDD_OPERATIONS wdt_op = {
            wdt_ctrl
};
static uint32_t g_wdt_period = 0;

/*******************************************************************/
#if 1
extern void sctrl_dpll_delay200us(void);

void wdt_init(void)
{
	sddev_register_dev(WDT_DEV_NAME, &wdt_op);
}

void wdt_exit(void)
{
	sddev_unregister_dev(WDT_DEV_NAME);
}

UINT32 wdt_ctrl(UINT32 cmd, void *param)
{
	UINT32 ret;
	UINT32 reg;
	UINT32 parameter;

	ret = WDT_SUCCESS;	

	#if CFG_JTAG_ENABLE
	goto ctrl_exit;
	#endif
	
	switch(cmd)
	{		
		case WCMD_POWER_DOWN:
			g_wdt_period = 0;

			parameter = PWD_ARM_WATCHDOG_CLK_BIT;
		    ret = sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_DOWN, (void *)&parameter);
		    if(ret !=0 )
				os_printf("clk powerdown fail\r\n");
			break;

		case WCMD_POWER_UP:
			parameter = PWD_ARM_WATCHDOG_CLK_BIT;
		    ret = sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, (void *)&parameter);
		    if(ret !=0 )
				os_printf("clk powerup fail\r\n");
			break;

		case WCMD_RELOAD_PERIOD:
			reg = WDT_1ST_KEY << WDT_KEY_POSI;
			reg |= (g_wdt_period & WDT_PERIOD_MASK) << WDT_PERIOD_POSI;
			REG_WRITE(WDT_CTRL_REG, reg);

			sctrl_dpll_delay200us();

			reg = WDT_2ND_KEY << WDT_KEY_POSI;
			reg |= (g_wdt_period & WDT_PERIOD_MASK) << WDT_PERIOD_POSI;
			REG_WRITE(WDT_CTRL_REG, reg);

            bk_misc_update_set_type(RESET_SOURCE_WATCHDOG);
			break;

		case WCMD_SET_PERIOD:				
			ASSERT(param);
			g_wdt_period = (*(UINT32 *)param);
			
			reg = WDT_1ST_KEY << WDT_KEY_POSI;
			reg |= ((*(UINT32 *)param) & WDT_PERIOD_MASK) << WDT_PERIOD_POSI;
			REG_WRITE(WDT_CTRL_REG, reg);

			sctrl_dpll_delay200us();

			reg = WDT_2ND_KEY << WDT_KEY_POSI;
			reg |= ((*(UINT32 *)param) & WDT_PERIOD_MASK) << WDT_PERIOD_POSI;
			REG_WRITE(WDT_CTRL_REG, reg);
			break;
			
		default:
			break;
	}

#if CFG_JTAG_ENABLE
ctrl_exit:	
#endif
    return ret;
}
#endif

// EOF
