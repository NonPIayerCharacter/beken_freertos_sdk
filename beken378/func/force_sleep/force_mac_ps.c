#include "sys_config.h"
#if (1 == CFG_USE_FORCE_LOWVOL_PS)
#include "arch.h"
#include "arm_arch.h"

#include "reg_mac_pl.h"
#include "reg_mac_core.h"

#include "target_util_pub.h"

#include "force_ps_pub.h"

#include "sys_ctrl.h"
#include "icu.h"
#include "co_math.h"
#include "intc_pub.h"
#include "hal_machw.h"

#include "reg_rc.h"

#include "txl_frame.h"
#include "vif_mgmt.h"
#include "phy_trident.h"
#include "sys_ctrl_pub.h"

UINT32 force_ps_mac_hwtimer_unmask = 0;
uint8_t prev_mac_state = 0;
extern bool rwnxl_get_status_in_doze(void);
extern void phy_init_after_wakeup(void);
extern void rwnxl_reset_handle(int dummy);

static void force_mac_ps_recover_mac_modem(void);

typedef struct mac_sleep_st {
	MAC_S_CFG_ST cfg;
	uint32_t mac_bakup;

} MAC_S_ST, *MAC_S_PTR;

MAC_S_ST g_mac_sleep = {
	.cfg = {
		.off_mac = 1,
		.off_modem = 1,
		.wakeup_mcu = 0,
	},
	.mac_bakup = 0,
};

static void force_mac_ps_disable_rf_clk(void)
{
	UINT32 reg;

	if ((REG_READ(SCTRL_CONTROL)) & MODEM_CLK480M_PWD_BIT) {
		/*rf 480m clock disabled*/
		return;
	}

	/*must be Disable BK7011 first, then shutdown clock*/
	rc_cntl_stat_set(0x0);

	/* Modem AHB clock disable*/
	reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
	reg &= ~PHY_HCLK_EN_BIT;
	REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg);

	/* Modem Subsystem clock 480m disable*/
	reg = REG_READ(SCTRL_CONTROL);
	REG_WRITE(SCTRL_CONTROL, reg | MODEM_CLK480M_PWD_BIT);
}

static void force_mac_ps_disable_mac_clk(void)
{
	UINT32 reg;
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	/* MAC AHB slave clock disable */
	reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
	reg &= ~MAC_HCLK_EN_BIT;
	REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg);

	/* Mac Subsystem clock 480m disable*/
	reg = REG_READ(SCTRL_CONTROL);
	REG_WRITE(SCTRL_CONTROL, reg | MAC_CLK480M_PWD_BIT);

	GLOBAL_INT_RESTORE();
}

static void force_mac_ps_rf_mac_power_down(void)
{
	UINT32 reg = 0;

	if (g_mac_sleep.cfg.off_mac)
		reg |= (MAC_PWD << MAC_PWD_POSI);

	if (g_mac_sleep.cfg.off_modem)
		reg |= (MODEM_PWD << MODEM_PWD_POSI);

	if ((g_mac_sleep.cfg.off_mac) || (g_mac_sleep.cfg.off_modem))
		REG_WRITE(SCTRL_PWR_MAC_MODEM, reg);
}

static void force_mac_ps_set_wakeup(void)
{
}

static void force_mac_ps_disable_mac_icu(void)
{
	UINT32 reg;
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	/*disable MAC interrupt*/
	reg = REG_READ(ICU_INTERRUPT_ENABLE);
	reg &= ~(CO_BIT(FIQ_MAC_TX_RX_MISC)
			 | CO_BIT(FIQ_MAC_TX_RX_TIMER)
			 | CO_BIT(FIQ_MAC_RX_TRIGGER)
			 | CO_BIT(FIQ_MAC_TX_TRIGGER)
			 | CO_BIT(FIQ_MAC_GENERAL)
			 | CO_BIT(FIQ_MAC_PROT_TRIGGER));
	REG_WRITE(ICU_INTERRUPT_ENABLE, reg);
	GLOBAL_INT_RESTORE();
}

static void force_mac_ps_disable_mac_hwtimer(uint32_t *mac_timer_enbit)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
	uint32_t timer_biten;

	timer_biten = nxmac_timers_int_un_mask_get();

	if (mac_timer_enbit)
		*mac_timer_enbit = timer_biten;

	GLOBAL_INT_RESTORE();
}

static void force_mac_ps_set_mac_to_idle(uint32_t *mac_status)
{
	uint32_t mac_state;
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	mac_state = nxmac_current_state_getf();

	if (mac_status)
		*mac_status = mac_state;

	FS_PRT("[mac] cur_status: %d\r\n", mac_state);

	if (mac_state != HW_IDLE) {
		uint32_t i_tmp = 0, y_tmp = 0;

		// enable idle int bit, for check status in loop
		nxmac_enable_idle_interrupt_setf(1);

		nxmac_next_state_setf(HW_IDLE);
		while (1) {
			if (nxmac_status_idle_interrupt_getf() == 1)
				break;
			else {
				i_tmp++;
				if (i_tmp > 1000) {
					i_tmp = 0;
					y_tmp ++;
					if (y_tmp > 50) {
						FS_FATAL("[mac] goto idle timeout\r\n");

						#if CONFIG_RECOVER_FORCE_IDLE_ISSUE
						/*TODO: root reason need be explored.if not reset mac fsm 
						 * system will hang / dead while here;
						 */
						nxmac_hw_fsm_reset_setf(1);
						#endif
						y_tmp = 0;
					}
					nxmac_next_state_setf(HW_IDLE);
				}
			}
		}
		nxmac_gen_int_ack_clear(NXMAC_IDLE_INTERRUPT_BIT);
	}

	GLOBAL_INT_RESTORE();
}

int force_mac_ps_entry(uint32_t *mac_status, uint32_t *mac_timer_enbit)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	g_mac_sleep.cfg.off_mac = 0;
	g_mac_sleep.cfg.off_modem = 1;
	g_mac_sleep.cfg.wakeup_mcu = 0;
	g_mac_sleep.mac_bakup = 0;

	force_mac_ps_disable_mac_icu();

	force_mac_ps_disable_mac_hwtimer(mac_timer_enbit);
	FS_PRT("0 hwt:%x\r\n", hal_machw_time());

	if(!(rwnxl_get_status_in_doze()))
	{
		// go to IDLE
		force_mac_ps_set_mac_to_idle(mac_status);

		// Ask HW to go to DOZE
		nxmac_next_state_setf(HW_DOZE);
	}

	// sleep mac
	force_mac_ps_disable_mac_clk();

	// sleep rf
	force_mac_ps_disable_rf_clk();

	// power down
	force_mac_ps_rf_mac_power_down();

	// set wakeup signal
	force_mac_ps_set_wakeup();

	GLOBAL_INT_RESTORE();

	return 0;
}

static void force_mac_ps_enable_rf_clk(void)
{
	UINT32 reg;
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	/* Modem AHB clock enable*/
	reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
	REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg | PHY_HCLK_EN_BIT);

	/* Modem Subsystem clock 480m enable*/
	reg = REG_READ(SCTRL_CONTROL);
	reg &= ~MODEM_CLK480M_PWD_BIT;
	REG_WRITE(SCTRL_CONTROL, reg);

	/*Enable BK7011:rc_en,ch0_en*/
	rc_cntl_stat_set(0x09);

	GLOBAL_INT_RESTORE();
}

static void force_mac_ps_enable_mac_clk(void)
{
	UINT32 reg;
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	/* MAC AHB slave clock enable*/
	reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
	REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg | MAC_HCLK_EN_BIT);

	/* Mac Subsystem clock 480m enable*/
	reg = REG_READ(SCTRL_CONTROL);
	reg &= ~MAC_CLK480M_PWD_BIT;
	REG_WRITE(SCTRL_CONTROL, reg);

	GLOBAL_INT_RESTORE();
}

static void force_mac_ps_rf_mac_power_up(void)
{
	UINT32 reg = 0;

	if (g_mac_sleep.cfg.off_mac)
		reg |= (MAC_PWU << MAC_PWD_POSI);

	if (g_mac_sleep.cfg.off_modem)
		reg |= (MODEM_PWU << MODEM_PWD_POSI);

	if ((g_mac_sleep.cfg.off_mac) || (g_mac_sleep.cfg.off_modem))
		REG_WRITE(SCTRL_PWR_MAC_MODEM, reg);
}

static void force_mac_ps_enable_mac_icu(void)
{
	UINT32 reg;
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	reg = REG_READ(ICU_INTERRUPT_ENABLE);
	reg |= (CO_BIT(FIQ_MAC_TX_RX_MISC)
			| CO_BIT(FIQ_MAC_TX_RX_TIMER)
			| CO_BIT(FIQ_MAC_RX_TRIGGER)
			| CO_BIT(FIQ_MAC_TX_TRIGGER)
			| CO_BIT(FIQ_MAC_GENERAL)
			| CO_BIT(FIQ_MAC_PROT_TRIGGER));
	reg &= ~(CO_BIT(FIQ_MAC_WAKEUP));
	REG_WRITE(ICU_INTERRUPT_ENABLE, reg);

	GLOBAL_INT_RESTORE();
}

static void force_mac_ps_set_mac_from_doze(void *fun, void *agc)
{
	UINT32 i_tmp = 0, y_tmp = 0;
	typedef void (*wait_cb_fn)(void *fn_agc);
	wait_cb_fn call_back;

	if (g_mac_sleep.cfg.off_mac) {
		if (nxmac_current_state_getf() == HW_IDLE)
			return;
	}

	// enable idle int bit, for check status in loop
	nxmac_enable_idle_interrupt_setf(1);

	// Start the Wake-Up from doze procedure
	nxmac_wake_up_from_doze_setf(1);

	// Wait for idle interrupt
	while (1) {
		if (i_tmp == 0) {
			if (fun) {
				call_back = (wait_cb_fn)fun;
				call_back(agc);
			}
		}

		if (nxmac_status_idle_interrupt_getf() == 1)
			break;
		else {
			i_tmp++;
			if (i_tmp > 1000) {
				i_tmp = 0;
				y_tmp ++;
				if (y_tmp > 50) {
					FS_WARN("[mac] wakeup mac timeout\r\n");
					y_tmp = 0;
				}
			}
		}
	}

	nxmac_gen_int_ack_clear(NXMAC_IDLE_INTERRUPT_BIT);

	// Wake-Up from doze procedure is done
	nxmac_wake_up_from_doze_setf(0);
}

static void force_mac_ps_en_mac_hwtimer(uint32_t *mac_timer_enbit)
{
}

static void force_mac_ps_clear_wakeup(void)
{
}

static void force_mac_ps_recover_mac_modem(void)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	if (g_mac_sleep.cfg.off_mac) {
		//hal_machw_disable_int();
		rwnxl_reset_handle(0);
	}

	if (g_mac_sleep.cfg.off_modem) {
#if (CFG_SOC_NAME == SOC_BK7231N)
		sctrl_fix_dpll_div();
		phy_wakeup_rf_reinit();
		phy_wakeup_wifi_reinit();
#elif (CFG_SOC_NAME == SOC_BK7238)
		phy_wakeup_rf_reinit();
		phy_wakeup_wifi_reinit();
#else
		phy_init_after_wakeup();
#endif
	}

	GLOBAL_INT_RESTORE();
}

int force_mac_ps_exit(uint32_t *mac_status, uint32_t *mac_timer_enbit)
{
	uint32_t mac_state = HW_IDLE;

	// power up
	force_mac_ps_rf_mac_power_up();

	// enable rf
	force_mac_ps_enable_rf_clk();

	// enable mac clk
	force_mac_ps_enable_mac_clk();

	// clear wakeup signal
	force_mac_ps_clear_wakeup();

	// wakeup mac
	force_mac_ps_set_mac_from_doze(force_mac_ps_recover_mac_modem, NULL);

	// recover previous status
	if (mac_status)
		mac_state = *mac_status;

	if (HW_IDLE != mac_state) {
		nxmac_next_state_setf(mac_state);
		int count = 0;
		while (nxmac_current_state_getf() != mac_state) {
			count++;
			if (count > 50000) {
				count = 0;
				FS_WARN("[mac] recover mac status timeout\r\n");
			}
		}
	}

	FS_PRT("1 hwt:%x,%x\r\n", hal_machw_time(), REG_READ(0xC0008088));
	force_mac_ps_en_mac_hwtimer(mac_timer_enbit);

	force_mac_ps_enable_mac_icu();

	return 0;
}
#endif

