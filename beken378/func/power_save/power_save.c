#include "intc_pub.h"
#include "rtos_pub.h"

#include "wdt_pub.h"
#include "gpio_pub.h"
#include "pwm_pub.h"
#include "mem_pub.h"
#include "icu_pub.h"

#include "fake_clock_pub.h"
#include "power_save.h"
#include "target_util_pub.h"
#include "sys_ctrl_pub.h"
#include "drv_model_pub.h"
#include "arm_arch.h"
#include "rwnx_config.h"
#include "ps.h"
#include "rwnx.h"
#include "uart_pub.h"
#include "mcu_ps_pub.h"
#include "error.h"
#include "start_type_pub.h"
#include "rtos_pub.h"

#if CFG_SUPPORT_BLE
#include "ble_pub.h"
#endif
#include "reg_rc.h"
#include "low_voltage_ps.h"
#include "low_voltage_compensation.h"
#include "phy_trident.h"
#include "mcu_ps.h"
#include "calendar_pub.h"
#include "bk_timer.h"
#include "bk_timer_pub.h"


/* Forward Declaration */
static void power_save_ieee_dtim_wakeup(void);
static UINT8 power_save_me_ps_set_all_state(UINT8 state);
static void power_save_sleep_status_set();

volatile static PS_MODE_STATUS bk_ps_mode = PS_NO_PS_MODE;
static UINT32 last_wk_tick = 0;
UINT32 last_rw_time = 0;

static STA_PS_INFO bk_ps_info = {
	.ps_dtim_period = 1,
	.ps_dtim_multi = 1,
	.listen_int = PS_DTIM_COUNT,
	.waited_beacon = STA_GET_INIT,
	.sleep_first = 1,
	.ps_can_sleep = 0,
	.ps_real_sleep = 0
};

/**
 * @r_wakeup_time  Radio Wakeup Time, MAC uses this value to switch on the
 *                 BB earlier then data is expected on air when coming out
 *                 of Power Save (unit: 32us).
 */
#if (CFG_SOC_NAME == SOC_BK7231)
static UINT16 r_wakeup_time = 50;
#elif (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
static UINT16 r_wakeup_time = 90;
#else
static UINT16 r_wakeup_time = 66;
#endif

static UINT32 int_enable_reg_save = 0;
static UINT8 ps_lock = 1;
static PS_FORBID_STATUS bk_forbid_code = 0;
static UINT16 bk_forbid_count = 0;
static UINT32 ps_dis_flag = 0;
static UINT16 beacon_len = 0;

#if CFG_LOW_LATENCY_PS
static UINT8 ps_data_low_latency = 0;
#endif

#if PS_USE_KEEP_TIMER
static beken2_timer_t ps_keep_timer = {0};
static UINT32 ps_keep_timer_status = 0;
static UINT32 ps_wait_timer_status = 0;
static UINT32 ps_keep_timer_period = 0;
static UINT32 ps_reseted_moniter_flag = 0;
static UINT32 ps_bcn_loss_max_count = 0;
static UINT32 ps_keep_timer_flag = 1;
#endif

#if PS_USE_WAIT_TIMER
static beken2_timer_t ps_wait_timer = {0};
#endif

#if (NX_HW_PARSER_TIM_ELEMENT)
uint32_t ps_hw_tim_cnt = 0;
uint32_t ps_hw_tim_cnt_limit = 10;
#define PS_HW_TIM_ALLOWED (ps_hw_tim_cnt < ps_hw_tim_cnt_limit)
#endif

#if PS_USE_KEEP_TIMER
void power_save_keep_timer_handler ( void *data );
#endif
extern void bmsg_null_sender ( void );

int net_if_is_up(void)
{
	return mhdr_get_station_status() == RW_EVT_STA_GOT_IP;
}

void power_save_wakeup_isr ( void )
{
}

void power_save_dtim_wake(UINT32 status)
{
	if (bk_ps_mode == PS_DTIM_PS_MODE &&
		bk_ps_info.ps_arm_wakeup_way == PS_ARM_WAKEUP_NONE) {
		UINT32 reg;

		if (status) {
			if (status & MAC_ARM_WAKEUP_EN_BIT) {
				/* FIQ mac wakeup interrupt disable */
				reg = REG_READ(ICU_INTERRUPT_ENABLE);
				reg &= ~(CO_BIT(FIQ_MAC_WAKEUP));
				REG_WRITE(ICU_INTERRUPT_ENABLE, reg);
				PS_DEBUG_UP_TRIGER;
#if 1
				bk_ps_info.ps_arm_wakeup_way = PS_ARM_WAKEUP_RW;
				power_save_ieee_dtim_wakeup();
#else
				power_save_ieee_dtim_wakeup();
#endif
			}
		}
	}
}


/*
 * This function will run in mac go to ps fiq,
 * only an actual emergency can put here,
 * can't operate wifi tx,rx,modem,rf here
 */
void power_save_gops_wait_idle_int_cb(void)
{
    // rf_ps_wakeup_isr_idle_int_cb();
}


#if CFG_SUPPORT_BLE
extern uint8_t ble_switch_mac_sleeped;
#endif
#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
extern void lv_ps_info_rf_sleep(bool pre_flag);
extern void lv_ps_info_rf_wakeup(bool restart_flag);
#endif
bool power_save_sleep(void)
{
	UINT32 ret = false;
	UINT32 reg;

	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	/* If already slept, return */
	if (1 == bk_ps_info.ps_real_sleep
#if CFG_SUPPORT_BLE
		|| ble_switch_mac_sleeped
#endif
		) {
		GLOBAL_INT_RESTORE();
		return ret;
	}

	/* If cannot enter ps, return */
	if (!(PS_STA_DTIM_CAN_SLEEP)) {
		GLOBAL_INT_RESTORE();
		return ret;
	}

	/* machw already in doze state */
	if (rwnxl_get_status_in_doze()) {
		GLOBAL_INT_RESTORE();
		return ret;
	}

	/*
	 * enable mac IDLE interrupt, this interrupt will be set whenever the
	 * stateCntrlReg.currentState transitions to IDLE state
	 */
	nxmac_enable_idle_interrupt_setf(1);
	PS_DEBUG_CK_TRIGER;

	/* if there are other mac interrupts pending for process */
	if (REG_READ((ICU_BASE + 19 * 4)) &
		 (CO_BIT(FIQ_MAC_TX_RX_MISC) |
		   CO_BIT(FIQ_MAC_TX_RX_TIMER) |
		   CO_BIT(FIQ_MAC_RX_TRIGGER) |
		   CO_BIT(FIQ_MAC_TX_TRIGGER) |
		   CO_BIT(FIQ_MAC_PROT_TRIGGER))) {
		GLOBAL_INT_RESTORE();
		return ret;
	}

	/* disable mac interrupts, mac wakeup interrupt (FIQ_MAC_WAKEUP) will be enabled later */
	reg = REG_READ(ICU_INTERRUPT_ENABLE);

	/* save current ICU interrupt register value which maybe restore if cannot enter ps */
	int_enable_reg_save = reg;
	reg &= ~(CO_BIT(FIQ_MAC_TX_RX_MISC) |
			 CO_BIT(FIQ_MAC_TX_RX_TIMER) |
			 CO_BIT(FIQ_MAC_RX_TRIGGER) |
			 CO_BIT(FIQ_MAC_TX_TRIGGER) |
			 CO_BIT(FIQ_MAC_GENERAL) |
			 CO_BIT(FIQ_MAC_PROT_TRIGGER));
	REG_WRITE(ICU_INTERRUPT_ENABLE, reg);


#if (1 == CFG_LOW_VOLTAGE_PS)
	/* disable radio controller */
	lv_ps_rf_pre_pwr_down = 1;
	rc_cntl_stat_set(0x00); //7011
#if (1 == CFG_LOW_VOLTAGE_PS_TEST)
	lv_ps_info_rf_sleep(1);
#endif
	// REG_WRITE((0x00802800+(20*4)), 0x00);//gpio19
#endif

#if NX_POWERSAVE
	last_rw_time = nxmac_monotonic_counter_2_lo_get();

	if (last_rw_time == 0xdead5555)
		bk_printf("XXXXXXXXXXXXXXXXXXXXXXXX TIME DEAD\r\n");

	ret = rwnxl_sleep(power_save_gops_wait_idle_int_cb, power_save_mac_idle_callback);

	if (false == ret) {
		PS_PRT("can't ps\r\n");

#if (1 == CFG_LOW_VOLTAGE_PS)
		rc_cntl_stat_set(0x09); //7011
		lv_ps_rf_pre_pwr_down = 0;
#if (1 == CFG_LOW_VOLTAGE_PS_TEST)
		lv_ps_info_rf_wakeup(1);
#endif
#endif

		/* restore previous saved ICU interrupt enable register value */
		REG_WRITE(ICU_INTERRUPT_ENABLE, int_enable_reg_save);
		GLOBAL_INT_RESTORE();
		return ret;
	}
#endif

	if (ps_lock) {
		ps_lock--;
	} else {
		PS_WPRT("error ps\r\n");
		GLOBAL_INT_RESTORE();
		return ret;
	}

#if(CFG_LV_PS_WITH_IDLE_TICK == 1)
	lv_ps_set_keep_timer_more(0);
#endif

	PS_WPRT("go ps\r\n");
#if CFG_USE_STA_PS
	/* mac is now slept, set real_sleep and wakeup_way */
	power_save_sleep_status_set();

	/* stop rf */
	sctrl_sta_rf_sleep();

	/* enable mac wakeup interrupt */
	reg = REG_READ ( ICU_INTERRUPT_ENABLE );
	reg |= ( CO_BIT ( FIQ_MAC_WAKEUP ) );
	REG_WRITE ( ICU_INTERRUPT_ENABLE, reg );
#endif
#if PS_USE_KEEP_TIMER
	/* disable ps keep timer */
	if (1 == ps_keep_timer_status) {
		rtos_lock_scheduling();
		bmsg_ps_sender(PS_BMSG_IOCTL_RF_KP_STOP);
		rtos_unlock_scheduling();
	}
#endif

	GLOBAL_INT_RESTORE();
	return true;
}

/*time = BI*1024*LIST*0.016*/
void power_save_wkup_time_cal ( UINT8 sleep_int )
{
	UINT32 tmp_r_wkup = r_wakeup_time + 12;
	nxmac_radio_wake_up_time_setf ( tmp_r_wkup );
}

int power_save_get_wkup_less_time()
{
	if (bk_ps_info.listen_mode == PS_LISTEN_MODE_DTIM)
		return bk_ps_info.ps_dtim_period * bk_ps_info.ps_dtim_multi
			   * bk_ps_info.ps_beacon_int * 15;

	return bk_ps_info.listen_int * bk_ps_info.ps_beacon_int * 15;
}

/**
 * power_save_mac_idle_callback
 *
 * called after MAC is set to IDLE state.
 */
void power_save_mac_idle_callback(void)
{
	uint32_t listen_interval = PS_DTIM_COUNT;

	listen_interval = power_save_get_listen_int();

	if (power_save_if_sleep_first()) {
		/* set radio wakeup time in advanced to BB wakeup */
		power_save_wkup_time_cal(listen_interval);

		/* disable HW TSF mgmt ??? */
		nxmac_tsf_mgt_disable_setf(0);

		/* set listen interval, MAC wakes up every listen_interval beacon intervals */
		nxmac_listen_interval_setf(listen_interval);

		/* set ATIM window??? only used in IBSS */
		nxmac_atim_w_setf(512);

		/* clear wakeUpSW bit */
		nxmac_wake_up_sw_setf(0);

		/* first clear beacon interval, delay, then set beacon interval, to fix rw sleep wakeup time */
		nxmac_beacon_int_setf(0);
		delay(1);
		nxmac_beacon_int_setf(bk_ps_info.ps_beacon_int);

		os_printf(" sleep_first %d\r\n", bk_ps_info.listen_mode);
		os_printf(" dtim period:%d multi:%d\r\n", bk_ps_info.ps_dtim_period, bk_ps_info.ps_dtim_multi);
	} else {
		if (bk_ps_info.listen_mode == PS_LISTEN_MODE_DTIM) {
			/* set radio wakeup time in advanced to BB wakeup */
			power_save_wkup_time_cal(listen_interval);

			/* set listen interval, MAC wakes up every listen_interval beacon intervals */
			nxmac_listen_interval_setf(listen_interval);
		}
	}

	bk_ps_info.sleep_count++;
}

UINT32 power_save_get_rf_ps_dtim_time(void)
{
	UINT32 tm;

	tm = bk_ps_info.ps_dtim_period * bk_ps_info.ps_dtim_multi * bk_ps_info.ps_beacon_int;
	return tm;
}

static void power_save_sleep_status_set(void)
{
	bk_ps_info.ps_real_sleep = 1;
	bk_ps_info.ps_arm_wakeup_way = PS_ARM_WAKEUP_NONE;
}

/**
 * Iterate all active STA vifs, and set each vif's prevent_sleep bit.
 */
static UINT8 power_save_set_all_vif_prevent_sleep(UINT32 prevent_bit)
{
	VIF_INF_PTR vif_entry = NULL;
	UINT32 i;

	for (i = 0; i < NX_VIRT_DEV_MAX; i++) {
		vif_entry = &vif_info_tab[i];

		if (vif_entry->active && vif_entry->type == VIF_STA) {
			vif_entry->prevent_sleep |= prevent_bit;
			#if (NX_HW_PARSER_TIM_ELEMENT)
			if (PS_HW_TIM_ALLOWED)
			{
				if(((mcu_ps_is_on())&&(lvc_calc_g_bundle_ready()))
					||((!mcu_ps_is_on())&&(lv_ps_wake_up_way != PS_DEEP_WAKEUP_GPIO)))
				{
					LV_PSC_PRT("%d ", power_save_get_hw_tim_cnt());
					nxmac_ack_tim_set_clearf(1);
					nxmac_gen_int_enable_set(nxmac_gen_int_enable_get() | NXMAC_TIM_SET_BIT);
					lvc_apply_clock_drift_tim();
				}
				else if((!mcu_ps_is_on()) && (lv_ps_wake_up_way == PS_DEEP_WAKEUP_GPIO))
				{
					lv_ps_force_software_beacon();
					lv_ps_wake_up_way = PS_DEEP_WAKEUP_NULL;
				}
					
			}
			#endif
		}
	}

	return 0;
}

/**
 * Iterate all active STA vifs, and clear each vif's prevent_sleep bit.
 */
static UINT8 power_save_clr_all_vif_prevent_sleep(UINT32 prevent_bit)
{
	VIF_INF_PTR vif_entry = NULL;
	UINT32 i;

	for (i = 0; i < NX_VIRT_DEV_MAX; i++) {
		vif_entry = &vif_info_tab[i];

		if (vif_entry->active && vif_entry->type == VIF_STA)
			vif_entry->prevent_sleep &= ~(prevent_bit);
	}

	return 0;
}

/*
 * This function will run in mac wakeup fiq,
 * only an actual emergency can put here,
 * can't operate wifi tx,rx,modem,rf here
 */
static void power_save_wkup_wait_idle_int_cb ( void )
{

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    #if (1 == CFG_LOW_VOLTAGE_PS)
        if(lv_ps_rf_reinit)
            return;
        #if (CFG_SOC_NAME == SOC_BK7231N)
        sctrl_fix_dpll_div();
        #endif
        phy_wakeup_rf_reinit();
        phy_wakeup_wifi_reinit();
        #if (1 == CFG_LOW_VOLTAGE_PS_TEST)
        extern void lv_ps_info_rf_ready(void);
        lv_ps_info_rf_ready();
        #endif
        lv_ps_rf_reinit = 1;
    #endif
#endif
}

void power_save_ble_lv_cb(void)
{
    power_save_wkup_wait_idle_int_cb();
}

#if CFG_SUPPORT_BLE
extern void ps_recover_ble_switch_mac_status(void);
#endif
void power_save_wakeup(void)
{
	UINT32 reg;
	PS_DEBUG_UP_TRIGER;

	/*
	 * reset waited_beacon to FALSE after wakeup, and if recevied a beacon, waited_beacon
	 * may set to TRUE if wait timer is not enabled.
	 */
	bk_ps_info.waited_beacon = STA_GET_FALSE;

#if (1 == CFG_LOW_VOLTAGE_PS)
	if (1 == lv_ps_rf_pre_pwr_down)
	{
		rc_cntl_stat_set(0x09); //7011
		lv_ps_rf_pre_pwr_down = 0;
	}
#endif

#if CFG_USE_STA_PS
	/* wakeup rf first */
	sctrl_sta_rf_wakeup();
#if CFG_SUPPORT_BLE
	rf_wifi_used_set();
#endif

	/* clear arm wakeup bit */
	reg = REG_READ(ICU_ARM_WAKEUP_EN);
	reg &= ~(MAC_ARM_WAKEUP_EN_BIT);
	REG_WRITE(ICU_ARM_WAKEUP_EN, reg);
#endif

#if NX_POWERSAVE
	/* wakeup mac: move mac from doze state, restore previous hw state */
	rwnxl_wakeup(power_save_wkup_wait_idle_int_cb);
#endif

	/* If waken from interrupt, waiting for beacon */
	if (bk_ps_info.ps_arm_wakeup_way == PS_ARM_WAKEUP_RW)
		power_save_set_all_vif_prevent_sleep((UINT32)(PS_VIF_WAITING_BCN));

	/* clear sleep first flag */
	bk_ps_info.sleep_first = 0;

	/* re-enable MAC interrupts, and clear FIQ mac wakeup interrupt */
	reg = REG_READ(ICU_INTERRUPT_ENABLE);
	reg |= CO_BIT(FIQ_MAC_TX_RX_MISC) |
		   CO_BIT(FIQ_MAC_TX_RX_TIMER) |
		   CO_BIT(FIQ_MAC_RX_TRIGGER) |
		   CO_BIT(FIQ_MAC_TX_TRIGGER) |
		   CO_BIT(FIQ_MAC_GENERAL) |
		   CO_BIT(FIQ_MAC_PROT_TRIGGER);
	reg &= ~(CO_BIT(FIQ_MAC_WAKEUP));
	REG_WRITE(ICU_INTERRUPT_ENABLE, reg);

	PS_DEBUG_UP_TRIGER;
	ASSERT(!ps_lock);
	ps_lock++;
}


void power_save_dtim_exit_check()
{
	if (power_save_wkup_event_get() & NEED_DISABLE_BIT) {
		power_save_dtim_rf_ps_disable_send_msg();
		power_save_wkup_event_clear(NEED_DISABLE_BIT);
	}
}

static void power_save_ieee_dtim_wakeup(void)
{
	if ((bk_ps_info.ps_arm_wakeup_way >  PS_ARM_WAKEUP_NONE &&
		bk_ps_info.ps_arm_wakeup_way <= PS_ARM_WAKEUP_USER) &&
		bk_ps_info.ps_real_sleep) {

		PS_DEBUG_UP_TRIGER;
		power_save_wakeup();

#if (1==CFG_LV_PS_WITH_IDLE_TICK)
		lv_ps_wakeup_set_timepoint();
#endif

		if (!bk_ps_info.ps_real_sleep)
			os_printf("ps r s not 0\r\n");

		bk_ps_info.ps_real_sleep = 0;
		bk_ps_info.ps_can_sleep = 1;

#if CFG_USE_MCU_PS && CFG_USE_TICK_CAL && (0 == CFG_LOW_VOLTAGE_PS)
		// When enabled MCU PS, compensate tick check
		mcu_ps_machw_cal();
#endif
		last_wk_tick = fclk_get_tick();
#if PS_USE_KEEP_TIMER

#if (1 == CFG_LOW_VOLTAGE_PS)
		// lv_ps_wakeup_wifi = 1;

		if (( !power_save_if_sleep_first() ) && (mcu_ps_is_on())) {
			bmsg_ps_sender ( PS_BMSG_IOCTL_RF_PS_TIMER_INIT );
		}
#else
		if ( !power_save_if_sleep_first() && ps_keep_timer_period ) {
			ps_keep_timer_flag = 1;
			bmsg_ps_sender(PS_BMSG_IOCTL_RF_KP_SET);
			PS_DEBUG_PWM_TRIGER;
		} else {
			//os_printf("errr %d %d\r\n", power_save_if_sleep_first(), ps_keep_timer_period);
		}
#endif /* PS_USE_KEEP_TIMER */

#endif
		/* If RF is assigned to WiFi, wakeup task that waits for mac wakeup */
#if CFG_SUPPORT_BLE
		if (!ble_switch_mac_sleeped)
#endif
			power_save_rf_ps_wkup_semlist_set();

		/* ??? explicit trigger timers */
		ke_evt_set(KE_EVT_KE_TIMER_BIT);
		ke_evt_set(KE_EVT_MM_TIMER_BIT);
		power_save_dtim_exit_check();
	}

#if CFG_SUPPORT_BLE
	if (!power_save_if_rf_sleep())
		ps_recover_ble_switch_mac_status();
#endif
}
/**
 * check whether mac/rf can sleep, and put them into powersave mode if possible.
 */
bool power_save_rf_sleep_check ( void )
{
#if (NX_POWERSAVE)
#if CFG_USE_STA_PS
#if PS_WAKEUP_MOTHOD_RW

	if (PS_STA_DTIM_CAN_SLEEP) {
		GLOBAL_INT_DECLARATION();

		/* If there are ke events need to be processed */
		if (ke_evt_get() != 0)
			return false;

		/* bus messages need to be processed */
		if (!bmsg_is_empty())
			return false;

		GLOBAL_INT_DISABLE();
		/* check whether mac/rf can sleep, and enter ps mode if possible */
		ps_sleep_check();
		GLOBAL_INT_RESTORE();
	}

#endif
#endif
#endif //(NX_POWERSAVE)
	return 0;
}

void power_save_me_ps_first_set_state ( UINT8 state )
{
	VIF_INF_PTR vif_entry;
	struct me_set_ps_disable_req *req;

	os_printf("%s %d\n", __func__, __LINE__);

	for_each_vif_entry(vif_entry) {
		if (vif_entry->type == VIF_STA && vif_entry->active) {

			req = KE_MSG_ALLOC(ME_PS_REQ, TASK_ME, TASK_NONE,
						me_set_ps_disable_req);

			if (!req)
				break;

			req->ps_disable = state;
			req->vif_idx = vif_entry->index;
			ke_msg_send(req);
		}
	}
}

static void power_save_sm_set_bcmc(UINT8 bcmc, UINT8 vif_idx)
{
	struct mm_set_ps_options_req *req;

	// Get a pointer to the kernel message
	req = KE_MSG_ALLOC(MM_SET_PS_OPTIONS_REQ, TASK_MM, TASK_NONE, mm_set_ps_options_req);

	if (req) {
		// Fill the message parameters
		req->dont_listen_bc_mc = bcmc;
		req->listen_interval = 0;
		req->vif_index = vif_idx;
		os_printf("%s %d %d %d\r\n", __FUNCTION__, req->dont_listen_bc_mc,
				  req->listen_interval, req->vif_index);
		// Set the PS options for this VIF
		ke_msg_send(req);
	}
}

UINT8 power_save_sm_set_all_bcmc(UINT8 bcmc)
{
	VIF_INF_PTR vif_entry = NULL;
	UINT32 i;

	for (i = 0; i < NX_VIRT_DEV_MAX; i++) {
		vif_entry = &vif_info_tab[i];

		if (vif_entry->active && vif_entry->type != VIF_STA) {
			os_printf("%s:%d %d is %d not STA!!!!\r\n", __FUNCTION__, __LINE__, i, vif_entry->type);
			return 0;
		}
	}

	for (i = 0; i < NX_VIRT_DEV_MAX; i++) {
		vif_entry = &vif_info_tab[i];

		if (vif_entry->active && vif_entry->type == VIF_STA)
			power_save_sm_set_bcmc(bcmc, i);
	}

	return 0;
}

static void power_save_me_ps_set_state(UINT8 state, UINT8 vif_idx)
{
	struct me_set_ps_disable_req *me_ps_ptr =
		KE_MSG_ALLOC(ME_SET_PS_DISABLE_REQ, TASK_ME, TASK_NONE,
			me_set_ps_disable_req);

	os_printf("%s:%d \r\n", __func__, __LINE__ );

	if (me_ps_ptr) {
		me_ps_ptr->ps_disable = state;
		me_ps_ptr->vif_idx = vif_idx;
		ke_msg_send(me_ps_ptr);
	}
}

/**
 * send ME_SET_PS_DISABLE_REQ ke_msg for each STA vif that is connected with AP
 *
 * @state      true if ps disable, false enable ps
 */
static UINT8 power_save_me_ps_set_all_state(UINT8 state)
{
	VIF_INF_PTR vif_entry = NULL;
	UINT32 i;

	if (state == false) {
		for (i = 0; i < NX_VIRT_DEV_MAX; i++) {
			vif_entry = &vif_info_tab[i];

			if (vif_entry->active && vif_entry->type != VIF_STA) {
				os_printf("%s:%d %d is %d not STA!!!!\r\n", __FUNCTION__, __LINE__, i, vif_entry->type);
				return 0;
			}
		}
	}

	for (i = 0; i < NX_VIRT_DEV_MAX; i++) {
		vif_entry = &vif_info_tab[i];

		if (vif_entry->active && vif_entry->type == VIF_STA)
			power_save_me_ps_set_state(state, i);
	}

	return 0;
}

void power_save_dtim_ps_init(void)
{
	bk_ps_info.sleep_count = 0;
	bk_ps_info.sleep_first = 1;
	os_printf("power_save_dtim_ps_init\n");
	bk_ps_info.ps_can_sleep = 1;
}

void power_save_dtim_ps_exit(void)
{
#if PS_USE_KEEP_TIMER
	power_save_keep_timer_stop();
#endif
#if PS_USE_WAIT_TIMER
	power_save_wait_timer_stop();
#endif

	/*
	 * set beacon interval to zero, may not needed.
	 * MACHW will reset beacon interval to zero if it
	 * moves out of DOZE state. And beacon interval is
	 * only used in DOZE state.
	 */
	nxmac_beacon_int_setf(0);
	delay(1);
	bk_ps_info.sleep_count = 0;
	bk_ps_info.ps_dtim_period = 1;
	bk_ps_info.ps_dtim_multi = 1;
	bk_ps_info.waited_beacon = STA_GET_INIT;
	bk_ps_info.sleep_first = 1;
	bk_ps_info.ps_can_sleep = 0;
	bk_ps_info.ps_real_sleep = 0;
}

/**
 * called when confirmation of MM_SET_PS_MODE_CFM is received. 802.11 power save mode
 * is enabled, Pwr Mgmt bit is set, and ps_env.ps_on = true.
 */
int power_save_dtim_enable_handler(void)
{
	UINT32 ps_time, multi;
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	if ((mhdr_get_station_status() >=  RW_EVT_STA_CONNECTED)) {
		ps_time = power_save_get_rf_ps_dtim_time();

		if (ps_time > 0 && ps_time < 75) {
			multi = 75 / ps_time + 1;
			power_save_set_dtim_multi(multi);
		} else {
			power_save_set_dtim_multi(1);
		}

		os_printf("enter %d ps,p:%d m:%d int:%d l:%d!\r\n", bk_ps_info.listen_mode,
				  bk_ps_info.ps_dtim_period, bk_ps_info.ps_dtim_multi,
				  bk_ps_info.ps_beacon_int, bk_ps_info.listen_int);

		/* 802.11 power save mode is enabled, now MAC can enter power save mode */
		power_save_dtim_ps_init();

		/* set to DTIM_PS_MODE */
		bk_ps_mode = PS_DTIM_PS_MODE;

#if PS_USE_WAIT_TIMER
		/*
		 * wait 20ms to enter ps to let mac has the time to process deauth/deassoc frames
		 * that sent from AP if we send [qos]null frames to AP.
		 */
		power_save_wait_timer_init();
#endif
	} else {
		os_printf("%s:%d %d %d--\r\n", __FUNCTION__, __LINE__, bk_ps_mode, mhdr_get_station_status());
	}

	GLOBAL_INT_RESTORE();
	return 0;
}

/**
 * called when confirmation of MM_SET_PS_MODE_CFM is received. 802.11 power save mode
 * is disabled, Pwr Mgmt bit is cleared, and ps_env.ps_on = false.
 */
int power_save_dtim_disable_handler(void)
{
	UINT32 wdt_val = 1;

	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
	bk_ps_mode = PS_NO_PS_MODE;

	if (bk_ps_info.ps_real_sleep == 1)
		os_printf("%s:%d err----\r\n", __FUNCTION__, __LINE__);

	/* restore timers masks */
	rwnxl_set_nxmac_timer_value();

	/* exit from PS */
	power_save_dtim_ps_exit();

#if CFG_SUPPORT_BLE
	rf_wifi_used_clr();
#endif

	if (power_save_wkup_event_get() & NEED_REBOOT_BIT) {
		sddev_control(WDT_DEV_NAME, WCMD_POWER_DOWN, NULL);
		os_printf("pswdt reboot\r\n");
		bk_misc_update_set_type(RESET_SOURCE_REBOOT);
		sddev_control(WDT_DEV_NAME, WCMD_SET_PERIOD, &wdt_val);
		sddev_control(WDT_DEV_NAME, WCMD_POWER_UP, NULL);

		while (1);
	}

	GLOBAL_INT_RESTORE();
	os_printf("exit dtim ps!\r\n" );
#if CFG_SUPPORT_BLE
	ps_recover_ble_switch_mac_status();
#endif
	return 0;
}


int power_save_dtim_enable(void)
{
	if (!net_if_is_up()) {
		os_printf("net %d not ip up\r\n", mhdr_get_station_status());
		return -1;
	}

	if (wpa_psk_cal_pending()) {
		os_printf("can't dtim, wpa_psk_cal is pending!\r\n");
		return -1;
	}

	if (g_wlan_general_param->role != CONFIG_ROLE_STA) {
		os_printf("can't dtim,role %d not only sta!\r\n", g_wlan_general_param->role);
		return -1;
	}

	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	if (bk_ps_mode != PS_NO_PS_MODE) {
		os_printf("can't dtim ps,ps in mode %d!\r\n", bk_ps_mode);
		GLOBAL_INT_RESTORE();
		return -1;
	}

	os_printf("first enable sleep \r\n");
	power_save_me_ps_first_set_state(PS_MODE_ON_DYN);

	GLOBAL_INT_RESTORE();
	return 0;
}

int power_save_dtim_disable(void)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	if (bk_ps_mode == PS_DTIM_PS_MODE) {
		GLOBAL_INT_RESTORE();
		power_save_me_ps_set_all_state(true);
		os_printf("start exit!\r\n");
		return 0;
	}
	GLOBAL_INT_RESTORE();

	return 0;
}


int power_save_dtim_rf_ps_disable_send_msg(void)
{
	if (bk_ps_mode == PS_DTIM_PS_MODE)
		bmsg_ps_sender(PS_BMSG_IOCTL_RF_DISABLE);

	return 0;
}

/**
 * manual wakeup mac
 */
void power_save_rf_dtim_manual_do_wakeup(void)
{
	UINT32 reg;

#if CFG_USE_AP_IDLE
	if (bk_wlan_has_role(VIF_AP) && ap_ps_enable_get()) {
		GLOBAL_INT_DECLARATION();
		GLOBAL_INT_DISABLE();
		power_save_rf_hold_bit_set(RF_HOLD_BY_AP_BIT);
		wifi_general_mac_state_set_active();
		GLOBAL_INT_RESTORE();
	}
#endif
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

#if CFG_SUPPORT_BLE
	if (ble_switch_mac_sleeped) {
		GLOBAL_INT_RESTORE();
		return;
	}
#endif

	rtos_lock_scheduling();
	PS_DEBUG_UP_TRIGER;

	if ((bk_ps_mode == PS_DTIM_PS_MODE) &&
		(bk_ps_info.ps_arm_wakeup_way == PS_ARM_WAKEUP_NONE ||
		 bk_ps_info.ps_arm_wakeup_way == PS_ARM_WAKEUP_UPING) &&
		(bk_ps_info.ps_real_sleep == 1)) {
		delay(1);
		PS_DEBUG_UP_TRIGER;

		if (bk_ps_info.ps_arm_wakeup_way == PS_ARM_WAKEUP_UPING)
			bk_ps_info.ps_arm_wakeup_way = PS_ARM_WAKEUP_RW;
		else
			bk_ps_info.ps_arm_wakeup_way = PS_ARM_WAKEUP_USER;

		reg = REG_READ(ICU_INTERRUPT_ENABLE);
		reg &= ~(CO_BIT(FIQ_MAC_WAKEUP));
		REG_WRITE(ICU_INTERRUPT_ENABLE, reg);
		power_save_ieee_dtim_wakeup();
		PS_PRT("m_r_u\r\n");
	}

	rtos_unlock_scheduling();
	GLOBAL_INT_RESTORE();
}

void power_save_set_dtim_period ( UINT8 period )
{
	if (bk_ps_info.ps_dtim_period != period)
		os_printf("new dtim period:%d\r\n", period);

	bk_ps_info.ps_dtim_period = period;
}

void power_save_set_dtim_count(UINT8 count)
{
	bk_ps_info.ps_dtim_count = count;
}

void power_save_cal_bcn_listen_int(UINT16 bcn_int)
{
	if (bcn_int != 0) {
		bk_ps_info.ps_beacon_int = bcn_int;
		//bk_ps_info.listen_int = PS_DTIM_COUNT;
#if (1 == CFG_LOW_VOLTAGE_PS)
		lv_ps_set_bcn_int(bcn_int << 10);
#endif
	}
}

void power_save_set_listen_int(UINT16 listen_int)
{
	if ((listen_int > 100) || (listen_int == 0))
		bk_ps_info.listen_int = PS_DTIM_COUNT;
	else
		bk_ps_info.listen_int = listen_int;

	os_printf("set listen intval:%d\r\n", bk_ps_info.listen_int, listen_int);
}

UINT8 power_save_get_listen_int(void)
{
	return bk_ps_info.listen_int;
}

void power_save_delay_sleep_check(void)
{
	bmsg_ps_sender(PS_BMSG_IOCTL_RF_TD_SET);
}

#if PS_USE_WAIT_TIMER
void power_save_wait_timer_stop(void)
{
	OSStatus err;

	if (rtos_is_oneshot_timer_running(&ps_wait_timer)) {
		err = rtos_stop_oneshot_timer(&ps_wait_timer);
		ASSERT(kNoErr == err);
	}
	ps_wait_timer_status = 0;
}

void power_save_wait_timer_real_handler(void)
{
	power_save_wait_timer_stop();
#if (0 == CFG_LOW_VOLTAGE_PS)
	if (PS_STA_DTIM_SWITCH)
		power_save_beacon_state_set(STA_GET_TRUE);
#else
	power_save_clr_all_vif_prevent_sleep((UINT32)(PS_VIF_WAITING_BCMC));
#endif
}

void power_save_wait_timer_handler(void *data)
{
	bmsg_ps_sender(PS_BMSG_IOCTL_WAIT_TM_HANDLER);
}

void power_save_wait_timer_init(void)
{
	UINT32 err;

	if (rtos_is_oneshot_timer_init(&ps_wait_timer)) {
		power_save_wait_timer_stop();
		err = rtos_deinit_oneshot_timer(&ps_wait_timer);
		ASSERT(kNoErr == err);
	}

	err = rtos_init_oneshot_timer(&ps_wait_timer,
								  20,
								  (timer_2handler_t)power_save_wait_timer_handler,
								  NULL,
								  NULL);
	ASSERT(kNoErr == err);
}

void power_save_wait_timer_set(void)
{
	if (PS_STA_DTIM_SWITCH)
		bmsg_ps_sender(PS_BMSG_IOCTL_WAIT_TM_SET);
}

void power_save_wait_timer_start(void)
{
	OSStatus err;

	if (rtos_is_oneshot_timer_init(&ps_wait_timer) && ps_wait_timer_status == 0) {
		ps_wait_timer_status = 1;
#if (0 == CFG_LOW_VOLTAGE_PS)
		power_save_beacon_state_set(STA_GET_FALSE);
#endif
		err = rtos_start_oneshot_timer(&ps_wait_timer);
		ASSERT(kNoErr == err);
	}
}
#else
void power_save_wait_set(UINT32 set)
{
	ps_wait_timer_status = set;
}

UINT32 power_save_wait_get(void)
{
	return ps_wait_timer_status;
}
#endif

#if PS_USE_KEEP_TIMER
void power_save_keep_timer_stop ( void )
{
	GLOBAL_INT_DECLARATION();
#if ( 1 == CFG_LOW_VOLTAGE_PS)
	UINT32 timer_channel;
	timer_channel = BKTIMER5;
	bk_timer_ctrl(CMD_TIMER_UNIT_DISABLE,&timer_channel);
#else
	OSStatus err;
	if (rtos_is_oneshot_timer_running(&ps_keep_timer)) {
		err = rtos_stop_oneshot_timer(&ps_keep_timer);
		ASSERT(kNoErr == err);
	}
#endif

	GLOBAL_INT_DISABLE();
	ps_keep_timer_status = 0;
	GLOBAL_INT_RESTORE();
}

void power_save_keep_timer_real_handler(void)
{
	GLOBAL_INT_DECLARATION();
	PS_DEBUG_PWM_TRIGER;
#if CFG_SUPPORT_BLE
	rf_wifi_used_clr();
#endif
	GLOBAL_INT_DISABLE();

	if ( ( PS_STA_DTIM_SWITCH )
		&& ((bk_ps_info.ps_arm_wakeup_way == PS_ARM_WAKEUP_RW)
#if (1 == CFG_LOW_VOLTAGE_PS)
		|| (bk_ps_info.ps_arm_wakeup_way == PS_ARM_WAKEUP_USER)
#endif
		)
		&& 0 == bk_ps_info.ps_real_sleep ) {
		{
#if (1 == CFG_LOW_VOLTAGE_PS)
			power_save_keep_timer_stop();

			bk_ps_info.ps_arm_wakeup_way = PS_ARM_WAKEUP_USER;
			ps_bcn_loss_max_count ++;
			if(ps_bcn_loss_max_count < PS_BCN_MAX_LOSS_LIMIT)
			{
				power_save_clr_all_vif_prevent_sleep((UINT32)(PS_VIF_WAITING_BCN));
			}
			else if(ps_bcn_loss_max_count == PS_BCN_MAX_LOSS_LIMIT)
			{
				os_printf("beacon loss %d, keep wakeup for 200ms more to catch beacon!\r\n",ps_bcn_loss_max_count);
				/* close hardware tim if beacon loss cnt > 5*/
				lv_ps_force_software_beacon();
				ps_bcn_loss_max_count = 0;
				power_save_set_keep_timer_time(200);
			}

			lv_ps_beacon_missing_handler();
#else
			if(ps_keep_timer_flag && (power_save_beacon_state_get() != STA_GET_TRUE))
			{
				PS_DBG("@%d ",__LINE__);
				ps_fake_data_rx_check();
				ps_keep_timer_flag = 0;
				bmsg_ps_sender(PS_BMSG_IOCTL_RF_KP_SET);
				GLOBAL_INT_RESTORE();
				return;
			}

			if(0 == ps_reseted_moniter_flag
			&& ps_bcn_loss_max_count < PS_BCN_MAX_LOSS_LIMIT)
			{
				power_save_beacon_state_set ( STA_GET_TRUE );
				power_save_clr_all_vif_prevent_sleep((UINT32)(PS_VIF_WAITING_BCN));
				ps_bcn_loss_max_count++;

				PS_DBG("@%d ", __LINE__);
				ps_run_td_timer(0);
			} else {
				//If more than 5 consecutive beacon loss happens, stay wakeup
				ps_reseted_moniter_flag = 0;
			}
#endif
		}

		GLOBAL_INT_RESTORE();
		delay(1);
		PS_DEBUG_PWM_TRIGER;
#if CFG_USE_STA_PS
		extern void bmsg_null_sender(void);
		bmsg_null_sender();
#endif
	} else {
		GLOBAL_INT_RESTORE();
	}
}

void power_save_keep_timer_handler(void *data)
{
	bmsg_ps_sender(PS_BMSG_IOCTL_RF_KP_HANDLER);
}

void power_save_set_keep_timer_time ( UINT32 time )
{
	if ( time >= 0 && time < 500 ) {
		GLOBAL_INT_DECLARATION();
		GLOBAL_INT_DISABLE();
#if(CFG_LV_PS_WITH_IDLE_TICK == 1)
		ps_keep_timer_period = time + lv_ps_get_keep_timer_more();
#else
		ps_keep_timer_period = time;
#endif
		power_save_keep_timer_init();
		GLOBAL_INT_RESTORE();
	}

	return;
}

void power_save_keep_timer_init ( void )
{
#if ( 1 == CFG_LOW_VOLTAGE_PS)
	timer_param_t param;
	param.channel = BKTIMER5;
	param.div = 1;              //timer0 timer1 timer2 26M // timer4 timer5 32K (n+1) division
	param.period = ps_keep_timer_period;
	param.t_Int_Handler= (TFUNC)power_save_keep_timer_real_handler;
	UINT32 timer_channel;
	timer_channel = param.channel;
	bk_timer_ctrl(CMD_TIMER_INIT_PARAM,&param);
	bk_timer_ctrl(CMD_TIMER_UNIT_ENABLE,&timer_channel);
	ps_keep_timer_status = 1;
#else
	UINT32 err;

	if ( rtos_is_oneshot_timer_init ( &ps_keep_timer ) )
	{
		power_save_keep_timer_stop();
		err = rtos_deinit_oneshot_timer ( &ps_keep_timer );
		ASSERT ( kNoErr == err );
	}

	os_printf ( "ps_keep_timer init %d\r\n", ps_keep_timer_period );

	if ( ps_keep_timer_period > 0 ) {
		err = rtos_init_oneshot_timer ( &ps_keep_timer,
		                        ps_keep_timer_period,
		                        ( timer_2handler_t ) power_save_keep_timer_handler,
		                        NULL,
		                        NULL );
		ASSERT ( kNoErr == err );
	}
#endif
}

void power_save_keep_timer_set ( void )
{
	OSStatus err;

	if (rtos_is_oneshot_timer_init(&ps_keep_timer) && ps_keep_timer_status == 0) {
		ps_keep_timer_status = 1;
		err = rtos_start_oneshot_timer(&ps_keep_timer);
		ASSERT(kNoErr == err);
	}
}

void power_save_set_reseted_flag ( void )
{
	ps_reseted_moniter_flag = 1;
}

UINT32 power_save_get_bcn_lost_count ( void )
{
	return ps_bcn_loss_max_count;
}
#endif /* PS_USE_KEEP_TIMER */

void power_save_rf_ps_wkup_semlist_init(void)
{
	co_list_init(&bk_ps_info.wk_list);
}

void *power_save_rf_ps_wkup_semlist_create(void)
{
	UINT32 ret;
	PS_DO_WKUP_SEM *sem_list = (PS_DO_WKUP_SEM *)os_malloc(sizeof(PS_DO_WKUP_SEM));

	if (!sem_list) {
		os_printf("semlist_wait NULL\r\n");
		return 0;
	}

	ret = rtos_init_semaphore(&sem_list->wkup_sema, 1);
	ASSERT(0 == ret);
	return sem_list;
}

void power_save_rf_ps_wkup_semlist_wait(void *sem_list_p)
{
	PS_DO_WKUP_SEM *sem_list = (PS_DO_WKUP_SEM *)sem_list_p;

	co_list_push_back(&bk_ps_info.wk_list, &sem_list->list);

#if CFG_SUPPORT_BLE
	if (!ble_switch_mac_sleeped)
#endif
	bmsg_ps_sender(PS_BMSG_IOCTL_RF_USER_WKUP);
}

void power_save_rf_ps_wkup_semlist_destroy(void *sem_list_p)
{
	UINT32 ret;
	PS_DO_WKUP_SEM *sem_list = (PS_DO_WKUP_SEM *)sem_list_p;

	ret = rtos_deinit_semaphore(&sem_list->wkup_sema);
	ASSERT(0 == ret);
}

void power_save_rf_ps_wkup_semlist_get(void *sem_list)
{
	UINT32 ret;

	if (sem_list) {
		ret = rtos_get_semaphore(&((PS_DO_WKUP_SEM *)sem_list)->wkup_sema, BEKEN_NEVER_TIMEOUT);
		ASSERT(0 == ret);
		GLOBAL_INT_DECLARATION();
		GLOBAL_INT_DISABLE();
		co_list_extract(&bk_ps_info.wk_list, &((PS_DO_WKUP_SEM *)sem_list)->list);
		GLOBAL_INT_RESTORE();
		ret = rtos_deinit_semaphore(&((PS_DO_WKUP_SEM *)sem_list)->wkup_sema);
		ASSERT(0 == ret);
		os_free(sem_list);
		sem_list = NULL;
	}
}

void power_save_rf_ps_wkup_semlist_set(void)
{
	UINT32 ret;

	rtos_lock_scheduling();
	while (!co_list_is_empty(&bk_ps_info.wk_list)) {
		PS_DO_WKUP_SEM *sem_list;
		sem_list = list2sem(co_list_pop_front(&bk_ps_info.wk_list));
		ret = rtos_set_semaphore(&sem_list->wkup_sema);
		ASSERT(0 == ret);
	}
	rtos_unlock_scheduling();
}

void power_save_beacon_state_set ( PS_STA_BEACON_STATE state )
{
	bk_ps_info.waited_beacon = state;
}

/* Called when received beacon frame */
void power_save_beacon_state_update(void)
{
	PS_DEBUG_RX_TRIGER;
#if CFG_SUPPORT_BLE
	rf_wifi_used_clr();
#endif

	if (PS_STA_DTIM_SWITCH) {
		if (power_save_if_ps_can_sleep() &&
			power_save_beacon_state_get() == STA_GET_INIT)
			power_save_beacon_state_set(STA_GET_FALSE);
	}

	if (PS_STA_DTIM_SWITCH && (power_save_beacon_state_get() != STA_GET_TRUE)) {
		power_save_beacon_state_set(STA_GET_TRUE);
#if PS_USE_KEEP_TIMER
		ps_bcn_loss_max_count = 0;
#endif
#if ( 1 == CFG_LOW_VOLTAGE_PS)
	if (1 == ps_keep_timer_status)
		power_save_keep_timer_stop();

	if (0 == ps_keep_timer_flag) {
		PS_DBG("@%d ", __LINE__);
		ps_run_td_timer(0);
	}
#else
		if (platform_is_in_interrupt_context() != RTOS_SUCCESS) {
			if (1 == ps_keep_timer_status)
				//bmsg_ps_sender(PS_BMSG_IOCTL_RF_KP_STOP);
				power_save_keep_timer_stop();
			if (0 == ps_keep_timer_flag) {
				PS_DBG("@%d ", __LINE__);
				ps_run_td_timer(0);
			}
		}
#endif
	}
}

void power_save_bcn_callback(uint8_t *data, int len, wifi_link_info_t *info)
{
	struct bcn_frame *bcn = (struct bcn_frame *)data;
	VIF_INF_PTR vif_entry;

	for_each_vif_entry(vif_entry) {
		if (vif_entry->type == VIF_STA && vif_entry->active)
			break;
	}

	if (!vif_entry)
		return;

	if (bcn->bcnint != bk_ps_info.ps_beacon_int) {
		os_printf("bcn interval changed %x %x\r\n", bcn->bcnint, bk_ps_info.ps_beacon_int);
		mm_send_connection_loss_ind(vif_entry);
	}
}

UINT8 power_save_if_sleep_first(void)
{
	return bk_ps_info.sleep_first;
}

INT8 power_save_if_sleep_at_first(void)
{
	return bk_ps_info.sleep_count < 6;
}

PS_STA_BEACON_STATE power_save_beacon_state_get(void)
{
	return bk_ps_info.waited_beacon;
}

PS_ARM_WAKEUP_WAY power_save_wkup_way_get(void)
{
	return bk_ps_info.ps_arm_wakeup_way;
}

UINT8 power_save_if_ps_can_sleep(void)
{
	return bk_ps_info.ps_can_sleep == 1;
}

UINT32 power_save_get_sleep_count(void)
{
	return bk_ps_info.sleep_count;
}

void power_save_ps_mode_set(PS_MODE_STATUS mode)
{
	bk_ps_mode = mode;
}

UINT16 power_save_radio_wkup_get(void)
{
	return r_wakeup_time;
}

void power_save_radio_wkup_set(UINT16 time)
{
	r_wakeup_time = time;
}

UINT32 power_save_wkup_event_get(void)
{
	return ps_dis_flag;
}

void power_save_wkup_event_set ( UINT32 value )
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
	ps_dis_flag |= value;
	GLOBAL_INT_RESTORE();
}

void power_save_wkup_event_clear ( UINT32 value )
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
	ps_dis_flag &= ~value;
	GLOBAL_INT_RESTORE();
}

UINT16 power_save_beacon_len_get ( void )
{
	return beacon_len;
}

void power_save_beacon_len_set ( UINT16 len )
{
	beacon_len = len + 4/*fcs*/ /*+25 radiotap*/;
}

UINT8 power_save_set_dtim_multi(UINT8 multi)
{
	bk_ps_info.ps_dtim_multi = multi;

	if (bk_ps_info.ps_dtim_multi > 0 && bk_ps_info.ps_dtim_multi < 100) {
		os_printf("set listen dtim:%d\r\n", bk_ps_info.ps_dtim_multi);
	} else {
		os_printf("set listen dtim:%d err,use default 1\r\n", bk_ps_info.ps_dtim_multi);
		bk_ps_info.ps_dtim_multi = 1;
	}

	bk_ps_info.listen_mode = PS_LISTEN_MODE_DTIM;
	return 0;
}

UINT16 power_save_forbid_trace(PS_FORBID_STATUS forbid)
{
	bk_forbid_count++;

	if (bk_forbid_code != forbid || (bk_forbid_count % 100 == 0)) {
		PS_DBG("front c:%d\r\n\r\n", bk_forbid_count);
		PS_DBG("ps_cd:%d %d\r\n", bk_forbid_code, forbid);
		bk_forbid_count = 0;
	}

	bk_forbid_code = forbid;
	return bk_forbid_count;
}

void power_save_dump(void)
{
	UINT32 i;
	extern UINT32 txl_cntrl_pck_get(void);

	os_printf("rf:%x\r\n", bk_ps_mode);
	os_printf("info dump\r\n");

	for (i = 0; i < sizeof(bk_ps_info); i++)
		os_printf(" %d 0x%x\r\n", i, *((UINT8 *)(&bk_ps_info) + i));

	os_printf("globel dump\r\n");
	os_printf("%d %d %d %d %d %d\r\n",
			  bk_ps_mode,
			  mhdr_get_station_status(),
			  g_wlan_general_param->role,
			  bk_ps_info.waited_beacon,
			  bk_ps_info.ps_can_sleep,
			  ps_lock);
	os_printf("env dump\r\n");
	os_printf("%d %d %d %d\r\n",
			  ps_env.ps_on,
			  me_env.ps_on,
			  beacon_len,
			  txl_cntrl_pck_get());
#if CFG_USE_MCU_PS
	os_printf("mcu dump\r\n");
	os_printf("%d %d\r\n",
			  peri_busy_count_get(),
			  mcu_prevent_get());
#endif
	os_printf("%d %d %d %d %d\r\n",
			  bk_ps_info.ps_dtim_period, bk_ps_info.ps_dtim_count,
			  bk_ps_info.ps_dtim_multi, bk_forbid_code);
#if CFG_USE_STA_PS
	sctrl_ps_dump();
#endif

	os_printf ( "PS_STA_DTIM_SWITCH\r\n");
	os_printf ( "bk_ps_mode:%d \r\n",bk_ps_mode);
	os_printf ( "mhdr_get_station_status:%d \r\n",mhdr_get_station_status());
	os_printf ( "g_wlan_general_param->role:%d \r\n",g_wlan_general_param->role);
	os_printf ( "\r\n");
	os_printf ( "PS_STA_DTIM_CAN_SLEEP\r\n");
	os_printf ( "bk_ps_info.waited_beacon:%d \r\n",bk_ps_info.waited_beacon);
	os_printf ( "power_save_wkup_way_get():%d \r\n", power_save_wkup_way_get());
	os_printf ( "bk_ps_info.ps_can_sleep:%d \r\n",bk_ps_info.ps_can_sleep);

	os_printf ( "\r\n");
	os_printf ( "ps_may_sleep()\r\n");
	os_printf ( "mcu_ps_info.mcu_ps_on:%d \r\n",mcu_ps_is_on());
	os_printf ( "peri_busy_count_get():%d \r\n", peri_busy_count_get());
	os_printf ( "mcu_prevent_get():%d \r\n",mcu_prevent_get());

	os_printf ( "\r\n");
	os_printf ( "ps_sleep_check()\r\n");
	os_printf ( "ps_env.ps_on:%d \r\n",ps_env.ps_on);
	os_printf ( "ps_env.prevent_sleep:%d \r\n", ps_env.prevent_sleep);
}

void power_save_wake_mac_rf_if_in_sleep(void)
{
    ps_set_rf_prevent();
    power_save_rf_dtim_manual_do_wakeup();

    power_save_rf_hold_bit_set(RF_HOLD_BY_MAC_USE_BIT);
}

void power_save_wake_mac_rf_end_clr_flag(void)
{
    if(ps_get_sleep_prevent() & PS_WAITING_RF_OPERATION)
    {
        ps_clear_rf_prevent();
    }

    power_save_rf_hold_bit_clear(RF_HOLD_BY_MAC_USE_BIT);
}

void power_save_check_clr_rf_prevent_flag(void)
{
}

void power_save_wake_rf_if_in_sleep(void)
{
}

void power_save_clr_temp_use_rf_flag(void)
{
    if(ps_get_sleep_prevent() & PS_WAITING_RF_OPERATION)
    {
        ps_clear_rf_prevent();
    }

    power_save_rf_hold_bit_clear(RF_HOLD_BY_TEMP_BIT);
}

void power_save_set_temp_use_rf_flag(void)
{
    ps_set_rf_prevent();
    power_save_rf_hold_bit_set(RF_HOLD_BY_TEMP_BIT);
}

void power_save_rf_hold_bit_set(UINT32 rf_hold_bit)
{
    UINT32 reg = rf_hold_bit;
    sddev_control(SCTRL_DEV_NAME, CMD_RF_HOLD_BIT_SET, &reg);
}

void power_save_rf_hold_bit_clear(UINT32 rf_hold_bit)
{
    UINT32 reg = rf_hold_bit;
    sddev_control(SCTRL_DEV_NAME, CMD_RF_HOLD_BIT_CLR, &reg);
}

UINT8 power_save_if_ps_rf_dtim_enabled ( void )
{
	return ( bk_ps_mode == PS_DTIM_PS_MODE );
}

PS_MODE_STATUS power_save_ps_mode_get ( void )
{
	return bk_ps_mode;
}

UINT8 power_save_if_rf_sleep(void)
{
#if CFG_USE_STA_PS
	if (bk_ps_info.ps_real_sleep == 1)
		return 1;
#endif
	return 0;
}

UINT32 power_save_time_to_sleep(void)
{
	INT32 less;

#if CFG_USE_STA_PS
	UINT32 tm;

	if (bk_ps_info.ps_dtim_count == 0)
		tm = bk_ps_info.ps_dtim_period * bk_ps_info.ps_dtim_multi * bk_ps_info.ps_beacon_int;
	else
		tm = (bk_ps_info.ps_dtim_period * (bk_ps_info.ps_dtim_multi - 1) + bk_ps_info.ps_dtim_count) * bk_ps_info.ps_beacon_int;

	less = tm - (((fclk_get_tick() - last_wk_tick) * FCLK_DURATION_MS) % tm);
#else
	less = 0;
#endif
	return less;
}

#if CFG_LOW_LATENCY_PS
void power_save_set_low_latency ( UINT8 value )
{
	ps_data_low_latency = value;
}

UINT8 power_save_low_latency_get ( void )
{
	return ps_data_low_latency;
}
#endif

#if(CFG_HW_PARSER_TIM_ELEMENT == 1)
void power_save_clear_hw_tim_cnt(void)
{
	ps_hw_tim_cnt = 0;
}

void power_save_increase_hw_tim_cnt(void)
{
	ps_hw_tim_cnt ++;
}

uint32_t power_save_get_hw_tim_cnt(void)
{
	return ps_hw_tim_cnt;
}

/**
 * Set the cnt_limit for hw_tim, once hw_tim_cnt is over cnt_limit, hw_tim will be disabled until next beacon received (hw_tim_cnt reset to 0).
 * @param cnt The cnt limit you want to pass, pass 0 if you want to disable hw_tim.
*/
void power_save_set_hw_tim_cnt_limit(uint32_t cnt)
{
	if (cnt == 0)
	{
		os_printf("hw tim disabled!\n");
	}
	ps_hw_tim_cnt_limit = cnt;
	lvc_calc_g_bundle_reset();
}
#endif

// eof

