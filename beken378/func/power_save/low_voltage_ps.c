#include "low_voltage_ps.h"
#include "power_save_pub.h"
#include "power_save.h"
#include "calendar_pub.h"
#include "mcu_ps.h"
#include "low_voltage_compensation.h"
#include "ps.h"
#include "mcu_ps_pub.h"
#include "icu_pub.h"
#include "sys_ctrl.h"
#include "phy_trident.h"
#include "bk7011_cal_pub.h"
#include "target_util_pub.h"
#if CFG_SUPPORT_BLE
#include "ble.h"
#endif
#include "mem_pub.h"

#define LV_PS_BEACON_LOSS_TIME_S            (30)

uint32_t lv_ps_beacon_interval = 0;
uint32_t lv_ps_start_flag = 0;
uint32_t lv_ps_enable_print = 0;
uint32_t lv_anchor_flag = 0;
uint32_t lv_ps_current_sleep_duration = 0;
uint64_t lv_ps_target_time = 0;
uint64_t lv_ps_bcn_tsf_field = 0;
uint32_t lv_ps_beacon_cnt_after_wakeup = 0;
uint32_t lv_ps_bcn_loss_flag_after_wakeup = 0;
uint64_t lv_ps_wakeup_mcu_timepoint = 0;
uint64_t lv_ps_wakeup_mac_timepoint = 0;
int32_t lv_ps_bcn_delay_duration = 0;
uint32_t lv_ps_bcn_frame_duration = 0;
uint64_t lv_ps_bcn_rxd_local_time = 0;
uint32_t lv_ps_tbtt_to_rxd_time = 0;
#if (AFTER_MISSING_STRATEGY == WAIT_UNTIL_RECVED)
uint32_t lv_ps_loss_bcn_count = 0;
#endif
int32_t lv_ps_pre_lead_wakeup_duration = 0;
uint32_t lv_ps_win_pri_compensation_factor;
uint32_t lv_ps_win_post_compensation_factor;
#if (AFTER_MISSING_STRATEGY == WAIT_ONCE)
uint32_t lv_ps_bcn_has_been_waiting = 0;
#endif
uint32_t lv_ps_bcn_cont_miss_bcn_count = 0;
uint64_t lv_ps_last_beacon_rev_timepoint = 0;
uint64_t lv_ps_first_beacon_change_rev_timepoint = 0;
#if CFG_USE_TICK_CAL
uint32_t lv_ps_sleep_cnt = 0;
#endif
#define TX_RECOVER_INIT     0
#define TX_RECOVER_SLEEP    1
#define TX_RECOVER_RECOVER  2
volatile UINT32 is_tx_recover = TX_RECOVER_INIT;

#if(CFG_HW_PARSER_TIM_ELEMENT == 1)
uint64_t lv_ps_tbtt_local;
int32_t lv_ps_tbtt_local_remainder;
#endif

static beken2_timer_t lv_ps_trigger_timer = {0};
static UINT32 lv_ps_trigger_timer_status = 0;

#if(CFG_LV_PS_WITH_IDLE_TICK == 1)
UINT32 lv_ps_mac_wakeup_flag = 1;
UINT32 lv_ps_keep_timer_more =0;
#endif
uint64_t lv_ps_arp_send_time = 0;
uint16_t lv_ps_dtim_period = 0;

uint8_t lv_ps_rf_pre_pwr_down = 0;
uint8_t lv_ps_rf_reinit = 0;
uint8_t lv_ps_wakeup_wifi = 1;
PS_DEEP_WAKEUP_WAY lv_ps_wake_up_way = PS_DEEP_WAKEUP_NULL;

static LIST_HEAD_DEFINE(lv_element);

/*******************************************************************************
* LV_PS_INFO DUMP
*******************************************************************************/
#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
static struct lv_ps_info_st lv_ps_info;
static uint64_t lv_ps_rf_start_local_time;
static uint64_t lv_ps_rf_ready_local_time;
static uint64_t lv_ps_rf_pend_local_time;
static uint64_t lv_ps_rf_end_local_time;
static bool lv_ps_connection_loss_flag;
static uint64_t lv_ps_disc_local_time;
static uint64_t lv_ps_rec_local_time;

static void lv_ps_info_reinit(void);

#define LV_PS_INFO_PRINT_TIMEOUT (cal_get_time_us() - lv_ps_info.mgmt.stat_start_time >= lv_ps_info.mgmt.print_period * 1000000)
#define sqr(x) ((x)*(x))
/// (u64)us ==> (u32)hour,(u32)min,(u32)second
#define US_TO_READABLE_VALUE(us) (uint32_t)(us/1000000/3600),(uint32_t)(us/1000000%3600/60),(uint32_t)(us/1000000%60)

/**
 * set print config.
 * statistic will be reinit if print already enabled.
*/
void lv_ps_info_print_switch(bool flag, uint32_t period)
{
	lv_ps_info.mgmt.print_enable = flag;
	lv_ps_info.mgmt.print_period = period;
	lv_ps_info_reinit();
}

/**
 * call by lv_ps_init only.
*/
void lv_ps_info_init(void)
{
	lv_ps_connection_loss_flag = 0;

	lv_ps_info.ap_bcn_delay_m_save = 0;
	lv_ps_info.tbtt_to_rxd_m_save = 0;

	lv_ps_info.mgmt.stat_index = 0;
	lv_ps_info.mgmt.stat_start_time = cal_get_time_us();

	memset(&lv_ps_info.beacon, 0, sizeof(LV_PS_BEACON_STAT));
	memset(&lv_ps_info.runtime, 0, sizeof(LV_PS_RUNTIME_STAT));
}

static void lv_ps_info_reinit(void)
{
	lv_ps_info.ap_bcn_delay_m_save = lv_ps_info.beacon.ap_bcn_delay_m;
	lv_ps_info.tbtt_to_rxd_m_save = lv_ps_info.beacon.tbtt_to_rxd_m;

	lv_ps_info.mgmt.stat_index += 1;
	lv_ps_info.mgmt.stat_start_time = cal_get_time_us();

	memset(&lv_ps_info.beacon, 0, sizeof(LV_PS_BEACON_STAT));
	memset(&lv_ps_info.runtime, 0, sizeof(LV_PS_RUNTIME_STAT));
}

void lv_ps_info_dump(void)
{
	// header
	os_printf("------------------------------------------\r\n");
	os_printf("index:%d  duration:%d s  start:%d:%d:%d  end:%d:%d:%d\r\n", 	lv_ps_info.mgmt.stat_index,
																			(uint32_t)(lv_ps_info.mgmt.stat_end_time-lv_ps_info.mgmt.stat_start_time)/1000000,
																			US_TO_READABLE_VALUE(lv_ps_info.mgmt.stat_start_time),
																			US_TO_READABLE_VALUE(lv_ps_info.mgmt.stat_end_time));
	os_printf("print_flag:%d\t\tperiod:%d\r\n", 							lv_ps_info.mgmt.print_enable,
																			lv_ps_info.mgmt.print_period);
	// beacon
	os_printf("------------------BEACON------------------\r\n");
	os_printf("inteval:\t\t%d\nlength_mean:\t\t%d\nframe_dura_mean:\t%d\nrecv_cnt:\t\t%d\ntim_cnt:\t\t%d\r\n",	lv_ps_info.beacon.interval,
																												lv_ps_info.beacon.length_m,
																												(uint32_t)lv_ps_info.beacon.frame_dura_m,
																												lv_ps_info.beacon.recv_count,
																												lv_ps_info.beacon.tim_count);
	os_printf("ap_bcn_delay_mean:\t%d\nap_bcn_delay_vari:\t%d\ntbtt_to_rxd_mean:\t%d\ntbtt_to_rxd_vari:\t%d\r\n",	lv_ps_info.beacon.ap_bcn_delay_m,
																												(uint32_t)lv_ps_info.beacon.ap_bcn_delay_v,
																												lv_ps_info.beacon.tbtt_to_rxd_m,
																												(uint32_t)lv_ps_info.beacon.tbtt_to_rxd_v);
	os_printf("loss_cnt:\t\t%d\nloss_times:\t\t%d\ndisc_cnt:\t\t%d\nreconn_tcost:\t\t%d\r\n\r\n",				lv_ps_info.beacon.loss_count,
																												lv_ps_info.beacon.loss_cnt_times,
																												lv_ps_info.beacon.disc_count,
																												(uint32_t)lv_ps_info.beacon.reconn_tcost);
	uint32_t bcn_count_total = lv_ps_info.beacon.recv_count + lv_ps_info.beacon.loss_count + lv_ps_info.beacon.tim_count;
	os_printf("beacon received rate =%6.2f%%\r\n",(float)(lv_ps_info.beacon.recv_count + lv_ps_info.beacon.tim_count) / bcn_count_total * 100);
	// runtime
	os_printf("------------------RUNTIME-----------------\r\n");
	os_printf("mcu_running_time:\t%d\nmcu_sleep_count:\t%d\nwakeup_tcost_total:\t%d\nsleep_tcost_total:\t%d\r\n",	(uint32_t)lv_ps_info.runtime.mcu_running_time,
																												lv_ps_info.runtime.mcu_sleep_count,
																												(uint32_t)lv_ps_info.runtime.wakeup_tcost_total,
																												(uint32_t)lv_ps_info.runtime.sleep_tcost_total);
	os_printf("rf_running_time:\t%d\nrf_sleep_count:\t\t%d\nrf_sleep_retry_count:\t%d\r\n",						(uint32_t)lv_ps_info.runtime.rf_running_time,
																												lv_ps_info.runtime.rf_sleep_count,
																												lv_ps_info.runtime.rf_sleep_retry_count);
	os_printf("rf_to_tim:\t\t%d\nrf_to_bcn:\t\t%d\nrf_init_tcost:\t\t%d\nrf_sleep_tcost:\t\t%d\r\n\r\n", 		(uint32_t)lv_ps_info.runtime.rf_to_tim,
																												(uint32_t)lv_ps_info.runtime.rf_to_bcn,
																												(uint32_t)lv_ps_info.runtime.rf_init_tcost,
																												(uint32_t)lv_ps_info.runtime.rf_sleep_tcost);
	uint64_t rf_to_bcn_time = (lv_ps_info.runtime.rf_to_bcn * lv_ps_info.beacon.recv_count + lv_ps_info.runtime.rf_to_tim * lv_ps_info.beacon.tim_count) 
								/ (lv_ps_info.beacon.recv_count + lv_ps_info.beacon.tim_count);
	os_printf("wakeup to beacon time mean =%6d us\r\n", (uint32_t)(lv_ps_info.runtime.wakeup_tcost_total + lv_ps_info.runtime.rf_init_tcost + rf_to_bcn_time));
	os_printf("------------------------------------------\r\n");
}

/**
 * record necessary information during runtime
 * and deal with information just before dump it.
*/
void lv_ps_info_calc(void)
{
	uint64_t duration;
	uint32_t bcn_cnt_expected, bcn_cnt_real;

	// accumulate time
	lv_ps_info.mgmt.stat_end_time = cal_get_time_us();
	duration = lv_ps_info.mgmt.stat_end_time - lv_ps_info.mgmt.stat_start_time;
	lv_ps_info.mgmt.total_time += duration;

	// check beacon count
	bcn_cnt_expected = duration / lv_ps_info.beacon.interval;
	bcn_cnt_real = lv_ps_info.beacon.recv_count + lv_ps_info.beacon.loss_count + lv_ps_info.beacon.tim_count;

	os_printf("\r\n");
	if(bcn_cnt_expected == bcn_cnt_real) {
		os_printf("bcn_count expected %d and get %d\r\n", bcn_cnt_expected, bcn_cnt_real);
	} else {
		os_printf("bcn_count expected %d but get %d\r\n", bcn_cnt_expected, bcn_cnt_real);
	}

	// handle with sleep count
	if(lv_ps_info.runtime.mcu_sleep_count > 1)
		lv_ps_info.runtime.mcu_sleep_count--;
	if(lv_ps_info.runtime.rf_sleep_count > 1)
		lv_ps_info.runtime.rf_sleep_count--;

	// calculate mean value by beacon count
	lv_ps_info.beacon.length_m /= lv_ps_info.beacon.recv_count;
	lv_ps_info.beacon.frame_dura_m /= lv_ps_info.beacon.recv_count;
	lv_ps_info.beacon.ap_bcn_delay_m /= (int32_t)lv_ps_info.beacon.recv_count;
	lv_ps_info.beacon.ap_bcn_delay_v /= lv_ps_info.beacon.recv_count;
	lv_ps_info.beacon.tbtt_to_rxd_m /= lv_ps_info.beacon.recv_count;
	lv_ps_info.beacon.tbtt_to_rxd_v /= lv_ps_info.beacon.recv_count;
	lv_ps_info.beacon.reconn_tcost /= lv_ps_info.beacon.disc_count;

	// runtime statistic
	lv_ps_info.runtime.mcu_running_time /= lv_ps_info.runtime.mcu_sleep_count;
	lv_ps_info.runtime.wakeup_tcost_total /= lv_ps_info.runtime.mcu_sleep_count;
	lv_ps_info.runtime.sleep_tcost_total /= lv_ps_info.runtime.mcu_sleep_count;
	lv_ps_info.runtime.rf_running_time /= lv_ps_info.runtime.rf_sleep_count;
	lv_ps_info.runtime.rf_init_tcost /= lv_ps_info.runtime.rf_sleep_count + 1;
	lv_ps_info.runtime.rf_sleep_tcost /= lv_ps_info.runtime.rf_sleep_count;
	lv_ps_info.runtime.rf_to_bcn /= lv_ps_info.beacon.recv_count;
	lv_ps_info.runtime.rf_to_tim /= lv_ps_info.beacon.tim_count;

	// variance statistic value should not be print during the first print
	if(0 == lv_ps_info.mgmt.stat_index) {
		lv_ps_info.beacon.tbtt_to_rxd_v = 0;
		lv_ps_info.beacon.ap_bcn_delay_v = 0;
	}
}

void lv_ps_info_recv_tim(void)
{
	if(0 == lv_ps_info.runtime.rf_sleep_count)
		return;
	lv_ps_info.beacon.tim_count += 1;
	lv_ps_info.runtime.rf_to_tim += cal_get_time_us() - lv_ps_rf_ready_local_time;
}

void lv_ps_info_recv_bcn(uint16_t len)
{
	if(0 == lv_ps_info.runtime.rf_sleep_count)
		return;
	uint32_t listen_interval = PS_DTIM_COUNT;
#if CFG_USE_STA_PS
	listen_interval = power_save_get_listen_int();
#endif
	lv_ps_info.beacon.interval = listen_interval * lv_ps_beacon_interval;
	lv_ps_info.beacon.length_m += len;
	lv_ps_info.beacon.frame_dura_m += lv_ps_bcn_frame_duration;

	lv_ps_info.beacon.recv_count += 1;
	lv_ps_info.beacon.ap_bcn_delay_m += lv_ps_bcn_delay_duration;
	lv_ps_info.beacon.ap_bcn_delay_v += sqr(lv_ps_bcn_delay_duration - lv_ps_info.ap_bcn_delay_m_save);
	lv_ps_info.beacon.tbtt_to_rxd_m += lv_ps_tbtt_to_rxd_time;
	lv_ps_info.beacon.tbtt_to_rxd_v += sqr(lv_ps_tbtt_to_rxd_time - lv_ps_info.tbtt_to_rxd_m_save);

	lv_ps_info.runtime.rf_to_bcn += lv_ps_bcn_rxd_local_time - lv_ps_rf_ready_local_time;
}

void lv_ps_info_disconnect(void)
{
	lv_ps_connection_loss_flag = 1;
	lv_ps_disc_local_time = cal_get_time_us();
	lv_ps_info.beacon.disc_count += 1;
}

void lv_ps_info_reconnect(void)
{
	if((lv_ps_info.mgmt.print_enable == 1)&&(lv_ps_connection_loss_flag == 1))
	{
		lv_ps_connection_loss_flag = 0;
		lv_ps_rec_local_time = cal_get_time_us();
		lv_ps_info.beacon.reconn_tcost += lv_ps_rec_local_time - lv_ps_disc_local_time;
	}
}

void lv_ps_info_mcu_wakeup(void)
{
	lv_ps_info.runtime.mcu_sleep_count += 1;
	if(1 == lv_ps_info.runtime.mcu_sleep_count)
		return;
#if (AFTER_MISSING_STRATEGY == WAIT_UNTIL_RECVED)
	if(lv_ps_loss_bcn_count)
		lv_ps_info.beacon.loss_count += 1;
	// only if last beaocn was received
	if(lv_ps_loss_bcn_count == 1)
		lv_ps_info.beacon.loss_cnt_times += 1;
#elif (AFTER_MISSING_STRATEGY == NO_WAIT)
	if (lv_ps_bcn_cont_miss_bcn_count)
		lv_ps_info.beacon.loss_count += 1;
#endif
}

void lv_ps_info_rf_wakeup(bool restart_flag)
{
	if(0 != lv_ps_info.runtime.rf_sleep_count)
		lv_ps_info.runtime.rf_running_time += lv_ps_rf_pend_local_time - lv_ps_rf_ready_local_time;
	if(0 == restart_flag) {
		lv_ps_info.runtime.rf_sleep_count += 1;
		lv_ps_rf_start_local_time = cal_get_time_us();
	} else {
		lv_ps_info.runtime.rf_sleep_retry_count += 1;
		lv_ps_rf_ready_local_time = cal_get_time_us();
	}
}

void lv_ps_info_rf_ready(void)
{
	if(0 == lv_ps_info.runtime.rf_sleep_count)
		return;
	lv_ps_rf_ready_local_time = cal_get_time_us();
	lv_ps_info.runtime.rf_init_tcost += lv_ps_rf_ready_local_time - lv_ps_rf_start_local_time;
}

void lv_ps_info_rf_sleep(bool pre_flag)
{
	if(0 == lv_ps_info.runtime.rf_sleep_count)
		return;
	if(0 == pre_flag) {
		lv_ps_rf_end_local_time = cal_get_time_us();
		lv_ps_info.runtime.rf_sleep_tcost += lv_ps_rf_end_local_time - lv_ps_rf_pend_local_time;
	} else {
		lv_ps_rf_pend_local_time = cal_get_time_us();
	}
}

void lv_ps_info_mcu_sleep(uint64_t current_time)
{
	if(0 == lv_ps_info.runtime.mcu_sleep_count)
		return;
	lv_ps_info.runtime.mcu_running_time += current_time - lv_ps_wakeup_mac_timepoint;
	lv_ps_info.runtime.wakeup_tcost_total += lv_ps_rf_start_local_time - lv_ps_wakeup_mac_timepoint;
	lv_ps_info.runtime.sleep_tcost_total += current_time - lv_ps_rf_end_local_time;
}
#endif

/*******************************************************************************
* LV_PS INIT FUNCTION
*******************************************************************************/
void lv_ps_init(void)
{
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    lv_ps_start_flag = 0;
    lv_ps_enable_print = 0;
    lv_anchor_flag = 0;
    lv_ps_current_sleep_duration = 0;
    lv_ps_bcn_tsf_field = 0;
    lv_ps_beacon_cnt_after_wakeup = 0;
    lv_ps_bcn_loss_flag_after_wakeup = 0;
    lv_ps_wakeup_mcu_timepoint = 0;
    lv_ps_wakeup_mac_timepoint = 0;
    lv_ps_bcn_delay_duration = 0;
    lv_ps_bcn_frame_duration = 0;
    lv_ps_bcn_rxd_local_time = 0;
    lv_ps_tbtt_to_rxd_time = 0;
#if (AFTER_MISSING_STRATEGY == WAIT_UNTIL_RECVED)
    lv_ps_loss_bcn_count = 0;
#endif
    lv_ps_pre_lead_wakeup_duration = 0;
    lv_ps_win_pri_compensation_factor = 0;
    lv_ps_win_post_compensation_factor = 0;
#if (AFTER_MISSING_STRATEGY == WAIT_ONCE)
    lv_ps_bcn_has_been_waiting = 0;
#endif
    lv_ps_bcn_cont_miss_bcn_count = 0;
    lv_ps_last_beacon_rev_timepoint = 0;
    lv_ps_wakeup_wifi = 1;
    lv_ps_wake_up_way = PS_DEEP_WAKEUP_NULL;
#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
    lv_ps_info_init();
#endif
    lv_ps_sleep_trigger_timer_init();
    lvc_calc_g_bundle_reset();
    GLOBAL_INT_RESTORE();
}

/*******************************************************************************
* TX CHECK FUNCTION
*******************************************************************************/
UINT32 lv_ps_check_tx_recovery(void)
{
    return (is_tx_recover != TX_RECOVER_SLEEP) ? 1: 0;
}

void lv_ps_set_tx_recovery(void)
{
//    rwnx_cal_recover_tx_setting();
    rwnx_cal_recover_wifi_setting();
    phy_exit_11b_low_power();
    //rwnxl_reset_handle(0);
    is_tx_recover = TX_RECOVER_RECOVER;
}

void lv_ps_clear_tx_recovery(void)
{
    is_tx_recover = TX_RECOVER_SLEEP;
}

void lv_ps_check_11b(void)
{
    if(mm_ap_beacon_rate_is_11b())
    {
        phy_enter_11b_low_power();
    }
    else
    {
        phy_exit_11b_low_power();
    }
}

extern void net_send_gratuitous_arp(void);
extern void rwn_set_tx_low_rate_once(void);
void lv_ps_send_arp(void)
{
    if(cal_get_time_us() - lv_ps_arp_send_time > LOW_VOL_ARP_SEND_INTERVAL * 1000000)
    {
        bmsg_ps_sender(PS_BMSG_IOCTL_ARP_TX);
    }
}

void lv_ps_update_arp_send_time(void)
{
    lv_ps_arp_send_time = cal_get_time_us();
}

void lv_ps_keepalive_arp_tx(void)
{
    rwn_set_tx_low_rate_once();
    net_send_gratuitous_arp();
}

/*******************************************************************************
* LV_PS_SLEEP FUNCTION
*******************************************************************************/
extern uint32_t lvc_general_sleep_flag;

extern void ps_send_connection_loss(void);
static uint32_t lv_ps_check_beacon_loss(void);

bool lv_ps_sleep_check( UINT32 sleep_tick)
{
	uint32_t ret = 0;
	uint32_t debug_print_flag = 0;
	uint64_t time_saved, delta_ms;
	LV_PS_ELEMENT_ENV * lv_elem = NULL;
	uint8_t lv_ele_role = LV_TYPE_NONE;


#if (NX_POWERSAVE && CFG_USE_STA_PS && PS_WAKEUP_MOTHOD_RW)

	GLOBAL_INT_DECLARATION();

	if( ( bk_wlan_has_role(VIF_STA) ) && ( !lv_ps_is_got_anchor_point() ) ) {
		debug_print_flag = 1;
		goto check_exit;
	}

	if( !ps_may_sleep() ) {
		debug_print_flag = 2;
		goto check_exit;
	}

	if( !sctrl_if_mcu_can_sleep())
	{
		debug_print_flag = 3;
		goto check_exit;
	}

	if ((bk_wlan_has_role(VIF_STA)) && ( lv_ps_wakeup_wifi ))
	{
		lv_ps_calc_sleep_duration();
		lv_ps_wakeup_wifi = 0;
	}

	lv_elem = lv_ps_pop_element();

	if (lv_elem) {
		lv_ps_target_time = lv_elem->lv_target_time;
		lv_ele_role = lv_elem->lv_type;
	}
	#if CFG_SUPPORT_BLE
	else if (!ble_thread_is_up() && (!bk_wlan_has_role(VIF_STA))) {
	#else
	else if (!bk_wlan_has_role(VIF_STA))
	#endif
		/*keep lv sleep mode util wakeup by gpio*/
		lv_ps_target_time = -1;
	}
	else {
		debug_print_flag = 4;
		goto check_exit;
	}

	/*calculate sleep duration,if too short ,do not enter into low voltage ps*/
	if(false == lv_ps_rosc_timer_setting( sleep_tick)) //set rtc time
	{
		debug_print_flag = 4;
		goto check_exit;
	}
	GLOBAL_INT_DISABLE();

	/*save current us, fot sleep duration compensation*/
	time_saved = cal_get_time_us();

	/*enter lv ps*/
	lv_ps_sleep(); ////enter lv ps

	/*tick compensation*/
	delta_ms = (cal_get_time_us() - time_saved) / 1000;
	fclk_update_tick( (uint32_t) BK_MS_TO_TICKS(delta_ms) - 1 );

#if CFG_USE_TICK_CAL
	/*tick calibrationt*/
	lv_ps_sleep_cnt++;
	lv_ps_cal_tick();
#endif

	GLOBAL_INT_RESTORE();

#endif //(NX_POWERSAVE)

check_exit:

	if (lv_elem) {
		if (lv_elem->object_cb) {
			if (((lv_ele_role == LV_TYPE_WIFI) && (lv_ps_wake_up_way == PS_DEEP_WAKEUP_GPIO))) {
				lv_ps_push_element(lv_elem);
			}
			else
				lv_elem->object_cb(lv_elem->lv_target_time,false);
	}

		if (lv_ele_role == LV_TYPE_WIFI) {
			if (lv_ps_element_next() == LV_TYPE_BT) {
				LV_PS_ELEMENT_ENV * next_elem = lv_ps_pop_element();
				next_elem->object_cb(next_elem->lv_target_time,false);
				os_free(next_elem);
			}
		}
		os_free(lv_elem);
	}
	lv_ps_element_check_pass();
	if(debug_print_flag)
	{
		WFI();
		//os_printf(":%d \r\n", debug_print_flag);
/*		os_printf("debug_print_flag:%d\r\n", debug_print_flag);
		os_printf("debug_print0:%d\r\n", power_save_if_ps_rf_dtim_enabled());
		os_printf("debug_print1:%d\r\n", (ke_evt_get() != 0));
		os_printf("debug_print2:%d\r\n", (!bmsg_is_empty()));
		os_printf("debug_print3:%d\r\n", (power_save_beacon_state_get() == STA_GET_TRUE));
		os_printf("debug_print4:%d\r\n", (power_save_wkup_way_get() == PS_ARM_WAKEUP_USER));
		os_printf("debug_print5:%d\r\n", power_save_if_ps_can_sleep());//bk_ps_info.ps_can_sleep == 1
		os_printf("\r\n");*/
	}
#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
	if(lv_ps_info.mgmt.print_enable && LV_PS_INFO_PRINT_TIMEOUT) {
		time_saved = cal_get_time_us();
		lv_ps_info_calc();
		lv_ps_info_dump();
		lv_ps_info_reinit();
		os_printf("ps info dump time cost %d us\r\n\r\n", (uint32_t)(cal_get_time_us() - time_saved));
	}
#endif
	return ret;
}

void lv_ps_sleep(void)
{
	UINT32 param;

#if (CHIP_U_MCU_WKUP_USE_TIMER && ((CFG_SOC_NAME != SOC_BK7231)))
	param = ( 0xfffff  & ( ~PWD_TIMER_26M_CLK_BIT ) & ( ~PWD_TIMER_32K_CLK_BIT ) & ( ~PWD_UART2_CLK_BIT )
				& ( ~PWD_UART1_CLK_BIT )
			);
#else
	param = ( 0xfffff & ( ~PWD_MCU_WAKE_PWM_BIT ) & ( ~PWD_UART2_CLK_BIT )
				& ( ~PWD_UART1_CLK_BIT )
			);
#endif

	/* sctrl_mcu_sleep(param) */
	sctrl_mcu_sleep( param ); // call lv_ps_wakeup_set_timepoint() inside;

	/*sctrl_mcu_wakeup */
	sctrl_mcu_wakeup();

    #if 0
#if(CFG_LV_PS_WITH_IDLE_TICK == 1)

	if(lv_ps_get_mac_wakeup_flag() == 1)
	{
		lv_ps_clear_tx_recovery();
		power_save_dtim_wake ( MAC_ARM_WAKEUP_EN_BIT );
	}
	else
	{
		delay_us(500);//if there is no delay, watch dog reset will happen, reason is not clear
	}
#else
	lv_ps_clear_tx_recovery();
	power_save_dtim_wake ( MAC_ARM_WAKEUP_EN_BIT );

#endif
    #endif
}

uint64_t lv_ps_wakeup_set_timepoint(void)
{

#if(CFG_LV_PS_WITH_IDLE_TICK == 1)
    lv_ps_wakeup_mcu_timepoint = cal_get_time_us()-MCU_TO_MAC_WAKEUP_DURATION;
#else
    lv_ps_wakeup_mcu_timepoint = cal_get_time_us();
#endif
#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
    lv_ps_info_mcu_wakeup();
#endif
    return lv_ps_wakeup_mcu_timepoint;
}

#if CFG_USE_TICK_CAL
void lv_ps_cal_tick(void)
{
	uint32_t listen_interval = PS_DTIM_COUNT;
	uint32_t sleep_ms;
#if CFG_USE_STA_PS
	listen_interval = power_save_get_listen_int();
#endif

	sleep_ms = listen_interval * lv_ps_beacon_interval / 1000;
	if( lv_ps_sleep_cnt * sleep_ms >= 15000 )
	{
		fclk_cal_tick();
		lv_ps_sleep_cnt = 0;
	}
}
#endif

uint32_t lv_ps_set_bcn_int(uint32_t interval)
{
	lv_ps_beacon_interval = interval;

	return interval;
}

void lv_ps_set_bcn_data(uint64_t bcn_tsf, uint32_t bcn_int,
		uint32_t duration_of_frame, uint32_t duration_to_timestamp)
{
	uint64_t tbtt_tsf = (bcn_tsf / bcn_int) * bcn_int;

	lv_ps_bcn_tsf_field = bcn_tsf;
	lv_ps_beacon_interval = bcn_int;
	lv_ps_bcn_delay_duration = (int32_t)(bcn_tsf - tbtt_tsf - duration_to_timestamp);
	lv_ps_bcn_frame_duration = duration_of_frame;
}

void lv_ps_set_bcn_timing(uint64_t local_time, uint64_t duration_tbtt_to_rxd)
{
	lv_ps_bcn_rxd_local_time = local_time;
#if (CFG_HW_PARSER_TIM_ELEMENT == 1)
	lv_ps_tbtt_to_rxd_time = (uint32_t)duration_tbtt_to_rxd;
#else
	if (duration_tbtt_to_rxd < LV_PS_TBTT_TO_RXD_MAX)
	{
		lv_ps_tbtt_to_rxd_time = (uint32_t)duration_tbtt_to_rxd;
	}
	else
	{
		lv_ps_tbtt_to_rxd_time = lv_ps_bcn_frame_duration + lv_ps_bcn_delay_duration + LV_PS_NORMAL_BCN_RX_OFFSET;
	}
#endif
}

uint32_t lv_ps_recv_beacon(void)
{
	lv_ps_beacon_cnt_after_wakeup ++;

#if (AFTER_MISSING_STRATEGY == WAIT_UNTIL_RECVED)
	lv_ps_loss_bcn_count = 0;
#endif

	lv_ps_last_beacon_rev_timepoint = cal_get_time_us();
	return lv_ps_beacon_cnt_after_wakeup;
}

void lv_ps_recv_beacon_change(void)
{
	lv_ps_first_beacon_change_rev_timepoint = cal_get_time_us();
}

uint32_t lv_ps_set_start_flag(void)
{
	if(ps_may_sleep())
	{
	    os_null_printf("nxmac_tsf_mgt_enable:0x%x\r\n", nxmac_mac_cntrl_1_get());
		nxmac_tsf_mgt_disable_setf(0);

		lv_ps_start_flag += 1;
	}
	else
		lv_ps_start_flag = 0;

	return 0;
}

void lv_ps_clear_start_flag(void)
{
	lv_ps_start_flag = 0;
}

uint32_t lv_ps_get_start_flag(void)
{
	return lv_ps_start_flag;
}

uint32_t lv_ps_is_super_anchor_point(void)
{
	return ((lv_anchor_flag > 2) ? 1:0);
}

void lv_ps_set_anchor_point(void)
{
	lv_anchor_flag += 1;
}

void lv_ps_clear_anchor_point(void)
{
	lv_ps_beacon_cnt_after_wakeup = 0;
	lv_ps_bcn_loss_flag_after_wakeup = 0;
}

uint32_t lv_ps_is_got_anchor_point(void)
{
	return (0 != lv_anchor_flag);
}

uint32_t lv_ps_get_sleep_duration(void)
{
	/** 
	 * wakeup_offset is delta time between rosc_timer_interrupt and mcu_wake_up. 
	 * it contains hardware_boot, flash_on_delay and delay_function(immediately after mcu_wake_up) time cost.
	 * different boards use different offset value.
	*/
	if (lv_ps_current_sleep_duration > (MCU_WAKEUP_OFFSET))
	{
		return lv_ps_current_sleep_duration - MCU_WAKEUP_OFFSET;
	}
	else
	{
		os_printf("sleep duration: %d\r\n",lv_ps_current_sleep_duration);
		return 0;
	}
}

uint32_t lv_ps_beacon_missing_handler(void)
{
	lv_ps_bcn_loss_flag_after_wakeup = 1;
#if (AFTER_MISSING_STRATEGY == WAIT_UNTIL_RECVED)
	lv_ps_loss_bcn_count ++;
#endif
	lvc_general_sleep_flag = 0;
#if(CFG_HW_PARSER_TIM_ELEMENT == 1)
	power_save_increase_hw_tim_cnt();
#endif

	lv_ps_check_beacon_loss();

	if(power_save_get_listen_int() > lv_ps_dtim_period)
	{
		lv_ps_send_arp();
	}
	return 0;
}

uint32_t lv_ps_get_keep_timer_duration(void)
{
	uint32_t value;

//    bk_printf ("%d, %d\r\n",lv_ps_pre_lead_wakeup_duration,lv_ps_win_post_compensation_factor);
//	ASSERT(lv_ps_pre_lead_wakeup_duration);
	value = (lv_ps_pre_lead_wakeup_duration + PS_KEEP_TIMER_VALID_DURATION_MS * 1000
				+ lv_ps_win_post_compensation_factor * CELL_DURATION
				- DURATION_WAKEUP_STABILIZATION_US) / 1000;
	return value;
}

#define LVPS_DURA_CALC_DEBUG	0

#if LVPS_DURA_CALC_DEBUG
#define LVPS_DATA_TBL_SIZE	64
struct lvps_dura_calc_s {
#define LVPS_F_LEAD_OVER_WKUP_TIME	0x10
#define LVPS_F_TSF_OVER_BCN_INT		0x20
#define LVPS_F_DURATION_ZERO		0x40
#define LVPS_F_WAIT_BCMC		0x80
	uint32_t flags;
	uint32_t pri_tagt;
	uint32_t tot_time;
	uint32_t pri_tbtt;
	uint32_t post_tbtt;
	int32_t bcn_delay;
	uint32_t tbtt_rxd;
	uint32_t post_rxd;
	int32_t next_lead;
};
struct lvps_dura_calc_s lvps_dura_data[LVPS_DATA_TBL_SIZE];
uint32_t lvps_dura_index = 0;
uint32_t lvps_debug_count = 0;
int32_t g_prev_duration_target_lead = 8000;

void dump_lvps_dura_data(void)
{
	int i;
	struct lvps_dura_calc_s *p_dura_calc_data;
	uint32_t pri_tbtt;
	uint32_t tot_cnt = 0, devi_cnt = 0;
	uint32_t tot_sum = 0, devi_sum = 0;
	uint32_t tot_avg = 0, devi_avg = 0;
	uint32_t tot_max = 0, tot_min = 0xFFFFFFFF;
	uint32_t devi_max = 0, devi_min = 0xFFFFFFFF;
	uint32_t lead_devi;
	int32_t lead_devi_orig;

	for (i = 0; i < LVPS_DATA_TBL_SIZE; i++) {
		p_dura_calc_data = &lvps_dura_data[i];
		pri_tbtt = p_dura_calc_data->tot_time - p_dura_calc_data->post_tbtt;
		if (i != 0) {
			tot_cnt++;
			tot_sum += p_dura_calc_data->tot_time;
			if (tot_max < p_dura_calc_data->tot_time)
				tot_max = p_dura_calc_data->tot_time;
			if (tot_min > p_dura_calc_data->tot_time)
				tot_min = p_dura_calc_data->tot_time;
		}
		if (p_dura_calc_data->pri_tagt) {
			devi_cnt++;
			lead_devi_orig = pri_tbtt - p_dura_calc_data->pri_tagt;
			lead_devi = (lead_devi_orig <= 0)? (0 - lead_devi_orig) : lead_devi_orig;
			devi_sum += lead_devi;
			if (lead_devi > devi_max)
				devi_max = lead_devi;
			if (lead_devi < devi_min)
				devi_min = lead_devi;
		} else {
			lead_devi = 0;
			lead_devi_orig = 0;
		}
		bk_printf("=== %u: flags %08x, total %u, prit %u, tagt %u, diff %d, post %u, nlead %d;",
				i,
				p_dura_calc_data->flags,
				p_dura_calc_data->tot_time,
				pri_tbtt,
				p_dura_calc_data->pri_tagt,
				lead_devi_orig,
				p_dura_calc_data->post_tbtt,
				p_dura_calc_data->next_lead);
		bk_printf(" bcn_dly %d, post_rx %u\r\n",
				p_dura_calc_data->bcn_delay,
				p_dura_calc_data->post_rxd);
	}
	if (devi_cnt)
		devi_avg = devi_sum / devi_cnt;
	tot_avg = tot_sum / tot_cnt;

	bk_printf("=== tot: sum %u, cnt %u, max %u, min %u, avg %u\r\n",
			tot_sum, tot_cnt, tot_max, tot_min, tot_avg);
	bk_printf("=== devi: sum %u, cnt %u, max %u, min %u, avg %u\r\n",
			devi_sum, devi_cnt, devi_max, devi_min, devi_avg);
	bk_printf("==================================================\r\n");
	bk_printf("==================================================\r\n");
	bk_printf("==================================================\r\n");
}
extern int32_t g_duration_target_lead;
#endif

static void ps_lv_wifi_cb(uint64_t target_time, bool check_pass)
{
#if(CFG_LV_PS_WITH_IDLE_TICK == 1)
    if(lv_ps_get_mac_wakeup_flag() == 1)
    {
        lv_ps_clear_tx_recovery();
        power_save_dtim_wake ( MAC_ARM_WAKEUP_EN_BIT );
    }
    else
    {
        delay_us(500);//if there is no delay, watch dog reset will happen, reason is not clear
    }
#else
    lv_ps_clear_tx_recovery();
    power_save_dtim_wake ( MAC_ARM_WAKEUP_EN_BIT );
    lv_ps_wakeup_wifi = 1;
    if(check_pass)
        lv_ps_wakeup_mac_timepoint = target_time;
    else
        lv_ps_wakeup_mac_timepoint = lv_ps_wakeup_mcu_timepoint;
#endif
}

uint32_t lv_ps_calc_sleep_duration(void)
{
	uint32_t distance_2_prv_tbtt = 0;
#if LVPS_DURA_CALC_DEBUG
	uint32_t case_type = 0;
#endif
	int32_t lead_value;
	int32_t duration;
	int32_t delta_time;
	uint64_t curr_local_time;
	uint32_t listen_interval = PS_DTIM_COUNT;
#if LVPS_DURA_CALC_DEBUG
	if (lvps_dura_index == 0 && lvps_debug_count) {
		GLOBAL_INT_DECLARATION();
		GLOBAL_INT_DISABLE();
		lv_ps_enable_print = 1;
		dump_lvps_dura_data();
		lv_ps_enable_print = 0;
		GLOBAL_INT_RESTORE();
	}
	struct lvps_dura_calc_s *p_dura_calc_data = &lvps_dura_data[lvps_dura_index];
	p_dura_calc_data->flags = 0;
#endif
#if CFG_USE_STA_PS
	listen_interval = power_save_get_listen_int();
#endif

	curr_local_time = cal_get_time_us();
	if (lv_ps_beacon_cnt_after_wakeup) {
		/*The first case: recv beacon after wakeup of low voltage*/
		if (lv_ps_bcn_cont_miss_bcn_count)
			lv_ps_win_pri_compensation_factor = 1;
		else
			lv_ps_win_pri_compensation_factor = 0;
		lv_ps_win_post_compensation_factor = 0;

		distance_2_prv_tbtt = (curr_local_time - lv_ps_bcn_rxd_local_time) + lv_ps_tbtt_to_rxd_time;
#if LVPS_DURA_CALC_DEBUG
		case_type = 1;

		if (distance_2_prv_tbtt > lv_ps_beacon_interval) {
			case_type = 2;
		}

		if (lv_ps_bcn_loss_flag_after_wakeup) {
			case_type = 3;
		}
		delta_time = curr_local_time - lv_ps_wakeup_mac_timepoint;
		if ((case_type == 1 || case_type == 3)
				&& p_dura_calc_data->flags == 0
				&& !lv_ps_bcn_cont_miss_bcn_count && lvps_dura_index) {
			p_dura_calc_data->pri_tagt = g_prev_duration_target_lead;
		} else {
			p_dura_calc_data->pri_tagt = 0;
		}
		p_dura_calc_data->post_tbtt = distance_2_prv_tbtt;
#endif

		lead_value = lvc_get_lead_duration();
		lv_ps_pre_lead_wakeup_duration = lead_value + LEAD_FORCE_TIME +
						lv_ps_win_pri_compensation_factor * CELL_DURATION;
		distance_2_prv_tbtt = distance_2_prv_tbtt % lv_ps_beacon_interval;
		duration = (32 * (int32_t)(listen_interval * lv_ps_beacon_interval - distance_2_prv_tbtt
									- lv_ps_pre_lead_wakeup_duration) / 1000);

		lvc_general_sleep_flag = 1;
#if LVPS_DURA_CALC_DEBUG
		p_dura_calc_data->bcn_delay = lv_ps_bcn_delay_duration;
		p_dura_calc_data->tbtt_rxd = lv_ps_tbtt_to_rxd_time;
		p_dura_calc_data->post_rxd = curr_local_time - lv_ps_bcn_rxd_local_time;
#endif
#if (AFTER_MISSING_STRATEGY == WAIT_ONCE)
		lv_ps_bcn_has_been_waiting = 0;
#endif
		lv_ps_bcn_cont_miss_bcn_count = 0;
	} else {
		LV_PSC_PRT("\n");
#if LVPS_DURA_CALC_DEBUG
		/*The case: beacon is missing all the while after wakeup of low voltage*/
		case_type = 4;
#endif
		delta_time = curr_local_time - lv_ps_wakeup_mac_timepoint;
		if (lv_ps_pre_lead_wakeup_duration > delta_time) {
			distance_2_prv_tbtt = 0;
#if LVPS_DURA_CALC_DEBUG
			p_dura_calc_data->flags |= LVPS_F_LEAD_OVER_WKUP_TIME;
#endif
		} else {
			distance_2_prv_tbtt = delta_time - lv_ps_pre_lead_wakeup_duration;
		}

#if LVPS_DURA_CALC_DEBUG
		p_dura_calc_data->post_tbtt = distance_2_prv_tbtt;
		if (distance_2_prv_tbtt >= lv_ps_beacon_interval) {
			p_dura_calc_data->flags |= LVPS_F_TSF_OVER_BCN_INT;
		}
#endif
		distance_2_prv_tbtt = distance_2_prv_tbtt % lv_ps_beacon_interval;

		lead_value = lvc_get_lead_duration();

#if (AFTER_MISSING_STRATEGY == WAIT_UNTIL_RECVED)
		if (lv_ps_loss_bcn_count < 5) {
			lv_ps_win_pri_compensation_factor = lv_ps_loss_bcn_count;
		} else {
			lv_ps_win_pri_compensation_factor = 10;
            bk_printf("bcn loss cnt = %d, line = %d\r\n",lv_ps_loss_bcn_count,__LINE__);
		}
		lv_ps_win_post_compensation_factor = lv_ps_win_pri_compensation_factor;

		lv_ps_pre_lead_wakeup_duration = lead_value + LEAD_FORCE_TIME + lv_ps_win_pri_compensation_factor * CELL_DURATION;
		duration = (32 * (int32_t)(DTIM_COUNT_WHEN_MISSING_BEACON * lv_ps_beacon_interval - distance_2_prv_tbtt
										- lv_ps_pre_lead_wakeup_duration) / 1000);

#elif (AFTER_MISSING_STRATEGY == WAIT_ONCE_ON_CONT_LOSS)
		if (lv_ps_bcn_cont_miss_bcn_count == 0) {
			lv_ps_win_pri_compensation_factor = 2;
			lv_ps_win_post_compensation_factor = 2;
			lv_ps_pre_lead_wakeup_duration = lead_value + LEAD_FORCE_TIME + lv_ps_win_pri_compensation_factor * CELL_DURATION;
			duration = (32 * (int32_t)(listen_interval * lv_ps_beacon_interval - distance_2_prv_tbtt
										- lv_ps_pre_lead_wakeup_duration) / 1000);

		PS_DBG("%d,%d,%d,%d,%d line = %d\r\n", duration, distance_2_prv_tbtt,lv_ps_pre_lead_wakeup_duration,lead_value,lv_ps_win_pri_compensation_factor,__LINE__);
		} else if (lv_ps_bcn_cont_miss_bcn_count == 1) {
			lv_ps_win_pri_compensation_factor = 4;
			lv_ps_win_post_compensation_factor = 4;
			lv_ps_pre_lead_wakeup_duration = lead_value + LEAD_FORCE_TIME + lv_ps_win_pri_compensation_factor * CELL_DURATION;
			duration = (32 * (DTIM_COUNT_WHEN_MISSING_BEACON * lv_ps_beacon_interval - distance_2_prv_tbtt
										- lv_ps_pre_lead_wakeup_duration) / 1000);

			PS_DBG("%d,%d,%d,%d,%d line = %d\r\n", duration, distance_2_prv_tbtt,lv_ps_pre_lead_wakeup_duration,lead_value,lv_ps_win_pri_compensation_factor,__LINE__);
		} else {
			if (lv_ps_bcn_cont_miss_bcn_count & 0x1) {
				lv_ps_win_pri_compensation_factor = 2;
				lv_ps_win_post_compensation_factor = 2;
			} else {
				lv_ps_win_pri_compensation_factor = 4;
				lv_ps_win_post_compensation_factor = 0;
			}
			lv_ps_pre_lead_wakeup_duration = lead_value + LEAD_FORCE_TIME + lv_ps_win_pri_compensation_factor * CELL_DURATION;
			duration = (32 * (int32_t)(listen_interval * lv_ps_beacon_interval - distance_2_prv_tbtt
										- lv_ps_pre_lead_wakeup_duration) / 1000);

		PS_DBG("%d,%d,%d,%d,%d line = %d\r\n", duration, distance_2_prv_tbtt,lv_ps_pre_lead_wakeup_duration,lead_value,lv_ps_win_pri_compensation_factor,__LINE__);
		}
#elif (AFTER_MISSING_STRATEGY == WAIT_ONCE)
		lv_ps_win_pri_compensation_factor = 4;
		lv_ps_win_post_compensation_factor = 4;
		lv_ps_pre_lead_wakeup_duration = lead_value + LEAD_FORCE_TIME + lv_ps_win_pri_compensation_factor * CELL_DURATION;
		if (0 == lv_ps_bcn_has_been_waiting) {
			duration = (32 * (int32_t)(DTIM_COUNT_WHEN_MISSING_BEACON * lv_ps_beacon_interval - distance_2_prv_tbtt
										- lv_ps_pre_lead_wakeup_duration) / 1000);
			lv_ps_bcn_has_been_waiting = 1;
		} else {
			duration = (32 * (int32_t)(listen_interval * lv_ps_beacon_interval - distance_2_prv_tbtt
										- lv_ps_pre_lead_wakeup_duration) / 1000);
			lv_ps_bcn_has_been_waiting = 0;
		}
#elif (AFTER_MISSING_STRATEGY == NO_WAIT)
		lv_ps_win_pri_compensation_factor = lv_ps_bcn_cont_miss_bcn_count + 1;
		lv_ps_win_post_compensation_factor = lv_ps_bcn_cont_miss_bcn_count * 2;
		lv_ps_pre_lead_wakeup_duration = lead_value + LEAD_FORCE_TIME + lv_ps_win_pri_compensation_factor * CELL_DURATION;
		duration = (32 * (int32_t)(listen_interval * lv_ps_beacon_interval - distance_2_prv_tbtt
							- lv_ps_pre_lead_wakeup_duration) / 1000);
#endif
		lvc_general_sleep_flag = 0;
		lv_ps_bcn_cont_miss_bcn_count++;

#if LVPS_DURA_CALC_DEBUG
		p_dura_calc_data->bcn_delay = 0;
		p_dura_calc_data->tbtt_rxd = 0;
		p_dura_calc_data->post_rxd = 0;
		p_dura_calc_data->pri_tagt = 0;
#endif
	}

	if (duration < 0) {
		os_printf("duration: %d\r\n",duration);
		duration = 0;
#if LVPS_DURA_CALC_DEBUG
		p_dura_calc_data->flags |= LVPS_F_DURATION_ZERO;
#endif
	}

	if (mcu_ps_is_on()) {
		LV_PS_ELEMENT_ENV *elem = os_malloc(sizeof(LV_PS_ELEMENT_ENV));
		memset(elem,0,sizeof(LV_PS_ELEMENT_ENV));
		elem->lv_target_time = curr_local_time + ((62*duration + (duration>>1))>>1);
		elem->lv_type = LV_TYPE_WIFI;
		elem->object_cb = ps_lv_wifi_cb;
		lv_ps_push_element(elem);
	}
#if LVPS_DURA_CALC_DEBUG
	g_prev_duration_target_lead = g_duration_target_lead;
	p_dura_calc_data->next_lead = lv_ps_pre_lead_wakeup_duration;
	p_dura_calc_data->flags |= case_type;
	p_dura_calc_data->tot_time = delta_time;
	lvps_dura_index = (++lvps_dura_index) % LVPS_DATA_TBL_SIZE;
	lvps_debug_count++;
#endif

	return duration;
}
extern UINT8 power_save_set_all_vif_prevent_sleep ( UINT32 prevent_bit );

static uint32_t lv_ps_check_beacon_loss(void)
{
    uint64_t current_timepoint = 0, loss_during;

    if(lv_ps_get_start_flag() == 0)
        return 0;

#if (AFTER_MISSING_STRATEGY == WAIT_UNTIL_RECVED)
    if(lv_ps_loss_bcn_count <= 1)
        return 0;
#endif

    current_timepoint = cal_get_time_us();

    if(current_timepoint > lv_ps_last_beacon_rev_timepoint) {
        loss_during = current_timepoint - lv_ps_last_beacon_rev_timepoint;
    } else {
//        loss_during = (0xffffffffffffffffu - lv_ps_last_beacon_rev_timepoint) + current_timepoint;
        ASSERT(0);
    }
    loss_during = loss_during / 1000000;

    //os_printf("loss: %u, %u\r\n", loss_during, LV_PS_BEACON_LOSS_TIME_S);
    if (loss_during >= LV_PS_BEACON_LOSS_TIME_S) {
#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
        lv_ps_info_disconnect();
#endif
        ps_send_connection_loss();
        os_printf("low voltage detect beacon loss\r\n");

        return 1;
    }

    return 0;
}

uint32_t lv_ps_check_beacon_changed(void)
{
    uint64_t current_timepoint = 0, loss_during;
    current_timepoint = cal_get_time_us();

    if(current_timepoint > lv_ps_first_beacon_change_rev_timepoint) {
        loss_during = current_timepoint - lv_ps_first_beacon_change_rev_timepoint;
    } else {
//        loss_during = (0xffffffffffffffffu - lv_ps_last_beacon_rev_timepoint) + current_timepoint;
        ASSERT(0);
    }
    loss_during = loss_during / 1000000;

    //os_printf("loss: %u, %u\r\n", loss_during, LV_PS_BEACON_LOSS_TIME_S);
    if (loss_during >= LV_PS_BEACON_LOSS_TIME_S) {
        return 1;
    }

    return 0;
}


/*******************************************************************************
* LV_PS SLEEP_TRIGGER_TIMER
*******************************************************************************/
void lv_ps_sleep_trigger_timer_stop(void)
{
	OSStatus err;

	if (rtos_is_oneshot_timer_running(&lv_ps_trigger_timer)) {
		err = rtos_stop_oneshot_timer(&lv_ps_trigger_timer);
		ASSERT(kNoErr == err);
	}
	lv_ps_trigger_timer_status = 0;
}

void lv_ps_sleep_trigger_timer_real_handler(void)
{
	lv_ps_sleep_trigger_timer_stop();
	extern void bmsg_null_sender(void);
	bmsg_null_sender();
}

void lv_ps_sleep_trigger_timer_init(void)
{
	UINT32 err;

	if (rtos_is_oneshot_timer_init(&lv_ps_trigger_timer)) {
		lv_ps_sleep_trigger_timer_real_handler();
		err = rtos_deinit_oneshot_timer(&lv_ps_trigger_timer);
		ASSERT(kNoErr == err);
	}

	err = rtos_init_oneshot_timer(&lv_ps_trigger_timer,
								  10,
								  (timer_2handler_t)lv_ps_sleep_trigger_timer_real_handler,
								  NULL,
								  NULL);
	ASSERT(kNoErr == err);
}

void lv_ps_sleep_trigger_timer_start(void)
{
	OSStatus err;

	if (rtos_is_oneshot_timer_init(&lv_ps_trigger_timer) && lv_ps_trigger_timer_status == 0) {
		lv_ps_trigger_timer_status = 1;
		err = rtos_start_oneshot_timer(&lv_ps_trigger_timer);
		ASSERT(kNoErr == err);
	}
}

#if(CFG_LV_PS_WITH_IDLE_TICK == 1)
void lv_ps_set_mac_wakeup_flag(UINT32 flag)
{
	lv_ps_mac_wakeup_flag = flag;
}

UINT32 lv_ps_get_mac_wakeup_flag(void)
{
	return lv_ps_mac_wakeup_flag;
}

void lv_ps_set_keep_timer_more(UINT32 value)
{
	lv_ps_keep_timer_more = value;
}

UINT32 lv_ps_get_keep_timer_more(void)
{
	return lv_ps_keep_timer_more;
}

UINT32 lv_ps_calc_rosc_period( UINT32 sleep_tick)
{
	static UINT32 sleep_time;
	UINT32 sleep_idle;
	UINT32 sleep_count;
	static uint64_t sleep_time_cal;
	static uint64_t curr_time;

	sleep_idle = 32*BK_TICKS_TO_MS(sleep_tick) - MCU_WAKEUP_OFFSET;

	/*if mac wakeup, it is necessary to calculate next mac wakeup point*/
	if(lv_ps_get_mac_wakeup_flag() == 1)
	{
		lv_ps_calc_sleep_duration();
		sleep_time = lv_ps_get_sleep_duration();
		sleep_time_cal = cal_get_time_us(); /*record current us*/
	}
	curr_time = cal_get_time_us();

	if(sleep_time > 32*(curr_time - sleep_time_cal)/1000)
	{
		sleep_time -= 32*(curr_time - sleep_time_cal)/1000;
		sleep_time_cal = curr_time;
	}
	else
	{
		sleep_time = 0;
	}

	/*take the minimum value as sleep duration*/
	sleep_count = _min(sleep_time,sleep_idle);

	/*if waked by ditm, then it is necessary to wakeup mac*/
	if(sleep_time <= sleep_count + MCU_SLEEP_DURATION_MIN * 32)
	{
		lv_ps_set_mac_wakeup_flag(1);
	}
	else
	{
		lv_ps_set_mac_wakeup_flag(0);
	}

	/*if this time need to wake mac but sleep duration is too short for mcu sleep, then do not sleep mcu and wakeup mac immediately, and keep 10ms more */
	if((sleep_count < MCU_SLEEP_DURATION_MIN * 32)&&(sleep_count == sleep_time))
	{
		lv_ps_set_keep_timer_more(KEEP_MORE_FOR_IDLE);
		lv_ps_clear_tx_recovery();
		power_save_dtim_wake ( MAC_ARM_WAKEUP_EN_BIT );
		return 0;
	}
	else if((sleep_count < MCU_SLEEP_DURATION_MIN * 32))
	{
		os_printf("rosc: %d %d %d\r\n", sleep_time,sleep_idle,MCU_SLEEP_DURATION_MIN * 32);
		return 0;
	}

	return sleep_count;
}

#else
UINT32 lv_ps_calc_rosc_period( UINT32 sleep_tick)
{
    UINT32 sleep_time;

    if (cal_get_time_us() > (lv_ps_target_time - MCU_SLEEP_DURATION_MIN*1000)) {
        return 0;
    }

    lv_ps_current_sleep_duration = 32 * (lv_ps_target_time -cal_get_time_us())/1000;
    sleep_time = lv_ps_get_sleep_duration();

    if(sleep_time < MCU_SLEEP_DURATION_MIN * 32)
    {
        os_printf("rosc: %d %d\r\n", sleep_time,MCU_SLEEP_DURATION_MIN * 32);
        return 0;
    }
    return sleep_time;
}
#endif

bool lv_ps_rosc_timer_setting( UINT32 sleep_tick)
{
	UINT32 rosc_period;
	sctrl_disable_rosc_timer();
	rosc_period = lv_ps_calc_rosc_period(sleep_tick);
	if(rosc_period == 0)
	{
		return false;
	}
	sctrl_enable_rosc_timer(rosc_period);
	return true;
}

void lv_ps_force_software_beacon(void)
{
#if(CFG_HW_PARSER_TIM_ELEMENT == 1)
	nxmac_gen_int_enable_set(nxmac_gen_int_enable_get() & ~ NXMAC_TIM_SET_BIT);
	lvc_calc_g_bundle_reset();
#endif
}

static void insertionSort(struct list_head *head)
{
    struct list_head *i, *j, *next;

    for (i = head->next->next; i != head; i = next) {
        next = i->next;
        LV_PS_ELEMENT_ENV *i_entry = list_entry(i, LV_PS_ELEMENT_ENV, node);

        for (j = i->prev; j != head; j = j->prev) {
            LV_PS_ELEMENT_ENV *j_entry = list_entry(j,LV_PS_ELEMENT_ENV, node);
            if (j_entry->lv_target_time <= i_entry->lv_target_time)
                break;
        }

        list_del(i);
        list_add_tail(i, j->next);
    }
}

bool lv_ps_push_element(LV_PS_ELEMENT_ENV *elem)
{
    LV_PS_ELEMENT_ENV *node;
    LIST_HEADER_T *list = &lv_element;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    uint8_t size = list_size(list);
    if (size > 1) {
        bk_printf("assert:lv_ps_push_element %d\r\n",size);
    }

    if (list_empty(list)) {
        list_add_head(&elem->node,&lv_element);
    } else {
        node = (LV_PS_ELEMENT_ENV *)list->next;

        if (node->lv_target_time < elem->lv_target_time) {
            list_add_tail(&elem->node,&lv_element);
        } else {
            list_add_head(&elem->node,&lv_element);
        }
        insertionSort(&lv_element);
    }
    GLOBAL_INT_RESTORE();
    return true;
}

LV_PS_ELEMENT_ENV *lv_ps_pop_element(void)
{
    LIST_HEADER_T *tmp;
    LIST_HEADER_T *pos;
    LV_PS_ELEMENT_ENV *node;

    LIST_HEADER_T *list = &lv_element;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    node = NULLPTR;
    list_for_each_safe(pos, tmp, list)
    {
        list_del(pos);
        node = list_entry(pos, LV_PS_ELEMENT_ENV, node);
        break;
    }
    GLOBAL_INT_RESTORE();
    return node;
}

void lv_ps_element_check_pass(void)
{
    LIST_HEADER_T *list = &lv_element;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    if (!list_empty(list)) {
        LV_PS_ELEMENT_ENV *node = (LV_PS_ELEMENT_ENV *)list->next;
        int32_t diff = (int32_t)(node->lv_target_time - cal_get_time_us());

        if ((node->lv_type == LV_TYPE_BT && diff < 5000) || (node->lv_type == LV_TYPE_WIFI && diff < 0)) {
            node = (LV_PS_ELEMENT_ENV *)lv_ps_pop_element();
            if (node->object_cb) {
                node->object_cb(node->lv_target_time, true);
            }
            os_free(node);
        }
    }
    GLOBAL_INT_RESTORE();
}

uint8_t lv_ps_element_next(void)
{
    LIST_HEADER_T *list = &lv_element;
    uint8_t ele = LV_TYPE_NONE;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    if (!list_empty(list)) {
        LV_PS_ELEMENT_ENV *node = (LV_PS_ELEMENT_ENV *)list->next;
        ele = node->lv_type;
    }
    GLOBAL_INT_RESTORE();
    return ele;
}

void lv_ps_element_bt_del(void)
{
    LV_PS_ELEMENT_ENV *node;
    LIST_HEADER_T *list = &lv_element;
    LIST_HEADER_T *tmp;
    LIST_HEADER_T *pos;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    list_for_each_safe(pos, tmp, list)
    {
        node = list_entry(pos, LV_PS_ELEMENT_ENV, node);
        if (node->lv_type == LV_TYPE_BT) {
            list_del(pos);
            os_free(node);
        }
    }
    GLOBAL_INT_RESTORE();
}


// eof

