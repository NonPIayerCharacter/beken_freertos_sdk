#ifndef _LOW_VOLTAGE_PS_H_
#define _LOW_VOLTAGE_PS_H_

#include "include.h"
#include "uart_pub.h"
#include "doubly_list.h"
#include "manual_ps_pub.h"

#define LV_PS_DEBUG                1

#if LV_PS_DEBUG
#define LV_PS_PRT                 os_printf
#else
#define LV_PS_PRT                 os_null_printf
#endif

#define DURATION_AHEAD_OF_TBTT_US         (5 * 1000)  /*uS*/ /*obsoleting macro*/
#define DURATION_BEACON_INTERVAL_US       (100 * 1024)
#define DURATION_WAKEUP_STABILIZATION_US  (0 * 1000)

#define CELL_DURATION                     (1000)  /* US*/
#define DTIM_COUNT_WHEN_MISSING_BEACON    (1)

#define LEAD_FORCE_TIME                     (0000)  /* US*/
#define LV_PS_TBTT_TO_RXD_MAX		(30 * 1000) /* US */
#define LV_PS_NORMAL_BCN_RX_OFFSET	(400)	    /* US */


#define WAIT_UNTIL_RECVED          (0)
#define WAIT_ONCE                  (1)
#define NO_WAIT                    (2)
#define WAIT_ONCE_ON_CONT_LOSS     (3)
#if(CFG_HW_PARSER_TIM_ELEMENT == 1)
#define AFTER_MISSING_STRATEGY    NO_WAIT
#else
#define AFTER_MISSING_STRATEGY    NO_WAIT
#endif

#define PS_KEEP_TIMER_VALID_DURATION_MS               (8)

#define MCU_SLEEP_DURATION_MIN     (4)  /* ms*/

#if (CFG_SOC_NAME == SOC_BK7238)
#define MCU_WAKEUP_OFFSET          (48)  /* VALUE/32 ms, related to flash_on_delay configuration*/
#elif (CFG_SOC_NAME == SOC_BK7231N)
#define MCU_WAKEUP_OFFSET          (16)  /* VALUE/32 ms, related to flash_on_delay configuration*/
#else
#define MCU_WAKEUP_OFFSET          (0)  /* VALUE/32 ms, related to flash_on_delay configuration*/
#endif

#if(CFG_LV_PS_WITH_IDLE_TICK == 1)
#define KEEP_MORE_FOR_IDLE              (10)  /* ms , sometimes if wakeup by idle and want to sleep again ,next wakeup need to wakeup mac,but the duration is not enough, for example < 4ms, just do not sleep for this time and wakeup mac automatically*/
#define MCU_TO_MAC_WAKEUP_DURATION      (1450)  /* us , if takes idle into consideration, it may no need to wakeup mcu before mac wakeup, so unable to get mcu wakeup timepoint, it can be calculate by mac_wakeup_point - mcu_to_mac_wakeup duration,bk7238 takes 1.4ms from mcu wakeup to mac wakeup*/
#endif

#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
/// management
typedef struct lv_ps_info_mgmt
{
    /// print management
    bool print_enable;
    uint32_t print_period;
    uint64_t total_time;
    uint32_t stat_index;
    uint64_t stat_start_time;
    uint64_t stat_end_time;
} LV_PS_PRINT_MGMT;

/// beacon statistic
typedef struct lv_ps_beacon_stat
{
    /// basic record
    uint32_t interval;
    uint32_t length_m;
    uint64_t frame_dura_m;

    /// receive by software
    uint32_t recv_count;
    int32_t ap_bcn_delay_m;
    uint64_t ap_bcn_delay_v;
    uint32_t tbtt_to_rxd_m;
    uint64_t tbtt_to_rxd_v;

    /// receive by hardware
    uint32_t tim_count;

    /// beacon lost
    uint32_t loss_count;
    uint32_t loss_cnt_times;

    /// disconnect
    uint32_t disc_count;
    uint64_t reconn_tcost;

} LV_PS_BEACON_STAT;

/// runtime statistic
typedef struct lv_ps_runtime_stat
{
    /// mcu runtime
    uint64_t mcu_running_time;
    uint32_t mcu_sleep_count;

    /// rf runtime
    uint64_t rf_running_time;
    uint32_t rf_sleep_count;
    uint32_t rf_sleep_retry_count;
    uint64_t rf_to_tim;
    uint64_t rf_to_bcn;

    /// execution runtime
    uint64_t rf_init_tcost;
    uint64_t rf_sleep_tcost;
    uint64_t sleep_tcost_total;
    uint64_t wakeup_tcost_total;
} LV_PS_RUNTIME_STAT;

typedef struct lv_ps_info_st
{
    /// for variance calculate
    int32_t ap_bcn_delay_m_save;
    uint32_t tbtt_to_rxd_m_save;

    struct lv_ps_info_mgmt     mgmt;
    struct lv_ps_beacon_stat    beacon;
    struct lv_ps_runtime_stat   runtime;
} LV_PS_INFO_ST;

void lv_ps_info_print_switch(bool flag, uint32_t period);
#endif

enum lv_type_t
{
    LV_TYPE_NONE    = 0,
    LV_TYPE_BT      = 1,
    LV_TYPE_WIFI    = 2,

    LV_TYPE_MAX,
};

typedef void (*lv_ps_object_cb)(uint64_t target_time, bool check_pass);

typedef struct lv_ps_element_env
{
    LIST_HEADER_T       node;
    uint8_t             lv_type;
    uint64_t            lv_target_time;
    //need todo after wakeup
    lv_ps_object_cb     object_cb;

}LV_PS_ELEMENT_ENV;

extern bool lv_ps_push_element(LV_PS_ELEMENT_ENV *elem);
extern LV_PS_ELEMENT_ENV *lv_ps_pop_element(void);
extern void lv_ps_element_check_pass(void);
extern uint8_t lv_ps_element_next(void);
extern void lv_ps_element_bt_del(void);
extern void lv_ps_bt_wakeup(void);

extern uint64_t lv_ps_wakeup_mac_timepoint;
extern uint32_t lv_ps_beacon_cnt_after_wakeup ;
#if(CFG_HW_PARSER_TIM_ELEMENT == 1)
extern uint64_t lv_ps_tbtt_local;
extern int32_t lv_ps_tbtt_local_remainder;
#endif
extern uint8_t lv_ps_rf_pre_pwr_down;
extern uint8_t lv_ps_rf_reinit;
extern uint8_t lv_ps_wakeup_wifi;
extern uint16_t lv_ps_dtim_period;
extern PS_DEEP_WAKEUP_WAY lv_ps_wake_up_way;

extern void lv_ps_init(void);
extern uint32_t lv_ps_get_keep_timer_duration(void);
void lv_ps_set_bcn_data(uint64_t bcn_tsf, uint32_t bcn_int,
uint32_t duration_of_frame, uint32_t duration_to_timestamp);
void lv_ps_set_bcn_timing(uint64_t local_time, uint64_t duration_tbtt_to_rxd);
uint32_t lv_ps_get_sleep_duration(void);
void lv_ps_clear_anchor_point(void);
uint64_t lv_ps_wakeup_set_timepoint(void);
uint32_t lv_ps_calc_sleep_duration(void);
bool lv_ps_rosc_timer_setting( UINT32 sleep_tick);
void lv_ps_force_software_beacon(void);
uint32_t lv_ps_is_got_anchor_point(void);
uint32_t lv_ps_is_super_anchor_point(void);
uint32_t lv_ps_beacon_missing_handler(void);
uint32_t lv_ps_get_start_flag(void);
void lv_ps_clear_start_flag(void);
uint32_t lv_ps_set_start_flag(void);
uint32_t lv_ps_recv_beacon(void);
void lv_ps_set_anchor_point(void);
uint32_t lv_ps_set_bcn_int(uint32_t interval);
bool lv_ps_sleep_check( UINT32 sleep_tick);
void lv_ps_sleep(void);
void lv_ps_recv_beacon_change(void);
uint32_t lv_ps_check_beacon_changed(void);

#if CFG_USE_TICK_CAL
void lv_ps_cal_tick(void);
#endif
UINT32 lv_ps_check_tx_recovery(void);
void lv_ps_set_tx_recovery(void);
void lv_ps_check_11b(void);
void lv_ps_send_arp(void);
void lv_ps_update_arp_send_time(void);
void lv_ps_keepalive_arp_tx(void);
void lv_ps_sleep_trigger_timer_stop(void);
void lv_ps_sleep_trigger_timer_init(void);
void lv_ps_sleep_trigger_timer_start(void);

#if(CFG_LV_PS_WITH_IDLE_TICK == 1)
void lv_ps_set_mac_wakeup_flag(UINT32 flag);
UINT32 lv_ps_get_mac_wakeup_flag(void);
void lv_ps_set_keep_timer_more(UINT32 value);
UINT32 lv_ps_get_keep_timer_more(void);
#endif

#endif
// eof

