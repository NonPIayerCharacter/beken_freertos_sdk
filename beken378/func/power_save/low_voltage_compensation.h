#ifndef _LOW_VOLTAGE_COMPENSATION_H_
#define _LOW_VOLTAGE_COMPENSATION_H_

#include "include.h"
#include "uart_pub.h"

#define LV_PSC_DEBUG                0
#define LV_PS_BUNDLE_DEBUG          0

#if LV_PSC_DEBUG
#define LV_PSC_PRT                 os_printf
#define LV_PSC_NULL_PRT            os_null_printf
#else
#define LV_PSC_PRT                 os_null_printf
#define LV_PSC_NULL_PRT            os_null_printf
#endif


#define SMOOTHED_LEAD_VALUE_CALC   1

#if LV_PS_BUNDLE_DEBUG
#if SMOOTHED_LEAD_VALUE_CALC
#define BUNDLE_MAX_COUNT           32   /* a power of 2*/
#else
#define BUNDLE_MAX_COUNT           8   /* a power of 2*/
#endif
#else
#define BUNDLE_MAX_COUNT           8   /* a power of 2*/
#endif

#if SMOOTHED_LEAD_VALUE_CALC
#define ITEM_COUNT_IN_BUNDLE       1
#else
#define ITEM_COUNT_IN_BUNDLE       4
#endif
#define GENERAL_BEACON_INTERVAL_US (100 * 1024)

#define LVC_SUCCESS                (0)
#define LVC_FAILURE                (1)
#define LVC_CELL_FULL              (2)

typedef struct _lv_item_
{
	uint32_t wakeup_time_up_to_bcn;
	int32_t wakeup_time_up_to_tbtt;
#if SMOOTHED_LEAD_VALUE_CALC
	int32_t duration_clock_drift;
#endif
}BCN_ITEM_T;

typedef struct _lv_bundle_
{
	uint32_t item_index;
	BCN_ITEM_T lv_item[ITEM_COUNT_IN_BUNDLE];

#if LV_PS_BUNDLE_DEBUG
	int32_t duration_of_lead;
	int32_t duration_of_setting;
	int32_t duration_of_compensation;
	int32_t drift_spur_detected;
#endif
}BCN_BUNDLE_T;

extern uint64_t g_dpll_lock_delay;

uint32_t lvc_recv_bcn_handler(uint64_t tsf, uint32_t tsf_offset);
int32_t lvc_get_lead_duration(void);
#if(CFG_HW_PARSER_TIM_ELEMENT == 1)
uint32_t lvc_calc_clock_drift_md(int32_t smoothed_clock_drift);
void lvc_calc_g_bundle_reset(void);
bool lvc_calc_g_bundle_ready(void);
uint32_t lvc_recv_bcn_handler_tim(void);
void lvc_apply_clock_drift_tim(void);
void lvc_update_clock_drift_tim(uint64_t delta_wakeup, uint64_t wakeup_to_tbtt);
#endif

#endif // _LOW_VOLTAGE_COMPENSATION_H_
// eof

