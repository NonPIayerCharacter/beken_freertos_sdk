/**
****************************************************************************************
*
* @file rwip_int.h
*
* @brief RW IP internal SW main module
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef _RWIP_INT_H_
#define _RWIP_INT_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the RW IP stacks/modules
 *
 * This module contains the primitives that allow an application accessing and running the
 * RW IP protocol stacks / modules.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"          // stack configuration

#include <stdint.h>               // standard integer definitions
#include <stdbool.h>              // standard boolean definitions


/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
/// structure that represents ISO timer
typedef struct rwip_iso_timer_
{
    /// Target BTS time (in us)
    uint32_t           tgt_bts[RWIP_ISO_TIMER_FIFO_DEPTH];
    /// Current ISO timer index
    uint8_t            curr_idx;
    /// Next ISO timer index
    uint8_t            next_idx;
    /// Number of ISO timers programmed
    uint8_t            prog_nb;
} rwip_iso_timer_t;
#endif // (BLE_EMB_PRESENT && BLE_ISO_PRESENT)

/// RWIP Environment structure
struct rwip_env_tag
{
    #if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
    rwip_iso_timer_t iso_timer;
    #endif // (BLE_EMB_PRESENT && BLE_ISO_PRESENT)

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Arbiter target timer  (integer part, in half slots)
    rwip_time_t          timer_arb_target;
    /// Alarm target timer (integer part, in half slots)
    uint32_t          timer_alarm_target;
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Common target timer (in half slots)
    uint32_t          timer_common_target;
    /// Last Sampled time (used for time conversion)
    rwip_time_t       last_samp_time;

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Contains sleep duration accumulated timing error (32kHz: 1/2 half us | 32.768kHz: 1/256 half-us)
    uint32_t          sleep_acc_error;
    /// Power_up delay (in LP clock cycle unit, depends on Low power clock frequency)
    uint32_t          lp_cycle_wakeup_delay;
    /// Duration of sleep and wake-up algorithm (depends on CPU speed) expressed in half us.
    uint16_t          sleep_algo_dur;
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Prevent sleep bit field
    uint16_t          prevent_sleep;
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// External wake-up support
    bool              ext_wakeup_enable;
    #if (!BLE_EMB_PRESENT)
    /// BTS sampling clock half microseconds residual (0 or 1)
    uint8_t           samp_hus_residual;
    #endif // (!BLE_EMB_PRESENT)
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    uint32_t irq_mask;
};


/*
 * GLOBAL DEFINITIONS
 ****************************************************************************************
 */

/// RW SW environment
extern struct rwip_env_tag rwip_env;


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 * Initialization of the RW IP Common core driver
 *
 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 */
void rwip_driver_init(uint8_t init_type);


///@} ROOT

#endif // _RWIP_INT_H_
