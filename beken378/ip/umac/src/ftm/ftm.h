/**
 ****************************************************************************************
 *
 * @file ftm.h
 *
 * @brief Functions and structures used by the UMAC FTM (Fine Timing Measurement) module.
 *
 * Copyright (C) RivieraWaves 2011-2021
 *
 ****************************************************************************************
 */

#ifndef _FTM_H_
#define _FTM_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "ke_task.h"
#include "sta_mgmt.h"
#include "scanu.h"
#include "scan.h"

#if NX_FTM_INITIATOR

/** @defgroup FTM FTM
 * @ingroup UMAC
 * @brief Declaration of all structures and functions used by the FTM module.
 * @{
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Number of FTM Task instances
#define FTM_IDX_MAX        (1)

/// Timeout to receive acknowledge of the action frame
#define DEFAULT_ACTION_FRAME_TIMEOUT     (5000)

/// Timeout to receive the second FTM measurement
#define DEFAULT_MEAS_FRAME_TIMEOUT       (2000)

/// Timeout to schedule measurement request
#define DEFAULT_MEAS_REQUEST_TIME        (100)

/// Time to remain on a dedicated channel
#define DEFAULT_REMAIN_ON_CHANNEL_TIME   (1000)

/// Timeout to send the FTM measurement
#define DEFAULT_MEAS_FRAME_SENT_TIMEOUT  (1000)

/// Max number of consecutive RoC allowed
#define MAX_ROC_ATTEMPTS  (5)

/// SIFS Duration (in ps)
#define FTM_SIFS_DURATION_IN_PS          (16 * 1000000)

/// Internal code identifying a FTM action frame
enum ftm_frame_type
{
    /// Initial frame Request
    FTM_INITIAL_FRAME_REQ = 0,
    /// Start Fine Timing Measurement Request
    FTM_START_FRAME_REQ,
    /// Measurement frame
    FTM_MEAS_RSP,
    /// Stop Fine Timing Measurement Request
    FTM_STOP_FRAME_REQ,
};

/// STA Info Table
struct ftm_sta
{
    /// MAC address of the STA
    struct mac_addr bssid;
    /// RSSI of the scanned BSS (in dBm)
    int8_t rssi;
    /// Network channel
    struct scan_chan_tag chan;
};

/// Declaration of FTM environment.
struct ftm_env_tag
{
    /// Pointer to the confirmation structure allocated when starting the procedure
    struct ftm_done_ind *ind;
    /// Sum of all measurement for one burst
    uint64_t ftm_sum_meas;
    /// VIF index
    uint8_t vif_idx;
    /// Nb of measurements received
    uint8_t ftm_meas_count;
    /// Nb of FTM measurements per bursts
    uint8_t ftm_per_burst;
    /// Current measurements per bursts
    uint8_t current_ftm_per_burst;
    /// list of all STA that support FTM
    struct ftm_sta ftm_list_sta[FTM_RSP_MAX];
    /// Number of FTM responders we want to measure with
    uint8_t nb_ftm_rsp;
    /// Current STA on which the FTM procedure is processed
    uint8_t ftm_current_idx;
    #if NX_FAKE_FTM_RSP
    /// MAC address of the AP
    struct mac_addr ap_mac_addr;
    #endif
};

/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */
/// FTM module environment declaration.
extern struct ftm_env_tag ftm_env;

/**
 ****************************************************************************************
 * @brief Start scan to find the different STA
 ****************************************************************************************
 */
void ftm_scan(void);

/**
 ****************************************************************************************
 * @brief Send the confirmation that the request to start the FTM has been received
 *
 * @param[in] status   status (success or not of the FTM start procedure)
 * @param[in] vif_idx  VIF index
 ****************************************************************************************
 */
void ftm_send_start_cfm(uint8_t status, uint8_t vif_idx);

/**
 ****************************************************************************************
 * @brief Close ongoing FTM session
 *
 ****************************************************************************************
 */
void ftm_close_session(void);

/**
 ****************************************************************************************
 * @brief Send FTM action frame to STA.
 *
 * @param[in]  action        Action to perform
 * @param[in]  dest_addr     Destination Address
 *
 * @return true if the frame has been successfully sent, else false.
 ****************************************************************************************
 */
bool ftm_send_action_frame(uint8_t action, struct mac_addr *dest_addr);

/**
 ****************************************************************************************
 * @brief Switch to the channel on which the STA is camped.
 *
 * @param[in]  operation_code  operation of RoC to proceed
 *
 ****************************************************************************************
 */
void ftm_remain_on_channel(uint8_t operation_code);

/**
 ****************************************************************************************
 * @brief Ordering the result of the scan with the rssi
 *
 * @param[in]  nb_bssid      nb of bssid of the scan
 * @param[in]  list_result   list of the bssid that should be ordered
 *
 ****************************************************************************************
 */
void sort_bssid_by_rssi(uint8_t nb_bssid, struct mac_scan_result *list_result[]);

/**
 ****************************************************************************************
 * @brief Handle the scheduling of the differents measurements.
 *
 ****************************************************************************************
 */
void ftm_measurement_scheduling(void);

/**
 ****************************************************************************************
 * @brief Get the address of the FTM parameters in the frame
 *
 * @param[in]  frame      Address of the frame
 *
 * @return the address of the FTM parameters or 0 in case there is no FTM parameters
 ****************************************************************************************
 */
uint32_t ftm_get_ftm_params(uint32_t frame);

/**
 *****************************************************************************************
 * @brief Compute duration it took to transmit the received frame
 *
 * @param[in] rx_leg_inf Rx frame legacy information
 *
 * @return air time used to transmit the received frame, in ps.
 ****************************************************************************************
 */
uint64_t ftm_frame_duration_ps(const struct rx_leg_info *rx_leg_inf);

/// @} end of group

#endif // NX_FTM_INITIATOR
#endif // _FTM_H_
