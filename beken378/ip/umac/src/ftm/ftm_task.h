/**
 ****************************************************************************************
 *
 * @file ftm_task.h
 *
 * Copyright (C) RivieraWaves 2016-2021
 *
 * @brief FTM Task Kernel Interface Declaration.
 *
 ****************************************************************************************
 */

#ifndef _FTM_TASK_H_
#define _FTM_TASK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "ftm.h"

#if NX_FTM_INITIATOR

/**
 * @defgroup TASK_FTM TASK_FTM
 * @ingroup FTM
 * @brief Declaration of all structures and functions used by the FTM module.
 * @{
 */

/*
 * DEFINES
 ****************************************************************************************
 */


#define TRACE_FTM(component, ...)  do {os_printf(__VA_ARGS__); os_printf("\n");} while(0)
#ifndef TR_MAC
#define TR_MAC(m) m
#endif
#ifndef TR_32
#define TR_32(a) (a)
#endif


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Possible states of the FTM task.
enum ftm_state_tag
{
    /// IDLE State.
    FTM_IDLE,
    /// SCANNING State
    FTM_SCANNING,
    /// Waiting that Remain on Channel start State
    FTM_WAITING_ROC_START,
    /// Waiting that Remain on Channel cancel State
    FTM_WAITING_ROC_CANCEL,
    /// FTM Establishing FTM session State
    FTM_ESTABLISHING_SESSION,
    /// FTM initial measurement
    FTM_MEAS_INITIAL_EXCHANGE,
    /// FTM measurement
    FTM_MEAS_EXCHANGE,
    /// Waiting for session to be closed
    FTM_CLOSING_SESSION,
    /// Number of states.
    FTM_STATE_MAX,
};

/// Messages that are logically related to the task.
enum ftm_msg_tag
{
    /// Request to start the FTM Measurements
    FTM_START_REQ = KE_FIRST_MSG(TASK_FTM),
    /// Confirmation of the FTM start.
    FTM_START_CFM,
    /// Message sent once the requested measurements have been done
    FTM_DONE_IND,
    /// Request Measurement to the peer STA
    FTM_MEASUREMENT_REQ,
    /// Timeout message for procedures requiring a response from the peer STA
    FTM_PEER_RSP_TIMEOUT_IND,
    /// Request for closing the session after last measurement
    FTM_CLOSE_SESSION_REQ,
    #if NX_FAKE_FTM_RSP
    FTM_AP_MEASUREMENT,
    #endif
};

/*
 * API MESSAGES STRUCTURES
 ****************************************************************************************
 */

/// Structure containing the parameters of the @ref FTM_START_REQ message.
struct ftm_start_req
{
    /// Index of the VIF for which the FTM is started
    uint8_t vif_idx;
    /// Number of FTMs per Burst
    uint8_t ftm_per_burst;
    /// Number of FTM responders on which we do the measurements
    uint8_t nb_ftm_rsp;
};

/// Structure containing the parameters of the @ref FTM_START_CFM message.
struct ftm_start_cfm
{
    /// Status of the FTM starting procedure
    uint8_t status;
    /// Index of the VIF for which the FTM is started
    uint8_t vif_idx;
};

/// Structure containing the parameters of the @ref FTM_DONE_IND message.
struct ftm_done_ind
{
    /// Index of the VIF for which the FTM is started
    uint8_t vif_idx;
    /// Results
    struct mac_ftm_results results;
};

/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */

/// Task Information
extern const struct ke_state_handler ftm_default_handler;
extern ke_state_t ftm_state[FTM_IDX_MAX];

/// @} end of group

#endif // NX_FTM_INITIATOR
#endif // _FTM_TASK_H_
