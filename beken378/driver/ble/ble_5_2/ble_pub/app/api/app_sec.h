/**
 ****************************************************************************************
 *
 * @file app_sec.h
 *
 * @brief Application Security Entry Point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_SEC
 * @{
 ****************************************************************************************
 */

#ifndef APP_SEC_H_
#define APP_SEC_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_APP_SEC)

#include <stdint.h>          // Standard Integer Definition
#include "gap.h"
#include "gapc_msg.h"
/*
 * DEFINES
 ****************************************************************************************
 */
typedef enum{
	APP_SEC_ERROR_NO_ERROR,
	APP_SEC_ERROR_PARAM_INVALID,
	APP_SEC_ERROR_PARAM_UNSUPPORT,
}sec_err_t;

typedef enum{
	APP_SEC_PAIRING_SUCCEED,
	APP_SEC_PAIRING_FAILED,
	APP_SEC_MAX,
}sec_notice_t;

typedef void (*sec_notice_cb_t)(sec_notice_t notice, void *param);

/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */
struct app_pairing_cfg
{
    /// IO capabilities (@see gap_io_cap)
    uint8_t iocap;
    /// Authentication (@see gap_auth)
    /// Note in BT 4.1 the Auth Field is extended to include 'Key Notification' and
    /// in BT 4.2 the Secure Connections'.
    uint8_t auth;
    ///Initiator key distribution (@see gap_kdist)
    uint8_t ikey_dist;
    ///Responder key distribution (@see gap_kdist)
    uint8_t rkey_dist;

    /// Device security requirements (minimum security level). (@see gap_sec_req)
    uint8_t sec_req;
};

struct app_sec_env_tag
{
	// Bond status
	bool bonded;
	bool peer_pairing_recv;
	bool peer_encrypt_recv;

	uint32_t passkey;
	/// Long Term Key information (if info = GAPC_LTK_EXCH)
	//@trc_union parent.info == GAPC_LTK_EXCH
	struct gapc_ltk ltk;
	struct gapc_ltk peer_ltk;

	/// Identity Resolving Key information (if info = GAPC_IRK_EXCH)
	//@trc_union parent.info == GAPC_IRK_EXCH
	struct gapc_irk irk;
	struct gapc_irk peer_irk;

	/// Connection Signature Resolving Key information (if info = GAPC_CSRK_EXCH)
	//@trc_union parent.info == GAPC_CSRK_EXCH
	struct gap_sec_key csrk;
	struct gap_sec_key peer_csrk;

	struct app_pairing_cfg pairing_param;
	sec_notice_cb_t sec_notice_cb;
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/// Application Security Environment
extern struct app_sec_env_tag app_sec_env;

/// Table of message handlers
extern const struct app_subtask_handlers app_sec_handlers;

/*
 * GLOBAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the Application Security Module
 ****************************************************************************************
 */
void app_sec_init(void);

/**
 ****************************************************************************************
 * @brief Set pairing param and cb function
 ****************************************************************************************
 */
uint8_t app_sec_config(struct app_pairing_cfg *param, sec_notice_cb_t func);

/**
 ****************************************************************************************
 * @brief Get Application Security Module BOND status
 ****************************************************************************************
 */
bool app_sec_get_bond_status(void);

/**
 ****************************************************************************************
 * @brief Application Security config ble tk value
 ****************************************************************************************
 */
bool app_sec_set_tk_passkey(uint32_t passkey);

/**
 ****************************************************************************************
 * @brief Remove all bond data stored in NVDS
 ****************************************************************************************
 */
void app_sec_remove_bond(void);

/**
 ****************************************************************************************
 * @brief Send a security request to the peer device. This function is used to require the
 * central to start the encryption with a LTK that would have shared during a previous
 * bond procedure.
 *
 * @param[in]   - conidx: Connection Index
 ****************************************************************************************
 */
void app_sec_send_security_req(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief request to Start a bonding procedure.This function is used to master of the connection
 *
 * @param[in]   - conidx: Connection Index
 ****************************************************************************************
 */
void app_sec_send_bond_cmd(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief request to Start an Encryption procedure.This function is used to master of the connection
 *
 * @param[in]   - conidx: Connection Index
 ****************************************************************************************
 */
void app_sec_send_encryption_cmd(uint8_t conidx);

#endif //(BLE_APP_SEC)

#endif // APP_SEC_H_

/// @} APP_SEC
