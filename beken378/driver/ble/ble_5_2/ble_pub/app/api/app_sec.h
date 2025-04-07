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
#include "gapm_msg.h"
#include "flash.h"
#include "BkDriverFlash.h"

/*
 * DEFINES
 ****************************************************************************************
 */
  //store bonding info in flash
#define APP_SEC_BOND_STORE    1

#define MAX_BOND_NUM          BLE_RAL_MAX
#define MAX_BOND_NUM_MASK    ((1 << MAX_BOND_NUM) - 1)
#define INVAILD_IDX           MAX_BOND_NUM

#if (APP_SEC_BOND_STORE)
#define CRC_DEFAULT_VALUE     0xFFFFFFFF
#define MAX_CRC_FAIL_TIMES    3
#endif

/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */
typedef struct
{
    struct gapc_irk peer_irk;
    struct gapc_ltk ltk;
    uint32_t crc;
}bond_info_t;

struct app_sec_env_tag
{
    // Bond status
    uint8_t bonded;

    uint32_t passkey;
    uint8_t matched_peer_idx;

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
    #if (APP_SEC_BOND_STORE)
    uint8_t flash_write_idx[MAX_BOND_NUM - 1];
    uint8_t flash_write_num;
    bk_logic_partition_t *flash_bond_ptr;
    #endif
    bond_info_t bond_info[MAX_BOND_NUM];

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
sec_err_t app_sec_config(struct app_pairing_cfg *param, sec_notice_cb_t func);

#if (APP_SEC_BOND_STORE)
/**
 ****************************************************************************************
 * @brief Store Bond Info In Flash
 ****************************************************************************************
 */
sec_err_t app_sec_store_bond_info_in_flash(void);
#endif

/**
 ****************************************************************************************
 * @brief Get Application Security Module BOND status
 ****************************************************************************************
 */
uint8_t app_sec_get_bond_status(void);

/**
 ****************************************************************************************
 * @brief Get next availiable bond idx
 ****************************************************************************************
 */
uint8_t app_sec_get_free_bond_idx(void);

/**
 ****************************************************************************************
 * @brief Check If Peer Device in bonded list
    If found, store idx in app_sec_env.matched_peer_idx.
 ****************************************************************************************
 */
sec_err_t app_sec_search_bond_list(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Application Security config ble tk value
 ****************************************************************************************
 */
bool app_sec_check_tk_passkey(uint32_t passkey);

/**
 ****************************************************************************************
 * @brief Remove all bond data stored in NVDS
 ****************************************************************************************
 */
sec_err_t app_sec_remove_bond(uint8_t idx);

/**
 ****************************************************************************************
 * @brief Send a security request to the peer device. This function is used to require the
 * central to start the encryption with a LTK that would have shared during a previous
 * bond procedure.
 *
 * @param[in]   - conidx: Connection Index
 ****************************************************************************************
 */
sec_err_t app_sec_send_security_req(uint8_t conidx);

/**
 *************************************************************************************************
 * @brief request to Start a bonding procedure.This function is used to master of the connection
 *
 * @param[in]   - conidx: Connection Index
 *************************************************************************************************
 */
sec_err_t app_sec_send_pairing_req(uint8_t conidx);

/**
 *************************************************************************************************
 * @brief respond to received pairing request.This function is used to slave of the connection
 *
 * @param[in]   - conidx: Connection Index
 *************************************************************************************************
 */
sec_err_t app_sec_send_pairing_rsp(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief disply the key value to the peer device in PassKey Entry Method
 *
 * @param[in]   - conidx: Connection Index
 * @param[in]   - passkey: the key value used in PassKey Entry Method
 ****************************************************************************************
 */
sec_err_t app_sec_tk_exchange_cfm(uint8_t conidx, uint32_t passkey);

/**
 ****************************************************************************************
 * @brief user checks if the numbers dispalyed on each side are the same
 *
 * @param[in]   - conidx: Connection Index
 * @param[in]   - accept: numbers displayed on each side are the same
 ****************************************************************************************
 */
sec_err_t app_sec_nc_exchange_cfm(uint8_t conidx, bool accept);

/**
 **************************************************************************************************
 * @brief request to Start an Encryption procedure.This function is used to master of the connection
 *
 * @param[in]   - conidx: Connection Index
 **************************************************************************************************
 */
sec_err_t app_sec_send_encryption_cmd(uint8_t conidx);

#endif //(BLE_APP_SEC)

#endif // APP_SEC_H_

/// @} APP_SEC
