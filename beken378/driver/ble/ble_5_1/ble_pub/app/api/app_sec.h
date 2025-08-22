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
#include "gapc_task.h"
#include "gapm_task.h"
#include "flash.h"
#include "BkDriverFlash.h"

/*
 * DEFINES
 ****************************************************************************************
 */
 //store bonding info in flash
#define APP_SEC_BOND_STORE    1
#define MAX_BOND_NUM          BLE_RAL_MAX

#if (MAX_BOND_NUM > BLE_RAL_MAX)
    #error "Number of bonded devices cannot be more than size of RAL"
#endif

#define MAX_BOND_NUM_MASK    ((1 << MAX_BOND_NUM) - 1)
#define INVALID_IDX           MAX_BOND_NUM

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
    // pairing level
    struct gapc_bond_auth auth;

    // peer irk
    struct gapc_irk peer_irk;

    // ltk for encrypt
    struct gapc_ltk ltk;

    #if BLE_APP_SIGN_WRITE
    // csrk & counter for singed write
    bool local_csrk_present;
    uint32_t sign_counter;
    struct gap_sec_key csrk;
    bool peer_csrk_present;
    uint32_t peer_sign_counter;
    struct gap_sec_key peer_csrk;
    #endif

    uint32_t crc;
}bond_info_t;

typedef struct
{
    uint8_t matched_peer_idx;

    struct gapc_bond_auth auth;

    /// Long Term Key information
    struct gapc_ltk ltk;
    struct gapc_ltk peer_ltk;

    /// Identity Resolving Key information
    struct gapc_irk irk;
    struct gapc_irk peer_irk;

    #if BLE_APP_SIGN_WRITE
    /// Connection Signature Resolving Key information
    bool local_csrk_present;
    uint32_t sign_counter;
    struct gap_sec_key csrk;
    bool peer_csrk_present;
    uint32_t peer_sign_counter;
    struct gap_sec_key peer_csrk;
    #endif
}sec_info_t;

struct app_sec_env_tag
{
    // Each bit corresponds to whether a bonding entry is valid. For example,
    // bonded = 0b101 means there are two valid bonding entries located at group 0 and group 2.
    uint8_t                 bonded;

    // Pairing parameters set by user
    struct app_pairing_cfg  pairing_param;

    // Security notice cb set by user
    sec_notice_cb_t         sec_notice_cb;

    // The bonding information of all bonded devices
    bond_info_t             bond_info[MAX_BOND_NUM];

    // The secutiry information of currently connected devices
    sec_info_t              sec_info[BLE_CONNECTION_MAX];

    #if (APP_SEC_BOND_STORE)
    // Indicates which bonding information entry needs to be written to flash
    bool                 flash_write_idx[MAX_BOND_NUM];

    // Point to the flash information structure that stores bonding information
    bk_logic_partition_t *  flash_bond_ptr;
    #endif
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
 * Calculate the CRC32 value of a memory buffer.
 *
 * @param crc accumulated CRC32 value, must be 0 on first call
 * @param buf buffer to calculate CRC32 value for
 * @param size bytes in buffer
 *
 * @return calculated CRC32 value
 */
uint32_t app_sec_crc32(uint32_t crc, const void *buf, size_t size);

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
bool app_sec_set_tk_passkey(uint32_t passkey);

/**
 ****************************************************************************************
 * @brief Remove all bond data stored in NVDS
 ****************************************************************************************
 */
sec_err_t app_sec_remove_bond(uint8_t idx, bool disconnect);

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
 ****************************************************************************************
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
 * @param[in]   - accept: accept pairing request
 *************************************************************************************************
 */
sec_err_t app_sec_send_pairing_rsp(uint8_t conidx, bool accept);

/**
 ****************************************************************************************
 * @brief disply the key value to the peer device in PassKey Entry Method
 *
 * @param[in]   - conidx: Connection Index
 * @param[in]   - passkey: the key value used in PassKey Entry Method
 * @param[in]   - accept: accept to reply tk
 ****************************************************************************************
 */
sec_err_t app_sec_tk_exchange_cfm(uint8_t conidx, uint32_t passkey, bool accept);

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
 ****************************************************************************************
 */
sec_err_t app_sec_send_encryption_cmd(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief if the peer device is not bonded, request to Start a bonding procedure,
 * otherwise, Start an Encryption procedure.
 *
 * @param[in]   - conidx: Connection Index
 ****************************************************************************************
 */
sec_err_t app_sec_master_security_start(uint8_t conidx);

#endif //(BLE_APP_SEC)

#endif // APP_SEC_H_

/// @} APP_SEC
