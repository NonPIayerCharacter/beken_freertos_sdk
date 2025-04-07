/**
 ****************************************************************************************
 *
 * @file rwble_hl_config.h
 *
 * @brief Configuration of the BLE protocol stack (max number of supported connections,
 * type of partitioning, etc.)
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

#ifndef RWBLE_HL_CONFIG_H_
#define RWBLE_HL_CONFIG_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 * @name BLE stack configuration
 * @{
 ****************************************************************************************
 */

#include "rwble_hl_error.h"

/*
 * DEFINES - Mandatory for BLE Host Layers
 ****************************************************************************************
 */

/// Maximum time to remain advertising when in the Limited
/// Discover able mode: TGAP(lim_adv_timeout)
/// required value: 180s: (18000 in 10 ms step)
#define GAP_TMR_LIM_ADV_TIMEOUT                             (18000)

/// Minimum time to perform scanning when performing
/// the General Discovery procedure on 1M PHY: TGAP(gen_disc_scan_min)
/// recommended value: 10.24s: (1024 in 10 ms step)
#define GAP_TMR_GEN_DISC_SCAN_1M                            (1024)

/// Minimum time to perform scanning when performing
/// the General Discovery procedure on Coded PHY: TGAP(gen_disc_scan_min_coded)
/// recommended value: 30.72s: (3072 in 10 ms step)
#define GAP_TMR_GEN_DISC_SCAN_CODED                         (3072)

/// Minimum time to perform scanning when performing the
/// Limited Discovery procedure on 1M PHY: TGAP(lim_disc_scan_min)
/// recommended value: 10.24s: (1024 in 10 ms step)
#define GAP_TMR_LIM_DISC_SCAN_1M                            (1024)

/// Minimum time to perform scanning when performing the
/// Limited Discovery procedure on Coded PHY: TGAP(lim_disc_scan_min_coded)
/// recommended value: 30.72s: (3072 in 10 ms step)
#define GAP_TMR_LIM_DISC_SCAN_CODED                         (3072)

/// TGAP(private_addr_int)
/// recommended value: 15 minutes

/// Minimum time interval between private address change
/// Minimum value 1s
#define GAP_TMR_PRIV_ADDR_MIN                             (0x0001)
/// Maximum time interval between private address change
/// 0xA1B8 (approximately 11.5 hours)
#define GAP_TMR_PRIV_ADDR_MAX                             (0xA1B8)


/// L2CAP Signaling transaction Timer duration in milliseconds
/// TGAP(conn_param_timeout) recommended value: 30 s: (30 000 ms)
#define GAP_SIG_TRANS_TIMEOUT_MS                          (30000)

/// SMP L2CAP transaction Timer duration in milliseconds  30 s: (30 000 ms)
#define GAP_SMP_TRANS_TIMEOUT_MS                          (30000)

/// Maximal authorized MTU / MPS value - Depends on memory size available
#ifndef CFG_MAX_LE_MTU
#define GAP_LE_MTU_MAX                                     (2048)
#define GAP_LE_MPS_MAX                                     (2048)
#else
#define GAP_LE_MTU_MAX                                     (CFG_MAX_LE_MTU)
#define GAP_LE_MPS_MAX                                     (CFG_MAX_LE_MTU)
#endif // (CFG_MAX_LE_MTU)

/// Maximum GAP device name size
#define GAP_MAX_NAME_SIZE                                  (0x20)

/// 30 seconds transaction timer (30000 ms)
#define GATT_TRANSACTION_TIMEOUT                           (30000)

/// Maximum attribute value length
#ifndef CFG_ATT_VAL_MAX
#define GATT_MAX_VALUE                                     (GAP_LE_MTU_MAX)
#else
#define GATT_MAX_VALUE                                     (CFG_ATT_VAL_MAX)
#endif // CFG_ATT_VAL_MAX

/// Maximum number of services returned by callback
#define GATT_DISCOVER_SVC_ATT_MAX                          (20)

/// Number of L2CAP COC channel that can be created per connection
#ifndef CFG_L2CAP_COC_CHAN_PER_CON_NB
#define L2CAP_COC_CHAN_PER_CON_NB                                  (10)
#else
#define L2CAP_COC_CHAN_PER_CON_NB                                  (CFG_L2CAP_COC_CHAN_PER_CON_NB)
#endif // CFG_L2CAP_COC_CHAN_PER_CON_NB

/// Total Number of L2CAP channel and GATT bearer that can be allocated in environment heap
#ifndef CFG_L2CAP_CHAN_IN_ENV_NB
#define L2CAP_CHAN_IN_ENV_NB                                   (10)
#else
#define L2CAP_CHAN_IN_ENV_NB                                   (CFG_L2CAP_CHAN_IN_ENV_NB)
#endif // CFG_L2CAP_CHAN_IN_ENV_NB

/// Memory limit of prepare write queue
#define GATT_PREP_WRITE_QUEUE_MEM_LIMIT                    (GATT_MAX_VALUE)
/// Memory limit of GATT operation (to consider that there insufficient memory to perform procedure)
#define GATT_MEM_LIMIT                                     (COMMON_BUF_BIG_POOL_SIZE + (COMMON_BUF_SMALL_POOL_SIZE >> 1))

/******************************************************************************************/
/* -------------------------   BLE PARTITIONING      -------------------------------------*/
/******************************************************************************************/


/******************************************************************************************/
/* --------------------------   INTERFACES        ----------------------------------------*/
/******************************************************************************************/

#if BLE_APP_PRESENT
#define APP_MAIN_TASK       TASK_BLE_APP
#elif AHI_TL_SUPPORT
#define APP_MAIN_TASK       TASK_BLE_AHI
#else
#define APP_MAIN_TASK       TASK_BLE_NONE
#endif // BLE_APP_PRESENT

// Host Controller Interface (Host side)
#define BLEHL_HCIH_ITF            HCIH_ITF


/******************************************************************************************/
/* --------------------------   COEX SETUP        ----------------------------------------*/
/******************************************************************************************/

///WLAN coex
#define BLEHL_WLAN_COEX          RW_WLAN_COEX
///WLAN test mode
#define BLEHL_WLAN_COEX_TEST     RW_WLAN_COEX_TEST

/******************************************************************************************/
/* --------------------------        GAF          ----------------------------------------*/
/******************************************************************************************/
#if BLE_GAF_PRESENT
#define BLE_GAF           (1)
#else //(CFG_GAF)
#define BLE_GAF           (0)
#endif //(CFG_GAF)

/******************************************************************************************/
/* --------------------------   PROFILES          ----------------------------------------*/
/******************************************************************************************/
#ifdef CFG_PRF
#define BLE_PROFILES      (1)
/// Number of Profile tasks managed by GAP manager.
#define BLE_NB_PROFILES   (CFG_NB_PRF)
#include "rwprf_config.h"
#else
#define BLE_PROFILES      (0)
#define BLE_NB_PROFILES   (0)
#endif // CFG_PRF

/******************************************************************************************/
/* --------------------------   HOST MODULES      ----------------------------------------*/
/******************************************************************************************/
#define CFG_GATT_CLI

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    /// Support GATT Client
    #if defined(CFG_GATT_CLI)
        #define BLE_GATT_CLI         1
    #else
        #define BLE_GATT_CLI         0
    #endif // defined(CFG_GATT_CLI)
#else
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

#define BLE_GAPM                    1
#if (BLE_OBSERVER)
// must be equals to sizeof(struct gapm_actv_scan_tag)
#define BLE_GAPM_HEAP_ENV_SIZE      (192 + KERNEL_HEAP_MEM_RESERVED)
#elif (BLE_PERIPHERAL)
// must be equals to sizeof(struct gapm_actv_adv_tag)
#define BLE_GAPM_HEAP_ENV_SIZE      (36 + KERNEL_HEAP_MEM_RESERVED)
#else //(BLE_PERIPHERAL)
#define BLE_GAPM_HEAP_ENV_SIZE      0
#endif //(BLE_PERIPHERAL)

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#define BLE_GAPC                    1
// must be equals to sizeof(gapc_con_t) + KERNEL_HEAP_MEM_RESERVED
#if (BLE_GATT_CLI)
#define BLE_GAPC_HEAP_ENV_SIZE      (120 + KERNEL_HEAP_MEM_RESERVED)
#else
#define BLE_GAPC_HEAP_ENV_SIZE      (116 + KERNEL_HEAP_MEM_RESERVED)
#endif
#else //(BLE_CENTRAL || BLE_PERIPHERAL)
#define BLE_GAPC                    0
#define BLE_GAPC_HEAP_ENV_SIZE      0
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#define BLE_L2CAP                   (1)
#define BLE_L2CAP_CHAN_PER_CON      (L2CAP_COC_CHAN_PER_CON_NB + BLE_GATT + BLE_GAPC + BLE_L2CAP + BLE_ISO_MODE_0_PROFILE)
#define BLE_GATT                    1

#if (BLE_GAF)
#define BLE_GATT_USER_NB            (8 + BLE_GATT + BLE_GATT_CLI) // 2 users reserved for GATT and GAP services / clients
#define BLE_GATT_BEARER_PER_CON     (8 + BLE_GATT)
#else //(BLE_GAF)
#if (BLE_PROFILES)
#define BLE_GATT_USER_NB            (BLE_NB_PROFILES + BLE_GATT + BLE_GATT_CLI) // 2 users reserved for GATT and GAP services / clients
#if (BLE_NB_PROFILES > L2CAP_COC_CHAN_PER_CON_NB)
#define BLE_GATT_BEARER_PER_CON     (L2CAP_COC_CHAN_PER_CON_NB + BLE_GATT)
#else // (BLE_NB_PROFILES <= L2CAP_COC_CHAN_PER_CON_NB)
#define BLE_GATT_BEARER_PER_CON     (BLE_NB_PROFILES + BLE_GATT)
#endif // (BLE_NB_PROFILES > L2CAP_COC_CHAN_PER_CON_NB)
#else // (BLE_PROFILES)
#define BLE_GATT_USER_NB            ((BLE_GATT * 2) + (BLE_GATT_CLI * 2)) // 1 client + service for GATT and GAP + 1 client + 1 service for upper layer application
#define BLE_GATT_BEARER_PER_CON     ((BLE_GATT * 2) + BLE_GATT_CLI)       // 1 Legacy bearer + 2 Enhanced Bearer: for server and one client for upper layer application
#endif // (BLE_PROFILES)
#endif //(BLE_GAF)

// Must be equals to (sizeof(gatt_con_env_t) and consider 4 * BLE_GATT_BEARER_PER_CON)
#if (BLE_GATT_CLI)
#define BLE_GATT_CON_ENV_SIZE       (84 + (4 * BLE_GATT_BEARER_PER_CON))
#else
#define BLE_GATT_CON_ENV_SIZE       (80 + (4 * BLE_GATT_BEARER_PER_CON))
#endif
#define BLE_GATT_BEARER_ENV_SIZE    (sizeof(common_list_t) + 8 + KERNEL_HEAP_MEM_RESERVED)
#define BLE_GATT_HEAP_ENV_SIZE      (BLE_GATT_CON_ENV_SIZE + KERNEL_HEAP_MEM_RESERVED)
// Must be equals to (sizeof(l2cap_con_env_t) and consider 4 * BLE_L2CAP_CHAN_PER_CON)
#define BLE_L2CAP_CON_ENV_SIZE      (112 + (4 * BLE_L2CAP_CHAN_PER_CON))
#define BLE_L2CAP_CHANNEL_ENV_SIZE  (sizeof(common_list_t) + (BLE_HL_MSG_API * 4) + 28 + KERNEL_HEAP_MEM_RESERVED)
#define BLE_L2CAP_HEAP_ENV_SIZE     (BLE_L2CAP_CON_ENV_SIZE + KERNEL_HEAP_MEM_RESERVED)
#define BLE_L2CAP_NB_RX_BUF_AVAIL   10
#else //(BLE_CENTRAL || BLE_PERIPHERAL)
#define BLE_L2CAP                   0
#define BLE_GATT                    0
#define BLE_GATT_CLI                0
#define BLE_GATT_HEAP_ENV_SIZE      0
#define BLE_L2CAP_HEAP_ENV_SIZE     0
#define BLE_HL_MSG_API              0
#define BLE_GATT_BEARER_ENV_SIZE    0
#define BLE_L2CAP_CHANNEL_ENV_SIZE  0
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

/******************************************************************************************/
/* --------------------------   HL MESSAGE API    ----------------------------------------*/
/******************************************************************************************/

#define CFG_HL_MSG_API

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    /// Support HL Message API
    #if defined(CFG_HL_MSG_API)
        #define BLE_HL_MSG_API        1
    #else
        #define BLE_HL_MSG_API        0
    #endif // defined(CFG_GATT_CLI)
#else
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/******************************************************************************************/
/* --------------------------   GATT              ----------------------------------------*/
/******************************************************************************************/
/// Size of the heap
#if (BLE_CENTRAL || BLE_PERIPHERAL)
    /// Can be tuned based on supported profiles
    #define BLEHL_HEAP_DB_SIZE                     (3072)
    /// Needed "security" heap size for reception of max supported MTU through prepare write procedure
    /// If retention memory is used, this additional heap can be part of the size allocated for the retention memory
    #define BLEHL_HEAP_DATA_THP_SIZE               (GAP_LE_MTU_MAX)
    /// message size per connection
    #define BLEHL_HEAP_MSG_SIZE_PER_CON            (100)

    #define BLEHL_HEAP_MSG_SIZE                    (((BLEHL_HEAP_MSG_SIZE_PER_CON * BLE_CONNECTION_MAX) > BLEHL_HEAP_DATA_THP_SIZE) \
                                                        ? (BLEHL_HEAP_MSG_SIZE_PER_CON * BLE_CONNECTION_MAX) : BLEHL_HEAP_DATA_THP_SIZE)
#elif (BLE_MESH)
    #define BLEHL_HEAP_MSG_SIZE                    (1024)
    #define BLEHL_HEAP_DB_SIZE                     (3072)
#else
    #define BLEHL_HEAP_MSG_SIZE                    (256)
    #define BLEHL_HEAP_DB_SIZE                     (0)
#endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */


/// Number of BLE HL tasks
#define BLEHL_TASK_SIZE       BLE_HOST_TASK_SIZE + BLE_PRF_TASK_SIZE

/// Size of environment variable needed on BLE Host Stack for one link
#define BLEHL_HEAP_ENV_SIZE  ( BLE_CONNECTION_MAX * ( BLE_GAPC_HEAP_ENV_SIZE       \
                                                    + BLE_GATT_HEAP_ENV_SIZE       \
                                                    + BLE_L2CAP_HEAP_ENV_SIZE )    \
                             + BLE_ACTIVITY_MAX * BLE_GAPM_HEAP_ENV_SIZE \
                             + L2CAP_CHAN_IN_ENV_NB * (BLE_GATT_BEARER_ENV_SIZE + BLE_L2CAP_CHANNEL_ENV_SIZE))


/// @} BLE stack configuration
/// @} ROOT

#endif // RWBLE_HL_CONFIG_H_
