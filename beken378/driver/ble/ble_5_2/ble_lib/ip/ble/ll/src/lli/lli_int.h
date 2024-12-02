/**
 ****************************************************************************************
 *
 * @file lli_int.h
 *
 * @brief Link layer ISO internal definition
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

#ifndef LLI_INT_H_
#define LLI_INT_H_

/**
 ****************************************************************************************
 * @addtogroup LLI Link layer ISO
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_ISO_PRESENT)
#include <stdint.h>          // Standard integer
#include <stdbool.h>         // Standard boolean
#include "lli.h"             // LLI global definitions

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of instances of the LLI task
#define LLI_IDX_MAX                       (1)
/// Invalid trigger offset - used to know if trigger is disabled
#define LLI_INVALID_TRIGGER_OFFSET        (0x80000000)
/*
 * MACROS
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// ISO Channel type
enum lli_iso_chan_type
{
    #if (BLE_CIS)
    /// Connected Isochronous Stream Channel
    LLI_ISO_CHAN_CIS = 0,
    #endif // (BLE_CIS)
    #if (BLE_BIS)
    /// Broadcast Isochronous Stream Channel
    LLI_ISO_CHAN_BIS,
    #endif // (BLE_BIS)
    #if (BLE_ISO_MODE_0)
    /// Audio Mode 0 Channel
    LLI_ISO_CHAN_AM0,
    #endif // (BLE_ISO_MODE_0)
    /// Undefined channel
    LLI_ISO_CHAN_UNDEF,
};

/// ISO Group type
enum lli_iso_group_type
{
    #if (BLE_CIS)
    /// Connected Isochronous Group
    LLI_ISO_GROUP_CIG = 0,
    #endif // (BLE_CIS)
    #if (BLE_BIS)
    /// Broadcast Isochronous Group
    LLI_ISO_GROUP_BIG,
    #endif // (BLE_BIS)
    /// Undefined Group
    LLI_ISO_GROUP_UNDEF,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// ISO Channel environment structure (common for BIS and CIS)
struct lli_chan_env
{
    #if (BLE_CIS)
    common_list_hdr_t               hdr;
    #endif // (BLE_CIS)
    /// Channel Type (BIS or CIS), (@see enum lli_iso_chan_type)
    uint8_t                     type;
    /// Channel Handle
    uint8_t                     hdl;
    /// Framing mode (@see enum iso_frame)
    uint8_t                     framing;
};

/// ISO Group environment structure (common for BIS and CIS)
struct lli_group_env
{
    /// Group Type (BIG or CIG), (@see enum lli_iso_group_type)
    uint8_t                     type;
    /// group handle associated with channels
    uint8_t                     hdl;
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 *****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Get environment structure for an ISO channel identified by its channel handle.
 *
 * @param[in]  chan_hdl Allocated channel handle
 * @param[in]  iso_type Isochronous Channel Type (@see enum lli_iso_chan_type)
 * @param[out] pp_env   Address of pointer containing address of allocated environment
 *
 * @return - COMMON_ERROR_NO_ERROR           if channel handle is valid and environment has been found
 *         - COMMON_ERROR_INVALID_HCI_PARAM  if channel handle is not valid
 *         - COMMON_ERROR_COMMAND_DISALLOWED if either environment has not been found or found environment
 *                                       is not environment for an ISO channel
 ****************************************************************************************
 */
uint8_t lli_chan_env_get(uint8_t chan_hdl, uint8_t iso_type, struct lli_chan_env **pp_env);

/**
 ****************************************************************************************
 * @brief Allocate new Isochronous channel environment structure and if needed check if an
 * activity is available in LLM
 *
 * @param[in]  act_id      Activity Identifier associated with channel
 * @param[in]  iso_type    Isochronous Channel type (@see enum lli_iso_chan_type)
 * @param[in]  size        Size of environment variable to allocate
 * @param[out] pp_iso_env  Allocate environment variable
 *
 * @return - COMMON_ERROR_NO_ERROR                    If environment correctly allocated
 *         - COMMON_ERROR_CONN_REJ_LIMITED_RESOURCES  If not possible to allocate new channel memory
 *         - COMMON_ERROR_CON_LIMIT_EXCEED            If no activity can be assigned to new channel
 ****************************************************************************************
 */
uint8_t lli_chan_create(uint8_t act_id, uint8_t iso_type, uint16_t size, struct lli_chan_env **pp_env);

/**
 ****************************************************************************************
 * @brief Free an allocated Isochronous channel environment
 *
 * @param[in] chan_hdl    Channel handle
 ****************************************************************************************
 */
void lli_chan_cleanup(uint16_t chan_hdl);

#if (BLE_BIS || BLE_CIS)
/**
 ****************************************************************************************
 * @brief Get environment structure for an ISO Group identified by its group handle.
 *
 * @param[in]  grp_hdl    Allocated group handle
 * @param[in]  iso_type   Isochronous Group Type (@see enum lli_iso_group_type)
 * @param[out] pp_env     Address of pointer containing address of allocated environment
 *
 * @return - COMMON_ERROR_NO_ERROR           if channel handle is valid and environment has been found
 *         - COMMON_ERROR_INVALID_HCI_PARAM  if channel handle is not valid
 *         - COMMON_ERROR_COMMAND_DISALLOWED if either environment has not been found or found environment
 *                                       is not environment for an ISO stream
 ****************************************************************************************
 */
uint8_t lli_group_env_get(uint16_t grp_hdl, uint8_t iso_type, struct lli_group_env **pp_env);

/**
 ****************************************************************************************
 * @brief Allocate new Isochronous Group environment structure and if needed check if an
 * activity is available in LLM
 *
 * @param[in]  iso_type Isochronous Group type (@see enum lli_iso_group_type)
 * @param[in]  size     Size of environment variable to allocate
 * @param[out] pp_env   Allocate environment variable
 *
 * @return - COMMON_ERROR_NO_ERROR                    If environment correctly allocated
 *         - COMMON_ERROR_CONN_REJ_LIMITED_RESOURCES  If not possible to allocate new channel memory
 *         - COMMON_ERROR_CON_LIMIT_EXCEED            If no activity can be assigned to new stream
 ****************************************************************************************
 */
uint8_t lli_group_create(uint8_t iso_type, uint16_t size, struct lli_group_env **pp_env);

/**
 ****************************************************************************************
 * @brief Free an allocated Isochronous Group environment
 *
 * @param[in] grp_hdl    Group handle
 ****************************************************************************************
 */
void lli_group_cleanup(uint16_t grp_hdl);
#endif // (BLE_BIS || BLE_CIS)


#if (BLE_CIS)
/**
 ****************************************************************************************
 * @brief Initialization of the LLI CI module
 *
 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 ****************************************************************************************
 */
void lli_ci_init(uint8_t init_type);

/**
 ****************************************************************************************
 * Check if ACL link stop is related to current CIS channel
 *
 * @param[in] link_id     Link Identifier
 * @param[in] p_iso_env  CIS environment
 * @param[in] reason Link termination reason
 ****************************************************************************************
 */
void lli_cis_link_stop_ind(uint8_t link_id, struct lli_chan_env* p_iso_env, uint8_t reason);


/**
 ****************************************************************************************
 * Load specific CIS TX / RX data path
 *
 * @param[in] p_env          Channel Environment
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 * @param[in] data_path_type Type of RX/TX data path interface (@see enum iso_dp_type)
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_cis_data_path_set(struct lli_chan_env *p_env, uint8_t direction, uint8_t data_path_type);

/**
 ****************************************************************************************
 * Remove specific CIS TX or RX data path
 *
 * @param[in] p_env          Channel Environment
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_cis_data_path_remove(struct lli_chan_env *p_env, uint8_t direction);

/**
 ****************************************************************************************
 * Configure Trigger for specific stream
 *
 * @param[in] p_env          Channel Environment
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 * @param[in] enable         Enable or disable trigger offset
 * @param[in] trigger_offset Trigger offset in microseconds
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_cis_trigger_set(struct lli_chan_env *p_env, uint8_t direction, bool enable, int32_t offset);

/**
 ****************************************************************************************
 * Read CIS statistics
 *
 * @param[in]  p_env                    Channel Environment
 * @param[out] tx_unacked_packets       Pointer to number of Tx unacked packets
 * @param[out] tx_flushed_packets       Pointer to number of Tx flushed packets
 * @param[out] tx_last_subevent_packets Pointer to number of Tx last subevent packets
 * @param[out] retransmitted_packets    Pointer to number of retransmitted packets
 * @param[out] crc_error_packets        Pointer to number of CRC error packets
 * @param[out] rx_unreceived_packets    Pointer to number of Rx unreceived packets
 * @param[out] duplicate_packets        Pointer to number of duplicate packets
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_cis_stats_get(struct lli_chan_env *p_env, uint32_t* tx_unacked_packets, uint32_t* tx_flushed_packets, uint32_t* tx_last_subevent_packets,
                                                      uint32_t* retransmitted_packets, uint32_t* crc_error_packets, uint32_t* rx_unreceived_packets,
                                                      uint32_t* duplicate_packets);
#endif // (BLE_CIS)

#if (BLE_BIS)

/**
 ****************************************************************************************
 * @brief Initialization of the LLI BI module
 *
 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 ****************************************************************************************
 */
void lli_bi_init(uint8_t init_type);

/**
 ****************************************************************************************
 * Load specific BIS TX / RX data path
 *
 * @param[in] p_env          Channel Environment
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 * @param[in] data_path_type Type of RX/TX data path interface (@see enum iso_dp_type)
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_bis_data_path_set(struct lli_chan_env *p_env, uint8_t direction, uint8_t data_path_type);

/**
 ****************************************************************************************
 * Remove specific CIS TX / RX data path
 *
 * @param[in] p_env          Channel Environment
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_bis_data_path_remove(struct lli_chan_env *p_env, uint8_t direction);

/**
 ****************************************************************************************
 * Configure Trigger for specific stream
 *
 * @param[in] p_env          Channel Environment
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 * @param[in] enable         Enable or disable trigger offset
 * @param[in] trigger_offset Trigger offset in microseconds
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_bis_trigger_set(struct lli_chan_env *p_env, uint8_t direction, bool enable, int32_t offset);

/**
 ****************************************************************************************
 * Read BIS statistics
 *
 * @param[in]  p_env                    Channel Environment
 * @param[out] crc_error_packets        Pointer to number of CRC error packets
 * @param[out] rx_unreceived_packets    Pointer to number of Rx unreceived packets
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_bis_stats_get(struct lli_chan_env *p_env, uint32_t* crc_error_packets, uint32_t* rx_unreceived_packets);
#endif // (BLE_BIS)

#if (BLE_ISO_MODE_0)
/**
 ****************************************************************************************
 * Check if ACL link stop is related to current AM0 channel
 *
 * @param[in] link_id     Link Identifier
 * @param[in] p_iso_env   AM0 environment
 * @param[in] reason      Link termination reason
 ****************************************************************************************
 */
void lli_am0_link_stop_ind(uint8_t link_id, struct lli_chan_env* p_iso_env, uint8_t reason);

/**
 ****************************************************************************************
 * Configure Trigger for specific stream
 *
 * @param[in] p_env          Channel Environment
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 * @param[in] enable         Enable or disable trigger offset
 * @param[in] trigger_offset Trigger offset in microseconds
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_am0_trigger_set(struct lli_chan_env *p_env, uint8_t direction, bool enable, int32_t offset);
#endif // (BLE_ISO_MODE_0)

/**
 ****************************************************************************************
 * Load specific Isochronous TX / RX data path
 *
 * @param[in]     chan_hdl                 Channel Handle
 * @param[in]     direction                Data Path direction (@see enum iso_rx_tx_select)
 * @param[in]     data_path_type           Type of data path interface (@see enum iso_dp_type)
 * @param[in|out] pp_dp                    Pointer to data path
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_data_path_load(uint16_t chan_hdl, uint8_t direction, uint8_t data_path_type, const struct data_path_itf** pp_dp);
/**
 ****************************************************************************************
 * Remove the Isochronous TX / RX data path if enabled
 *
 * @param[in]     chan_hdl                 Channel Handle
 * @param[in]     direction                Data Path direction (@see enum iso_rx_tx_select)
 * @param[in|out] pp_dp                    Pointer to data path
 *
 * @return status of the executed procedure (@see enum common_error)
 ****************************************************************************************
 */
uint8_t lli_data_path_remove(uint16_t chan_hdl, uint8_t direction, const struct data_path_itf** pp_dp);

/**
 ****************************************************************************************
 * Initialize the Isochronous TX / RX data path
 *
 * @param[in]     chan_hdl                 Channel Handle
 * @param[in]     direction                Data Path direction (@see enum iso_rx_tx_select)
 * @param[in|out] pp_dp                    Pointer to data path
 ****************************************************************************************
 */
void lli_data_path_init(uint16_t chan_hdl, uint8_t direction, const struct data_path_itf** pp_dp);

#endif // (BLE_ISO_PRESENT)
/// @} LLI

#endif // LLI_INT_H_
