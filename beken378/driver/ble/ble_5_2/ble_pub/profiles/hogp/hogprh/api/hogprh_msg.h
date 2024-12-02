/**
 ****************************************************************************************
 *
 * @file hogprh_msg.h
 *
 * @brief Header file - HID Over GATT Profile Report Host Role - Message API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 *
 ****************************************************************************************
 */


#ifndef _HOGPRH_MSG_H_
#define _HOGPRH_MSG_H_

/**
 ****************************************************************************************
 * @addtogroup HOGPRH
 * @ingroup Profile
 * @brief HID Over GATT Profile Report Host - Message API
 *
 * This module is responsible for handling the messages coming in and out of the
 * @ref HOGPRH monitor block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


#include "rwip_task.h" // Task definitions
#include "hogp_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximal number of hids instances that can be handled
#define HOGPRH_NB_HIDS_INST_MAX              (2)
/// Maximal number of Report Char. that can be added in the DB for one HIDS - Up to 11
#define HOGPRH_NB_REPORT_INST_MAX            (5)

/// Maximal length of Report Map Char. Value
#define HOGPRH_REPORT_MAP_MAX_LEN            (512)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/*@TRACE*/
enum hogprh_msg_id
{
    /// Start the HID Over GATT profile - at connection
    HOGPRH_ENABLE_REQ    = MSG_ID(HOGPRH, 0x00),
    ///Confirm that cfg connection has finished with discovery results, or that normal cnx started
    HOGPRH_ENABLE_RSP    = MSG_ID(HOGPRH, 0x01),

    /// Read Characteristic Value Request
    HOGPRH_READ_INFO_REQ = MSG_ID(HOGPRH, 0x02),
    /// Read Characteristic Value Request
    HOGPRH_READ_INFO_RSP = MSG_ID(HOGPRH, 0x03),

    /// Write/Configure peer device attribute Request
    HOGPRH_WRITE_REQ     = MSG_ID(HOGPRH, 0x04),
    /// Write/Configure peer device attribute Response
    HOGPRH_WRITE_RSP     = MSG_ID(HOGPRH, 0x05),

    /// Report value send to APP (after Notification)
    HOGPRH_REPORT_IND    = MSG_ID(HOGPRH, 0x06),
};

/// Characteristics
enum hogprh_chars
{
    /// Report Map
    HOGPRH_CHAR_REPORT_MAP,
    /// HID Information
    HOGPRH_CHAR_HID_INFO,
    /// HID Control Point
    HOGPRH_CHAR_HID_CTNL_PT,
    /// Protocol Mode
    HOGPRH_CHAR_PROTOCOL_MODE,
    /// Report
    HOGPRH_CHAR_REPORT,

    HOGPRH_CHAR_MAX = HOGPRH_CHAR_REPORT + HOGPRH_NB_REPORT_INST_MAX,
};

/// Characteristic descriptors
enum hogprh_descs
{
    /// Report Map Char. External Report Reference Descriptor
    HOGPRH_DESC_REPORT_MAP_EXT_REP_REF,
    /// Report Char. Report Reference
    HOGPRH_DESC_REPORT_REF,
    /// Report Client Config
    HOGPRH_DESC_REPORT_CFG = HOGPRH_DESC_REPORT_REF + HOGPRH_NB_REPORT_INST_MAX,

    HOGPRH_DESC_MAX = HOGPRH_DESC_REPORT_CFG + HOGPRH_NB_REPORT_INST_MAX,
};

/// Peer HID service info that can be read/write
enum hogprh_info
{
    /// Protocol Mode
    HOGPRH_PROTO_MODE,
    /// Report Map
    HOGPRH_REPORT_MAP,
    /// Report Map Char. External Report Reference Descriptor
    HOGPRH_REPORT_MAP_EXT_REP_REF,

    /// HID Information
    HOGPRH_HID_INFO,
    /// HID Control Point
    HOGPRH_HID_CTNL_PT,
    /// Report
    HOGPRH_REPORT,
    /// Report Char. Report Reference
    HOGPRH_REPORT_REF,
    /// Report Notification config
    HOGPRH_REPORT_NTF_CFG,

    HOGPRH_INFO_MAX,
};

/*
 * APIs Structure
 ****************************************************************************************
 */

///Structure containing the characteristics handles, value handles and descriptors
typedef struct hogprh_content
{
    /// Service info
    prf_svc_t               svc;

    /// Included service info
    prf_incl_svc_t          incl_svc;

    /// Characteristic info:
    prf_char_t          chars[HOGPRH_CHAR_MAX];

    /// Descriptor handles:
    prf_desc_t     descs[HOGPRH_DESC_MAX];

    /// Number of Report Char. that have been found
    uint8_t                 report_nb;
} hogprh_content_t;

/// Parameters of the @ref HOGPRH_ENABLE_REQ message
struct hogprh_enable_req
{
    /// Connection Index
    uint8_t          conidx;
    /// Connection type
    uint8_t          con_type;

    /// Number of HIDS instances
    uint8_t          hids_nb;
    /// Existing handle values hids
    hogprh_content_t hids[HOGPRH_NB_HIDS_INST_MAX];
};

/// Parameters of the @ref HOGPRH_ENABLE_RSP message
struct hogprh_enable_rsp
{
    /// Connection Index
    uint8_t          conidx;
    ///status
    uint16_t         status;
    /// Number of HIDS instances
    uint8_t          hids_nb;
    /// Existing handle values hids
    hogprh_content_t hids[HOGPRH_NB_HIDS_INST_MAX];
};

/// Information data
union hogprh_data
{
    /// Protocol Mode
    ///  - info = HOGPRH_PROTO_MODE
    uint8_t                 proto_mode;

    /// HID Information value
    ///  - info = HOGPRH_HID_INFO
    hogp_hid_info_t         hid_info;

    /// HID Control Point value to write
    ///  - info = HOGPRH_HID_CTNL_PT
    uint8_t                 hid_ctnl_pt;

    /// Report information
    ///  - info = HOGPRH_REPORT
    hogp_report_t           report;

    ///Notification Configuration Value
    ///  - info = HOGPRH_REPORT_NTF_CFG
    uint16_t                report_cfg;

    /// HID report Reference
    ///  - info = HOGPRH_REPORT_REF
    hogp_report_ref_t       report_ref;

    /// HID report MAP info
    ///  - info = HOGPRH_REPORT_MAP
    hogp_report_t           report_map;

    /// HID report MAP reference
    ///  - info = HOGPRH_REPORT_MAP_EXT_REP_REF
    hogp_report_map_ref_t   report_map_ref;
};

///Parameters of the @ref HOGPRH_READ_INFO_REQ message
struct hogprh_read_info_req
{
    /// Connection Index
    uint8_t conidx;
    ///Characteristic info @see enum hogprh_info
    uint8_t info;
    /// HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
    uint8_t hid_idx;
    /// HID Report Index: only relevant for:
    ///  - info = HOGPRH_REPORT
    ///  - info = HOGPRH_REPORT_REF
    ///  - info = HOGPRH_REPORT_NTF_CFG
    uint8_t report_idx;
};

///Parameters of the @ref HOGPRH_READ_INFO_RSP message
struct hogprh_read_info_rsp
{
    /// Connection Index
    uint8_t           conidx;
    /// status of the request
    uint16_t          status;
    ///Characteristic info @see enum hogprh_info
    uint8_t           info;
    /// HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
    uint8_t           hid_idx;
    /// HID Report Index: only relevant for:
    ///  - info = HOGPRH_REPORT
    ///  - info = HOGPRH_REPORT_REF
    ///  - info = HOGPRH_REPORT_NTF_CFG
    uint8_t           report_idx;
    /// Information data
    union hogprh_data data;
};

///Parameters of the @ref HOGPRH_WRITE_REQ message
struct hogprh_write_req
{
    /// Connection Index
    uint8_t           conidx;
    ///Characteristic info @see enum hogprh_info
    uint8_t           info;
    /// HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
    uint8_t           hid_idx;
    /// HID Report Index: only relevant for:
    ///  - info = HOGPRH_REPORT
    ///  - info = HOGPRH_REPORT_NTF_CFG
    uint8_t           report_idx;
    /// Write type ( Write without Response True or Write Request)
    /// - only valid for HOGPRH_REPORT
    bool              wr_cmd;
    /// Information data
    union hogprh_data data;
};

///Parameters of the @ref HOGPRH_WRITE_RSP message
struct hogprh_write_rsp
{
    /// Connection Index
    uint8_t  conidx;
    /// status of the request
    uint16_t status;
    ///Characteristic info @see enum hogprh_info
    uint8_t  info;
    /// HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
    uint8_t  hid_idx;
    /// HID Report Index: only relevant for:
    ///  - info = HOGPRH_REPORT
    ///  - info = HOGPRH_REPORT_REF
    ///  - info = HOGPRH_REPORT_NTF_CFG
    uint8_t  report_idx;
};

///Parameters of the @ref HOGPRH_BOOT_REPORT_IND message
struct hogprh_report_ind
{
    uint8_t       conidx;
    /// HIDS Instance
    uint8_t       hid_idx;
    /// HID Report Index
    uint8_t       report_idx;
    /// Report data
    hogp_report_t report;
};

/// @} HOGPRH

#endif /* _HOGPRH_MSG_H_ */
