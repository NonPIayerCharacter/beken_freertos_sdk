/**
 ****************************************************************************************
 *
 * @file hogpbh_msg.h
 *
 * @brief Header file - HID Over GATT Profile Boot Host Role - Message API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


#ifndef _HOGPBH_MSG_H_
#define _HOGPBH_MSG_H_

/**
 ****************************************************************************************
 * @addtogroup HOGPBH
 * @ingroup Profile
 * @brief HID Over GATT Profile Boot Host - Message API
 *
 * This module is responsible for handling the messages coming in and out of the
 * @ref HOGPBH monitor block of the BLE Host.
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
#define HOGPBH_NB_HIDS_INST_MAX              (2)

/// Maximal Length of Boot Report Char. Value
#define HOGPBH_BOOT_REPORT_MAX_LEN           (8)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/*@TRACE*/
enum hogpbh_msg_id
{
    /// Start the HID Over GATT profile - at connection
    HOGPBH_ENABLE_REQ      = MSG_ID(HOGPBH, 0x00),
    ///Confirm that cfg connection has finished with discovery results, or that normal cnx started
    HOGPBH_ENABLE_RSP      = MSG_ID(HOGPBH, 0x01),

    /// Read Characteristic Value Request
    HOGPBH_READ_INFO_REQ   = MSG_ID(HOGPBH, 0x02),
    /// Read Characteristic Value Response
    HOGPBH_READ_INFO_RSP   = MSG_ID(HOGPBH, 0x03),

    /// Write/Configure peer device attribute Request
    HOGPBH_WRITE_REQ       = MSG_ID(HOGPBH, 0x04),
    /// Write/Configure peer device attribute Response
    HOGPBH_WRITE_RSP       = MSG_ID(HOGPBH, 0x05),

    /// Boot Report value send to APP (after Read Request or Notification)
    HOGPBH_BOOT_REPORT_IND = MSG_ID(HOGPBH, 0x06),
};


/// Characteristics
enum hogpbh_chars
{
    /// Protocol Mode
    HOGPBH_CHAR_PROTO_MODE,
    /// Boot Keyboard Input Report
    HOGPBH_CHAR_BOOT_KB_IN_REPORT,
    /// Boot Keyboard Output Report
    HOGPBH_CHAR_BOOT_KB_OUT_REPORT,
    /// Boot Mouse Input Report
    HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT,

    HOGPBH_CHAR_MAX,
};


/// Characteristic descriptors
enum hogpbh_descs
{
    /// Boot Keyboard Input Report Client Config
    HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG,
    /// Boot Mouse Input Report Client Config
    HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG,

    HOGPBH_DESC_MAX,
};


/// Peer HID service info that can be read/write
enum hogpbh_info
{
    /// Protocol Mode
    HOGPBH_PROTO_MODE,
    /// Boot Keyboard Input Report
    HOGPBH_BOOT_KB_IN_REPORT,
    /// Boot Keyboard Output Report
    HOGPBH_BOOT_KB_OUT_REPORT,
    /// Boot Mouse Input Report
    HOGPBH_BOOT_MOUSE_IN_REPORT,

    /// Boot Keyboard Input Report Client Config
    HOGPBH_BOOT_KB_IN_NTF_CFG,
    /// Boot Mouse Input Report Client Config
    HOGPBH_BOOT_MOUSE_IN_NTF_CFG,

    HOGPBH_INFO_MAX,
};

/*
 * APIs Structure
 ****************************************************************************************
 */

///Structure containing the characteristics handles, value handles and descriptors
typedef struct hogpbh_content
{
    /// service info
    prf_svc_t           svc;

    /// characteristic info:
    ///  - Protocol Mode
    ///  - Boot Keyboard Input Report
    ///  - Boot Keyboard Output Report
    ///  - Boot Mouse Input Report
    prf_char_t      chars[HOGPBH_CHAR_MAX];

    /// Descriptor handles:
    ///  - Boot Keyboard Input Client Cfg
    ///  - Boot Mouse Input Client Cfg
    prf_desc_t descs[HOGPBH_DESC_MAX];
} hogpbh_content_t;

/// Parameters of the @ref HOGPBH_ENABLE_REQ message
struct hogpbh_enable_req
{
    /// Connection Index
    uint8_t          conidx;
    /// Connection type
    uint8_t          con_type;
    /// Number of HIDS instances
    uint8_t          hids_nb;
    /// Existing handle values hids
    hogpbh_content_t hids[HOGPBH_NB_HIDS_INST_MAX];
};

/// Parameters of the @ref HOGPBH_ENABLE_RSP message
struct hogpbh_enable_rsp
{
    /// Connection Index
    uint8_t          conidx;
    ///status
    uint16_t         status;
    /// Number of HIDS instances
    uint8_t          hids_nb;
    /// Existing handle values hids
    hogpbh_content_t hids[HOGPBH_NB_HIDS_INST_MAX];
};

/// Information data
union hogpbh_data
{
    /// Protocol Mode
    ///  - info = HOGPBH_PROTO_MODE
    uint8_t              proto_mode;
    ///Notification Configuration Value
    ///  - info = HOGPBH_BOOT_KB_IN_NTF_CFG
    ///  - info = HOGPBH_BOOT_MOUSE_IN_NTF_CFG
    uint16_t             ntf_cfg;

    /// Boot report information
    ///  - info = HOGPBH_BOOT_KB_IN_REPORT
    ///  - info = HOGPBH_BOOT_KB_OUT_REPORT
    ///  - info = HOGPBH_BOOT_MOUSE_IN_REPORT
    hogp_report_t        boot_report;
};



///Parameters of the @ref HOGPBH_READ_INFO_REQ message
struct hogpbh_read_info_req
{
    /// Connection Index
    uint8_t conidx;
    ///Characteristic info @see enum hogpbh_info
    uint8_t info;
    /// HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
    uint8_t hid_idx;
};

///Parameters of the @ref HOGPBH_READ_INFO_RSP message
struct hogpbh_read_info_rsp
{
    /// Connection Index
    uint8_t           conidx;
    /// status of the request
    uint16_t          status;
    ///Characteristic info @see enum hogpbh_info
    uint8_t           info;
    /// HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
    uint8_t           hid_idx;
    /// Information data
    union hogpbh_data data;
};


///Parameters of the @ref HOGPBH_WRITE_REQ message
struct hogpbh_write_req
{
    /// Connection Index
    uint8_t           conidx;
    ///Characteristic info @see enum hogpbh_info
    uint8_t           info;
    /// HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
    uint8_t           hid_idx;
    /// Write type ( Write without Response True or Write Request)
    /// - only valid for HOGPBH_BOOT_KB_OUT_REPORT
    bool              wr_cmd;
    /// Information data
    union hogpbh_data data;
};


///Parameters of the @ref HOGPBH_WRITE_RSP message
struct hogpbh_write_rsp
{
    /// Connection Index
    uint8_t  conidx;
    /// status of the request
    uint16_t status;
    ///Characteristic info @see enum hogpbh_info
    uint8_t  info;
    /// HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
    uint8_t  hid_idx;
};


///Parameters of the @ref HOGPBH_BOOT_REPORT_IND message
struct hogpbh_boot_report_ind
{
    /// Connection Index
    uint8_t              conidx;
    ///Characteristic info @see enum hogpbh_info
    uint8_t              info;
    /// HIDS Instance
    uint8_t              hid_idx;
    /// Boot Report data
    hogp_report_t        boot_report;
};

/// @} HOGPBH

#endif /* _HOGPBH_MSG_H_ */
