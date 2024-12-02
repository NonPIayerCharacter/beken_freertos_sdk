/**
 ****************************************************************************************
 *
 * @file hogpd_msg.h
 *
 * @brief Header file - HID Over GATT Profile Device - Message API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 *
 ****************************************************************************************
 */


#ifndef _HOGPD_MSG_H_
#define _HOGPD_MSG_H_

/**
 ****************************************************************************************
 * @addtogroup HOGPD
 * @ingroup Profile
 * @brief HID Over GATT Profile - Message API.
 *
 * This module is responsible for handling the messages coming in and out of the
 * @ref HOGPD block of the BLE Host.
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

/// Maximal number of HIDS that can be added in the DB
#define HOGPD_NB_HIDS_INST_MAX              (2)
/// Maximal number of Report Char. that can be added in the DB for one HIDS - Up to 11
#define HOGPD_NB_REPORT_INST_MAX            (5)
/// Valid Feature mask
#define HOGPD_CFG_MASK                      0x3F
/// Report Notification Enabled (to be shift for each report index)
#define HOGPD_CFG_REPORT_NTF_EN             0x40

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Messages for HID Over GATT Profile Device Role
/*@TRACE*/
enum hogpd_msg_id
{
    /// Restore bond data the HID Over GATT Profile Device Role Task
    HOGPD_ENABLE_REQ            = MSG_ID(HOGPD, 0x00),
    /// Response of enabled request
    HOGPD_ENABLE_RSP            = MSG_ID(HOGPD, 0x01),

    /// Request sending of a report to the host - notification
    HOGPD_REPORT_UPD_REQ        = MSG_ID(HOGPD, 0x02),
    /// Response sending of a report to the host
    HOGPD_REPORT_UPD_RSP        = MSG_ID(HOGPD, 0x03),

    /// Request from peer device to Read or update a report value
    HOGPD_REPORT_READ_REQ_IND   = MSG_ID(HOGPD, 0x04),
    /// Confirmation for peer device for Reading or Updating a report value
    HOGPD_REPORT_READ_CFM       = MSG_ID(HOGPD, 0x05),

    /// Request from peer device to Read or update a report value
    HOGPD_REPORT_WRITE_REQ_IND  = MSG_ID(HOGPD, 0x06),
    /// Confirmation for peer device for Reading or Updating a report value
    HOGPD_REPORT_WRITE_CFM      = MSG_ID(HOGPD, 0x07),

    /// Inform Device APP that Protocol Mode Characteristic Value has been written on Device
    HOGPD_PROTO_MODE_REQ_IND    = MSG_ID(HOGPD, 0x08),
    /// Confirm if the new protocol mode value
    HOGPD_PROTO_MODE_CFM        = MSG_ID(HOGPD, 0x09),

    /// Inform Device APP that a Client Characteristic Configuration has been modified
    HOGPD_NTF_CFG_IND           = MSG_ID(HOGPD, 0x0A),
    /// Inform APP that HID Control Point Characteristic Value has been written on Device
    HOGPD_CTNL_PT_IND           = MSG_ID(HOGPD, 0x0B),
};

/// Report Char. Configuration Flag Values
enum hogpd_report_cfg
{
    /// Input Report
    HOGPD_CFG_REPORT_IN     = 0x01,
    /// Output Report
    HOGPD_CFG_REPORT_OUT    = 0x02,
    // HOGPD_CFG_REPORT_FEAT can be used as a mask to check Report type
    /// Feature Report
    HOGPD_CFG_REPORT_FEAT   = 0x03,
    /// Input report with Write capabilities
    HOGPD_CFG_REPORT_WR     = 0x10,
};

/// Features Flag Values bit field
enum hogpd_cfg_bf
{
    /// Keyboard Device
    HOGPD_CFG_KEYBOARD_POS          = 0,
    HOGPD_CFG_KEYBOARD_BIT          = COMMON_BIT(HOGPD_CFG_KEYBOARD_POS),

    /// Mouse Device
    HOGPD_CFG_MOUSE_POS             = 1,
    HOGPD_CFG_MOUSE_BIT             = COMMON_BIT(HOGPD_CFG_MOUSE_POS),

    /// Protocol Mode present
    HOGPD_CFG_PROTO_MODE_POS        = 2,
    HOGPD_CFG_PROTO_MODE_BIT        = COMMON_BIT(HOGPD_CFG_PROTO_MODE_POS),

    /// Extended Reference Present
    HOGPD_CFG_MAP_EXT_REF_POS       = 3,
    HOGPD_CFG_MAP_EXT_REF_BIT       = COMMON_BIT(HOGPD_CFG_MAP_EXT_REF_POS),

    /// Boot Keyboard Report write capability
    HOGPD_CFG_BOOT_KB_WR_POS        = 4,
    HOGPD_CFG_BOOT_KB_WR_BIT        = COMMON_BIT(HOGPD_CFG_BOOT_KB_WR_POS),

    /// Boot Mouse Report write capability
    HOGPD_CFG_BOOT_MOUSE_WR_POS     = 5,
    HOGPD_CFG_BOOT_MOUSE_WR_BIT     = COMMON_BIT(HOGPD_CFG_BOOT_MOUSE_WR_POS),
};

/// Type of reports
enum hogpd_report_type
{
    /// The Report characteristic is used to exchange data between a HID Device and a HID Host.
    HOGPD_REPORT,
    /// The Report Map characteristic
    HOGPD_REPORT_MAP,
    /// Boot Keyboard Input Report
    HOGPD_BOOT_KEYBOARD_INPUT_REPORT,
    /// Boot Keyboard Output Report
    HOGPD_BOOT_KEYBOARD_OUTPUT_REPORT,
    /// Boot Mouse Input Report
    HOGPD_BOOT_MOUSE_INPUT_REPORT,
};

/*
 * APIs Structures
 ****************************************************************************************
 */

/// External Report Reference
typedef struct hogpd_ext_ref
{
    /// External Report Reference - Included Service
    uint16_t inc_svc_hdl;
    /// External Report Reference - Characteristic UUID
    uint16_t rep_ref_uuid;
} hogpd_ext_ref_t;

/// Database Creation Service Instance Configuration structure
typedef struct hogpd_hids_cfg
{
    /// Service Features (@see enum hogpd_cfg)
    uint8_t             svc_features;
    /// Number of Report Char. instances to add in the database
    uint8_t             report_nb;
    /// Report Char. Configuration (@see enum hogpd_report_cfg)
    uint8_t             report_char_cfg[HOGPD_NB_REPORT_INST_MAX];
    /// Report id number
    uint8_t             report_id[HOGPD_NB_REPORT_INST_MAX];
    /// HID Information Char. Values
    hogp_hid_info_t     hid_info;
    /// External Report Reference
    hogpd_ext_ref_t     ext_ref;
} hogpd_hids_cfg_t;

/// Parameters of the @ref HOGPD_CREATE_DB_REQ message
struct hogpd_db_cfg
{
    /// Number of HIDS to add
    uint8_t          hids_nb;
    /// Initial configuration for each HIDS instance
    hogpd_hids_cfg_t cfg[HOGPD_NB_HIDS_INST_MAX];
};


/// Parameters of the @ref HOGPD_ENABLE_REQ message
struct hogpd_enable_req
{
    ///Connection index
    uint8_t  conidx;
    /// Notification Configurations
    uint16_t ntf_cfg[HOGPD_NB_HIDS_INST_MAX];
};

/// Parameters of the @ref HOGPD_ENABLE_RSP message
struct hogpd_enable_rsp
{
    ///Connection index
    uint8_t  conidx;
    /// status of the request
    uint16_t status;
};

///Parameters of the @ref HOGPD_NTF_CFG_IND message
struct hogpd_ntf_cfg_ind
{
    /// Connection Index
    uint8_t  conidx;
    /// Notification Configurations
    uint16_t ntf_cfg[HOGPD_NB_HIDS_INST_MAX];
};

/// Inform Device APP that Protocol Mode Characteristic Value has been written on Device
struct hogpd_proto_mode_req_ind
{
    /// Connection Index
    uint8_t  conidx;
    /// Token value that must be returned in confirmation
    uint16_t token;
    /// HIDS Instance
    uint8_t  hid_idx;
    /// New Protocol Mode Characteristic Value
    uint8_t  proto_mode;
};

/// Confirm if the new protocol mode value
struct hogpd_proto_mode_cfm
{
    /// Connection Index
    uint8_t  conidx;
    /// Token value that provided in request
    uint16_t token;
    /// Status of the request
    uint16_t status;
    /// HIDS Instance
    uint8_t  hid_idx;
    /// New Protocol Mode Characteristic Value
    uint8_t  proto_mode;
};


/// Request sending of a report to the host - notification
struct hogpd_report_upd_req
{
    /// Connection Index
    uint8_t             conidx;
    /// HIDS Instance
    uint8_t             hid_idx;
    /// type of report (@see enum hogpd_report_type)
    uint8_t             report_type;
    /// Report Instance - 0 for boot reports and report map
    uint8_t             report_idx;
    /// Report Info
    hogp_report_t       report;
};

/// Response sending of a report to the host
struct hogpd_report_upd_rsp
{
    /// Connection Index
    uint8_t             conidx;
    /// Status of the request
    uint16_t            status;
};

/// Request from peer device to Read or update a report value
struct hogpd_report_read_req_ind
{
    /// Connection Index
    uint8_t             conidx;
    /// Token value that must be returned in confirmation
    uint16_t            token;
    /// HIDS Instance
    uint8_t             hid_idx;
    /// type of report (@see enum hogpd_report_type)
    uint8_t             report_type;
    /// Report Instance - 0 for boot reports and report map
    uint8_t             report_idx;
    /// Data offset requested for read value
    uint16_t            offset;
    /// Maximum data length is response value (starting from offset)
    uint16_t            max_length;
};

/// Confirmation for peer device for Reading or Updating a report value
struct hogpd_report_read_cfm
{
    /// Connection Index
    uint8_t             conidx;
    /// Token value that provided in request
    uint16_t            token;
    /// Status of the request
    uint16_t            status;
    /// Complete report length (without considering offset)
    uint16_t            tot_length;
    /// Report Info
    hogp_report_t       report;
};


/// Request from peer device to Read or update a report value
struct hogpd_report_write_req_ind
{
    /// Connection Index
    uint8_t             conidx;
    /// Token value that must be returned in confirmation
    uint16_t            token;
    /// HIDS Instance
    uint8_t             hid_idx;
    /// type of report (@see enum hogpd_report_type)
    uint8_t             report_type;
    /// Report Instance - 0 for boot reports and report map
    uint8_t             report_idx;
    /// Report Info
    hogp_report_t       report;
};

/// Confirmation for peer device for Reading or Updating a report value
struct hogpd_report_write_cfm
{
    /// Connection Index
    uint8_t             conidx;
    /// Token value that provided in request
    uint16_t            token;
    /// Status of the request
    uint16_t            status;
};

///Parameters of the @ref HOGPD_CTNL_PT_IND message
struct hogpd_ctnl_pt_ind
{
    /// Connection Index
    uint8_t conidx;
    /// HIDS Instance
    uint8_t hid_idx;
    /// New HID Control Point Characteristic Value
    uint8_t hid_ctnl_pt;
};

/// @} HOGPD

#endif /* _HOGPD_MSG_H_ */
