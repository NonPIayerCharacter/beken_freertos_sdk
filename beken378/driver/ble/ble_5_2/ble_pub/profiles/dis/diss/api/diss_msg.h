/**
 ****************************************************************************************
 *
 * @file diss_msg.h
 *
 * @brief Header file - Device Information Service Server - Message API.
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 *
 ****************************************************************************************
 */

#ifndef DISS_MSG_H_
#define DISS_MSG_H_

/**
 ****************************************************************************************
 * @addtogroup DISS Device Information Service Server
 * @brief Device Information Service Server - Message API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_task.h" // Task definitions
#include <stdint.h>
#include "prf_types.h"
#include "common_math.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///All features are supported
#define DIS_ALL_FEAT_SUP                     (0x01FF)


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Messages for Device Information Service Server
/*@TRACE*/
enum diss_msg_id
{
    ///Set the value of an attribute - Request
    DISS_SET_VALUE_REQ = MSG_ID(DISS, 0x00),
    ///Set the value of an attribute - Response
    DISS_SET_VALUE_RSP = MSG_ID(DISS, 0x01),
    /// Peer device request to get profile attribute value
    DISS_VALUE_REQ_IND = MSG_ID(DISS, 0x02),
    /// Peer device confirm value of requested attribute
    DISS_VALUE_CFM     = MSG_ID(DISS, 0x03),
};

/// Attribute Table Indexes
enum diss_val_id
{
    /// Manufacturer Name
    DIS_VAL_MANUFACTURER_NAME,
    /// Model Number
    DIS_VAL_MODEL_NB_STR,
    /// Serial Number
    DIS_VAL_SERIAL_NB_STR,
    /// HW Revision Number
    DIS_VAL_HARD_REV_STR,
    /// FW Revision Number
    DIS_VAL_FIRM_REV_STR,
    /// SW Revision Number
    DIS_VAL_SW_REV_STR,
    /// System Identifier Name
    DIS_VAL_SYSTEM_ID,
    /// IEEE Certificate
    DIS_VAL_IEEE,
    /// Plug and Play Identifier
    DIS_VAL_PNP_ID,

    DIS_VAL_MAX,
};

/// Database Configuration Flags (not used)
enum diss_features_bf
{
    ///Indicate if Manufacturer Name String Char. is supported
    DIS_MANUFACTURER_NAME_CHAR_SUP_POS          = 0,
    DIS_MANUFACTURER_NAME_CHAR_SUP_BIT          = COMMON_BIT(DIS_MANUFACTURER_NAME_CHAR_SUP_POS),

    ///Indicate if Model Number String Char. is supported
    DIS_MODEL_NB_STR_CHAR_SUP_POS               = 1,
    DIS_MODEL_NB_STR_CHAR_SUP_BIT               = COMMON_BIT(DIS_MODEL_NB_STR_CHAR_SUP_POS),

    ///Indicate if Serial Number String Char. is supported
    DIS_SERIAL_NB_STR_CHAR_SUP_POS              = 2,
    DIS_SERIAL_NB_STR_CHAR_SUP_BIT              = COMMON_BIT(DIS_SERIAL_NB_STR_CHAR_SUP_POS),

    ///Indicate if Hardware Revision String Char. supports indications
    DIS_HARD_REV_STR_CHAR_SUP_POS               = 3,
    DIS_HARD_REV_STR_CHAR_SUP_BIT               = COMMON_BIT(DIS_HARD_REV_STR_CHAR_SUP_POS),

    ///Indicate if Firmware Revision String Char. is writable
    DIS_FIRM_REV_STR_CHAR_SUP_POS               = 4,
    DIS_FIRM_REV_STR_CHAR_SUP_BIT               = COMMON_BIT(DIS_FIRM_REV_STR_CHAR_SUP_POS),

    ///Indicate if Software Revision String Char. is writable
    DIS_SW_REV_STR_CHAR_SUP_POS                 = 5,
    DIS_SW_REV_STR_CHAR_SUP_BIT                 = COMMON_BIT(DIS_SW_REV_STR_CHAR_SUP_POS),

    ///Indicate if System ID Char. is writable
    DIS_SYSTEM_ID_CHAR_SUP_POS                  = 6,
    DIS_SYSTEM_ID_CHAR_SUP_BIT                  = COMMON_BIT(DIS_SYSTEM_ID_CHAR_SUP_POS),

    ///Indicate if IEEE Char. is writable
    DIS_IEEE_CHAR_SUP_POS                       = 7,
    DIS_IEEE_CHAR_SUP_BIT                       = COMMON_BIT(DIS_IEEE_CHAR_SUP_POS),

    ///Indicate if PnP ID Char. is writable
    DIS_PNP_ID_CHAR_SUP_POS                     = 8,
    DIS_PNP_ID_CHAR_SUP_BIT                     = COMMON_BIT(DIS_PNP_ID_CHAR_SUP_POS),
};

/*
 * API MESSAGES STRUCTURES
 ****************************************************************************************
 */

/// Parameters for the database creation
struct diss_db_cfg
{
    /// Database configuration @see enum diss_features_bf
    uint16_t features;
};

/// Set the value of an attribute - Request
struct diss_set_value_req
{
    /// Value identifier (@see enum diss_val_id)
    uint8_t val_id;
    /// Value length
    uint8_t length;
    /// Value data
    uint8_t data[__ARRAY_EMPTY];
};

/// Set the value of an attribute - Response
struct diss_set_value_rsp
{
    /// Value identifier (@see enum diss_val_id)
    uint8_t  val_id;
    /// status of the request
    uint16_t status;
};

/// Peer device request to get profile attribute value
struct diss_value_req_ind
{
    /// Token value that must be returned in confirmation
    uint32_t token;
    /// Value identifier (@see enum diss_val_id)
    uint8_t  val_id;
};

/// Peer device  value of requested attribute
struct diss_value_cfm
{
    /// Token value provided in request
    uint32_t token;
    /// Value identifier (@see enum diss_val_id)
    uint8_t  val_id;
    /// Data length (starting from offset)
    uint8_t  length;
    /// Value data (starting from offset)
    uint8_t  data[__ARRAY_EMPTY];
};

/// @} DISS
#endif // DISS_MSG_H_
