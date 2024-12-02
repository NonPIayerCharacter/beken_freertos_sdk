/**
 ****************************************************************************************
 *
 * @file hogpd.h
 *
 * @brief Header file - HID Over GATT Profile Device - Native API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


#ifndef _HOGPD_H_
#define _HOGPD_H_

/**
 ****************************************************************************************
 * @addtogroup HOGPD
 * @ingroup Profile
 * @brief HID Over GATT Profile - Native API.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "hogpd_msg.h"


/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/*
 * NATIVE API CALLBACKS
 ****************************************************************************************
 */

/// HID over GATT device server callback set
typedef struct hogpd_cb
{
    /**
     ****************************************************************************************
     * @brief Inform that Bond data updated for the connection.
     *
     * @param[in] conidx        Connection index
     * @param[in] p_ntf_cfg     Pointer to array of Notification Configurations
     *                          Array of HOGPD_NB_HIDS_INST_MAX size.
     ****************************************************************************************
     */
    void (*cb_bond_data_upd)(uint8_t conidx, const uint16_t* p_ntf_cfg);

    /**
     ****************************************************************************************
     * @brief Inform that Peer device request to update the protocol mode
     *
     * @note Request must be confirmed using @see  hogpd_proto_upd_cfm function.
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token value that must be returned in confirmation
     * @param[in] hid_idx       HIDS Instance
     * @param[in] proto_mode    New Protocol Mode Characteristic Value
     ****************************************************************************************
     */
    void (*cb_proto_upd_req)(uint8_t conidx, uint16_t token, uint8_t hid_idx, uint8_t proto_mode);

    /**
     ****************************************************************************************
     * @brief Inform that Peer device request to read report information
     *
     * @note Request must be confirmed using @see  hogpd_report_read_cfm function.
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token value that must be returned in confirmation
     * @param[in] hid_idx       HIDS Instance
     * @param[in] report_idx    Report Instance - 0 for boot reports and report map
     * @param[in] report_type   Type of report (@see enum hogpd_report_type)
     * @param[in] offset        Data offset requested for read value
     * @param[in] max_length    Maximum data length is response value
     ****************************************************************************************
     */
    void (*cb_report_read_req)(uint8_t conidx, uint16_t token, uint8_t hid_idx, uint8_t report_idx, uint8_t report_type,
                               uint16_t offset, uint16_t max_length);

    /**
     ****************************************************************************************
     * @brief Inform that Peer device request to update report information
     *
     * @note Request must be confirmed using @see  hogpd_report_write_cfm function.
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token value that must be returned in confirmation
     * @param[in] hid_idx       HIDS Instance
     * @param[in] report_idx    Report Instance - 0 for boot reports and report map
     * @param[in] report_type   Type of report (@see enum hogpd_report_type)
     * @param[in] p_buf         Report information data
     ****************************************************************************************
     */
    void (*cb_report_write_req)(uint8_t conidx, uint16_t token, uint8_t hid_idx, uint8_t report_idx, uint8_t report_type,
                                common_buf_t* p_buf);

    /**
     ****************************************************************************************
     * @brief Inform that a control point update has been received
     *
     * @param[in] conidx        Connection index
     * @param[in] hid_idx       HIDS Instance
     * @param[in] value   New HID Control Point Characteristic Value
     ****************************************************************************************
     */
    void (*cb_ctnl_pt)(uint8_t conidx, uint8_t hid_idx, uint8_t value);

    /**
     ****************************************************************************************
     * @brief Inform that report update procedure is done
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     ****************************************************************************************
     */
    void (*cb_report_upd_cmp)(uint8_t conidx, uint16_t status);
} hogpd_cb_t;

/*
 * NATIVE API FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Restore bond data of a known peer device (at connection establishment)
 *
 * @param[in] conidx          Connection index
 * @param[in] p_ntf_cfg       Pointer to array of Notification Configurations
 *                            Array of HOGPD_NB_HIDS_INST_MAX size.
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpd_enable(uint8_t conidx, const uint16_t* p_ntf_cfg);

/**
 ****************************************************************************************
 * @brief Confirm Update of protocol mode update
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token value that provided in request
 * @param[in] status        Status of the request
 * @param[in] hid_idx       HIDS Instance
 * @param[in] proto_mode    New Protocol Mode Characteristic Value
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpd_proto_upd_cfm(uint8_t conidx, uint16_t token, uint16_t status, uint8_t hid_idx, uint8_t proto_mode);

/**
 ****************************************************************************************
 * @brief Provide data of the report that should be read
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token value that provided in request
 * @param[in] status        Status of the request
 * @param[in] tot_length    Complete report length (without considering offset)
 * @param[in] p_buf         Report information data - starting from requested offset
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpd_report_read_cfm(uint8_t conidx, uint16_t token, uint16_t status, uint16_t tot_length, common_buf_t* p_buf);


/**
 ****************************************************************************************
 * @brief Confirm or not update of report.
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token value that provided in request
 * @param[in] status        Status of the request
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpd_report_write_cfm(uint8_t conidx, uint16_t token, uint16_t status);

/**
 ****************************************************************************************
 * @brief Update report information
 *
 * Wait for @see cb_report_upd_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HIDS Instance
 * @param[in] report_idx    Report Instance - 0 for boot reports and report map
 * @param[in] report_type   Type of report (@see enum hogpd_report_type)
 * @param[in] p_buf         Report information data
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpd_report_upd(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx, uint8_t report_type, common_buf_t* p_buf);
/// @} HOGPD

#endif /* _HOGPD_H_ */
