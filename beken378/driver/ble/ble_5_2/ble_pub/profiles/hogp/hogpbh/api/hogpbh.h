/**
 ****************************************************************************************
 *
 * @file hogpbh.h
 *
 * @brief Header file - HID Over GATT Profile Boot Host Role - Native API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


#ifndef _HOGPBH_H_
#define _HOGPBH_H_

/**
 ****************************************************************************************
 * @addtogroup HOGPBH
 * @ingroup Profile
 * @brief HID Over GATT Profile Boot Host - Native API
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


#include "hogpbh_msg.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * NATIVE API CALLBACKS
 ****************************************************************************************
 */

/// HID Over GATT Boot Host client callback set
typedef struct hogpbh_cb
{
    /**
     ****************************************************************************************
     * @brief Completion of enable procedure
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hids_nb       Number of Discovered HIDS instances
     * @param[in] p_hids        Array of peer database description bond data (size = hids_nb)
     *
     ****************************************************************************************
     */
    void (*cb_enable_cmp)(uint8_t conidx, uint16_t status, uint8_t hids_nb, const hogpbh_content_t* p_hids);

    /**
     ****************************************************************************************
     * @brief Completion of read Characteristic Configuration procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
     * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
     *                           - HOGPBH_BOOT_KB_IN_NTF_CFG
     *                           - HOGPBH_BOOT_MOUSE_IN_NTF_CFG
     * @param[in] cfg_val       Configuration value
     *
     ****************************************************************************************
     */
    void (*cb_read_cfg_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t val_id, uint16_t cfg_val);

    /**
     ****************************************************************************************
     * @brief Completion of read Protocol Mode procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
     * @param[in] proto_mode    Protocol mode.
     *
     ****************************************************************************************
     */
    void (*cb_read_proto_mode_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t proto_mode);

    /**
     ****************************************************************************************
     * @brief Completion of read report procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
     * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
     *                           - HOGPBH_BOOT_KB_IN_REPORT
     *                           - HOGPBH_BOOT_KB_OUT_REPORT
     *                           - HOGPBH_BOOT_MOUSE_IN_REPORT
     * @param[in] p_report      Buffer that contains report data
     ****************************************************************************************
     */
    void (*cb_read_report_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t val_id, common_buf_t* p_report);

    /**
     ****************************************************************************************
     * @brief Completion of write Characteristic Configuration procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
     *                           - HOGPBH_BOOT_KB_IN_NTF_CFG
     *                           - HOGPBH_BOOT_MOUSE_IN_NTF_CFG
     *
     ****************************************************************************************
     */
    void (*cb_write_cfg_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t val_id);

    /**
     ****************************************************************************************
     * @brief Completion of write Protocol Mode procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
     *
     ****************************************************************************************
     */
    void (*cb_write_proto_mode_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx);

    /**
     ****************************************************************************************
     * @brief Completion of write report procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
     * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
     *                           - HOGPBH_BOOT_KB_IN_REPORT
     *                           - HOGPBH_BOOT_KB_OUT_REPORT
     *                           - HOGPBH_BOOT_MOUSE_IN_REPORT
     *
     ****************************************************************************************
     */
    void (*cb_write_report_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t val_id);


    /**
     ****************************************************************************************
     * @brief Function called when report information is received
     *
     * @param[in] conidx        Connection index
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
     * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
     *                           - HOGPBH_BOOT_KB_IN_REPORT
     *                           - HOGPBH_BOOT_MOUSE_IN_REPORT
     * @param[in] p_report      Buffer that contains report data
     ****************************************************************************************
     */
    void (*cb_report)(uint8_t conidx, uint8_t hid_idx, uint8_t val_id, common_buf_t* p_report);
} hogpbh_cb_t;

/*
 * NATIVE API FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Restore bond data of a known peer device (at connection establishment)
 *
 * Wait for @see cb_enable_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] con_type      Connection type
 * @param[in] p_rscs        Pointer to peer database description bond data
 * @param[in] hids_nb       Number of Discovered HIDS instances
 * @param[in] p_hids        Array of peer database description bond data (size = hids_nb)
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpbh_enable(uint8_t conidx, uint8_t con_type, uint8_t hids_nb, const hogpbh_content_t* p_hids);

/**
 ****************************************************************************************
 * @brief Perform a read Characteristic Configuration procedure.
 *
 * Wait for @see cb_read_cfg_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
 * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
 *                           - HOGPBH_BOOT_KB_IN_NTF_CFG
 *                           - HOGPBH_BOOT_MOUSE_IN_NTF_CFG
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpbh_read_cfg(uint8_t conidx, uint8_t hid_idx, uint8_t val_id);

/**
 ****************************************************************************************
 * @brief Perform a protocol mode read procedure.
 *
 * Wait for @see cb_read_proto_mode_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpbh_read_proto_mode(uint8_t conidx, uint8_t hid_idx);

/**
 ****************************************************************************************
 * @brief Perform a read report procedure.
 *
 * Wait for @see cb_read_report_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
 * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
 *                           - HOGPBH_BOOT_KB_IN_REPORT
 *                           - HOGPBH_BOOT_KB_OUT_REPORT
 *                           - HOGPBH_BOOT_MOUSE_IN_REPORT
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpbh_read_report(uint8_t conidx, uint8_t hid_idx, uint8_t val_id);

/**
 ****************************************************************************************
 * @brief Perform a write Characteristic Configuration procedure.
 *
 * Wait for @see cb_write_cfg_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
 * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
 *                           - HOGPBH_BOOT_KB_IN_NTF_CFG
 *                           - HOGPBH_BOOT_MOUSE_IN_NTF_CFG
 * @param[in] cfg_val       Configuration value
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpbh_write_cfg(uint8_t conidx, uint8_t hid_idx, uint8_t val_id, uint16_t cfg_val);


/**
 ****************************************************************************************
 * @brief Perform a protocol mode write procedure.
 *
 * Wait for @see cb_write_proto_mode_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
 * @param[in] proto_mode    Protocol mode
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpbh_write_proto_mode(uint8_t conidx, uint8_t hid_idx, uint8_t proto_mode);

/**
 ****************************************************************************************
 * @brief Perform a write report procedure.
 *
 * Wait for @see cb_write_report_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
 * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
 *                           - HOGPBH_BOOT_KB_IN_REPORT
 *                           - HOGPBH_BOOT_KB_OUT_REPORT
 *                           - HOGPBH_BOOT_MOUSE_IN_REPORT
 * @param[in] wr_cmd        Write type ( Write without Response True or Write Request)
 *                          only valid for HOGPBH_BOOT_KB_OUT_REPORT
 * @param[in] p_report      Buffer that contains report data
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogpbh_write_report(uint8_t conidx, uint8_t hid_idx, uint8_t val_id, bool wr_cmd, common_buf_t* p_report);

/// @} HOGPBH

#endif /* _HOGPBH_H_ */
