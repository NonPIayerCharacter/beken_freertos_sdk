/**
 ****************************************************************************************
 *
 * @file hogprh.h
 *
 * @brief Header file - HID Over GATT Profile Report Host Role - Native API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 *
 ****************************************************************************************
 */


#ifndef _HOGPRH_H_
#define _HOGPRH_H_

/**
 ****************************************************************************************
 * @addtogroup HOGPRH
 * @ingroup Profile
 * @brief HID Over GATT Profile Report Host - Native API
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


#include "hogprh_msg.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * NATIVE API CALLBACKS
 ****************************************************************************************
 */

/// HID Over GATT Boot Host client callback set
typedef struct hogprh_cb
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
    void (*cb_enable_cmp)(uint8_t conidx, uint16_t status, uint8_t hids_nb, const hogprh_content_t* p_hids);

    /**
     ****************************************************************************************
     * @brief Completion of read Protocol Mode procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     * @param[in] proto_mode    Protocol mode.
     *
     ****************************************************************************************
     */
    void (*cb_read_proto_mode_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t proto_mode);

    /**
     ****************************************************************************************
     * @brief Completion of read HID information procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     * @param[in] p_hid_info    Pointer to HID information data.
     *
     ****************************************************************************************
     */
    void (*cb_read_hid_info_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx,
                                 const hogp_hid_info_t* p_hid_info);

    /**
     ****************************************************************************************
     * @brief Completion of read report map procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     * @param[in] p_report_map  Buffer that contains report map data
     ****************************************************************************************
     */
    void (*cb_read_report_map_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, common_buf_t* p_report_map);

    /**
     ****************************************************************************************
     * @brief Completion of read report reference procedure.
     *
     * @param[in] conidx            Connection index
     * @param[in] status            Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx           HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     * @param[in] p_report_map_ref  Pointer to report map reference information
     ****************************************************************************************
     */
    void (*cb_read_report_map_ref_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx,
                                       const hogp_report_map_ref_t* p_report_map_ref);

    /**
     ****************************************************************************************
     * @brief Completion of read report Characteristic Configuration procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     * @param[in] report_idx    Index of the report in the service
     * @param[in] cfg_val       Configuration value
     *
     ****************************************************************************************
     */
    void (*cb_read_report_cfg_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx, uint16_t cfg_val);

    /**
     ****************************************************************************************
     * @brief Completion of read report procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     * @param[in] report_idx    Index of the report in the service
     * @param[in] p_report      Buffer that contains report data
     ****************************************************************************************
     */
    void (*cb_read_report_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx, common_buf_t* p_report);

    /**
     ****************************************************************************************
     * @brief Completion of read report reference procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     * @param[in] report_idx    Index of the report in the service
     * @param[in] p_report_ref  Pointer to report reference information
     ****************************************************************************************
     */
    void (*cb_read_report_ref_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx,
                                   const hogp_report_ref_t* p_report_ref);

    /**
     ****************************************************************************************
     * @brief Completion of write Protocol Mode procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     *
     ****************************************************************************************
     */
    void (*cb_write_proto_mode_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx);

    /**
     ****************************************************************************************
     * @brief Completion of write control point procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     ****************************************************************************************
     */
    void (*cb_write_ctnl_pt_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx);

    /**
     ****************************************************************************************
     * @brief Completion of write Characteristic Configuration procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] report_idx    Index of the report in the service
     ****************************************************************************************
     */
    void (*cb_write_report_cfg_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx);

    /**
     ****************************************************************************************
     * @brief Completion of write report procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (@see enum hl_err)
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     * @param[in] report_idx    Index of the report in the service
     ****************************************************************************************
     */
    void (*cb_write_report_cmp)(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx);

    /**
     ****************************************************************************************
     * @brief Function called when report information is received
     *
     * @param[in] conidx        Connection index
     * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
     * @param[in] report_idx    Index of the report in the service
     * @param[in] p_report      Buffer that contains report data
     ****************************************************************************************
     */
    void (*cb_report)(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx, common_buf_t* p_report);
} hogprh_cb_t;

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
uint16_t hogprh_enable(uint8_t conidx, uint8_t con_type, uint8_t hids_nb, const hogprh_content_t* p_hids);

/**
 ****************************************************************************************
 * @brief Perform a protocol mode read procedure.
 *
 * Wait for @see cb_read_proto_mode_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_read_proto_mode(uint8_t conidx, uint8_t hid_idx);

/**
 ****************************************************************************************
 * @brief Perform a HID information read procedure.
 *
 * Wait for @see cb_read_hid_info_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_read_hid_info(uint8_t conidx, uint8_t hid_idx);

/**
 ****************************************************************************************
 * @brief Perform a report map read procedure.
 *
 * Wait for @see cb_read_report_map_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_read_report_map(uint8_t conidx, uint8_t hid_idx);

/**
 ****************************************************************************************
 * @brief Perform a report map reference read procedure.
 *
 * Wait for @see cb_read_report_map_ref_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_read_report_map_ref(uint8_t conidx, uint8_t hid_idx);

/**
 ****************************************************************************************
 * @brief Perform a read report Characteristic Configuration procedure.
 *
 * Wait for @see cb_read_report_cfg_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] report_idx    Index of the report in the service
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_read_report_cfg(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx);

/**
 ****************************************************************************************
 * @brief Perform a read report procedure.
 *
 * Wait for @see cb_read_report_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] report_idx    Index of the report in the service
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_read_report(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx);

/**
 ****************************************************************************************
 * @brief Perform a read report reference procedure.
 *
 * Wait for @see cb_read_report_ref_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] report_idx    Index of the report in the service
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_read_report_ref(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx);

/**
 ****************************************************************************************
 * @brief Perform a protocol mode write procedure.
 *
 * Wait for @see cb_write_proto_mode_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] proto_mode    Protocol mode
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_write_proto_mode(uint8_t conidx, uint8_t hid_idx, uint8_t proto_mode);

/**
 ****************************************************************************************
 * @brief Perform a control point write procedure.
 *
 * Wait for @see cb_write_proto_mode_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] value         Control Point value
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_write_ctnl_pt(uint8_t conidx, uint8_t hid_idx, uint8_t value);

/**
 ****************************************************************************************
 * @brief Perform a write report Characteristic Configuration procedure.
 *
 * Wait for @see cb_write_report_cfg_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] report_idx    Index of the report in the service
 * @param[in] cfg_val       Configuration value
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_write_report_cfg(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx, uint16_t cfg_val);

/**
 ****************************************************************************************
 * @brief Perform a write report procedure.
 *
 * Wait for @see cb_write_report_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] report_idx    Index of the report in the service
 * @param[in] wr_cmd        Write type ( Write without Response True or Write Request)
 *                          only valid for HOGPRH_BOOT_KB_OUT_REPORT
 * @param[in] p_report      Buffer that contains report data
 *
 * @return Status of the function execution (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t hogprh_write_report(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx, bool wr_cmd, common_buf_t* p_report);

/// @} HOGPRH

#endif /* _HOGPRH_H_ */
