/**
 ****************************************************************************************
 *
 * @file hogpbh.c
 *
 * @brief HID Over GATT Profile - Boot Host Role Implementation
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HOGPBH
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BLE_HID_BOOT_HOST)

#include "hogpbh.h"
#include "gap.h"
#include "gatt.h"
#include "prf_utils.h"
#include "prf.h"

#include "common_utils.h"
#include "common_endian.h"

#include <string.h>
#include "kernel_mem.h"


/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of Client task instances

/// Content of HOGPBH dummy bit field
enum hogpbh_dummy_bf
{
    /// HID Instance
    HOGPBH_DUMMY_HID_IDX_MASK  = 0x00FF,
    HOGPBH_DUMMY_HID_IDX_LSB   = 0,
    /// Value Identifier
    HOGPBH_DUMMY_VAL_ID_MASK   = 0xFF00,
    HOGPBH_DUMMY_VAL_ID_LSB    = 8,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Environment variable for each Connections
typedef struct hogpbh_cnx_env
{
    ///HIDS characteristics
    hogpbh_content_t hids[HOGPBH_NB_HIDS_INST_MAX];
    ///Number of HIDS instances found
    uint8_t          nb_svc;
    /// Client is in discovering state
    bool             discover;
} hogpbh_cnx_env_t;

/// Client environment variable
typedef struct hogpbh_env
{
    /// profile environment
    prf_hdr_t            prf_env;
    /// Environment variable pointer for each connections
    hogpbh_cnx_env_t*    p_env[BLE_CONNECTION_MAX];
    /// GATT User local identifier
    uint8_t              user_lid;
} hogpbh_env_t;


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// State machine used to retrieve HID Service characteristics information
const prf_char_def_t hogpbh_hids_char[HOGPBH_CHAR_MAX] =
{
    /// Protocol Mode
    [HOGPBH_CHAR_PROTO_MODE]             = { GATT_CHAR_PROTOCOL_MODE,        ATT_REQ(PRES, MAND), PROP(RD) | PROP(WC) },
    /// Boot Keyboard Input Report
    [HOGPBH_CHAR_BOOT_KB_IN_REPORT]      = { GATT_CHAR_BOOT_KB_IN_REPORT,    ATT_REQ(PRES, OPT),  PROP(RD) | PROP(N)  },
    /// Boot Keyboard Output Report
    [HOGPBH_CHAR_BOOT_KB_OUT_REPORT]     = { GATT_CHAR_BOOT_KB_OUT_REPORT,   ATT_REQ(PRES, OPT),  PROP(RD) | PROP(WR) },
    /// Boot Keyboard Output Report
    [HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT]   = { GATT_CHAR_BOOT_MOUSE_IN_REPORT, ATT_REQ(PRES, OPT),  PROP(RD) | PROP(N)  },
};

/// State machine used to retrieve HID Service characteristic description information
const prf_desc_def_t hogpbh_hids_char_desc[HOGPBH_DESC_MAX] =
{
    /// Boot Keyboard Input Report Client Config
    [HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG]    = { GATT_DESC_CLIENT_CHAR_CFG, ATT_REQ(PRES, OPT), HOGPBH_CHAR_BOOT_KB_IN_REPORT    },
    /// Boot Mouse Input Report Client Config
    [HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG] = { GATT_DESC_CLIENT_CHAR_CFG, ATT_REQ(PRES, OPT), HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT },
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send discovery results to application.
 *
 * @param p_hogpbh_env    Client Role task environment
 * @param conidx        Connection index
 * @param status        Response status code
 *****************************************************************************************
 */
__STATIC void hogpbh_enable_cmp(hogpbh_env_t* p_hogpbh_env, uint8_t conidx, uint16_t status)
{
    const hogpbh_cb_t* p_cb = (const hogpbh_cb_t*) p_hogpbh_env->prf_env.p_cb;

    if(p_hogpbh_env != NULL)
    {
        hogpbh_cnx_env_t* p_con_env = p_hogpbh_env->p_env[conidx];

        p_cb->cb_enable_cmp(conidx, status, p_con_env->nb_svc, p_con_env->hids);

        if (status != GAP_ERR_NO_ERROR)
        {
            // clean-up environment variable allocated for task instance
            kernel_free(p_con_env);
            p_hogpbh_env->p_env[conidx] = NULL;
        }
        else
        {
            uint8_t svc_idx;
            p_con_env->discover = false;

            for(svc_idx = 0 ; svc_idx < p_con_env->nb_svc ; svc_idx++)
            {
                // Register profile handle to catch gatt indications
                gatt_cli_event_register(conidx, p_hogpbh_env->user_lid, p_con_env->hids[svc_idx].svc.shdl,
                                        p_con_env->hids[svc_idx].svc.ehdl);
            }
        }
    }
}

/**
 ****************************************************************************************
 * @brief Send read result to application,.
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the request
 * @param[in] dummy         Dummy token used to retrieve read on-going
 * @param[in] p_data        Pointer of buffer that contains data value
 ****************************************************************************************
 */
__STATIC void hogpbh_read_val_cmp(uint8_t conidx, uint16_t status, uint16_t dummy, common_buf_t* p_data)
{
    hogpbh_env_t* p_hogpbh_env = PRF_ENV_GET(HOGPBH, hogpbh);

    if(p_hogpbh_env != NULL)
    {
        const hogpbh_cb_t* p_cb = (const hogpbh_cb_t*) p_hogpbh_env->prf_env.p_cb;
        uint8_t hid_idx      = GETF(dummy, HOGPBH_DUMMY_HID_IDX);
        uint8_t val_id       = GETF(dummy, HOGPBH_DUMMY_VAL_ID);

        switch(val_id)
        {
            case HOGPBH_PROTO_MODE:
            {
                uint8_t proto_mode = 0;

                if(status == COMMON_BUF_ERR_NO_ERROR)
                {
                    proto_mode = common_buf_data(p_data)[0];
                }

                p_cb->cb_read_proto_mode_cmp(conidx, status, hid_idx, proto_mode);
            }break;

            // Boot Keyboard Input Report
            case HOGPBH_BOOT_KB_IN_REPORT:
            // Boot Keyboard Output Report
            case HOGPBH_BOOT_KB_OUT_REPORT:
            // Boot Mouse Input Report
            case HOGPBH_BOOT_MOUSE_IN_REPORT:
            {
                p_cb->cb_read_report_cmp(conidx, status, hid_idx, val_id, p_data);
            }break;

            // Boot Keyboard Input Report Client Config
            case HOGPBH_BOOT_KB_IN_NTF_CFG:
            // Boot Mouse Input Report Client Config
            case HOGPBH_BOOT_MOUSE_IN_NTF_CFG:
            {
                uint16_t ntf_cfg = 0;

                if(status == COMMON_BUF_ERR_NO_ERROR)
                {
                    ntf_cfg = common_btohs(common_read16p(common_buf_data(p_data)));
                }

                p_cb->cb_read_cfg_cmp(conidx, status, hid_idx, val_id, ntf_cfg);
            }break;

            default: { /* Nothing to do */ } break;
        }
    }
}



/**
 ****************************************************************************************
 * @brief Perform Value read procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
 * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
 ****************************************************************************************
 */
__STATIC uint16_t hogpbh_read_val(uint8_t conidx, uint8_t hid_idx, uint16_t val_id)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Client environment
    hogpbh_env_t* p_hogpbh_env = PRF_ENV_GET(HOGPBH, hogpbh);

    if(p_hogpbh_env != NULL)
    {
        if ((conidx < BLE_CONNECTION_MAX) && (p_hogpbh_env->p_env[conidx] != NULL) && (!p_hogpbh_env->p_env[conidx]->discover))
        {
            hogpbh_cnx_env_t* p_con_env = p_hogpbh_env->p_env[conidx];

            if (hid_idx >= p_con_env->nb_svc)
            {
                status = PRF_ERR_INVALID_PARAM;
            }
            else
            {
                uint16_t hdl;
                hogpbh_content_t* p_hids = &(p_con_env->hids[hid_idx]);

                switch(val_id)
                {
                    case HOGPBH_PROTO_MODE:            { hdl = p_hids->chars[HOGPBH_CHAR_PROTO_MODE].val_hdl;                } break;
                    case HOGPBH_BOOT_KB_IN_REPORT:     { hdl = p_hids->chars[HOGPBH_CHAR_BOOT_KB_IN_REPORT].val_hdl;         } break;
                    case HOGPBH_BOOT_KB_OUT_REPORT:    { hdl = p_hids->chars[HOGPBH_CHAR_BOOT_KB_OUT_REPORT].val_hdl;        } break;
                    case HOGPBH_BOOT_MOUSE_IN_REPORT:  { hdl = p_hids->chars[HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT].val_hdl;      } break;
                    case HOGPBH_BOOT_KB_IN_NTF_CFG:    { hdl = p_hids->descs[HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG].desc_hdl;    } break;
                    case HOGPBH_BOOT_MOUSE_IN_NTF_CFG: { hdl = p_hids->descs[HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG].desc_hdl; } break;
                    default:                           { hdl = GATT_INVALID_HDL;                                             } break;
                }

                if(hdl == GATT_INVALID_HDL)
                {
                    status = PRF_ERR_INEXISTENT_HDL;
                }
                else
                {
                    uint16_t dummy = 0;
                    SETF(dummy, HOGPBH_DUMMY_HID_IDX,  hid_idx);
                    SETF(dummy, HOGPBH_DUMMY_VAL_ID,   val_id);
                    // perform read request
                    status = gatt_cli_read(conidx, p_hogpbh_env->user_lid, dummy, hdl, 0, 0);
                }
            }
        }
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Send write result to application,.
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the request
 * @param[in] dummy         Dummy token used to retrieve read on-going
 ****************************************************************************************
 */
__STATIC void hogpbh_write_val_cmp(uint8_t conidx, uint16_t status, uint16_t dummy)
{
    hogpbh_env_t* p_hogpbh_env = PRF_ENV_GET(HOGPBH, hogpbh);

    if(p_hogpbh_env != NULL)
    {
        const hogpbh_cb_t* p_cb = (const hogpbh_cb_t*) p_hogpbh_env->prf_env.p_cb;
        uint8_t hid_idx      = GETF(dummy, HOGPBH_DUMMY_HID_IDX);
        uint8_t val_id       = GETF(dummy, HOGPBH_DUMMY_VAL_ID);

        switch(val_id)
        {
            case HOGPBH_PROTO_MODE:
            {
                p_cb->cb_write_proto_mode_cmp(conidx, status, hid_idx);
            }break;

            // Boot Keyboard Input Report
            case HOGPBH_BOOT_KB_IN_REPORT:
            // Boot Keyboard Output Report
            case HOGPBH_BOOT_KB_OUT_REPORT:
            // Boot Mouse Input Report
            case HOGPBH_BOOT_MOUSE_IN_REPORT:
            {
                p_cb->cb_write_report_cmp(conidx, status, hid_idx, val_id);
            }break;

            // Boot Keyboard Input Report Client Config
            case HOGPBH_BOOT_KB_IN_NTF_CFG:
            // Boot Mouse Input Report Client Config
            case HOGPBH_BOOT_MOUSE_IN_NTF_CFG:
            {
                p_cb->cb_write_cfg_cmp(conidx, status, hid_idx, val_id);
            }break;

            default: { /* Nothing to do */ } break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Perform Write procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Instance - From 0 to HOGPBH_NB_HIDS_INST_MAX-1
 * @param[in] val_id        Value Identifier (@see enum hogpbh_info)
 ****************************************************************************************
 */

__STATIC uint16_t hogpbh_write_val(uint8_t conidx, uint8_t hid_idx, uint8_t val_id, bool wr_cmd, common_buf_t* p_buf)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Client environment
    hogpbh_env_t* p_hogpbh_env = PRF_ENV_GET(HOGPBH, hogpbh);

    if(p_buf == NULL)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if(p_hogpbh_env != NULL)
    {
        if ((conidx < BLE_CONNECTION_MAX) && (p_hogpbh_env->p_env[conidx] != NULL) && (!p_hogpbh_env->p_env[conidx]->discover))
        {
            hogpbh_cnx_env_t* p_con_env = p_hogpbh_env->p_env[conidx];

            if (hid_idx >= p_con_env->nb_svc)
            {
                status = PRF_ERR_INVALID_PARAM;
            }
            else
            {
                uint16_t hdl;
                hogpbh_content_t* p_hids = &(p_con_env->hids[hid_idx]);

                switch(val_id)
                {
                    case HOGPBH_PROTO_MODE:            { hdl = p_hids->chars[HOGPBH_CHAR_PROTO_MODE].val_hdl;                } break;
                    case HOGPBH_BOOT_KB_IN_REPORT:     { hdl = p_hids->chars[HOGPBH_CHAR_BOOT_KB_IN_REPORT].val_hdl;         } break;
                    case HOGPBH_BOOT_KB_OUT_REPORT:    { hdl = p_hids->chars[HOGPBH_CHAR_BOOT_KB_OUT_REPORT].val_hdl;        } break;
                    case HOGPBH_BOOT_MOUSE_IN_REPORT:  { hdl = p_hids->chars[HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT].val_hdl;      } break;
                    case HOGPBH_BOOT_KB_IN_NTF_CFG:    { hdl = p_hids->descs[HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG].desc_hdl;    } break;
                    case HOGPBH_BOOT_MOUSE_IN_NTF_CFG: { hdl = p_hids->descs[HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG].desc_hdl; } break;
                    default:                           { hdl = GATT_INVALID_HDL;                                             } break;
                }

                if(hdl == GATT_INVALID_HDL)
                {
                    status = PRF_ERR_INEXISTENT_HDL;
                }
                else
                {
                    uint16_t dummy = 0;
                    SETF(dummy, HOGPBH_DUMMY_HID_IDX,  hid_idx);
                    SETF(dummy, HOGPBH_DUMMY_VAL_ID,   val_id);
                    // perform write request
                    status = gatt_cli_write(conidx, p_hogpbh_env->user_lid, dummy,
                                            wr_cmd ? GATT_WRITE_NO_RESP : GATT_WRITE, hdl, 0, p_buf);
                }
            }
        }
    }

    return (status);

}


/*
 * GATT USER CLIENT HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function is called when a full service has been found during a discovery procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] hdl           First handle value of following list
 * @param[in] disc_info     Discovery information (@see enum gatt_svc_disc_info)
 * @param[in] nb_att        Number of attributes
 * @param[in] p_atts        Pointer to attribute information present in a service
 ****************************************************************************************
 */
__STATIC void hogpbh_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint8_t disc_info,
                          uint8_t nb_att, const gatt_svc_att_t* p_atts)
{
    // Get the address of the environment
    hogpbh_env_t* p_hogpbh_env = PRF_ENV_GET(HOGPBH, hogpbh);

    if(p_hogpbh_env != NULL)
    {
        hogpbh_cnx_env_t* p_con_env = p_hogpbh_env->p_env[conidx];

        if(p_con_env != NULL)
        {
            if (p_con_env->nb_svc < HOGPBH_NB_HIDS_INST_MAX)
            {
                //Even if we get multiple responses we only store 1 range
                if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_START))
                {
                    p_con_env->hids[p_con_env->nb_svc].svc.shdl = hdl;
                }

                if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_END))
                {
                    p_con_env->hids[p_con_env->nb_svc].svc.ehdl = hdl + nb_att -1;
                }

                // Retrieve characteristics
                prf_extract_svc_info(hdl, nb_att, p_atts,
                                     HOGPBH_CHAR_MAX, hogpbh_hids_char, p_con_env->hids[p_con_env->nb_svc].chars,
                                     HOGPBH_DESC_MAX, hogpbh_hids_char_desc, p_con_env->hids[p_con_env->nb_svc].descs);

                if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_END))
                {
                    p_con_env->nb_svc++;
                }
            }
        }
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT client user discovery procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (@see enum hl_err)
 ****************************************************************************************
 */
__STATIC void hogpbh_discover_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    // Get the address of the environment
    hogpbh_env_t* p_hogpbh_env = PRF_ENV_GET(HOGPBH, hogpbh);

    if(p_hogpbh_env != NULL)
    {
        hogpbh_cnx_env_t* p_con_env = p_hogpbh_env->p_env[conidx];

        if (p_con_env->nb_svc > 0)
        {
            uint8_t svc_idx;
            for (svc_idx = 0; (svc_idx < p_con_env->nb_svc) && (status == GAP_ERR_NO_ERROR); svc_idx++)
            {
                status = prf_check_svc_validity(HOGPBH_CHAR_MAX, p_con_env->hids[svc_idx].chars, hogpbh_hids_char,
                                                HOGPBH_DESC_MAX, p_con_env->hids[svc_idx].descs,
                                                hogpbh_hids_char_desc);
            }
        }
        else
        {
            status = PRF_ERR_STOP_DISC_CHAR_MISSING;
        }

        hogpbh_enable_cmp(p_hogpbh_env, conidx, status);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called during a read procedure when attribute value is retrieved
 *        form peer device.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] p_data        Pointer to buffer that contains attribute value starting from offset
 ****************************************************************************************
 */
__STATIC void hogpbh_att_val_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint16_t offset,
                              common_buf_t* p_data)
{
    hogpbh_read_val_cmp(conidx, GAP_ERR_NO_ERROR, dummy, p_data);
}


/**
 ****************************************************************************************
 * @brief This function is called when GATT client user read procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (@see enum hl_err)
 ****************************************************************************************
 */
__STATIC void hogpbh_read_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    if(status != GAP_ERR_NO_ERROR)
    {
        hogpbh_read_val_cmp(conidx, status, dummy, NULL);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT client user write procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (@see enum hl_err)
 ****************************************************************************************
 */
__STATIC void hogpbh_write_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    hogpbh_write_val_cmp(conidx, status, dummy);
}


/**
 ****************************************************************************************
 * @brief This function is called when a notification or an indication is received onto
 *        register handle range (@see gatt_cli_event_register).
 *
 *        @see gatt_cli_val_event_cfm must be called to confirm event reception.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] evt_type      Event type triggered (@see enum gatt_evt_type)
 * @param[in] complete      True if event value if complete value has been received
 *                          False if data received is equals to maximum attribute protocol value.
 *                          In such case GATT Client User should perform a read procedure.
 * @param[in] hdl           Attribute handle
 * @param[in] p_data        Pointer to buffer that contains attribute value
 ****************************************************************************************
 */
__STATIC void hogpbh_att_val_evt_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint8_t evt_type, bool complete,
                        uint16_t hdl, common_buf_t* p_data)
{
    // Get the address of the environment
    hogpbh_env_t* p_hogpbh_env = PRF_ENV_GET(HOGPBH, hogpbh);

    if(p_hogpbh_env != NULL)
    {
        hogpbh_cnx_env_t* p_con_env = p_hogpbh_env->p_env[conidx];

        if(p_con_env != NULL)
        {
            uint8_t att_info = HOGPBH_INFO_MAX;
            uint8_t hid_idx;

            // BOOT Report - HID instance is unknown.
            for (hid_idx = 0; (hid_idx < p_con_env->nb_svc); hid_idx++)
            {
                if (hdl == p_con_env->hids[hid_idx].chars[HOGPBH_CHAR_BOOT_KB_IN_REPORT].val_hdl)
                {
                    att_info = HOGPBH_BOOT_KB_IN_REPORT;
                    break;
                }
                else if (hdl == p_con_env->hids[hid_idx].chars[HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT].val_hdl)
                {
                    att_info = HOGPBH_BOOT_MOUSE_IN_REPORT;
                    break;
                }
            }

            // Inform application about received report
            if (att_info != HOGPBH_INFO_MAX)
            {
                const hogpbh_cb_t* p_cb = (const hogpbh_cb_t*) p_hogpbh_env->prf_env.p_cb;
                p_cb->cb_report(conidx, hid_idx, att_info, p_data);
            }
        }
    }

    // confirm event reception
    gatt_cli_att_event_cfm(conidx, user_lid, token);

}

/**
 ****************************************************************************************
 * @brief Event triggered when a service change has been received or if an attribute
 *        transaction triggers an out of sync error.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] out_of_sync   True if an out of sync error has been received
 * @param[in] start_hdl     Service start handle
 * @param[in] end_hdl       Service end handle
 ****************************************************************************************
 */
__STATIC void hogpbh_svc_changed_cb(uint8_t conidx, uint8_t user_lid, bool out_of_sync, uint16_t start_hdl, uint16_t end_hdl)
{
    // Do Nothing
}

/// Client callback hander
__STATIC const gatt_cli_cb_t hogpbh_cb =
{
    .cb_discover_cmp    = hogpbh_discover_cmp_cb,
    .cb_read_cmp        = hogpbh_read_cmp_cb,
    .cb_write_cmp       = hogpbh_write_cmp_cb,
    .cb_att_val_get     = NULL,
    .cb_svc             = hogpbh_svc_cb,
    .cb_svc_info        = NULL,
    .cb_inc_svc         = NULL,
    .cb_char            = NULL,
    .cb_desc            = NULL,
    .cb_att_val         = hogpbh_att_val_cb,
    .cb_att_val_evt     = hogpbh_att_val_evt_cb,
    .cb_svc_changed     = hogpbh_svc_changed_cb,
};

/*
 * PROFILE NATIVE API
 ****************************************************************************************
 */

uint16_t hogpbh_enable(uint8_t conidx, uint8_t con_type, uint8_t hids_nb, const hogpbh_content_t* p_hids)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Client environment
    hogpbh_env_t* p_hogpbh_env = PRF_ENV_GET(HOGPBH, hogpbh);

    if((hids_nb > HOGPBH_NB_HIDS_INST_MAX) || ((con_type == PRF_CON_NORMAL) && (p_hids == NULL)))
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if(p_hogpbh_env != NULL)
    {
        if ((conidx < BLE_CONNECTION_MAX) && (p_hogpbh_env->p_env[conidx] == NULL))
        {
            // allocate environment variable for task instance
            p_hogpbh_env->p_env[conidx] = (hogpbh_cnx_env_t *) kernel_malloc(sizeof(hogpbh_cnx_env_t), KERNEL_MEM_ATT_DB);
            memset(p_hogpbh_env->p_env[conidx], 0, sizeof(hogpbh_cnx_env_t));

            if(p_hogpbh_env->p_env[conidx] != NULL)
            {
                // Config connection, start discovering
                if (con_type == PRF_CON_DISCOVERY)
                {
                    uint16_t gatt_svc_uuid = GATT_SVC_HID;

                    // start discovery
                    status = gatt_cli_discover_svc(conidx, p_hogpbh_env->user_lid, 0, GATT_DISCOVER_SVC_PRIMARY_BY_UUID, true,
                                                   GATT_MIN_HDL, GATT_MAX_HDL, GATT_UUID_16, (uint8_t*) &gatt_svc_uuid);

                    // Go to DISCOVERING state
                    p_hogpbh_env->p_env[conidx]->discover = true;
                }
                // normal connection, get saved att details
                else
                {
                    p_hogpbh_env->p_env[conidx]->nb_svc = hids_nb;
                    memcpy(p_hogpbh_env->p_env[conidx]->hids, p_hids, sizeof(hogpbh_content_t) * hids_nb);
                    status = GAP_ERR_NO_ERROR;

                    // send APP confirmation that can start normal connection to TH
                    hogpbh_enable_cmp(p_hogpbh_env, conidx, GAP_ERR_NO_ERROR);
                }
            }
            else
            {
                status = GAP_ERR_INSUFF_RESOURCES;
            }
        }
    }

    return (status);
}

uint16_t hogpbh_read_cfg(uint8_t conidx, uint8_t hid_idx, uint8_t val_id)
{
    uint16_t status;

    switch(val_id)
    {
        case HOGPBH_BOOT_KB_IN_NTF_CFG:
        case HOGPBH_BOOT_MOUSE_IN_NTF_CFG: { status = hogpbh_read_val(conidx, hid_idx, val_id); } break;
        default:                           { status = PRF_ERR_INVALID_PARAM; } break;
    }

    return (status);
}

uint16_t hogpbh_read_proto_mode(uint8_t conidx, uint8_t hid_idx)
{
    uint16_t status = hogpbh_read_val(conidx, hid_idx, HOGPBH_PROTO_MODE);
    return (status);
}

uint16_t hogpbh_read_report(uint8_t conidx, uint8_t hid_idx, uint8_t val_id)
{
    uint16_t status;

    switch(val_id)
    {
        case HOGPBH_BOOT_KB_IN_REPORT:
        case HOGPBH_BOOT_KB_OUT_REPORT:
        case HOGPBH_BOOT_MOUSE_IN_REPORT: { status = hogpbh_read_val(conidx, hid_idx, val_id); } break;
        default:                          { status = PRF_ERR_INVALID_PARAM;                    } break;
    }

    return (status);
}

uint16_t hogpbh_write_cfg(uint8_t conidx, uint8_t hid_idx, uint8_t val_id, uint16_t cfg_val)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    if (cfg_val > PRF_CLI_START_NTF)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else
    {
        switch(val_id)
        {
            case HOGPBH_BOOT_KB_IN_NTF_CFG:
            case HOGPBH_BOOT_MOUSE_IN_NTF_CFG:
            {
                common_buf_t* p_buf;
                // allocate a buffer to perform write
                if(common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, sizeof(uint16_t), GATT_BUFFER_TAIL_LEN) == COMMON_BUF_ERR_NO_ERROR)
                {
                    common_write16p(common_buf_data(p_buf), common_htobs(cfg_val));
                    status = hogpbh_write_val(conidx, hid_idx, val_id, false, p_buf);
                    common_buf_release(p_buf);
                }
            } break;
            default:  { status = PRF_ERR_INVALID_PARAM; } break;
        }
    }

    return (status);
}

uint16_t hogpbh_write_proto_mode(uint8_t conidx, uint8_t hid_idx, uint8_t proto_mode)
{
    common_buf_t* p_buf;
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    // allocate a buffer to perform write
    if(common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, sizeof(uint8_t), GATT_BUFFER_TAIL_LEN) == COMMON_BUF_ERR_NO_ERROR)
    {
        common_buf_data(p_buf)[0] = proto_mode;
        status = hogpbh_write_val(conidx, hid_idx, HOGPBH_PROTO_MODE, true, p_buf);
        common_buf_release(p_buf);
    }

    return (status);
}

uint16_t hogpbh_write_report(uint8_t conidx, uint8_t hid_idx, uint8_t val_id, bool wr_cmd, common_buf_t* p_report)
{
    uint16_t status;

    switch(val_id)
    {
        case HOGPBH_BOOT_KB_IN_REPORT:
        case HOGPBH_BOOT_MOUSE_IN_REPORT: { wr_cmd = false; }
        // No Break
        case HOGPBH_BOOT_KB_OUT_REPORT:   { status = hogpbh_write_val(conidx, hid_idx, val_id, wr_cmd, p_report); } break;
        default:                          { status = PRF_ERR_INVALID_PARAM;                                       } break;
    }

    return (status);
}


#if (BLE_HL_MSG_API)
/*
 * PROFILE MSG HANDLERS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief  Message handler example
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogpbh_enable_req_handler(kernel_msg_id_t const msgid, struct hogpbh_enable_req const *p_param,
                                       kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint16_t status = hogpbh_enable(p_param->conidx, p_param->con_type, p_param->hids_nb, p_param->hids);

    // send an error if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct hogpbh_enable_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPBH_ENABLE_RSP, src_id, dest_id, hogpbh_enable_rsp);
        if(p_rsp != NULL)
        {
            p_rsp->conidx  = p_param->conidx;
            p_rsp->status  = status;
            p_rsp->hids_nb = 0;
            kernel_msg_send(p_rsp);
        }
    }

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPBH_READ_INFO_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogpbh_read_info_req_handler(kernel_msg_id_t const msgid, struct hogpbh_read_info_req const *p_param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint8_t status;


    // check requested info
    switch (p_param->info)
    {
        // Protocol Mode
        case HOGPBH_PROTO_MODE:             { status = hogpbh_read_proto_mode(p_param->conidx, p_param->hid_idx);            } break;
        // Boot Keyboard Input Report
        case HOGPBH_BOOT_KB_IN_REPORT:
        // Boot Keyboard Output Report
        case HOGPBH_BOOT_KB_OUT_REPORT:
        // Boot Mouse Input Report
        case HOGPBH_BOOT_MOUSE_IN_REPORT:   { status = hogpbh_read_report(p_param->conidx, p_param->hid_idx, p_param->info); } break;

        // Boot Keyboard Input Report Client Config
        case HOGPBH_BOOT_KB_IN_NTF_CFG:
        // Boot Mouse Input Report Client Config
        case HOGPBH_BOOT_MOUSE_IN_NTF_CFG:  { status = hogpbh_read_cfg(p_param->conidx, p_param->hid_idx, p_param->info);    } break;

        default:                            { status = PRF_ERR_INVALID_PARAM;                                                } break;
    }

    // request cannot be performed
    if (status != GAP_ERR_NO_ERROR)
    {
        struct hogpbh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPBH_READ_INFO_RSP, src_id, dest_id, hogpbh_read_info_rsp);

        // set error status
        if(p_rsp != NULL)
        {
            p_rsp->conidx    = p_param->conidx;
            p_rsp->status    = status;
            p_rsp->info      = p_param->info;
            p_rsp->hid_idx   = p_param->hid_idx;
            p_rsp->data.boot_report.length = 0;
            kernel_msg_send(p_rsp);
        }
    }

    return (KERNEL_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPBH_WRITE_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogpbh_write_req_handler(kernel_msg_id_t const msgid, struct hogpbh_write_req const *p_param,
                                      kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint8_t status;

    // check requested info
    switch (p_param->info)
    {
        // Protocol Mode
        case HOGPBH_PROTO_MODE:
        {
            status = hogpbh_write_proto_mode(p_param->conidx, p_param->hid_idx, p_param->data.proto_mode);
        } break;
        // Boot Keyboard Input Report
        case HOGPBH_BOOT_KB_IN_REPORT:
        // Boot Keyboard Output Report
        case HOGPBH_BOOT_KB_OUT_REPORT:
        // Boot Mouse Input Report
        case HOGPBH_BOOT_MOUSE_IN_REPORT:
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            uint16_t report_len = p_param->data.boot_report.length;
            common_buf_t* p_buf = NULL;

            // allocate a buffer to perform write
            if(common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, report_len, GATT_BUFFER_TAIL_LEN) == COMMON_BUF_ERR_NO_ERROR)
            {
                common_buf_copy_data_from_mem(p_buf, p_param->data.boot_report.value, report_len);
                status = hogpbh_write_report(p_param->conidx, p_param->hid_idx, p_param->info, p_param->wr_cmd, p_buf);
                common_buf_release(p_buf);
            }
        } break;

        // Boot Keyboard Input Report Client Config
        case HOGPBH_BOOT_KB_IN_NTF_CFG:
        // Boot Mouse Input Report Client Config
        case HOGPBH_BOOT_MOUSE_IN_NTF_CFG:
        {
            status = hogpbh_write_cfg(p_param->conidx, p_param->hid_idx, p_param->info, p_param->data.ntf_cfg);
        } break;

        default: { status = PRF_ERR_INVALID_PARAM; } break;
    }

    // request cannot be performed
    if (status != GAP_ERR_NO_ERROR)
    {
        struct hogpbh_write_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPBH_WRITE_RSP, src_id, dest_id, hogpbh_write_rsp);

        // set error status
        if(p_rsp != NULL)
        {
            p_rsp->conidx = p_param->conidx;
            p_rsp->status = status;
            p_rsp->info   = p_param->info;
            p_rsp->hid_idx   = p_param->hid_idx;

            kernel_msg_send(p_rsp);
        }
    }

    return (KERNEL_MSG_CONSUMED);
}


/// Default State handlers definition
KERNEL_MSG_HANDLER_TAB(hogpbh)
{
    // Note: all messages must be sorted in ID ascending order

    {HOGPBH_ENABLE_REQ,        (kernel_msg_func_t)hogpbh_enable_req_handler     },
    {HOGPBH_READ_INFO_REQ,     (kernel_msg_func_t)hogpbh_read_info_req_handler  },
    {HOGPBH_WRITE_REQ,         (kernel_msg_func_t)hogpbh_write_req_handler      },
};


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
__STATIC void hogpbh_cb_enable_cmp(uint8_t conidx, uint16_t status, uint8_t hids_nb, const hogpbh_content_t* p_hids)
{
    struct hogpbh_enable_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPBH_ENABLE_RSP, PRF_DST_TASK(HOGPBH), PRF_SRC_TASK(HOGPBH),
                                                   hogpbh_enable_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx       = conidx;
        p_rsp->status       = status;
        p_rsp->hids_nb      = hids_nb;
        memcpy(p_rsp->hids, p_hids, sizeof(p_rsp->hids));

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogpbh_cb_read_cfg_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t val_id, uint16_t cfg_val)
{
    struct hogpbh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPBH_READ_INFO_RSP, PRF_DST_TASK(HOGPBH), PRF_SRC_TASK(HOGPBH),
                                                      hogpbh_read_info_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx       = conidx;
        p_rsp->status       = status;
        p_rsp->hid_idx      = hid_idx;
        p_rsp->info         = val_id;
        p_rsp->data.ntf_cfg = cfg_val;

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogpbh_cb_read_proto_mode_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t proto_mode)
{
    struct hogpbh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPBH_READ_INFO_RSP, PRF_DST_TASK(HOGPBH), PRF_SRC_TASK(HOGPBH),
                                                      hogpbh_read_info_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx          = conidx;
        p_rsp->status          = status;
        p_rsp->hid_idx         = hid_idx;
        p_rsp->info            = HOGPBH_PROTO_MODE;
        p_rsp->data.proto_mode = proto_mode;

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogpbh_cb_read_report_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t val_id, common_buf_t* p_report)
{
    uint16_t data_len = common_buf_data_len(p_report);
    struct hogpbh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC_DYN(HOGPBH_READ_INFO_RSP, PRF_DST_TASK(HOGPBH), PRF_SRC_TASK(HOGPBH),
                                                          hogpbh_read_info_rsp, data_len);

    if(p_rsp != NULL)
    {
        p_rsp->conidx        = conidx;
        p_rsp->status        = status;
        p_rsp->hid_idx       = hid_idx;
        p_rsp->info          = val_id;
        p_rsp->data.boot_report.length = data_len;
        common_buf_copy_data_to_mem(p_report, p_rsp->data.boot_report.value, data_len);

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogpbh_cb_write_cfg_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t val_id)
{
    struct hogpbh_write_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPBH_WRITE_RSP, PRF_DST_TASK(HOGPBH), PRF_SRC_TASK(HOGPBH),
                                                   hogpbh_write_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx       = conidx;
        p_rsp->status       = status;
        p_rsp->hid_idx      = hid_idx;
        p_rsp->info         = val_id;

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogpbh_cb_write_proto_mode_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx)
{
    struct hogpbh_write_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPBH_WRITE_RSP, PRF_DST_TASK(HOGPBH), PRF_SRC_TASK(HOGPBH),
                                                   hogpbh_write_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx       = conidx;
        p_rsp->status       = status;
        p_rsp->hid_idx      = hid_idx;
        p_rsp->info         = HOGPBH_PROTO_MODE;

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogpbh_cb_write_report_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t val_id)
{
    struct hogpbh_write_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPBH_WRITE_RSP, PRF_DST_TASK(HOGPBH), PRF_SRC_TASK(HOGPBH),
                                                   hogpbh_write_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx       = conidx;
        p_rsp->status       = status;
        p_rsp->hid_idx      = hid_idx;
        p_rsp->info         = val_id;

        kernel_msg_send(p_rsp);
    }
}


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
__STATIC void hogpbh_cb_report(uint8_t conidx, uint8_t hid_idx, uint8_t val_id, common_buf_t* p_report)
{
    uint16_t data_len = common_buf_data_len(p_report);
    // send boot report indication
    struct hogpbh_boot_report_ind *p_ind = KERNEL_MSG_ALLOC_DYN(HOGPBH_BOOT_REPORT_IND, PRF_DST_TASK(HOGPBH),
                                                            PRF_SRC_TASK(HOGPBH), hogpbh_boot_report_ind, data_len);

    if(p_ind != NULL)
    {
        p_ind->conidx       = conidx;
        p_ind->hid_idx       = hid_idx;
        p_ind->info          = val_id;
        p_ind->boot_report.length = data_len;
        common_buf_copy_data_to_mem(p_report, p_ind->boot_report.value, data_len);

        kernel_msg_send(p_ind);
    }
}

/// Default Message handle
__STATIC const hogpbh_cb_t hogpbh_msg_cb =
{
    .cb_enable_cmp            = hogpbh_cb_enable_cmp,
    .cb_read_cfg_cmp          = hogpbh_cb_read_cfg_cmp,
    .cb_read_proto_mode_cmp   = hogpbh_cb_read_proto_mode_cmp,
    .cb_read_report_cmp       = hogpbh_cb_read_report_cmp,
    .cb_write_cfg_cmp         = hogpbh_cb_write_cfg_cmp,
    .cb_write_proto_mode_cmp  = hogpbh_cb_write_proto_mode_cmp,
    .cb_write_report_cmp      = hogpbh_cb_write_report_cmp,
    .cb_report                = hogpbh_cb_report,

};
#endif // (BLE_HL_MSG_API)


/*
 * PROFILE DEFAULT HANDLERS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialization of the Client module.
 * This function performs all the initializations of the Profile module.
 *  - Creation of database (if it's a service)
 *  - Allocation of profile required memory
 *  - Initialization of task descriptor to register application
 *      - Task State array
 *      - Number of tasks
 *      - Default task handler
 *
 * @param[out]    p_env      Collector or Service allocated environment data.
 * @param[in|out] p_start_hdl  Service start handle (0 - dynamically allocated), only applies for services.
 * @param[in]     sec_lvl    Security level (@see enum gatt_svc_info_bf)
 * @param[in]     p_param    Configuration parameters of profile collector or service (32 bits aligned)
 * @param[in]     p_cb       Callback structure that handles event from profile
 *
 * @return status code to know if profile initialization succeed or not.
 ****************************************************************************************
 */
__STATIC uint16_t hogpbh_init(prf_data_t* p_env, uint16_t* p_start_hdl, uint8_t sec_lvl, uint8_t user_prio,
                            const void* p_params, const hogpbh_cb_t* p_cb)
{
    uint8_t conidx;
    // DB Creation Status
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t user_lid = GATT_INVALID_USER_LID;

    do
    {
        hogpbh_env_t* p_hogpbh_env;

        #if (BLE_HL_MSG_API)
        if(p_cb == NULL)
        {
            p_cb = &(hogpbh_msg_cb);
        }
        #endif // (BLE_HL_MSG_API)

        if(   (p_params == NULL) || (p_cb == NULL) || (p_cb->cb_enable_cmp == NULL) || (p_cb->cb_read_cfg_cmp == NULL)
           || (p_cb->cb_read_proto_mode_cmp == NULL)  || (p_cb->cb_read_report_cmp == NULL)  || (p_cb->cb_write_cfg_cmp == NULL)
           || (p_cb->cb_write_proto_mode_cmp == NULL)  || (p_cb->cb_write_report_cmp == NULL)  || (p_cb->cb_report == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // register HOGPBH user
        status = gatt_user_cli_register(HOGP_REPORT_MAX_LEN, user_prio, &hogpbh_cb, &user_lid);
        if(status != GAP_ERR_NO_ERROR) break;

        //-------------------- allocate memory required for the profile  ---------------------
        p_hogpbh_env = (hogpbh_env_t*) kernel_malloc(sizeof(hogpbh_env_t), KERNEL_MEM_ATT_DB);

        if(p_hogpbh_env != NULL)
        {
            // allocate HOGPBH required environment variable
            p_env->p_env = (prf_hdr_t *) p_hogpbh_env;

            // initialize environment variable
            p_hogpbh_env->prf_env.p_cb    = p_cb;
            #if (BLE_HL_MSG_API)
            p_env->desc.msg_handler_tab = hogpbh_msg_handler_tab;
            p_env->desc.msg_cnt         = ARRAY_LEN(hogpbh_msg_handler_tab);
            #endif // (BLE_HL_MSG_API)

            p_hogpbh_env->user_lid = user_lid;
            for (conidx = 0; conidx < BLE_CONNECTION_MAX; conidx++)
            {
                p_hogpbh_env->p_env[conidx] = NULL;
            }
        }
    } while(0);


    if((status != GAP_ERR_NO_ERROR) && (user_lid != GATT_INVALID_USER_LID))
    {
        gatt_user_unregister(user_lid);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Destruction of the profile module - due to a reset or profile remove.
 *
 * This function clean-up allocated memory.
 *
 * @param[in|out]    p_env        Collector or Service allocated environment data.
 * @param[in]        reason       Destroy reason (@see enum prf_destroy_reason)
 *
 * @return status of the destruction, if fails, profile considered not removed.
 ****************************************************************************************
 */
__STATIC uint16_t hogpbh_destroy(prf_data_t *p_env, uint8_t reason)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    hogpbh_env_t* p_hogpbh_env = (hogpbh_env_t*) p_env->p_env;

    if(reason != PRF_DESTROY_RESET)
    {
        status = gatt_user_unregister(p_hogpbh_env->user_lid);
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        uint8_t idx;

        // cleanup environment variable for each task instances
        for (idx = 0; idx < BLE_CONNECTION_MAX; idx++)
        {
            if (p_hogpbh_env->p_env[idx] != NULL)
            {
                kernel_free(p_hogpbh_env->p_env[idx]);
            }
        }

        // free profile environment variables
        p_env->p_env = NULL;
        kernel_free(p_hogpbh_env);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env          Collector or Service allocated environment data.
 * @param[in]        conidx       Connection index
 * @param[in]        p_con_param  Pointer to connection parameters information
 ****************************************************************************************
 */
__STATIC void hogpbh_con_create(prf_data_t *p_env, uint8_t conidx, const gap_con_param_t* p_con_param)
{
    // Nothing to do
}

/**
 ****************************************************************************************
 * @brief Handles Disconnection
 *
 * @param[in|out]    p_env      Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason
 ****************************************************************************************
 */
__STATIC void hogpbh_con_cleanup(prf_data_t *p_env, uint8_t conidx, uint16_t reason)
{
    hogpbh_env_t* p_hogpbh_env = (hogpbh_env_t*) p_env->p_env;

    // clean-up environment variable allocated for task instance
    if (p_hogpbh_env->p_env[conidx] != NULL)
    {
        kernel_free(p_hogpbh_env->p_env[conidx]);
        p_hogpbh_env->p_env[conidx] = NULL;
    }
}

/// HOGPBH Task interface required by profile manager
const prf_task_cbs_t hogpbh_itf =
{
    .cb_init          = (prf_init_cb) hogpbh_init,
    .cb_destroy       = hogpbh_destroy,
    .cb_con_create    = hogpbh_con_create,
    .cb_con_cleanup   = hogpbh_con_cleanup,
    .cb_con_upd       = NULL,
};

/**
 ****************************************************************************************
 * @brief Retrieve client profile interface
 *
 * @return Client profile interface
 ****************************************************************************************
 */
const prf_task_cbs_t* hogpbh_prf_itf_get(void)
{
    return &hogpbh_itf;
}

#endif /* (BLE_HID_BOOT_HOST) */

/// @} HOGPBH
