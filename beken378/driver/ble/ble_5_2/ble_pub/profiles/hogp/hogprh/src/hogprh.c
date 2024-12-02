/**
 ****************************************************************************************
 *
 * @file hogprh.c
 *
 * @brief HID Over GATT Profile - Report Host Role Implementation
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup HOGPRH
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_HID_REPORT_HOST)

#include "hogprh.h"
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

/// Content of HOGPRH dummy bit field
enum hogprh_dummy_bf
{
    /// HID Instance
    HOGPRH_DUMMY_HID_IDX_MASK      = 0x000F,
    HOGPRH_DUMMY_HID_IDX_LSB       = 0,
    /// Value Identifier
    HOGPRH_DUMMY_VAL_ID_MASK       = 0x00F0,
    HOGPRH_DUMMY_VAL_ID_LSB        = 4,
    /// Report Index
    HOGPRH_DUMMY_REPORT_IDX_MASK   = 0x7F00,
    HOGPRH_DUMMY_REPORT_IDX_LSB    = 8,
    /// Notification received
    HOGPRH_DUMMY_NTF_RECV_BIT      = 0x8000,
    HOGPRH_DUMMY_NTF_RECV_POS      = 15,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Environment variable for each Connections
typedef struct hogprh_cnx_env
{
    ///HIDS characteristics
    hogprh_content_t hids[HOGPRH_NB_HIDS_INST_MAX];
    ///Number of HIDS instances found
    uint8_t          nb_svc;
    /// Client is in discovering state
    bool             discover;
} hogprh_cnx_env_t;

/// Client environment variable
typedef struct hogprh_env
{
    /// profile environment
    prf_hdr_t            prf_env;
    /// Environment variable pointer for each connections
    hogprh_cnx_env_t*    p_env[BLE_CONNECTION_MAX];
    /// GATT User local identifier
    uint8_t              user_lid;
} hogprh_env_t;


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// State machine used to retrieve HID Service characteristics information
const prf_char_def_t hogprh_hids_char[HOGPRH_CHAR_REPORT + 1] =
{
    /// Report Map
    [HOGPRH_CHAR_REPORT_MAP]    = { GATT_CHAR_REPORT_MAP,    ATT_REQ(PRES, MAND), PROP(RD)              },
    /// HID Information
    [HOGPRH_CHAR_HID_INFO]      = { GATT_CHAR_HID_INFO,      ATT_REQ(PRES, MAND), PROP(RD)              },
    /// HID Control Point
    [HOGPRH_CHAR_HID_CTNL_PT]   = { GATT_CHAR_HID_CTNL_PT,   ATT_REQ(PRES, MAND), PROP(WC)              },
    /// Protocol Mode
    [HOGPRH_CHAR_PROTOCOL_MODE] = { GATT_CHAR_PROTOCOL_MODE, ATT_REQ(PRES, OPT),  (PROP(RD) | PROP(WC)) },
    /// Report
    [HOGPRH_CHAR_REPORT]        = { GATT_CHAR_REPORT,        ATT_REQ(PRES, MAND), PROP(RD)              },
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send discovery results to application.
 *
 * @param p_hogprh_env    Client Role task environment
 * @param conidx        Connection index
 * @param status        Response status code
 *****************************************************************************************
 */
__STATIC void hogprh_enable_cmp(hogprh_env_t* p_hogprh_env, uint8_t conidx, uint16_t status)
{
    const hogprh_cb_t* p_cb = (const hogprh_cb_t*) p_hogprh_env->prf_env.p_cb;

    if(p_hogprh_env != NULL)
    {
        hogprh_cnx_env_t* p_con_env = p_hogprh_env->p_env[conidx];

        p_cb->cb_enable_cmp(conidx, status, p_con_env->nb_svc, p_con_env->hids);

        if (status != GAP_ERR_NO_ERROR)
        {
            // clean-up environment variable allocated for task instance
            kernel_free(p_con_env);
            p_hogprh_env->p_env[conidx] = NULL;
        }
        else
        {
            uint8_t svc_idx;
            p_con_env->discover = false;

            for(svc_idx = 0 ; svc_idx < p_con_env->nb_svc ; svc_idx++)
            {
                // Register profile handle to catch gatt indications
                gatt_cli_event_register(conidx, p_hogprh_env->user_lid, p_con_env->hids[svc_idx].svc.shdl,
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
__STATIC void hogprh_read_val_cmp(uint8_t conidx, uint16_t status, uint16_t dummy, common_buf_t* p_data)
{
    hogprh_env_t* p_hogprh_env = PRF_ENV_GET(HOGPRH, hogprh);

    if(p_hogprh_env != NULL)
    {
        const hogprh_cb_t* p_cb = (const hogprh_cb_t*) p_hogprh_env->prf_env.p_cb;
        uint8_t hid_idx      = GETF(dummy, HOGPRH_DUMMY_HID_IDX);
        uint8_t val_id       = GETF(dummy, HOGPRH_DUMMY_VAL_ID);
        uint8_t report_idx   = GETF(dummy, HOGPRH_DUMMY_REPORT_IDX);
        bool    ntf_recv     = GETB(dummy, HOGPRH_DUMMY_NTF_RECV);

        switch(val_id)
        {
            // Protocol Mode
            case HOGPRH_PROTO_MODE:
            {
                uint8_t proto_mode = 0;

                if(status == COMMON_BUF_ERR_NO_ERROR)
                {
                    proto_mode = common_buf_data(p_data)[0];
                }

                p_cb->cb_read_proto_mode_cmp(conidx, status, hid_idx, proto_mode);
            }break;
            // HID Information
            case HOGPRH_HID_INFO:
            {
                hogp_hid_info_t hid_info;
                memset(&hid_info, 0, sizeof(hogp_hid_info_t));

                if(status == COMMON_BUF_ERR_NO_ERROR)
                {
                    hid_info.bcdHID       = common_btohs(common_read16p(common_buf_data(p_data)));
                    common_buf_head_release(p_data, 2);
                    hid_info.bCountryCode = common_buf_data(p_data)[0];
                    common_buf_head_release(p_data, 1);
                    hid_info.flags        = common_buf_data(p_data)[0];
                    common_buf_head_release(p_data, 1);
                }

                p_cb->cb_read_hid_info_cmp(conidx, status, hid_idx, &hid_info);
            }break;
            // Report Map
            case HOGPRH_REPORT_MAP:
            {
                p_cb->cb_read_report_map_cmp(conidx, status, hid_idx, p_data);
            }break;
            // Report Map Char. External Report Reference Descriptor
            case HOGPRH_REPORT_MAP_EXT_REP_REF:
            {
                hogp_report_map_ref_t report_map_ref;
                memset(&report_map_ref, 0, sizeof(hogp_report_map_ref_t));

                if(status == COMMON_BUF_ERR_NO_ERROR)
                {
                    uint8_t uuid_len = common_buf_data_len(p_data);
                    if(uuid_len > GATT_UUID_128_LEN)
                    {
                        status = ATT_ERR_UNLIKELY_ERR;
                    }
                    else
                    {
                        common_buf_copy_data_to_mem(p_data, report_map_ref.uuid, uuid_len);
                        report_map_ref.uuid_len = uuid_len;
                    }
                }

                p_cb->cb_read_report_map_ref_cmp(conidx, status, hid_idx, &report_map_ref);
            }break;

            // Report
            case HOGPRH_REPORT:
            {
                // read performed due to a notification reception
                if(ntf_recv)
                {
                    if(status == GAP_ERR_NO_ERROR)
                    {
                        p_cb->cb_report(conidx, hid_idx, report_idx, p_data);
                    } // else ignore
                }
                else
                {
                    p_cb->cb_read_report_cmp(conidx, status, hid_idx, report_idx, p_data);
                }
            }break;

            // Report Char. Report Reference
            case HOGPRH_REPORT_REF:
            {
                hogp_report_ref_t report_ref;
                memset(&report_ref, 0, sizeof(hogp_report_ref_t));

                if(status == COMMON_BUF_ERR_NO_ERROR)
                {
                    report_ref.id    = common_buf_data(p_data)[0];
                    common_buf_head_release(p_data, 1);
                    report_ref.type  = common_buf_data(p_data)[0];
                    common_buf_head_release(p_data, 1);
                }

                p_cb->cb_read_report_ref_cmp(conidx, status, hid_idx, report_idx, &report_ref);
            }break;
            // Report Notification config
            case HOGPRH_REPORT_NTF_CFG:
            {
                uint16_t ntf_cfg = 0;

                if(status == COMMON_BUF_ERR_NO_ERROR)
                {
                    ntf_cfg = common_btohs(common_read16p(common_buf_data(p_data)));
                }

                p_cb->cb_read_report_cfg_cmp(conidx, status, hid_idx, report_idx, ntf_cfg);
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
 * @param[in] hid_idx       HID Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] val_id        Value Identifier (@see enum hogprh_info)
 * @param[in] report_idx    HID service Report index
 ****************************************************************************************
 */
__STATIC uint16_t hogprh_read_val(uint8_t conidx, uint8_t hid_idx, uint16_t val_id, uint8_t report_idx)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Client environment
    hogprh_env_t* p_hogprh_env = PRF_ENV_GET(HOGPRH, hogprh);

    if(p_hogprh_env != NULL)
    {
        if ((conidx < BLE_CONNECTION_MAX) && (p_hogprh_env->p_env[conidx] != NULL) && (!p_hogprh_env->p_env[conidx]->discover))
        {
            hogprh_cnx_env_t* p_con_env = p_hogprh_env->p_env[conidx];

            if ((hid_idx >= p_con_env->nb_svc) || (report_idx >= HOGPRH_NB_REPORT_INST_MAX))
            {
                status = PRF_ERR_INVALID_PARAM;
            }
            else
            {
                uint16_t hdl;
                hogprh_content_t* p_hids = &(p_con_env->hids[hid_idx]);

                switch(val_id)
                {
                    case HOGPRH_PROTO_MODE:             { hdl = p_hids->chars[HOGPRH_CHAR_PROTOCOL_MODE].val_hdl;            } break;
                    case HOGPRH_HID_INFO:               { hdl = p_hids->chars[HOGPRH_CHAR_HID_INFO].val_hdl;                 } break;
                    case HOGPRH_REPORT_MAP:             { hdl = p_hids->chars[HOGPRH_CHAR_REPORT_MAP].val_hdl;               } break;
                    case HOGPRH_REPORT_MAP_EXT_REP_REF: { hdl = p_hids->descs[HOGPRH_DESC_REPORT_MAP_EXT_REP_REF].desc_hdl;  } break;
                    case HOGPRH_REPORT:                 { hdl = p_hids->chars[HOGPRH_CHAR_REPORT + report_idx].val_hdl;      } break;
                    case HOGPRH_REPORT_NTF_CFG:         { hdl = p_hids->descs[HOGPRH_DESC_REPORT_CFG + report_idx].desc_hdl; } break;
                    case HOGPRH_REPORT_REF:             { hdl = p_hids->descs[HOGPRH_DESC_REPORT_REF + report_idx].desc_hdl; } break;
                    default:                            { hdl = GATT_INVALID_HDL;                                            } break;
                }

                if(hdl == GATT_INVALID_HDL)
                {
                    status = PRF_ERR_INEXISTENT_HDL;
                }
                else
                {
                    uint16_t dummy = 0;
                    SETF(dummy, HOGPRH_DUMMY_HID_IDX,      hid_idx);
                    SETF(dummy, HOGPRH_DUMMY_VAL_ID,       val_id);
                    SETF(dummy, HOGPRH_DUMMY_REPORT_IDX,   report_idx);
                    // perform read request
                    status = gatt_cli_read(conidx, p_hogprh_env->user_lid, dummy, hdl, 0, 0);
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
__STATIC void hogprh_write_val_cmp(uint8_t conidx, uint16_t status, uint16_t dummy)
{
    hogprh_env_t* p_hogprh_env = PRF_ENV_GET(HOGPRH, hogprh);

    if(p_hogprh_env != NULL)
    {
        const hogprh_cb_t* p_cb = (const hogprh_cb_t*) p_hogprh_env->prf_env.p_cb;
        uint8_t hid_idx      = GETF(dummy, HOGPRH_DUMMY_HID_IDX);
        uint8_t val_id       = GETF(dummy, HOGPRH_DUMMY_VAL_ID);
        uint8_t report_idx   = GETF(dummy, HOGPRH_DUMMY_REPORT_IDX);

        switch(val_id)
        {
            // Protocol Mode
            case HOGPRH_PROTO_MODE:     { p_cb->cb_write_proto_mode_cmp(conidx, status, hid_idx);             } break;
            // HID Control Point
            case HOGPRH_HID_CTNL_PT:    { p_cb->cb_write_ctnl_pt_cmp(conidx, status, hid_idx);                } break;
            // Report Char. Report Reference
            case HOGPRH_REPORT:         { p_cb->cb_write_report_cmp(conidx, status, hid_idx, report_idx);     } break;
            // Report Notification config
            case HOGPRH_REPORT_NTF_CFG: { p_cb->cb_write_report_cfg_cmp(conidx, status, hid_idx, report_idx); } break;
            default:                    { /* Nothing to do */                                                 } break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Perform Write procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HID Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] val_id        Value Identifier (@see enum hogprh_info)
 * @param[in] report_idx    HID service Report index
 ****************************************************************************************
 */

__STATIC uint16_t hogprh_write_val(uint8_t conidx, uint8_t hid_idx, uint8_t val_id, uint8_t report_idx, bool wr_cmd,
                                   common_buf_t* p_buf)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Client environment
    hogprh_env_t* p_hogprh_env = PRF_ENV_GET(HOGPRH, hogprh);

    if(p_buf == NULL)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if(p_hogprh_env != NULL)
    {
        if ((conidx < BLE_CONNECTION_MAX) && (p_hogprh_env->p_env[conidx] != NULL) && (!p_hogprh_env->p_env[conidx]->discover))
        {
            hogprh_cnx_env_t* p_con_env = p_hogprh_env->p_env[conidx];

            if ((hid_idx >= p_con_env->nb_svc) || (report_idx >= HOGPRH_NB_REPORT_INST_MAX))
            {
                status = PRF_ERR_INVALID_PARAM;
            }
            else
            {
                uint16_t hdl;
                hogprh_content_t* p_hids = &(p_con_env->hids[hid_idx]);

                switch(val_id)
                {
                    case HOGPRH_PROTO_MODE:     { hdl = p_hids->chars[HOGPRH_CHAR_PROTOCOL_MODE].val_hdl;            } break;
                    case HOGPRH_HID_CTNL_PT:    { hdl = p_hids->chars[HOGPRH_CHAR_HID_CTNL_PT].val_hdl;              } break;
                    case HOGPRH_REPORT:         { hdl = p_hids->chars[HOGPRH_CHAR_REPORT + report_idx].val_hdl;      } break;
                    case HOGPRH_REPORT_NTF_CFG: { hdl = p_hids->descs[HOGPRH_DESC_REPORT_CFG + report_idx].desc_hdl; } break;
                    default:                    { hdl = GATT_INVALID_HDL;                                            } break;
                }

                if(hdl == GATT_INVALID_HDL)
                {
                    status = PRF_ERR_INEXISTENT_HDL;
                }
                else
                {
                    uint16_t dummy = 0;
                    SETF(dummy, HOGPRH_DUMMY_HID_IDX,      hid_idx);
                    SETF(dummy, HOGPRH_DUMMY_VAL_ID,       val_id);
                    SETF(dummy, HOGPRH_DUMMY_REPORT_IDX,   report_idx);
                    // perform write request
                    status = gatt_cli_write(conidx, p_hogprh_env->user_lid, dummy,
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
__STATIC void hogprh_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint8_t disc_info,
                            uint8_t nb_att, const gatt_svc_att_t* p_atts)
{
    // Get the address of the environment
    hogprh_env_t* p_hogprh_env = PRF_ENV_GET(HOGPRH, hogprh);

    if(p_hogprh_env != NULL)
    {
        hogprh_cnx_env_t* p_con_env = p_hogprh_env->p_env[conidx];

        if(p_con_env != NULL)
        {
            if (p_con_env->nb_svc < HOGPRH_NB_HIDS_INST_MAX)
            {
                uint8_t i;
                prf_char_def_t hids_char[HOGPRH_CHAR_MAX];
                prf_desc_def_t hids_char_desc[HOGPRH_DESC_MAX];
                uint8_t hid_idx = p_hogprh_env->p_env[conidx]->nb_svc;
                uint8_t att_cursor;

                //Even if we get multiple responses we only store 1 range
                if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_START))
                {
                    p_con_env->hids[p_con_env->nb_svc].svc.shdl = hdl;
                }

                if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_END))
                {
                    p_con_env->hids[p_con_env->nb_svc].svc.ehdl = hdl + nb_att -1;
                }

                // 1. Create characteristic reference
                memcpy(hids_char, hogprh_hids_char, sizeof(prf_char_def_t) * HOGPRH_CHAR_REPORT);

                for (i = HOGPRH_CHAR_REPORT; i < HOGPRH_CHAR_MAX; i++)
                {
                    hids_char[i] = hogprh_hids_char[HOGPRH_CHAR_REPORT];
                }

                // 2. create descriptor reference
                // Report Map Char. External Report Reference Descriptor
                hids_char_desc[HOGPRH_DESC_REPORT_MAP_EXT_REP_REF].char_code = HOGPRH_CHAR_REPORT_MAP;
                hids_char_desc[HOGPRH_DESC_REPORT_MAP_EXT_REP_REF].req_bf  = ATT_REQ(PRES, OPT);
                hids_char_desc[HOGPRH_DESC_REPORT_MAP_EXT_REP_REF].uuid      = GATT_DESC_EXT_REPORT_REF;

                for (i = 0; i < HOGPRH_NB_REPORT_INST_MAX; i++)
                {
                    // Report Char. Report Reference
                    hids_char_desc[HOGPRH_DESC_REPORT_REF + i].char_code = HOGPRH_CHAR_REPORT + i;
                    hids_char_desc[HOGPRH_DESC_REPORT_REF + i].req_bf  = ATT_REQ(PRES, OPT);
                    hids_char_desc[HOGPRH_DESC_REPORT_REF + i].uuid      = GATT_DESC_REPORT_REF;
                    // Report Client Config
                    hids_char_desc[HOGPRH_DESC_REPORT_CFG + i].char_code = HOGPRH_CHAR_REPORT + i;
                    hids_char_desc[HOGPRH_DESC_REPORT_CFG + i].req_bf  = ATT_REQ(PRES, OPT);
                    hids_char_desc[HOGPRH_DESC_REPORT_CFG + i].uuid      = GATT_DESC_CLIENT_CHAR_CFG;
                }


                // 3. Retrieve HID characteristics
                // Retrieve characteristics
                prf_extract_svc_info(hdl, nb_att, p_atts,
                                     HOGPRH_CHAR_MAX, hids_char, p_con_env->hids[hid_idx].chars,
                                     HOGPRH_DESC_MAX, hids_char_desc, p_con_env->hids[hid_idx].descs);

                // 4. Search for an included service
                for (att_cursor=0; att_cursor < nb_att; att_cursor++)
                {
                    switch(p_atts[att_cursor].att_type)
                    {
                        case GATT_ATT_INCL_SVC:
                        {
                            p_con_env->hids[hid_idx].incl_svc.handle    = hdl + att_cursor;
                            p_con_env->hids[hid_idx].incl_svc.start_hdl = p_atts[att_cursor].info.svc.start_hdl;
                            p_con_env->hids[hid_idx].incl_svc.end_hdl   = p_atts[att_cursor].info.svc.end_hdl;

                            // Extract included service information
                            switch(p_atts[att_cursor].uuid_type)
                            {
                                case GATT_UUID_16:
                                {
                                    p_con_env->hids[hid_idx].incl_svc.uuid_len = GATT_UUID_16_LEN;
                                } break;
                                case GATT_UUID_32:
                                {
                                    p_con_env->hids[hid_idx].incl_svc.uuid_len = GATT_UUID_32_LEN;

                                } break;
                                default:
                                case GATT_UUID_128:
                                {
                                    p_con_env->hids[hid_idx].incl_svc.uuid_len = GATT_UUID_128_LEN;
                                } break;
                            }

                            // Copy UUID data
                            memcpy(p_con_env->hids[hid_idx].incl_svc.uuid, p_atts[att_cursor].uuid,
                                   p_con_env->hids[hid_idx].incl_svc.uuid_len);

                        } break;
                        case GATT_ATT_VAL:
                        {
                            // count number of report in the service
                            if(gatt_uuid16_comp(p_atts[att_cursor].uuid, p_atts[att_cursor].uuid_type, GATT_CHAR_REPORT))
                            {
                                p_con_env->hids[hid_idx].report_nb++;
                            }
                        } break;
                        default: { /* Nothing  to do */ } break;
                    }
                }

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
__STATIC void hogprh_discover_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    // Get the address of the environment
    hogprh_env_t* p_hogprh_env = PRF_ENV_GET(HOGPRH, hogprh);

    if(p_hogprh_env != NULL)
    {
        hogprh_cnx_env_t* p_con_env = p_hogprh_env->p_env[conidx];

        if (p_con_env->nb_svc > 0)
        {
            uint8_t svc_idx;
            for (svc_idx = 0; (svc_idx < p_con_env->nb_svc) && (status == GAP_ERR_NO_ERROR); svc_idx++)
            {
                uint8_t report_idx;
                prf_char_def_t hids_char[HOGPRH_CHAR_MAX];
                prf_desc_def_t hids_char_desc[HOGPRH_DESC_MAX];

                // 1. Create characteristic reference
                memcpy(hids_char, hogprh_hids_char, sizeof(prf_char_def_t) * HOGPRH_CHAR_REPORT);
                for (report_idx = HOGPRH_CHAR_REPORT; report_idx < HOGPRH_CHAR_MAX; report_idx++)
                {
                    hids_char[report_idx] = hogprh_hids_char[HOGPRH_CHAR_REPORT];
                    hids_char[report_idx].req_bf = ATT_REQ(PRES, OPT);
                }

                // 2. create descriptor reference
                // Report Map Char. External Report Reference Descriptor
                hids_char_desc[HOGPRH_DESC_REPORT_MAP_EXT_REP_REF].char_code = HOGPRH_CHAR_REPORT_MAP;
                hids_char_desc[HOGPRH_DESC_REPORT_MAP_EXT_REP_REF].req_bf  = ATT_REQ(PRES, OPT);
                hids_char_desc[HOGPRH_DESC_REPORT_MAP_EXT_REP_REF].uuid      = GATT_DESC_EXT_REPORT_REF;

                for (report_idx = 0; report_idx < HOGPRH_NB_REPORT_INST_MAX; report_idx++)
                {
                    // Report Char. Report Reference
                    hids_char_desc[HOGPRH_DESC_REPORT_REF + report_idx].char_code =
                            HOGPRH_CHAR_REPORT + report_idx;
                    hids_char_desc[HOGPRH_DESC_REPORT_REF + report_idx].req_bf  = ATT_REQ(PRES, OPT);
                    hids_char_desc[HOGPRH_DESC_REPORT_REF + report_idx].uuid = GATT_DESC_REPORT_REF;

                    // Report Client Config
                    hids_char_desc[HOGPRH_DESC_REPORT_CFG + report_idx].char_code =
                            HOGPRH_CHAR_REPORT + report_idx;
                    hids_char_desc[HOGPRH_DESC_REPORT_CFG + report_idx].req_bf  = ATT_REQ(PRES, OPT);
                    hids_char_desc[HOGPRH_DESC_REPORT_CFG + report_idx].uuid = GATT_DESC_CLIENT_CHAR_CFG;
                }

                status = prf_check_svc_validity(HOGPRH_CHAR_MAX, p_con_env->hids[svc_idx].chars, hids_char,
                                                HOGPRH_DESC_MAX, p_con_env->hids[svc_idx].descs,
                                                hids_char_desc);
            }
        }
        else
        {
            status = PRF_ERR_STOP_DISC_CHAR_MISSING;
        }

        hogprh_enable_cmp(p_hogprh_env, conidx, status);
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
__STATIC void hogprh_att_val_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint16_t offset,
                              common_buf_t* p_data)
{
    hogprh_read_val_cmp(conidx, GAP_ERR_NO_ERROR, dummy, p_data);
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
__STATIC void hogprh_read_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    if(status != GAP_ERR_NO_ERROR)
    {
        hogprh_read_val_cmp(conidx, status, dummy, NULL);
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
__STATIC void hogprh_write_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    hogprh_write_val_cmp(conidx, status, dummy);
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
__STATIC void hogprh_att_val_evt_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint8_t evt_type, bool complete,
                        uint16_t hdl, common_buf_t* p_data)
{
    // Get the address of the environment
    hogprh_env_t* p_hogprh_env = PRF_ENV_GET(HOGPRH, hogprh);

    if(p_hogprh_env != NULL)
    {
        hogprh_cnx_env_t* p_con_env = p_hogprh_env->p_env[conidx];

        if(p_con_env != NULL)
        {
            uint8_t hid_idx;
            uint8_t report_idx;

            // Report - HID instance is unknown.
            for (hid_idx = 0; (hid_idx < p_con_env->nb_svc); hid_idx++)
            {
                for (report_idx = 0; (report_idx < HOGPRH_NB_REPORT_INST_MAX) ; report_idx++)
                {
                    // search for report index
                    if (hdl == p_con_env->hids[hid_idx].chars[HOGPRH_CHAR_REPORT + report_idx].val_hdl)
                    {
                        // check if a complete report is received
                        if(complete)
                        {
                            // if yes inform application
                            const hogprh_cb_t* p_cb = (const hogprh_cb_t*) p_hogprh_env->prf_env.p_cb;
                            p_cb->cb_report(conidx, hid_idx, report_idx, p_data);
                        }
                        else
                        {
                            // read value before informing application
                            uint16_t dummy = 0;
                            SETF(dummy, HOGPRH_DUMMY_HID_IDX,      hid_idx);
                            SETF(dummy, HOGPRH_DUMMY_VAL_ID,       HOGPRH_REPORT);
                            SETF(dummy, HOGPRH_DUMMY_REPORT_IDX,   report_idx);
                            SETB(dummy, HOGPRH_DUMMY_NTF_RECV,     true);
                            gatt_cli_read(conidx, p_hogprh_env->user_lid, dummy, hdl, 0, 0);
                        }

                        break;
                    }
                }
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
__STATIC void hogprh_svc_changed_cb(uint8_t conidx, uint8_t user_lid, bool out_of_sync, uint16_t start_hdl, uint16_t end_hdl)
{
    // Do Nothing
}

/// Client callback hander
__STATIC const gatt_cli_cb_t hogprh_cb =
{
    .cb_discover_cmp    = hogprh_discover_cmp_cb,
    .cb_read_cmp        = hogprh_read_cmp_cb,
    .cb_write_cmp       = hogprh_write_cmp_cb,
    .cb_att_val_get     = NULL,
    .cb_svc             = hogprh_svc_cb,
    .cb_svc_info        = NULL,
    .cb_inc_svc         = NULL,
    .cb_char            = NULL,
    .cb_desc            = NULL,
    .cb_att_val         = hogprh_att_val_cb,
    .cb_att_val_evt     = hogprh_att_val_evt_cb,
    .cb_svc_changed     = hogprh_svc_changed_cb,
};

/*
 * PROFILE NATIVE API
 ****************************************************************************************
 */

uint16_t hogprh_enable(uint8_t conidx, uint8_t con_type, uint8_t hids_nb, const hogprh_content_t* p_hids)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Client environment
    hogprh_env_t* p_hogprh_env = PRF_ENV_GET(HOGPRH, hogprh);

    if((hids_nb > HOGPRH_NB_HIDS_INST_MAX) || ((con_type == PRF_CON_NORMAL) && (p_hids == NULL)))
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if(p_hogprh_env != NULL)
    {
        if ((conidx < BLE_CONNECTION_MAX) && (p_hogprh_env->p_env[conidx] == NULL))
        {
            // allocate environment variable for task instance
            p_hogprh_env->p_env[conidx] = (hogprh_cnx_env_t *) kernel_malloc(sizeof(hogprh_cnx_env_t), KERNEL_MEM_ATT_DB);
            memset(p_hogprh_env->p_env[conidx], 0, sizeof(hogprh_cnx_env_t));

            if(p_hogprh_env->p_env[conidx] != NULL)
            {
                // Config connection, start discovering
                if (con_type == PRF_CON_DISCOVERY)
                {
                    uint16_t gatt_svc_uuid = GATT_SVC_HID;

                    // start discovery
                    status = gatt_cli_discover_svc(conidx, p_hogprh_env->user_lid, 0, GATT_DISCOVER_SVC_PRIMARY_BY_UUID, true,
                                                   GATT_MIN_HDL, GATT_MAX_HDL, GATT_UUID_16, (uint8_t*) &gatt_svc_uuid);

                    // Go to DISCOVERING state
                    p_hogprh_env->p_env[conidx]->discover = true;
                }
                // normal connection, get saved att details
                else
                {
                    p_hogprh_env->p_env[conidx]->nb_svc = hids_nb;
                    memcpy(p_hogprh_env->p_env[conidx]->hids, p_hids, sizeof(hogprh_content_t) * hids_nb);
                    status = GAP_ERR_NO_ERROR;

                    // send APP confirmation that can start normal connection to TH
                    hogprh_enable_cmp(p_hogprh_env, conidx, GAP_ERR_NO_ERROR);
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

uint16_t hogprh_read_proto_mode(uint8_t conidx, uint8_t hid_idx)
{
    uint16_t status = hogprh_read_val(conidx, hid_idx, HOGPRH_PROTO_MODE, 0);
    return (status);
}

uint16_t hogprh_read_hid_info(uint8_t conidx, uint8_t hid_idx)
{
    uint16_t status = hogprh_read_val(conidx, hid_idx, HOGPRH_HID_INFO, 0);
    return (status);
}

uint16_t hogprh_read_report_map(uint8_t conidx, uint8_t hid_idx)
{
    uint16_t status = hogprh_read_val(conidx, hid_idx, HOGPRH_REPORT_MAP, 0);
    return (status);
}

uint16_t hogprh_read_report_map_ref(uint8_t conidx, uint8_t hid_idx)
{
    uint16_t status = hogprh_read_val(conidx, hid_idx, HOGPRH_REPORT_MAP_EXT_REP_REF, 0);
    return (status);
}
uint16_t hogprh_read_report_cfg(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx)
{
    uint16_t status = hogprh_read_val(conidx, hid_idx, HOGPRH_REPORT_NTF_CFG, report_idx);
    return (status);
}

uint16_t hogprh_read_report(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx)
{
    uint16_t status = hogprh_read_val(conidx, hid_idx, HOGPRH_REPORT, report_idx);
    return (status);
}

uint16_t hogprh_read_report_ref(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx)
{
    uint16_t status = hogprh_read_val(conidx, hid_idx, HOGPRH_REPORT_REF, report_idx);
    return (status);
}

uint16_t hogprh_write_proto_mode(uint8_t conidx, uint8_t hid_idx, uint8_t proto_mode)
{
    common_buf_t* p_buf;
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    // allocate a buffer to perform write
    if(common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, sizeof(uint8_t), GATT_BUFFER_TAIL_LEN) == COMMON_BUF_ERR_NO_ERROR)
    {
        common_buf_data(p_buf)[0] = proto_mode;
        status = hogprh_write_val(conidx, hid_idx, HOGPRH_PROTO_MODE, 0, true, p_buf);
        common_buf_release(p_buf);
    }

    return (status);
}

uint16_t hogprh_write_ctnl_pt(uint8_t conidx, uint8_t hid_idx, uint8_t value)
{
    common_buf_t* p_buf;
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    // allocate a buffer to perform write
    if(common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, sizeof(uint8_t), GATT_BUFFER_TAIL_LEN) == COMMON_BUF_ERR_NO_ERROR)
    {
        common_buf_data(p_buf)[0] = value;
        status = hogprh_write_val(conidx, hid_idx, HOGPRH_HID_CTNL_PT, 0, true, p_buf);
        common_buf_release(p_buf);
    }

    return (status);
}

uint16_t hogprh_write_report_cfg(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx, uint16_t cfg_val)

{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    common_buf_t* p_buf;

    if (cfg_val > PRF_CLI_START_NTF)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else
    {
        // allocate a buffer to perform write
        if(common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, sizeof(uint16_t), GATT_BUFFER_TAIL_LEN) == COMMON_BUF_ERR_NO_ERROR)
        {
            common_write16p(common_buf_data(p_buf), common_htobs(cfg_val));
            status = hogprh_write_val(conidx, hid_idx, HOGPRH_REPORT_NTF_CFG, report_idx, false, p_buf);
            common_buf_release(p_buf);
        }
    }

    return (status);
}

uint16_t hogprh_write_report(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx, bool wr_cmd, common_buf_t* p_report)
{
    uint16_t status = hogprh_write_val(conidx, hid_idx, HOGPRH_REPORT, report_idx, wr_cmd, p_report);
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
__STATIC int hogprh_enable_req_handler(kernel_msg_id_t const msgid, struct hogprh_enable_req const *p_param,
                                       kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint16_t status = hogprh_enable(p_param->conidx, p_param->con_type, p_param->hids_nb, p_param->hids);

    // send an error if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct hogprh_enable_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_ENABLE_RSP, src_id, dest_id, hogprh_enable_rsp);
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
 * @brief Handles reception of the @ref HOGPRH_READ_INFO_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogprh_read_info_req_handler(kernel_msg_id_t const msgid, struct hogprh_read_info_req const *p_param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint8_t status;
    uint8_t conidx     = p_param->conidx;
    uint8_t hid_idx    = p_param->hid_idx;
    uint8_t report_idx = p_param->report_idx;


    // check requested info
    switch (p_param->info)
    {
        // Protocol Mode
        case HOGPRH_PROTO_MODE:             { status = hogprh_read_proto_mode(conidx, hid_idx);             } break;
        // HID Information
        case HOGPRH_HID_INFO:               { status = hogprh_read_hid_info(conidx, hid_idx);               } break;
        // Report Map
        case HOGPRH_REPORT_MAP:             { status = hogprh_read_report_map(conidx, hid_idx);             } break;
        // Report Map Char. External Report Reference Descriptor
        case HOGPRH_REPORT_MAP_EXT_REP_REF: { status = hogprh_read_report_map_ref(conidx, hid_idx);         } break;
        // Report
        case HOGPRH_REPORT:                 { status = hogprh_read_report(conidx, hid_idx, report_idx);     } break;
        // Report Char. Report Reference
        case HOGPRH_REPORT_REF:             { status = hogprh_read_report_ref(conidx, hid_idx, report_idx); } break;
        // Report Notification config
        case HOGPRH_REPORT_NTF_CFG:         { status = hogprh_read_report_cfg(conidx, hid_idx, report_idx); } break;

        default:                            { status = PRF_ERR_INVALID_PARAM;                               } break;
    }

    // request cannot be performed
    if (status != GAP_ERR_NO_ERROR)
    {
        struct hogprh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_READ_INFO_RSP, src_id, dest_id, hogprh_read_info_rsp);

        // set error status
        if(p_rsp != NULL)
        {
            p_rsp->conidx  = conidx;
            p_rsp->status  = status;
            p_rsp->info    = p_param->info;
            p_rsp->hid_idx = hid_idx;
            p_rsp->data.report.length = 0;

            kernel_msg_send(p_rsp);
        }
    }

    return (KERNEL_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPRH_WRITE_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogprh_write_req_handler(kernel_msg_id_t const msgid, struct hogprh_write_req const *p_param,
                                      kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint8_t status;

    // check requested info
    switch (p_param->info)
    {
        // Protocol Mode
        case HOGPRH_PROTO_MODE:
        {
            status = hogprh_write_proto_mode(p_param->conidx, p_param->hid_idx, p_param->data.proto_mode);
        } break;
        // HID Control Point
        case HOGPRH_HID_CTNL_PT:
        {
            status = hogprh_write_ctnl_pt(p_param->conidx, p_param->hid_idx, p_param->data.hid_ctnl_pt);
        } break;

        // Report Char. Report Reference
        case HOGPRH_REPORT:
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            uint16_t report_len = p_param->data.report.length;
            common_buf_t* p_buf = NULL;

            // allocate a buffer to perform write
            if(common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, report_len, GATT_BUFFER_TAIL_LEN) == COMMON_BUF_ERR_NO_ERROR)
            {
                common_buf_copy_data_from_mem(p_buf, p_param->data.report.value, report_len);
                status = hogprh_write_report(p_param->conidx, p_param->hid_idx, p_param->report_idx, p_param->wr_cmd, p_buf);
                common_buf_release(p_buf);
            }
        } break;

        // Report Notification config
        case HOGPRH_REPORT_NTF_CFG:
        {
            status = hogprh_write_report_cfg(p_param->conidx, p_param->hid_idx, p_param->report_idx, p_param->data.report_cfg);
        } break;

        default: { status = PRF_ERR_INVALID_PARAM; } break;
    }

    // request cannot be performed
    if (status != GAP_ERR_NO_ERROR)
    {
        struct hogprh_write_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_WRITE_RSP, src_id, dest_id, hogprh_write_rsp);

        // set error status
        if(p_rsp != NULL)
        {
            p_rsp->conidx     = p_param->conidx;
            p_rsp->status     = status;
            p_rsp->info       = p_param->info;
            p_rsp->hid_idx    = p_param->hid_idx;
            p_rsp->report_idx = p_param->report_idx;

            kernel_msg_send(p_rsp);
        }
    }

    return (KERNEL_MSG_CONSUMED);
}


/// Default State handlers definition
KERNEL_MSG_HANDLER_TAB(hogprh)
{
    // Note: all messages must be sorted in ID ascending order

    {HOGPRH_ENABLE_REQ,        (kernel_msg_func_t)hogprh_enable_req_handler     },
    {HOGPRH_READ_INFO_REQ,     (kernel_msg_func_t)hogprh_read_info_req_handler  },
    {HOGPRH_WRITE_REQ,         (kernel_msg_func_t)hogprh_write_req_handler      },
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
__STATIC void hogprh_cb_enable_cmp(uint8_t conidx, uint16_t status, uint8_t hids_nb, const hogprh_content_t* p_hids)
{
    struct hogprh_enable_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_ENABLE_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                   hogprh_enable_rsp);

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
 * @brief Completion of read Protocol Mode procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the procedure execution (@see enum hl_err)
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 * @param[in] proto_mode    Protocol mode.
 *
 ****************************************************************************************
 */
__STATIC void hogprh_cb_read_proto_mode_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t proto_mode)
{
    struct hogprh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_READ_INFO_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                      hogprh_read_info_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx          = conidx;
        p_rsp->status          = status;
        p_rsp->hid_idx         = hid_idx;
        p_rsp->report_idx      = 0;
        p_rsp->info            = HOGPRH_PROTO_MODE;
        p_rsp->data.proto_mode = proto_mode;

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogprh_cb_read_hid_info_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx,
                                          const hogp_hid_info_t* p_hid_info)
{
    struct hogprh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_READ_INFO_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                      hogprh_read_info_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx          = conidx;
        p_rsp->status          = status;
        p_rsp->hid_idx         = hid_idx;
        p_rsp->report_idx      = 0;
        p_rsp->info            = HOGPRH_HID_INFO;
        memcpy(&(p_rsp->data.hid_info), p_hid_info, sizeof(hogp_hid_info_t));

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogprh_cb_read_report_map_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, common_buf_t* p_report_map)
{
    uint16_t data_len = common_buf_data_len(p_report_map);
    struct hogprh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC_DYN(HOGPRH_READ_INFO_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                          hogprh_read_info_rsp, data_len);

    if(p_rsp != NULL)
    {
        p_rsp->conidx        = conidx;
        p_rsp->status        = status;
        p_rsp->hid_idx       = hid_idx;
        p_rsp->report_idx    = 0;
        p_rsp->info          = HOGPRH_REPORT_MAP;
        p_rsp->data.report_map.length = data_len;
        common_buf_copy_data_to_mem(p_report_map, p_rsp->data.report_map.value, data_len);

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogprh_cb_read_report_map_ref_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx,
                                                const hogp_report_map_ref_t* p_report_map_ref)
{
    struct hogprh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_READ_INFO_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                      hogprh_read_info_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx          = conidx;
        p_rsp->status          = status;
        p_rsp->hid_idx         = hid_idx;
        p_rsp->report_idx      = 0;
        p_rsp->info            = HOGPRH_REPORT_MAP_EXT_REP_REF;
        memcpy(&(p_rsp->data.report_map_ref), p_report_map_ref, sizeof(hogp_report_map_ref_t));

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogprh_cb_read_report_cfg_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx,
                                            uint16_t cfg_val)
{
    struct hogprh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_READ_INFO_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                      hogprh_read_info_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx          = conidx;
        p_rsp->status          = status;
        p_rsp->hid_idx         = hid_idx;
        p_rsp->report_idx      = report_idx;
        p_rsp->info            = HOGPRH_REPORT_NTF_CFG;
        p_rsp->data.report_cfg = cfg_val;

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogprh_cb_read_report_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx,
                                        common_buf_t* p_report)
{
    uint16_t data_len = common_buf_data_len(p_report);
    struct hogprh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC_DYN(HOGPRH_READ_INFO_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                          hogprh_read_info_rsp, data_len);

    if(p_rsp != NULL)
    {
        p_rsp->conidx        = conidx;
        p_rsp->status        = status;
        p_rsp->hid_idx       = hid_idx;
        p_rsp->report_idx    = report_idx;
        p_rsp->info          = HOGPRH_REPORT;
        p_rsp->data.report.length = data_len;
        common_buf_copy_data_to_mem(p_report, p_rsp->data.report.value, data_len);

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogprh_cb_read_report_ref_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx,
                                            const hogp_report_ref_t* p_report_ref)
{
    struct hogprh_read_info_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_READ_INFO_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                      hogprh_read_info_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx          = conidx;
        p_rsp->status          = status;
        p_rsp->hid_idx         = hid_idx;
        p_rsp->report_idx      = report_idx;
        p_rsp->info            = HOGPRH_REPORT_REF;
        memcpy(&(p_rsp->data.report_ref), p_report_ref, sizeof(hogp_report_ref_t));

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogprh_cb_write_proto_mode_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx)
{
    struct hogprh_write_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_WRITE_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                   hogprh_write_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx       = conidx;
        p_rsp->status       = status;
        p_rsp->hid_idx      = hid_idx;
        p_rsp->hid_idx      = 0;
        p_rsp->info         = HOGPRH_PROTO_MODE;

        kernel_msg_send(p_rsp);
    }
}

/**
 ****************************************************************************************
 * @brief Completion of write control point procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the procedure execution (@see enum hl_err)
 * @param[in] hid_idx       HID Service Instance - From 0 to HOGPRH_NB_HIDS_INST_MAX-1
 ****************************************************************************************
 */
__STATIC void hogprh_cb_write_ctnl_pt_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx)
{
    struct hogprh_write_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_WRITE_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                   hogprh_write_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx       = conidx;
        p_rsp->status       = status;
        p_rsp->hid_idx      = hid_idx;
        p_rsp->hid_idx      = 0;
        p_rsp->info         = HOGPRH_HID_CTNL_PT;

        kernel_msg_send(p_rsp);
    }
}

/**
 ****************************************************************************************
 * @brief Completion of write Characteristic Configuration procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the procedure execution (@see enum hl_err)
 * @param[in] report_idx    Index of the report in the service
 ****************************************************************************************
 */
__STATIC void hogprh_cb_write_report_cfg_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx)
{
    struct hogprh_write_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_WRITE_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                   hogprh_write_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx       = conidx;
        p_rsp->status       = status;
        p_rsp->hid_idx      = hid_idx;
        p_rsp->hid_idx      = report_idx;
        p_rsp->info         = HOGPRH_REPORT_NTF_CFG;

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogprh_cb_write_report_cmp(uint8_t conidx, uint16_t status, uint8_t hid_idx, uint8_t report_idx)
{
    struct hogprh_write_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPRH_WRITE_RSP, PRF_DST_TASK(HOGPRH), PRF_SRC_TASK(HOGPRH),
                                                   hogprh_write_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx       = conidx;
        p_rsp->status       = status;
        p_rsp->hid_idx      = hid_idx;
        p_rsp->hid_idx      = report_idx;
        p_rsp->info         = HOGPRH_REPORT;

        kernel_msg_send(p_rsp);
    }
}

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
__STATIC void hogprh_cb_report(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx, common_buf_t* p_report)
{
    uint16_t data_len = common_buf_data_len(p_report);
    // send report indication
    struct hogprh_report_ind *p_ind = KERNEL_MSG_ALLOC_DYN(HOGPRH_REPORT_IND, PRF_DST_TASK(HOGPRH),
                                                       PRF_SRC_TASK(HOGPRH), hogprh_report_ind, data_len);

    if(p_ind != NULL)
    {
        p_ind->conidx        = conidx;
        p_ind->hid_idx       = hid_idx;
        p_ind->report_idx    = report_idx;
        p_ind->report.length = data_len;
        common_buf_copy_data_to_mem(p_report, p_ind->report.value, data_len);

        kernel_msg_send(p_ind);
    }
}

/// Default Message handle
__STATIC const hogprh_cb_t hogprh_msg_cb =
{
    .cb_enable_cmp              = hogprh_cb_enable_cmp,
    .cb_read_proto_mode_cmp     = hogprh_cb_read_proto_mode_cmp,
    .cb_read_hid_info_cmp       = hogprh_cb_read_hid_info_cmp,
    .cb_read_report_map_cmp     = hogprh_cb_read_report_map_cmp,
    .cb_read_report_map_ref_cmp = hogprh_cb_read_report_map_ref_cmp,
    .cb_read_report_cfg_cmp     = hogprh_cb_read_report_cfg_cmp,
    .cb_read_report_cmp         = hogprh_cb_read_report_cmp,
    .cb_read_report_ref_cmp     = hogprh_cb_read_report_ref_cmp,
    .cb_write_proto_mode_cmp    = hogprh_cb_write_proto_mode_cmp,
    .cb_write_ctnl_pt_cmp       = hogprh_cb_write_ctnl_pt_cmp,
    .cb_write_report_cfg_cmp    = hogprh_cb_write_report_cfg_cmp,
    .cb_write_report_cmp        = hogprh_cb_write_report_cmp,
    .cb_report                  = hogprh_cb_report,
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
__STATIC uint16_t hogprh_init(prf_data_t* p_env, uint16_t* p_start_hdl, uint8_t sec_lvl, uint8_t user_prio,
                            const void* p_params, const hogprh_cb_t* p_cb)
{
    uint8_t conidx;
    // DB Creation Status
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t user_lid = GATT_INVALID_USER_LID;

    do
    {
        hogprh_env_t* p_hogprh_env;

        #if (BLE_HL_MSG_API)
        if(p_cb == NULL)
        {
            p_cb = &(hogprh_msg_cb);
        }
        #endif // (BLE_HL_MSG_API)

        if(   (p_params == NULL) || (p_cb == NULL) || (p_cb->cb_enable_cmp == NULL) || (p_cb->cb_read_proto_mode_cmp == NULL)
           || (p_cb->cb_read_hid_info_cmp == NULL)  || (p_cb->cb_read_report_map_cmp == NULL)
           || (p_cb->cb_read_report_map_ref_cmp == NULL) || (p_cb->cb_read_report_cfg_cmp == NULL)
           || (p_cb->cb_read_report_cmp == NULL) || (p_cb->cb_read_report_ref_cmp == NULL)
           || (p_cb->cb_write_proto_mode_cmp == NULL) || (p_cb->cb_write_ctnl_pt_cmp == NULL)
           || (p_cb->cb_write_report_cfg_cmp == NULL) || (p_cb->cb_write_report_cmp == NULL)  || (p_cb->cb_report == NULL)  )
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // register HOGPRH user
        status = gatt_user_cli_register(HOGP_REPORT_MAX_LEN, user_prio, &hogprh_cb, &user_lid);
        if(status != GAP_ERR_NO_ERROR) break;

        //-------------------- allocate memory required for the profile  ---------------------
        p_hogprh_env = (hogprh_env_t*) kernel_malloc(sizeof(hogprh_env_t), KERNEL_MEM_ATT_DB);

        if(p_hogprh_env != NULL)
        {
            // allocate HOGPRH required environment variable
            p_env->p_env = (prf_hdr_t *) p_hogprh_env;

            // initialize environment variable
            p_hogprh_env->prf_env.p_cb    = p_cb;
            #if (BLE_HL_MSG_API)
            p_env->desc.msg_handler_tab = hogprh_msg_handler_tab;
            p_env->desc.msg_cnt         = ARRAY_LEN(hogprh_msg_handler_tab);
            #endif // (BLE_HL_MSG_API)

            p_hogprh_env->user_lid = user_lid;
            for (conidx = 0; conidx < BLE_CONNECTION_MAX; conidx++)
            {
                p_hogprh_env->p_env[conidx] = NULL;
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
__STATIC uint16_t hogprh_destroy(prf_data_t *p_env, uint8_t reason)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    hogprh_env_t* p_hogprh_env = (hogprh_env_t*) p_env->p_env;

    if(reason != PRF_DESTROY_RESET)
    {
        status = gatt_user_unregister(p_hogprh_env->user_lid);
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        uint8_t idx;

        // cleanup environment variable for each task instances
        for (idx = 0; idx < BLE_CONNECTION_MAX; idx++)
        {
            if (p_hogprh_env->p_env[idx] != NULL)
            {
                kernel_free(p_hogprh_env->p_env[idx]);
            }
        }

        // free profile environment variables
        p_env->p_env = NULL;
        kernel_free(p_hogprh_env);
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
__STATIC void hogprh_con_create(prf_data_t *p_env, uint8_t conidx, const gap_con_param_t* p_con_param)
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
__STATIC void hogprh_con_cleanup(prf_data_t *p_env, uint8_t conidx, uint16_t reason)
{
    hogprh_env_t* p_hogprh_env = (hogprh_env_t*) p_env->p_env;

    // clean-up environment variable allocated for task instance
    if (p_hogprh_env->p_env[conidx] != NULL)
    {
        kernel_free(p_hogprh_env->p_env[conidx]);
        p_hogprh_env->p_env[conidx] = NULL;
    }
}

/// HOGPRH Task interface required by profile manager
const prf_task_cbs_t hogprh_itf =
{
    .cb_init          = (prf_init_cb) hogprh_init,
    .cb_destroy       = hogprh_destroy,
    .cb_con_create    = hogprh_con_create,
    .cb_con_cleanup   = hogprh_con_cleanup,
    .cb_con_upd       = NULL,
};

/**
 ****************************************************************************************
 * @brief Retrieve client profile interface
 *
 * @return Client profile interface
 ****************************************************************************************
 */
const prf_task_cbs_t* hogprh_prf_itf_get(void)
{
    return &hogprh_itf;
}

#endif /* (BLE_HID_REPORT_HOST) */

/// @} HOGPRH
