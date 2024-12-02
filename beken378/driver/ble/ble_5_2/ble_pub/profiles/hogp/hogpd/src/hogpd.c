/**
 ****************************************************************************************
 *
 * @file hogpd.c
 *
 * @brief HID Over GATT Profile HID Device Role Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HOGPD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwprf_config.h"

#if (BLE_HID_DEVICE)
#include "hogpd.h"
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

#define HIDS_CFG_FLAG_MANDATORY_MASK        ((uint32_t)0x000FD)
#define HIDS_MANDATORY_ATT_NB               (7)
#define HIDS_CFG_FLAG_MAP_EXT_MASK          ((uint32_t)0x00102)
#define HIDS_MAP_EXT_ATT_NB                 (2)
#define HIDS_CFG_FLAG_PROTO_MODE_MASK       ((uint32_t)0x00600)
#define HIDS_PROTO_MODE_ATT_NB              (2)
#define HIDS_CFG_FLAG_KEYBOARD_MASK         ((uint32_t)0x0F800)
#define HIDS_KEYBOARD_ATT_NB                (5)
#define HIDS_CFG_FLAG_MOUSE_MASK            ((uint32_t)0x70000)
#define HIDS_MOUSE_ATT_NB                   (3)

#define HIDS_CFG_REPORT_MANDATORY_MASK      ((uint32_t)0x7)
#define HIDS_REPORT_MANDATORY_ATT_NB        (3)
#define HIDS_CFG_REPORT_IN_MASK             ((uint32_t)0x8)
#define HIDS_REPORT_IN_ATT_NB               (1)
/// number of attribute index for a report
#define HIDS_REPORT_NB_IDX                  (4)

/// Boot KB Input Report Notification Configuration Bit Mask
#define HOGPD_BOOT_KB_IN_NTF_CFG_MASK       (0x40)
/// Boot KB Input Report Notification Configuration Bit Mask
#define HOGPD_BOOT_MOUSE_IN_NTF_CFG_MASK    (0x80)
/// Boot Report Notification Configuration Bit Mask
#define HOGPD_REPORT_NTF_CFG_MASK           (0x20)


/// HOGPD Attributes database handle list
enum hogpd_att_db_handles
{
    HOGPD_IDX_SVC,

    /// Included Service
    HOGPD_IDX_INCL_SVC,

    /// HID Information
    HOGPD_IDX_HID_INFO_CHAR,
    HOGPD_IDX_HID_INFO_VAL,

    /// HID Control Point
    HOGPD_IDX_HID_CTNL_PT_CHAR,
    HOGPD_IDX_HID_CTNL_PT_VAL,

    /// Report Map
    HOGPD_IDX_REPORT_MAP_CHAR,
    HOGPD_IDX_REPORT_MAP_VAL,
    HOGPD_IDX_REPORT_MAP_EXT_REP_REF,

    /// Protocol Mode
    HOGPD_IDX_PROTO_MODE_CHAR,
    HOGPD_IDX_PROTO_MODE_VAL,

    /// Boot Keyboard Input Report
    HOGPD_IDX_BOOT_KB_IN_REPORT_CHAR,
    HOGPD_IDX_BOOT_KB_IN_REPORT_VAL,
    HOGPD_IDX_BOOT_KB_IN_REPORT_NTF_CFG,

    /// Boot Keyboard Output Report
    HOGPD_IDX_BOOT_KB_OUT_REPORT_CHAR,
    HOGPD_IDX_BOOT_KB_OUT_REPORT_VAL,

    /// Boot Mouse Input Report
    HOGPD_IDX_BOOT_MOUSE_IN_REPORT_CHAR,
    HOGPD_IDX_BOOT_MOUSE_IN_REPORT_VAL,
    HOGPD_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG,

    /// number of attributes that are uniq in the service
    HOGPD_ATT_UNIQ_NB,

    /// Report
    HOGPD_IDX_REPORT_CHAR = HOGPD_ATT_UNIQ_NB,
    HOGPD_IDX_REPORT_VAL,
    HOGPD_IDX_REPORT_REP_REF,
    HOGPD_IDX_REPORT_NTF_CFG,

    HOGPD_IDX_NB,

    /// maximum number of attribute that can be present in the HID service
    HOGPD_ATT_MAX = HOGPD_ATT_UNIQ_NB + (4* (HOGPD_NB_REPORT_INST_MAX)),
};


/*
 * TYPES DEFINITION
 ****************************************************************************************
 */


/// ongoing operation information
typedef struct hogpd_buf_meta
{
    /// Connection index targeted
    uint8_t   conidx;
    /// notification targeted handle
    uint16_t  hdl;
} hogpd_buf_meta_t;

/// HIDS service cfg
typedef struct hogpd_svc_cfg
{
    /// Service Features (@see enum hogpd_cfg)
    uint16_t            features;
    /// Number of attribute present in service
    uint8_t             nb_att;
    /// Number of Report Char. instances to add in the database
    uint8_t             nb_report;
    /// Handle offset where report are available - to enhance handle search
    uint8_t             report_hdl_offset;
    /// Current Protocol Mode
    uint8_t             proto_mode;
    /// Report Char. Configuration (@see enum hogpd_report_cfg)
    uint8_t             report_char_cfg[HOGPD_NB_REPORT_INST_MAX];
    /// Report id number
    uint8_t             report_id[HOGPD_NB_REPORT_INST_MAX];
    /// HID Information Char. Values
    hogp_hid_info_t     hid_info;
    /// External Report Reference - Characteristic UUID
    uint16_t            rep_ref_uuid;
} hogpd_svc_cfg_t;

/// HOGPD Server Environment Variable
typedef struct hogpd_env
{
    /// profile environment
    prf_hdr_t       prf_env;
    /// Operation Event TX wait queue
    common_list_t       wait_queue;
    /// Supported Features
    hogpd_svc_cfg_t svcs[HOGPD_NB_HIDS_INST_MAX];
    /// Notification configuration per service and configuration
    uint16_t        ntf_cfg[HOGPD_NB_HIDS_INST_MAX][BLE_CONNECTION_MAX];
    /// Service Attribute Start Handle
    uint16_t        start_hdl;
    /// Number of HIDS added in the database
    uint8_t         hids_nb;
    /// GATT user local identifier
    uint8_t         user_lid;
    /// Operation On-going
    bool            op_ongoing;
    /// Prevent recursion in execute_operation function
    bool            in_exe_op;

} hogpd_env_t;


/*
 * ATTRIBUTES DATABASE
 ****************************************************************************************
 */

/// Full Database Description - Used to add attributes into the database
const gatt_att16_desc_t hogpd_att_db[HOGPD_IDX_NB] =
{
    // HID Service Declaration
    [HOGPD_IDX_SVC]                          = { GATT_DECL_PRIMARY_SERVICE,      PROP(RD),                         0                                          },
    // HID Service Declaration
    [HOGPD_IDX_INCL_SVC]                     = { GATT_DECL_INCLUDE,              PROP(RD),                         0                                          },

    // HID Information Characteristic Declaration
    [HOGPD_IDX_HID_INFO_CHAR]                = { GATT_DECL_CHARACTERISTIC,       PROP(RD),                         0                                          },
    // HID Information Characteristic Value
    [HOGPD_IDX_HID_INFO_VAL]                 = { GATT_CHAR_HID_INFO,             PROP(RD),                         OPT(NO_OFFSET)                             },

    // HID Control Point Characteristic Declaration
    [HOGPD_IDX_HID_CTNL_PT_CHAR]             = { GATT_DECL_CHARACTERISTIC,       PROP(RD),                         0                                          },
    // HID Control Point Characteristic Value
    [HOGPD_IDX_HID_CTNL_PT_VAL]              = { GATT_CHAR_HID_CTNL_PT,          PROP(WC),                         OPT(NO_OFFSET) | sizeof(uint8_t)           },

    // Report Map Characteristic Declaration
    [HOGPD_IDX_REPORT_MAP_CHAR]              = { GATT_DECL_CHARACTERISTIC,       PROP(RD),                         0                                          },
    // Report Map Characteristic Value
    [HOGPD_IDX_REPORT_MAP_VAL]               = { GATT_CHAR_REPORT_MAP,           PROP(RD),                         HOGP_REPORT_MAP_MAX_LEN                    },
    // Report Map Characteristic - External Report Reference Descriptor
    [HOGPD_IDX_REPORT_MAP_EXT_REP_REF]       = { GATT_DESC_EXT_REPORT_REF,       PROP(RD),                         OPT(NO_OFFSET)                             },

    // Protocol Mode Characteristic Declaration
    [HOGPD_IDX_PROTO_MODE_CHAR]              = { GATT_DECL_CHARACTERISTIC,       PROP(RD),                         0                                          },
    // Protocol Mode Characteristic Value
    [HOGPD_IDX_PROTO_MODE_VAL]               = { GATT_CHAR_PROTOCOL_MODE,        PROP(RD) | PROP(WC),              OPT(NO_OFFSET) | sizeof(uint8_t)           },

    // Boot Keyboard Input Report Characteristic Declaration
    [HOGPD_IDX_BOOT_KB_IN_REPORT_CHAR]       = { GATT_DECL_CHARACTERISTIC,       PROP(RD),                         0                                          },
    // Boot Keyboard Input Report Characteristic Value
    [HOGPD_IDX_BOOT_KB_IN_REPORT_VAL]        = { GATT_CHAR_BOOT_KB_IN_REPORT,    PROP(RD) | PROP(N),               HOGP_BOOT_REPORT_MAX_LEN                   },
    // Boot Keyboard Input Report Characteristic - Client Characteristic Configuration Descriptor
    [HOGPD_IDX_BOOT_KB_IN_REPORT_NTF_CFG]    = { GATT_DESC_CLIENT_CHAR_CFG,      PROP(RD) | PROP(WR),              OPT(NO_OFFSET)                             },

    // Boot Keyboard Output Report Characteristic Declaration
    [HOGPD_IDX_BOOT_KB_OUT_REPORT_CHAR]      = { GATT_DECL_CHARACTERISTIC,       PROP(RD),                         0                                          },
    // Boot Keyboard Output Report Characteristic Value
    [HOGPD_IDX_BOOT_KB_OUT_REPORT_VAL]       = { GATT_CHAR_BOOT_KB_OUT_REPORT,   PROP(RD) | PROP(WR) | PROP(WC),   OPT(NO_OFFSET) | HOGP_BOOT_REPORT_MAX_LEN  },

    // Boot Mouse Input Report Characteristic Declaration
    [HOGPD_IDX_BOOT_MOUSE_IN_REPORT_CHAR]    = { GATT_DECL_CHARACTERISTIC,       PROP(RD),                         0                                          },
    // Boot Mouse Input Report Characteristic Value
    [HOGPD_IDX_BOOT_MOUSE_IN_REPORT_VAL]     = { GATT_CHAR_BOOT_MOUSE_IN_REPORT, PROP(RD) | PROP(N),               OPT(NO_OFFSET) | HOGP_BOOT_REPORT_MAX_LEN  },
    // Boot Mouse Input Report Characteristic - Client Characteristic Configuration Descriptor
    [HOGPD_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG] = { GATT_DESC_CLIENT_CHAR_CFG,      PROP(RD) | PROP(WR) | PROP(WC),   OPT(NO_OFFSET)                             },

    // Report Characteristic Declaration
    [HOGPD_IDX_REPORT_CHAR]                  = { GATT_DECL_CHARACTERISTIC,       PROP(RD),                         0                                          },
    // Report Characteristic Value
    [HOGPD_IDX_REPORT_VAL]                   = { GATT_CHAR_REPORT,               PROP(RD),                         HOGP_REPORT_MAX_LEN                        },
    // Report Characteristic - Report Reference Descriptor
    [HOGPD_IDX_REPORT_REP_REF]               = { GATT_DESC_REPORT_REF,           PROP(RD),                         OPT(NO_OFFSET)                             },
    // Report Characteristic - Client Characteristic Configuration Descriptor
    [HOGPD_IDX_REPORT_NTF_CFG]               = { GATT_DESC_CLIENT_CHAR_CFG,      PROP(RD) | PROP(WR) | PROP(WC),   OPT(NO_OFFSET)                             },
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Retrieve Attribute handle from service and attribute index
 *
 * @param[in] p_hogpd_env  HID Service environment
 * @param[in] svc_idx      HID Service index
 * @param[in] att_idx      Attribute index
 * @param[in] report_idx   Report index
 *
 * @return HID attribute handle or INVALID HANDLE if nothing found
 ****************************************************************************************
 */
__STATIC uint16_t hogpd_get_att_handle(hogpd_env_t *p_hogpd_env, uint8_t svc_idx, uint8_t att_idx, uint8_t report_idx)
{
    uint16_t handle  = GATT_INVALID_HDL;
    uint8_t i = 0;

    // Sanity check
    if (   (svc_idx < p_hogpd_env ->hids_nb) && (att_idx < HOGPD_IDX_NB)
        && ((att_idx < HOGPD_ATT_UNIQ_NB) || (report_idx < p_hogpd_env->svcs[svc_idx].nb_report)))
    {
        handle = p_hogpd_env->start_hdl;

        for (i = 0 ; i < svc_idx ; i++)
        {
            // update start handle for next service - only useful if multiple service, else not used.
            handle +=  p_hogpd_env->svcs[i].nb_att;
        }

        // increment index according to expected index
        if (att_idx < HOGPD_ATT_UNIQ_NB)
        {
            handle += att_idx;

            // check if Keyboard feature active
            if (!GETB(p_hogpd_env->svcs[svc_idx].features, HOGPD_CFG_KEYBOARD))
            {
               if (att_idx > HOGPD_IDX_BOOT_KB_OUT_REPORT_VAL)
               {
                   handle -= HIDS_KEYBOARD_ATT_NB;
               }
               // Error Case
               else if (   (att_idx >= HOGPD_IDX_BOOT_KB_IN_REPORT_CHAR)
                        && (att_idx <= HOGPD_IDX_BOOT_KB_OUT_REPORT_VAL))
               {
                   handle = GATT_INVALID_HDL;
               }
            }

            // check if Mouse feature active
            if (!GETB(p_hogpd_env->svcs[svc_idx].features, HOGPD_CFG_MOUSE))
            {
               if(att_idx > HOGPD_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG)
               {
                   handle -= HIDS_MOUSE_ATT_NB;
               }
               // Error Case
               else if (   (att_idx >= HOGPD_IDX_BOOT_MOUSE_IN_REPORT_CHAR)
                        && (att_idx <= HOGPD_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG))
               {
                   handle = GATT_INVALID_HDL;
               }
            }

            // check if Protocol Mode feature active
            if (!GETB(p_hogpd_env->svcs[svc_idx].features, HOGPD_CFG_PROTO_MODE))
            {
               if (att_idx > HOGPD_IDX_PROTO_MODE_VAL)
               {
                   handle -= HIDS_PROTO_MODE_ATT_NB;
               }
               // Error Case
               else if ((att_idx >= HOGPD_IDX_PROTO_MODE_CHAR)
                       && (att_idx <= HOGPD_IDX_PROTO_MODE_VAL))
               {
                   handle = GATT_INVALID_HDL;
               }
            }

            // check if Ext Ref feature active
            if (!GETB(p_hogpd_env->svcs[svc_idx].features, HOGPD_CFG_MAP_EXT_REF))
            {
               if (att_idx > HOGPD_IDX_REPORT_MAP_EXT_REP_REF)
               {
                   handle -= HIDS_MAP_EXT_ATT_NB;
               }
               else if (att_idx > HOGPD_IDX_INCL_SVC)
               {
                   handle -= 1;
               }
               // Error Case
               else if ((att_idx == HOGPD_IDX_INCL_SVC) || (att_idx == HOGPD_IDX_REPORT_MAP_EXT_REP_REF))
               {
                   handle = GATT_INVALID_HDL;
               }
            }
        }
        else
        {
            handle += p_hogpd_env->svcs[svc_idx].report_hdl_offset;

            // increment attribute handle with other reports
            for (i = 0 ; i < report_idx; i++)
            {
                handle += HIDS_REPORT_MANDATORY_ATT_NB;
                // check if it's a Report input
                if ((p_hogpd_env->svcs[svc_idx].features & (HOGPD_CFG_REPORT_NTF_EN << i)) != 0)
                {
                    handle += HIDS_REPORT_IN_ATT_NB;
                }
            }

            // Error check
            if (   (att_idx == HOGPD_IDX_REPORT_NTF_CFG)
                && ((p_hogpd_env->svcs[svc_idx].features & (HOGPD_CFG_REPORT_NTF_EN << report_idx)) != 0))
            {
                handle = GATT_INVALID_HDL;
            }
            else
            {
                // update handle cursor
                handle += att_idx - HOGPD_ATT_UNIQ_NB;
            }
        }
    }

    return handle;
}

/**
 ****************************************************************************************
 * @brief Retrieve Service and attribute index form attribute handle
 *
 * @param[in] p_hogpd_env    HID Service environment
 * @param[out] handle        Attribute handle
 * @param[out] p_svc_idx     HID Service index
 * @param[out] p_att_idx     Attribute index
 * @param[out] p_report_idx  Report Index
 *
 * @return Success if attribute and service index found, else Application error
 ****************************************************************************************
 */
__STATIC uint16_t hogpd_get_att_idx(hogpd_env_t* p_hogpd_env, uint16_t handle, uint8_t *p_svc_idx, uint8_t *p_att_idx,
                                    uint8_t *p_report_idx)
{
    uint16_t hdl_cursor = p_hogpd_env->start_hdl;
    uint16_t status = PRF_APP_ERROR;

    // invalid index
    *p_att_idx = HOGPD_IDX_NB;

    // Browse list of services
    // handle must be greater than current index
    for (*p_svc_idx = 0; (*p_svc_idx < p_hogpd_env->hids_nb) && (handle >= hdl_cursor); (*p_svc_idx)++)
    {
        // check if handle is on current service
        if (handle >= (hdl_cursor + p_hogpd_env->svcs[*p_svc_idx].nb_att))
        {
            hdl_cursor += p_hogpd_env->svcs[*p_svc_idx].nb_att;
            continue;
        }

        // if we are here, we are sure that handle is valid
        status = GAP_ERR_NO_ERROR;
        *p_report_idx = 0;

        // check if handle is in reports or not
        if (handle < (hdl_cursor + p_hogpd_env->svcs[*p_svc_idx].report_hdl_offset))
        {
            if (handle == hdl_cursor)
            {
                *p_att_idx = HOGPD_IDX_SVC;
                break;
            }

            // check if Ext Ref feature active
            if (GETB(p_hogpd_env->svcs[*p_svc_idx].features, HOGPD_CFG_MAP_EXT_REF))
            {
                hdl_cursor += 1;
                if (handle == hdl_cursor)
                {
                    *p_att_idx = HOGPD_IDX_INCL_SVC;
                    break;
                }
            }

            // check if handle is in mandatory range
            hdl_cursor += HIDS_MANDATORY_ATT_NB;
            if (handle <= hdl_cursor)
            {
                *p_att_idx = HOGPD_IDX_REPORT_MAP_VAL - (hdl_cursor - handle - 1);
                break;
            }

            // check if Ext Ref feature active
            if (GETB(p_hogpd_env->svcs[*p_svc_idx].features, HOGPD_CFG_MAP_EXT_REF))
            {
                hdl_cursor += 1;
                if (handle == hdl_cursor)
                {
                    *p_att_idx = HOGPD_IDX_REPORT_MAP_EXT_REP_REF;
                    break;
                }
            }

            // check if Protocol Mode feature active
            if (GETB(p_hogpd_env->svcs[*p_svc_idx].features, HOGPD_CFG_PROTO_MODE))
            {
                hdl_cursor += HIDS_PROTO_MODE_ATT_NB;
                if (handle <= hdl_cursor)
                {
                    *p_att_idx = HOGPD_IDX_PROTO_MODE_VAL - (hdl_cursor - handle - 1);
                    break;
                }
            }

            // check if Keyboard feature active
            if (GETB(p_hogpd_env->svcs[*p_svc_idx].features, HOGPD_CFG_KEYBOARD))
            {
                hdl_cursor += HIDS_KEYBOARD_ATT_NB;
                if (handle <= hdl_cursor)
                {
                    *p_att_idx = HOGPD_IDX_BOOT_KB_OUT_REPORT_VAL - (hdl_cursor - handle - 1);
                    break;
                }
            }

            // check if Mouse feature active
            if (GETB(p_hogpd_env->svcs[*p_svc_idx].features, HOGPD_CFG_MOUSE))
            {
                hdl_cursor += HIDS_MOUSE_ATT_NB;
                if (handle <= hdl_cursor)
                {
                    *p_att_idx = HOGPD_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG - (hdl_cursor - handle - 1);
                    break;
                }
            }

            // not expected
            BLE_ASSERT_ERR(0);
        }
        else
        {
            // add handle offset
            hdl_cursor += p_hogpd_env->svcs[*p_svc_idx].report_hdl_offset;

            for (*p_report_idx = 0; (*p_report_idx < p_hogpd_env->svcs[*p_svc_idx].nb_report); (*p_report_idx)++)
            {
                hdl_cursor += HIDS_REPORT_MANDATORY_ATT_NB;
                if (handle <= hdl_cursor)
                {
                    *p_att_idx = HOGPD_IDX_REPORT_REP_REF - (hdl_cursor - handle - 1);
                    break;
                }

                if ((p_hogpd_env->svcs[*p_svc_idx].features & (HOGPD_CFG_REPORT_NTF_EN << *p_report_idx)) != 0)
                {
                    hdl_cursor += HIDS_REPORT_IN_ATT_NB;
                    if (handle == hdl_cursor)
                    {
                        *p_att_idx = HOGPD_IDX_REPORT_NTF_CFG;
                        break;
                    }
                }
            }

            // not expected
            BLE_ASSERT_ERR(*p_att_idx != HOGPD_IDX_NB);
        }
        // loop not expected here
        break;
    }

    return (status);
}



/**
 ****************************************************************************************
 * @brief  This function manages report notification
 ****************************************************************************************
 */
__STATIC void hogpd_exe_operation(hogpd_env_t* p_hogpd_env)
{
    if(!p_hogpd_env->in_exe_op)
    {
        p_hogpd_env->in_exe_op = true;

        while(!common_list_is_empty(&(p_hogpd_env->wait_queue)) && !(p_hogpd_env->op_ongoing))
        {
            uint16_t status = GAP_ERR_NO_ERROR;
            common_buf_t* p_buf = (common_buf_t*) common_list_pop_front(&(p_hogpd_env->wait_queue));
            hogpd_buf_meta_t* p_meta = (hogpd_buf_meta_t*) common_buf_metadata(p_buf);
            uint8_t conidx = p_meta->conidx;


            status = gatt_srv_event_send(conidx, p_hogpd_env->user_lid, 0, GATT_NOTIFY, p_meta->hdl, p_buf);

            if(status == GAP_ERR_NO_ERROR)
            {
                p_hogpd_env->op_ongoing = true;
            }

            common_buf_release(p_buf);

            if(!p_hogpd_env->op_ongoing)
            {
                const hogpd_cb_t* p_cb = (const hogpd_cb_t*) p_hogpd_env->prf_env.p_cb;
                // Inform application that event has been sent
                p_cb->cb_report_upd_cmp(conidx, status);
            }
        }

        p_hogpd_env->in_exe_op = false;
    }
}

/*
 * GATT USER SERVICE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function is called when peer want to read local attribute database value.
 *
 *        @see gatt_srv_att_read_get_cfm shall be called to provide attribute value
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] max_length    Maximum data length to return
 ****************************************************************************************
 */
__STATIC void hogpd_cb_att_read_get(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
                                    uint16_t max_length)
{
    hogpd_env_t *p_hogpd_env  = PRF_ENV_GET(HOGPD, hogpd);
    common_buf_t*    p_buf        = NULL;
    uint16_t     att_val_len  = 0;
    bool         confirm_send = true;
    uint16_t     status       = PRF_APP_ERROR;

   do
    {
        uint8_t hid_idx, att_idx, report_idx;

        if(p_hogpd_env == NULL) break;

        // Retrieve attribute index
        status = hogpd_get_att_idx(p_hogpd_env, hdl, &hid_idx, &att_idx, &report_idx);
        if (status != GAP_ERR_NO_ERROR) break;


        // check which attribute is requested by peer device
        switch (att_idx)
        {
            //  ------------ READ report value requested
            case HOGPD_IDX_BOOT_KB_IN_REPORT_VAL:
            case HOGPD_IDX_BOOT_KB_OUT_REPORT_VAL:
            case HOGPD_IDX_BOOT_MOUSE_IN_REPORT_VAL:
            case HOGPD_IDX_REPORT_VAL:
            case HOGPD_IDX_REPORT_MAP_VAL:
            {
                const hogpd_cb_t* p_cb  = (const hogpd_cb_t*) p_hogpd_env->prf_env.p_cb;
                uint8_t report_type = 0;
                // retrieve report type
                switch (att_idx)
                {
                    // An Input Report
                    case HOGPD_IDX_BOOT_KB_IN_REPORT_VAL:    { report_type = HOGPD_BOOT_KEYBOARD_INPUT_REPORT;  } break;
                    // Boot Keyboard input report
                    case HOGPD_IDX_BOOT_KB_OUT_REPORT_VAL:   { report_type = HOGPD_BOOT_KEYBOARD_OUTPUT_REPORT; } break;
                    // Boot Mouse input report
                    case HOGPD_IDX_BOOT_MOUSE_IN_REPORT_VAL: { report_type = HOGPD_BOOT_MOUSE_INPUT_REPORT;     } break;
                    // Normal report
                    case HOGPD_IDX_REPORT_VAL:               { report_type = HOGPD_REPORT;                      } break;
                    // Report MAP
                    case HOGPD_IDX_REPORT_MAP_VAL:           { report_type = HOGPD_REPORT_MAP;                  } break;
                    default:                                 { BLE_ASSERT_ERR(0);                                   } break;
                }

                p_cb->cb_report_read_req(conidx, token, hid_idx, report_idx, report_type, offset, max_length);
                confirm_send = false;
            } break;
            default:
            {
                if(offset != 0) // does not support read with offset
                {
                    status = PRF_APP_ERROR;
                }
                else if(common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_HEADER_LEN) != COMMON_BUF_ERR_NO_ERROR)
                {
                    status = ATT_ERR_INSUFF_RESOURCE;
                }
            } break;
        }

        // check if there is nothing more to do
        if((status != GAP_ERR_NO_ERROR) || (!confirm_send)) break;

        switch (att_idx)
        {
            //  ------------ READ active protocol mode
            case HOGPD_IDX_PROTO_MODE_VAL:
            {
                common_buf_tail(p_buf)[0] = p_hogpd_env->svcs[hid_idx].proto_mode;
                common_buf_tail_reserve(p_buf, 1);
            } break;

            //  ------------ READ Notification configuration
            case HOGPD_IDX_BOOT_KB_IN_REPORT_NTF_CFG:
            {
                uint16_t ntf_cfg = (GETB(p_hogpd_env->ntf_cfg[hid_idx][conidx], HOGPD_CFG_KEYBOARD))
                                 ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;
                common_write16p(common_buf_tail(p_buf), common_htobs(ntf_cfg));
                common_buf_tail_reserve(p_buf, 2);
            } break;

            case HOGPD_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG:
            {
                uint16_t ntf_cfg = (GETB(p_hogpd_env->ntf_cfg[hid_idx][conidx], HOGPD_CFG_MOUSE))
                                 ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;
                common_write16p(common_buf_tail(p_buf), common_htobs(ntf_cfg));
                common_buf_tail_reserve(p_buf, 2);
            } break;

            case HOGPD_IDX_REPORT_NTF_CFG:
            {
                uint16_t ntf_cfg = ((p_hogpd_env->ntf_cfg[hid_idx][conidx] & (HOGPD_CFG_REPORT_NTF_EN << report_idx)) != 0)
                                 ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;
                common_write16p(common_buf_tail(p_buf), common_htobs(ntf_cfg));
                common_buf_tail_reserve(p_buf, 2);
            } break;


            //  ------------ READ static information
            case HOGPD_IDX_HID_INFO_VAL:
            {
                hogp_hid_info_t* p_hid_info = &(p_hogpd_env->svcs[hid_idx].hid_info);

                common_write16p(common_buf_tail(p_buf), common_htobs(p_hid_info->bcdHID));
                common_buf_tail_reserve(p_buf, 2);
                common_buf_tail(p_buf)[0] = p_hid_info->bCountryCode;
                common_buf_tail_reserve(p_buf, 1);
                common_buf_tail(p_buf)[0] = p_hid_info->flags;
                common_buf_tail_reserve(p_buf, 1);
            } break;

            case HOGPD_IDX_REPORT_MAP_EXT_REP_REF:
            {
                common_write16p(common_buf_tail(p_buf), common_htobs(p_hogpd_env->svcs[hid_idx].rep_ref_uuid));
                common_buf_tail_reserve(p_buf, 2);
            } break;

            case HOGPD_IDX_REPORT_REP_REF:
            {
                // Pack report ID
                common_buf_tail(p_buf)[0] = p_hogpd_env->svcs[hid_idx].report_id[report_idx];
                common_buf_tail_reserve(p_buf, 1);
                // Pack report Type
                common_buf_tail(p_buf)[0] = (p_hogpd_env->svcs[hid_idx].report_char_cfg[report_idx] & HOGPD_CFG_REPORT_FEAT);
                common_buf_tail_reserve(p_buf, 1);
            } break;

            default: { /* Nothing to do */ } break;
        }
    } while(0);

    // Immediately send confirmation message
    if(confirm_send)
    {
        gatt_srv_att_read_get_cfm(conidx, user_lid, token, status, att_val_len, p_buf);
    }
    // release allocated buffer
    if(p_buf != NULL)
    {
        common_buf_release(p_buf);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called during a write procedure to modify attribute handle.
 *
 *        @see gatt_srv_att_val_set_cfm shall be called to accept or reject attribute
 *        update.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Value offset
 * @param[in] p_data        Pointer to buffer that contains data to write starting from offset
 ****************************************************************************************
 */
__STATIC void hogpd_cb_att_val_set(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
                                   common_buf_t* p_data)
{
    hogpd_env_t *p_hogpd_env = PRF_ENV_GET(HOGPD, hogpd);
    bool confirm_send = true;
    uint16_t status = PRF_APP_ERROR;

    do
    {
        const hogpd_cb_t* p_cb  = (const hogpd_cb_t*) p_hogpd_env->prf_env.p_cb;
        uint8_t hid_idx, att_idx, report_idx;

        if((p_hogpd_env == NULL) || (offset != 0)) break; // reject write if offset != 0

        status = hogpd_get_att_idx(p_hogpd_env, hdl, &hid_idx, &att_idx, &report_idx);
        if (status != GAP_ERR_NO_ERROR) break;

        // check which attribute is requested by peer device
        switch (att_idx)
        {
            // Control point value updated
            case HOGPD_IDX_HID_CTNL_PT_VAL:
            {
                uint8_t ctnl_pt_val = common_buf_data(p_data)[0];
                p_cb->cb_ctnl_pt(conidx, hid_idx, ctnl_pt_val);
                status = GAP_ERR_NO_ERROR;
            } break;

            // Modification of protocol mode requested
            case HOGPD_IDX_PROTO_MODE_VAL:
            {
                uint8_t proto_mode = common_buf_data(p_data)[0];
                p_cb->cb_proto_upd_req(conidx, token, hid_idx, proto_mode);
                confirm_send = false;
            } break;

            // Modification of report value requested
            case HOGPD_IDX_BOOT_KB_IN_REPORT_VAL:
            case HOGPD_IDX_BOOT_KB_OUT_REPORT_VAL:
            case HOGPD_IDX_BOOT_MOUSE_IN_REPORT_VAL:
            case HOGPD_IDX_REPORT_VAL:
            {
                uint8_t report_type = 0;
                // retrieve report type
                switch (att_idx)
                {
                    // An Input Report
                    case HOGPD_IDX_BOOT_KB_IN_REPORT_VAL:    { report_type = HOGPD_BOOT_KEYBOARD_INPUT_REPORT;  } break;
                    // Boot Keyboard input report
                    case HOGPD_IDX_BOOT_KB_OUT_REPORT_VAL:   { report_type = HOGPD_BOOT_KEYBOARD_OUTPUT_REPORT; } break;
                    // Boot Mouse input report
                    case HOGPD_IDX_BOOT_MOUSE_IN_REPORT_VAL: { report_type = HOGPD_BOOT_MOUSE_INPUT_REPORT;     } break;
                    // Normal report
                    case HOGPD_IDX_REPORT_VAL:               { report_type = HOGPD_REPORT;                      } break;
                    default:                                 { BLE_ASSERT_ERR(0);                                   } break;
                }

                p_cb->cb_report_write_req(conidx, token, hid_idx, report_idx, report_type, p_data);
                confirm_send = false;
            } break;

            // Notification configuration update
            case HOGPD_IDX_BOOT_KB_IN_REPORT_NTF_CFG:
            case HOGPD_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG:
            case HOGPD_IDX_REPORT_NTF_CFG:
            {
                uint16_t mask = 0;
                uint16_t ntf_cfg = common_btohs(common_read16p(common_buf_data(p_data)));

                // set notification config update mask
                switch (att_idx)
                {
                    // An Input Report
                    case HOGPD_IDX_REPORT_NTF_CFG:               { mask = (HOGPD_CFG_REPORT_NTF_EN << report_idx); } break;
                    // Boot Keyboard input report
                    case HOGPD_IDX_BOOT_KB_IN_REPORT_NTF_CFG:    { mask = HOGPD_CFG_KEYBOARD_BIT;                  } break;
                    // Boot Mouse input report
                    case HOGPD_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG: { mask = HOGPD_CFG_MOUSE_BIT;                     } break;
                    default:                                     { BLE_ASSERT_ERR(0);                                  } break;
                }

                status = GAP_ERR_NO_ERROR;

                // Stop notification
                if (ntf_cfg == PRF_CLI_STOP_NTFIND)
                {
                    p_hogpd_env->ntf_cfg[hid_idx][conidx] &= ~mask;
                }
                // Start notification
                else if(ntf_cfg == PRF_CLI_START_NTF)
                {
                    p_hogpd_env->ntf_cfg[hid_idx][conidx] |= mask;
                }
                // Provided value is incorrect
                else
                {
                    status = PRF_APP_ERROR;
                }

                // inform application about updated notification configuration
                if (status == GAP_ERR_NO_ERROR)
                {
                    uint8_t svc_idx;
                    uint16_t ntf_cfg[HOGPD_NB_HIDS_INST_MAX];
                    for (svc_idx = 0 ; svc_idx < HOGPD_NB_HIDS_INST_MAX; svc_idx++)
                    {
                        ntf_cfg[svc_idx] = p_hogpd_env->ntf_cfg[svc_idx][conidx];
                    }

                    p_cb->cb_bond_data_upd(conidx, ntf_cfg);
                }
            } break;

            default: { status = PRF_APP_ERROR; } break;
        }


    } while(0);


    if(confirm_send)
    {
        gatt_srv_att_val_set_cfm(conidx, user_lid, token, status);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT server user has initiated event send to peer
 *        device or if an error occurs.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (@see enum hl_err)
 ****************************************************************************************
 */
__STATIC void hogpd_cb_event_sent(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    // Consider notification done
    hogpd_env_t* p_hogpd_env = PRF_ENV_GET(HOGPD, hogpd);
    if(p_hogpd_env != NULL)
    {
        const hogpd_cb_t* p_cb = (const hogpd_cb_t*) p_hogpd_env->prf_env.p_cb;
        p_hogpd_env->op_ongoing = false;
        // Inform application that event has been sent
        p_cb->cb_report_upd_cmp(conidx, status);

        // continue operation execution
        hogpd_exe_operation(p_hogpd_env);
    }
}

/// Service callback hander
__STATIC const gatt_srv_cb_t hogpd_cb =
{
        .cb_event_sent    = hogpd_cb_event_sent,
        .cb_att_read_get  = hogpd_cb_att_read_get,
        .cb_att_event_get = NULL,
        .cb_att_info_get  = NULL, // Does not support a write starting from != 0 offset
        .cb_att_val_set   = hogpd_cb_att_val_set,
};

/*
 * PROFILE NATIVE HANDLERS
 ****************************************************************************************
 */

uint16_t hogpd_enable(uint8_t conidx, const uint16_t* p_ntf_cfg)
{
    hogpd_env_t* p_hogpd_env = PRF_ENV_GET(HOGPD, hogpd);
    uint16_t status = PRF_ERR_REQ_DISALLOWED;

    if(p_hogpd_env != NULL)
    {
        // check state of the task
        if (gapc_get_conhdl(conidx) != GAP_INVALID_CONHDL)
        {
            uint8_t svc_idx;
            for (svc_idx = 0; svc_idx < p_hogpd_env->hids_nb; svc_idx++)
            {
                // Retrieve notification configuration
                p_hogpd_env->ntf_cfg[svc_idx][conidx]   = p_ntf_cfg[svc_idx];
            }
             status = GAP_ERR_NO_ERROR;
        }
    }

    return (status);
}

uint16_t hogpd_proto_upd_cfm(uint8_t conidx, uint16_t token, uint16_t status, uint8_t hid_idx, uint8_t proto_mode)
{
    uint16_t ret_status;

    hogpd_env_t* p_hogpd_env = PRF_ENV_GET(HOGPD, hogpd);

    if(p_hogpd_env != NULL)
    {
        // give back result of report write done at application level
        ret_status = gatt_srv_att_val_set_cfm(conidx, p_hogpd_env->user_lid, token, status);

        // check if procedure exists and protocole has been updated by application
        if((status == GAP_ERR_NO_ERROR) && (ret_status == GAP_ERR_NO_ERROR))
        {
            p_hogpd_env->svcs[hid_idx].proto_mode = proto_mode;
        }
    }
    else
    {
        ret_status = PRF_ERR_REQ_DISALLOWED;
    }

    return (ret_status);
}

uint16_t hogpd_report_read_cfm(uint8_t conidx, uint16_t token, uint16_t status, uint16_t tot_length, common_buf_t* p_buf)
{
    hogpd_env_t* p_hogpd_env = PRF_ENV_GET(HOGPD, hogpd);

    if(p_hogpd_env != NULL)
    {
        // Return buffer that contains report data requested by peer device
        status = gatt_srv_att_read_get_cfm(conidx, p_hogpd_env->user_lid, token, status, tot_length, p_buf);
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    return (status);
}

uint16_t hogpd_report_write_cfm(uint8_t conidx, uint16_t token, uint16_t status)
{
    hogpd_env_t* p_hogpd_env = PRF_ENV_GET(HOGPD, hogpd);

    if(p_hogpd_env != NULL)
    {
        // give back result of report write done at application level
        status = gatt_srv_att_val_set_cfm(conidx, p_hogpd_env->user_lid, token, status);
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    return (status);
}

uint16_t hogpd_report_upd(uint8_t conidx, uint8_t hid_idx, uint8_t report_idx, uint8_t report_type, common_buf_t* p_buf)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;

    hogpd_env_t* p_hogpd_env = PRF_ENV_GET(HOGPD, hogpd);

    if(p_buf == NULL)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if((p_hogpd_env != NULL) && (gapc_get_conhdl(conidx) != GAP_INVALID_CONHDL))
    {
        uint16_t hdl;
        uint8_t  att_idx = HOGPD_IDX_NB;
        uint16_t max_report_len= 0;
        uint16_t feature_mask = 0;
        uint8_t  exp_prot_mode = 0;

        // According to the report type retrieve:
        // - Attribute index
        // - Attribute max length
        // - Feature to use
        // - Expected protocol mode
        switch(report_type)
        {
            // An Input Report
            case HOGPD_REPORT:
            {
                att_idx = HOGPD_IDX_REPORT_VAL;
                max_report_len = HOGP_REPORT_MAX_LEN;
                feature_mask = HOGPD_CFG_REPORT_NTF_EN << report_idx;
                exp_prot_mode = HOGP_REPORT_PROTOCOL_MODE;
            } break;

            // Boot Keyboard input report
            case HOGPD_BOOT_KEYBOARD_INPUT_REPORT:
            {
                att_idx = HOGPD_IDX_BOOT_KB_IN_REPORT_VAL;
                max_report_len = HOGP_BOOT_REPORT_MAX_LEN;
                feature_mask = HOGPD_CFG_KEYBOARD_BIT;
                exp_prot_mode = HOGP_BOOT_PROTOCOL_MODE;
            } break;

            // Boot Mouse input report
            case HOGPD_BOOT_MOUSE_INPUT_REPORT:
            {
                att_idx = HOGPD_IDX_BOOT_MOUSE_IN_REPORT_VAL;
                max_report_len = HOGP_BOOT_REPORT_MAX_LEN;
                feature_mask = HOGPD_CFG_MOUSE_BIT;
                exp_prot_mode = HOGP_BOOT_PROTOCOL_MODE;
            } break;

            default: { /* Nothing to do */ } break;
        }

        // retrieve attribute handle
        hdl = hogpd_get_att_handle(p_hogpd_env, hid_idx, att_idx, report_idx);

        // check if attribute is found
        if(hdl == GATT_INVALID_HDL)
        {
            // check if it's an unsupported feature
            if (  (feature_mask != 0) && (hid_idx < p_hogpd_env->hids_nb)
                && (report_idx < p_hogpd_env->svcs[hid_idx].nb_report)
                && ((p_hogpd_env->svcs[hid_idx].features & feature_mask) == 0))
            {
                status = PRF_ERR_FEATURE_NOT_SUPPORTED;
            }
            // or an invalid param
            else
            {
                status = PRF_ERR_INVALID_PARAM;
            }
        }
        // check if length is valid
        else if (common_buf_data_len(p_buf) > max_report_len)
        {
            status = PRF_ERR_UNEXPECTED_LEN;
        }
        // check if notification is enabled
        else if ((p_hogpd_env->ntf_cfg[hid_idx][conidx] & feature_mask) == 0)
        {
            status = PRF_ERR_NTF_DISABLED;
        }
        // check if protocol mode is valid
        else if (  (p_hogpd_env->svcs[hid_idx].proto_mode != exp_prot_mode)
                && (GETB(p_hogpd_env->svcs[hid_idx].features, HOGPD_CFG_PROTO_MODE)))
        {
            status = PRF_ERR_REQ_DISALLOWED;
        }
        else
        {
            // store buffer context
            hogpd_buf_meta_t* p_buf_meta = (hogpd_buf_meta_t*) common_buf_metadata(p_buf);
            p_buf_meta->conidx = conidx;
            p_buf_meta->hdl    = hdl;

            // acquire buffer
            common_buf_acquire(p_buf);
            status = GAP_ERR_NO_ERROR;

            // put event on wait queue
            common_list_push_back(&(p_hogpd_env->wait_queue), &(p_buf->hdr));
            // execute operation
            hogpd_exe_operation(p_hogpd_env);
        }
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
 * @brief Handles reception of the @ref HOGPD_ENABLE_REQ message.
 * The handler enables the HID Over GATT Profile Device Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogpd_enable_req_handler(kernel_msg_id_t const msgid, struct hogpd_enable_req const *p_param,
                                      kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Counter for HIDS instance
    struct hogpd_enable_rsp *p_rsp;
    uint16_t status = hogpd_enable(p_param->conidx, p_param->ntf_cfg);

    // Send back response
    p_rsp = KERNEL_MSG_ALLOC(HOGPD_ENABLE_RSP, src_id, dest_id, hogpd_enable_rsp);
    if(p_rsp != NULL)
    {
        p_rsp->conidx = p_param->conidx;
        p_rsp->status = status;
        kernel_msg_send(p_rsp);
    }

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPD_REPORT_UPD_REQ message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogpd_report_upd_req_handler(kernel_msg_id_t const msgid, struct hogpd_report_upd_req const *p_param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint16_t status;
    common_buf_t* p_buf = NULL;

    // allocate report buffer
    if(common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, p_param->report.length, GATT_BUFFER_TAIL_LEN) == COMMON_BUF_ERR_NO_ERROR)
    {
        common_buf_copy_data_from_mem(p_buf, p_param->report.value, p_param->report.length);

        // send report
        status = hogpd_report_upd(p_param->conidx, p_param->hid_idx, p_param->report_idx, p_param->report_type,
                                  p_buf);

        common_buf_release(p_buf);
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        // send report update response
        struct hogpd_report_upd_rsp *p_rsp = KERNEL_MSG_ALLOC(HOGPD_REPORT_UPD_RSP, src_id, dest_id, hogpd_report_upd_rsp);
        if(p_rsp != NULL)
        {
            p_rsp->conidx = p_param->conidx;
            p_rsp->status = status;
            kernel_msg_send(p_rsp);
        }
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPD_REPORT_READ_CFM message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogpd_report_read_cfm_handler(kernel_msg_id_t const msgid, struct hogpd_report_read_cfm const *p_param,
                                      kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    common_buf_t* p_buf = NULL;
    uint16_t status = p_param->status;

    // allocate report buffer
    if(   (status == GAP_ERR_NO_ERROR)
       && common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, p_param->report.length, GATT_BUFFER_TAIL_LEN) == COMMON_BUF_ERR_NO_ERROR)
    {
        common_buf_copy_data_from_mem(p_buf, p_param->report.value, p_param->report.length);
    }
    else
    {
        status = ATT_ERR_INSUFF_RESOURCE;
    }

    hogpd_report_read_cfm(p_param->conidx, p_param->token, p_param->status, p_param->tot_length, p_buf);

    if(p_buf != NULL)
    {
        common_buf_release(p_buf);
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPD_REPORT_WRITE_CFM message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogpd_report_write_cfm_handler(kernel_msg_id_t const msgid, struct hogpd_report_write_cfm const *p_param,
                                            kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    hogpd_report_write_cfm(p_param->conidx, p_param->token, p_param->status);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPD_PROTO_MODE_CFM message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int hogpd_proto_mode_cfm_handler(kernel_msg_id_t const msgid, struct hogpd_proto_mode_cfm const *p_param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    hogpd_proto_upd_cfm(p_param->conidx, p_param->token, p_param->status, p_param->hid_idx, p_param->proto_mode);

    return (KERNEL_MSG_CONSUMED);
}

/// Default State handlers definition
KERNEL_MSG_HANDLER_TAB(hogpd)
{
    // Note: all messages must be sorted in ID ascending order

    { HOGPD_ENABLE_REQ,              (kernel_msg_func_t) hogpd_enable_req_handler       },
    { HOGPD_REPORT_UPD_REQ,          (kernel_msg_func_t) hogpd_report_upd_req_handler   },
    { HOGPD_REPORT_READ_CFM,         (kernel_msg_func_t) hogpd_report_read_cfm_handler  },
    { HOGPD_REPORT_WRITE_CFM,        (kernel_msg_func_t) hogpd_report_write_cfm_handler },
    { HOGPD_PROTO_MODE_CFM,          (kernel_msg_func_t) hogpd_proto_mode_cfm_handler   },
};


/**
 ****************************************************************************************
 * @brief Inform that Bond data updated for the connection.
 *
 * @param[in] conidx        Connection index
 * @param[in] p_ntf_cfg     Pointer to array of Notification Configurations
 *                          Array of HOGPD_NB_HIDS_INST_MAX size.
 ****************************************************************************************
 */
__STATIC void hogpd_cb_bond_data_upd(uint8_t conidx, const uint16_t* p_ntf_cfg)
{
    struct hogpd_ntf_cfg_ind* p_ind =  KERNEL_MSG_ALLOC(HOGPD_NTF_CFG_IND, PRF_DST_TASK(HOGPD),
                                                    PRF_SRC_TASK(HOGPD), hogpd_ntf_cfg_ind);

    if(p_ind != NULL)
    {
        p_ind->conidx = conidx;
        memcpy(p_ind->ntf_cfg, p_ntf_cfg, sizeof(p_ind->ntf_cfg));
        kernel_msg_send(p_ind);
    }
}

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
__STATIC void hogpd_cb_proto_upd_req(uint8_t conidx, uint16_t token, uint8_t hid_idx, uint8_t proto_mode)
{
    struct hogpd_proto_mode_req_ind* p_req_ind =  KERNEL_MSG_ALLOC(HOGPD_PROTO_MODE_REQ_IND,
                                                               PRF_DST_TASK(HOGPD), PRF_SRC_TASK(HOGPD),
                                                               hogpd_proto_mode_req_ind);

    if(p_req_ind != NULL)
    {
        p_req_ind->conidx      = conidx;
        p_req_ind->token       = token;
        p_req_ind->hid_idx     = hid_idx;
        p_req_ind->proto_mode  = proto_mode;
        kernel_msg_send(p_req_ind);
    }
}

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
__STATIC void hogpd_cb_report_read_req(uint8_t conidx, uint16_t token, uint8_t hid_idx, uint8_t report_idx,
                                       uint8_t report_type, uint16_t offset, uint16_t max_length)
{
    struct hogpd_report_read_req_ind* p_req_ind =  KERNEL_MSG_ALLOC(HOGPD_REPORT_READ_REQ_IND,
                                                                PRF_DST_TASK(HOGPD), PRF_SRC_TASK(HOGPD),
                                                                hogpd_report_read_req_ind);

    if(p_req_ind != NULL)
    {
        p_req_ind->conidx      = conidx;
        p_req_ind->token       = token;
        p_req_ind->hid_idx     = hid_idx;
        p_req_ind->report_idx  = report_idx;
        p_req_ind->report_type = report_type;
        p_req_ind->offset      = offset;
        p_req_ind->max_length  = max_length;
        kernel_msg_send(p_req_ind);
    }
}

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
__STATIC void hogpd_cb_report_write_req(uint8_t conidx, uint16_t token, uint8_t hid_idx, uint8_t report_idx,
                                        uint8_t report_type, common_buf_t* p_buf)
{
    uint16_t data_len = ((p_buf == NULL) ? 0 : common_buf_data_len(p_buf));
    struct hogpd_report_write_req_ind* p_req_ind =  KERNEL_MSG_ALLOC_DYN(HOGPD_REPORT_WRITE_REQ_IND,
                                                                     PRF_DST_TASK(HOGPD), PRF_SRC_TASK(HOGPD),
                                                                     hogpd_report_write_req_ind, data_len);

    if(p_req_ind != NULL)
    {
        p_req_ind->conidx        = conidx;
        p_req_ind->token         = token;
        p_req_ind->hid_idx       = hid_idx;
        p_req_ind->report_idx    = report_idx;
        p_req_ind->report_type   = report_type;
        p_req_ind->report.length = data_len;

        if(data_len > 0)
        {
            common_buf_copy_data_to_mem(p_buf, p_req_ind->report.value, data_len);
        }
        kernel_msg_send(p_req_ind);
    }
}

/**
 ****************************************************************************************
 * @brief Inform that a control point update has been received
 *
 * @param[in] conidx        Connection index
 * @param[in] hid_idx       HIDS Instance
 * @param[in] value   New HID Control Point Characteristic Value
 ****************************************************************************************
 */
__STATIC void hogpd_cb_ctnl_pt(uint8_t conidx, uint8_t hid_idx, uint8_t value)
{
    struct hogpd_ctnl_pt_ind* p_ind =  KERNEL_MSG_ALLOC(HOGPD_CTNL_PT_IND, PRF_DST_TASK(HOGPD),
                                                    PRF_SRC_TASK(HOGPD), hogpd_ctnl_pt_ind);

    if(p_ind != NULL)
    {
        p_ind->conidx      = conidx;
        p_ind->hid_idx     = hid_idx;
        p_ind->hid_ctnl_pt = value;

        kernel_msg_send(p_ind);
    }
}

/**
 ****************************************************************************************
 * @brief Inform that report update procedure is done
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the procedure execution (@see enum hl_err)
 ****************************************************************************************
 */
__STATIC void hogpd_cb_report_upd_cmp(uint8_t conidx, uint16_t status)
{
    struct hogpd_report_upd_rsp* p_rsp =  KERNEL_MSG_ALLOC(HOGPD_REPORT_UPD_RSP, PRF_DST_TASK(HOGPD),
                                                       PRF_SRC_TASK(HOGPD), hogpd_report_upd_rsp);

    if(p_rsp != NULL)
    {
        p_rsp->conidx      = conidx;
        p_rsp->status     = status;

        kernel_msg_send(p_rsp);
    }
}

/// Default Message handle
__STATIC const hogpd_cb_t hogpd_msg_cb =
{
    .cb_bond_data_upd    = hogpd_cb_bond_data_upd,
    .cb_proto_upd_req    = hogpd_cb_proto_upd_req,
    .cb_report_read_req  = hogpd_cb_report_read_req,
    .cb_report_write_req = hogpd_cb_report_write_req,
    .cb_ctnl_pt          = hogpd_cb_ctnl_pt,
    .cb_report_upd_cmp   = hogpd_cb_report_upd_cmp,
};
#endif // (BLE_HL_MSG_API)

/*
 * PROFILE DEFAULT HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the HOGPD module.
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
__STATIC uint16_t hogpd_init(prf_data_t *p_env, uint16_t *p_start_hdl, uint8_t sec_lvl, uint8_t user_prio,
                          struct hogpd_db_cfg *p_params, const hogpd_cb_t* p_cb)
{
    //------------------ create the attribute database for the profile -------------------

    // DB Creation Status
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t user_lid = GATT_INVALID_USER_LID;
    hogpd_env_t* p_hogpd_env = NULL;
    // array of service description used to allocate the service
    gatt_att16_desc_t* p_hids_db[HOGPD_NB_HIDS_INST_MAX];
    memset(p_hids_db, 0, sizeof(p_hids_db));
    // Service Instance Counter
    uint8_t svc_idx;

    do
    {
        // Total number of attributes
        uint8_t tot_nb_att = 0;
        // Service content flag - Without Report Characteristics
        uint32_t cfg_flag[HOGPD_NB_HIDS_INST_MAX][(HOGPD_ATT_MAX/32) +1];
        // Report Index Counter
        uint8_t report_idx;
        uint16_t shdl;

        // ensure that everything is initialized
        memset(cfg_flag, 0, sizeof(cfg_flag));

        #if (BLE_HL_MSG_API)
        if(p_cb == NULL)
        {
            p_cb = &(hogpd_msg_cb);
        }
        #endif // (BLE_HL_MSG_API)

        if(   (p_params == NULL) || (p_start_hdl == NULL) || (p_cb == NULL) || (p_cb->cb_bond_data_upd == NULL)
           || (p_cb->cb_proto_upd_req == NULL) || (p_cb->cb_report_read_req == NULL) || (p_cb->cb_report_write_req == NULL)
           || (p_cb->cb_ctnl_pt == NULL) || (p_cb->cb_report_upd_cmp == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }
        // Check number of HID instances
        if ((p_params->hids_nb == 0) || (p_params->hids_nb > HOGPD_NB_HIDS_INST_MAX))
        {
            // Invalid number of service instances
            status = PRF_ERR_INVALID_PARAM;
        }

        // register HOGPD user
        status = gatt_user_srv_register(HOGP_REPORT_MAP_MAX_LEN, user_prio, &hogpd_cb, &user_lid);
        if(status != GAP_ERR_NO_ERROR) break;


        //-------------------- allocate memory required for the profile  ---------------------
        p_hogpd_env = (hogpd_env_t *) kernel_malloc(sizeof(hogpd_env_t), KERNEL_MEM_ATT_DB);

        if(p_hogpd_env == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        // For each required HIDS instance
        for (svc_idx = 0; ((svc_idx < p_params->hids_nb) && (status == GAP_ERR_NO_ERROR)); svc_idx++)
        {
            // Check number of Report Char. instances
            if (p_params->cfg[svc_idx].report_nb > HOGPD_NB_REPORT_INST_MAX)
            {
                // Too many Report Char. Instances
                status = PRF_ERR_INVALID_PARAM;
                break;
            }

            // Initialize service configuration
            p_hogpd_env->svcs[svc_idx].features       = p_params->cfg[svc_idx].svc_features & HOGPD_CFG_MASK;
            p_hogpd_env->svcs[svc_idx].nb_report      = p_params->cfg[svc_idx].report_nb;
            p_hogpd_env->svcs[svc_idx].hid_info       = p_params->cfg[svc_idx].hid_info;
            p_hogpd_env->svcs[svc_idx].rep_ref_uuid   = p_params->cfg[svc_idx].ext_ref.rep_ref_uuid;
            memcpy(p_hogpd_env->svcs[svc_idx].report_char_cfg, p_params->cfg[svc_idx].report_char_cfg, HOGPD_NB_REPORT_INST_MAX);
            memcpy(p_hogpd_env->svcs[svc_idx].report_id,       p_params->cfg[svc_idx].report_id,       HOGPD_NB_REPORT_INST_MAX);

            p_hids_db[svc_idx] = (gatt_att16_desc_t *) kernel_malloc(HOGPD_ATT_MAX * sizeof(gatt_att16_desc_t), KERNEL_MEM_NON_RETENTION);

            if(p_hids_db[svc_idx] == NULL)
            {
                status = GAP_ERR_INSUFF_RESOURCES;
                break;
            }

            cfg_flag[svc_idx][0] = HIDS_CFG_FLAG_MANDATORY_MASK;
            p_hogpd_env->svcs[svc_idx].nb_att = HIDS_MANDATORY_ATT_NB;

            // copy default definition of the HID attribute database
            memcpy(p_hids_db[svc_idx], hogpd_att_db, sizeof(gatt_att16_desc_t) * HOGPD_ATT_UNIQ_NB);

            //--------------------------------------------------------------------
            // Compute cfg_flag[i] without Report Characteristics
            //--------------------------------------------------------------------
            if (GETB(p_params->cfg[svc_idx].svc_features, HOGPD_CFG_MAP_EXT_REF))
            {
                cfg_flag[svc_idx][0]            |= HIDS_CFG_FLAG_MAP_EXT_MASK;
                p_hogpd_env->svcs[svc_idx].nb_att += HIDS_MAP_EXT_ATT_NB;
                // store reference handle in database
                p_hids_db[svc_idx][HOGPD_IDX_INCL_SVC].ext_info = p_params->cfg[svc_idx].ext_ref.inc_svc_hdl;
            }

            if (GETB(p_params->cfg[svc_idx].svc_features, HOGPD_CFG_PROTO_MODE))
            {
                cfg_flag[svc_idx][0]            |= HIDS_CFG_FLAG_PROTO_MODE_MASK;
                p_hogpd_env->svcs[svc_idx].nb_att += HIDS_PROTO_MODE_ATT_NB;
            }

            if (GETB(p_params->cfg[svc_idx].svc_features, HOGPD_CFG_KEYBOARD))
            {
                cfg_flag[svc_idx][0]            |= HIDS_CFG_FLAG_KEYBOARD_MASK;
                p_hogpd_env->svcs[svc_idx].nb_att += HIDS_KEYBOARD_ATT_NB;

                if (GETB(p_params->cfg[svc_idx].svc_features, HOGPD_CFG_BOOT_KB_WR))
                {
                    // Adds write permissions on report
                    p_hids_db[svc_idx][HOGPD_IDX_BOOT_KB_IN_REPORT_VAL].info |= (PROP(WR));
                }
            }

            if (GETB(p_params->cfg[svc_idx].svc_features, HOGPD_CFG_MOUSE))
            {
                cfg_flag[svc_idx][0]            |= HIDS_CFG_FLAG_MOUSE_MASK;
                p_hogpd_env->svcs[svc_idx].nb_att += HIDS_MOUSE_ATT_NB;

                if (GETB(p_params->cfg[svc_idx].svc_features, HOGPD_CFG_BOOT_MOUSE_WR))
                {
                    // Adds write permissions on report
                    p_hids_db[svc_idx][HOGPD_IDX_BOOT_MOUSE_IN_REPORT_VAL].info |= (PROP(WR));
                }
            }

            // set report handle offset
            p_hogpd_env->svcs[svc_idx].report_hdl_offset = p_hogpd_env->svcs[svc_idx].nb_att;

            //--------------------------------------------------------------------
            // Update cfg_flag_rep[i] with Report Characteristics
            //--------------------------------------------------------------------
            for (report_idx = 0; report_idx < p_params->cfg[svc_idx].report_nb; report_idx++)
            {
                uint16_t perm = 0;
                uint16_t report_offset = HIDS_REPORT_NB_IDX*report_idx;

                // update config for current report
                cfg_flag[svc_idx][(HOGPD_IDX_REPORT_CHAR + report_offset) >> 5]    |=
                        (1 << ((HOGPD_IDX_REPORT_CHAR + report_offset) & 0x1F));

                cfg_flag[svc_idx][(HOGPD_IDX_REPORT_VAL + report_offset) >> 5]     |=
                        (1 << ((HOGPD_IDX_REPORT_VAL + report_offset) & 0x1F));

                cfg_flag[svc_idx][(HOGPD_IDX_REPORT_REP_REF + report_offset) >> 5] |=
                        (1 << ((HOGPD_IDX_REPORT_REP_REF + report_offset) & 0x1F));

                p_hogpd_env->svcs[svc_idx].nb_att += HIDS_REPORT_MANDATORY_ATT_NB;

                // copy default definition of the HID report database
                memcpy(&(p_hids_db[svc_idx][HOGPD_IDX_REPORT_CHAR + report_offset]), &(hogpd_att_db[HOGPD_IDX_REPORT_CHAR]),
                        sizeof(gatt_att16_desc_t) * HIDS_REPORT_NB_IDX);

                // according to the report type, update value property
                switch (p_params->cfg[svc_idx].report_char_cfg[report_idx] & HOGPD_CFG_REPORT_FEAT)
                {
                    // Input Report
                    case HOGPD_CFG_REPORT_IN:
                    {
                        // add notification permission on report
                        perm = PROP(RD) | PROP(N);
                        // Report Char. supports NTF => Client Characteristic Configuration Descriptor
                        cfg_flag[svc_idx][(HOGPD_IDX_REPORT_NTF_CFG + report_offset) >> 5] |=
                                (1 << ((HOGPD_IDX_REPORT_NTF_CFG + report_offset) & 0x1F));

                        p_hogpd_env->svcs[svc_idx].nb_att += HIDS_REPORT_IN_ATT_NB;

                        // update feature flag
                        p_hogpd_env->svcs[svc_idx].features |= (HOGPD_CFG_REPORT_NTF_EN << report_idx);

                        // check if attribute value could be written
                        if ((p_params->cfg[svc_idx].report_char_cfg[report_idx] & HOGPD_CFG_REPORT_WR)
                                == HOGPD_CFG_REPORT_WR)
                        {
                            perm |= PROP(WR);
                        }
                    } break;

                    // Output Report
                    case HOGPD_CFG_REPORT_OUT:  { perm = PROP(RD) | PROP(WR) | PROP(WC); } break;
                    // Feature Report
                    case HOGPD_CFG_REPORT_FEAT: { perm = PROP(RD) | PROP(WR);            } break;

                    default:                    { status = PRF_ERR_INVALID_PARAM;        } break;
                }

                p_hids_db[svc_idx][HOGPD_IDX_REPORT_VAL + (HIDS_REPORT_NB_IDX*report_idx)].info  = perm;
            }

            // increment total number of attributes to allocate.
            tot_nb_att += p_hogpd_env->svcs[svc_idx].nb_att;
        }

        // Check that attribute list can be allocated.
        if (status == GAP_ERR_NO_ERROR)
        {
            status = gatt_db_handle_range_reserve(user_lid, tot_nb_att, p_start_hdl);
        }

        if (status != GAP_ERR_NO_ERROR) break;

        // used start handle calculated when handle range reservation has been performed
        shdl = *p_start_hdl;
        p_hogpd_env->start_hdl = *p_start_hdl;
        p_hogpd_env->hids_nb = p_params->hids_nb;

        // allocate services
        for (svc_idx = 0; ((svc_idx < p_params->hids_nb) && (status == GAP_ERR_NO_ERROR)); svc_idx++)
        {
            // Add GAP service
            status = gatt_db_svc16_add(user_lid, sec_lvl, GATT_SVC_HID, HOGPD_ATT_MAX,
                                       (uint8_t *)&(cfg_flag[svc_idx][0]), p_hids_db[svc_idx], 0, &shdl);
            if(status != GAP_ERR_NO_ERROR) break;

            // update start handle for next service
            shdl += p_hogpd_env->svcs[svc_idx].nb_att;

            // by default in Report protocol mode.
            p_hogpd_env->svcs[svc_idx].proto_mode = HOGP_REPORT_PROTOCOL_MODE;
        }

        if (status != GAP_ERR_NO_ERROR) break;

        // Init HOGPD required environment variable
        p_env->p_env = (prf_hdr_t *) p_hogpd_env;
        p_hogpd_env->start_hdl   = *p_start_hdl;
        p_hogpd_env->user_lid    = user_lid;
        p_hogpd_env->op_ongoing  = false;
        p_hogpd_env->in_exe_op   = false;
        memset(p_hogpd_env->ntf_cfg, 0, sizeof(p_hogpd_env->ntf_cfg));
        common_list_init(&(p_hogpd_env->wait_queue));

        // initialize profile environment variable
        p_hogpd_env->prf_env.p_cb    = p_cb;
        #if (BLE_HL_MSG_API)
        p_env->desc.msg_handler_tab  = hogpd_msg_handler_tab;
        p_env->desc.msg_cnt          = ARRAY_LEN(hogpd_msg_handler_tab);
        #endif // (BLE_HL_MSG_API)

    } while(0);

    // Clean-up allocated service description structures
    for (svc_idx = 0; svc_idx < HOGPD_NB_HIDS_INST_MAX ; svc_idx++)
    {
        if(p_hids_db[svc_idx] != NULL)
        {
            kernel_free(p_hids_db[svc_idx]);
        }
    }

    if((status != GAP_ERR_NO_ERROR) && (user_lid != GATT_INVALID_USER_LID))
    {
        if(p_hogpd_env != NULL)
        {
            kernel_free(p_hogpd_env);
        }

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
__STATIC uint16_t hogpd_destroy(prf_data_t *p_env, uint8_t reason)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    hogpd_env_t *p_hogpd_env = (hogpd_env_t *) p_env->p_env;

    if(reason != PRF_DESTROY_RESET)
    {
        status = gatt_user_unregister(p_hogpd_env->user_lid);
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        if(reason != PRF_DESTROY_RESET)
        {
            // remove buffer in wait queue
            while(!common_list_is_empty(&p_hogpd_env->wait_queue))
            {
                common_buf_t* p_buf = (common_buf_t*) common_list_pop_front(&p_hogpd_env->wait_queue);
                common_buf_release(p_buf);
            }
        }

        // free profile environment variables
        p_env->p_env = NULL;
        kernel_free(p_hogpd_env);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief @brief Handles Connection creation
 *
 * @param[in|out]    env          Collector or Service allocated environment data.
 * @param[in]        conidx       Connection index
 * @param[in]        p_con_param  Pointer to connection parameters information
 ****************************************************************************************
 */
__STATIC void hogpd_con_create(prf_data_t *p_env, uint8_t conidx, const gap_con_param_t* p_con_param)
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
__STATIC void hogpd_con_cleanup(prf_data_t *p_env, uint8_t conidx, uint16_t reason)
{
    hogpd_env_t *p_hogpd_env = (hogpd_env_t *) p_env->p_env;
    uint8_t svc_idx;
    BLE_ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // Reset the notification configuration to ensure that no notification will be sent on
    // a disconnected link
    for (svc_idx = 0; svc_idx < p_hogpd_env->hids_nb; svc_idx++)
    {
        p_hogpd_env->ntf_cfg[svc_idx][conidx] = 0;
    }
}

/// HOGPD Task interface required by profile manager
const prf_task_cbs_t hogpd_itf =
{
    .cb_init          = (prf_init_cb) hogpd_init,
    .cb_destroy       = hogpd_destroy,
    .cb_con_create    = hogpd_con_create,
    .cb_con_cleanup   = hogpd_con_cleanup,
    .cb_con_upd       = NULL,
};

/**
 ****************************************************************************************
 * @brief Retrieve service profile interface
 *
 * @return service profile interface
 ****************************************************************************************
 */
const prf_task_cbs_t *hogpd_prf_itf_get(void)
{
    return &hogpd_itf;
}

#endif /* BLE_HID_DEVICE */

/// @} HOGPD
