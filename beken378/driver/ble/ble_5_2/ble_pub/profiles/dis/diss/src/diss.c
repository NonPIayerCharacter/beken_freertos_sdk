/**
 ****************************************************************************************
 *
 * @file diss.c
 *
 * @brief Device Information Service Server Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup DISS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwprf_config.h"

#if (BLE_DIS_SERVER)
#include "diss_msg.h"
#include "diss.h"
#include "gatt.h"
#include "prf_utils.h"
#include "prf.h"

#include <string.h>
#include "kernel_mem.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximal length for Characteristic values - 128 bytes
#define DIS_VAL_MAX_LEN                         (128)
///System ID string length
#define DIS_SYS_ID_LEN                          (0x08)
///IEEE Certif length (min 6 bytes)
#define DIS_IEEE_CERTIF_MIN_LEN                 (0x06)
///PnP ID length
#define DIS_PNP_ID_LEN                          (0x07)

/// DISS Attributes database handle list
enum diss_att_db_handles
{
    DIS_IDX_SVC,

    DIS_IDX_MANUFACTURER_NAME_CHAR,
    DIS_IDX_MANUFACTURER_NAME_VAL,

    DIS_IDX_MODEL_NB_STR_CHAR,
    DIS_IDX_MODEL_NB_STR_VAL,

    DIS_IDX_SERIAL_NB_STR_CHAR,
    DIS_IDX_SERIAL_NB_STR_VAL,

    DIS_IDX_HARD_REV_STR_CHAR,
    DIS_IDX_HARD_REV_STR_VAL,

    DIS_IDX_FIRM_REV_STR_CHAR,
    DIS_IDX_FIRM_REV_STR_VAL,

    DIS_IDX_SW_REV_STR_CHAR,
    DIS_IDX_SW_REV_STR_VAL,

    DIS_IDX_SYSTEM_ID_CHAR,
    DIS_IDX_SYSTEM_ID_VAL,

    DIS_IDX_IEEE_CHAR,
    DIS_IDX_IEEE_VAL,

    DIS_IDX_PNP_ID_CHAR,
    DIS_IDX_PNP_ID_VAL,

    DIS_IDX_NB,
};

/// Content of DISS token
enum diss_token_bf
{
    /// GATT procedure token
    DISS_TOKEN_GATT_TOKEN_MASK = 0x0000FFFF,
    DISS_TOKEN_GATT_TOKEN_LSB  = 0,
    /// Connection index
    DISS_TOKEN_CONIDX_MASK     = 0x00FF0000,
    DISS_TOKEN_CONIDX_LSB      = 16,
    /// Data offset requested
    DISS_TOKEN_OFFSET_MASK     = 0xFF000000,
    DISS_TOKEN_OFFSET_LSB      = 24,
};

/*
 * TYPES DEFINITION
 ****************************************************************************************
 */

/// Value element
struct diss_val_elmt
{
    /// list element header
    struct common_list_hdr hdr;
    /// value identifier
    uint8_t value;
    /// value length
    uint8_t length;
    /// value data
    uint8_t data[__ARRAY_EMPTY];
};

///Device Information Service Server Environment Variable
typedef struct diss_env
{
    /// profile environment
    prf_hdr_t prf_env;
    /// List of values set by application
    struct common_list values;
    /// Service Attribute Start Handle
    uint16_t start_hdl;
    /// Services features
    uint16_t features;
    /// GATT user local identifier
    uint8_t  user_lid;
} diss_env_t;


/*
 * DIS ATTRIBUTES DATABASE
 ****************************************************************************************
 */

/// Full DIS Database Description - Used to add attributes into the database
const gatt_att16_desc_t diss_att_db[DIS_IDX_NB] =
{
    // Device Information Service Declaration
    [DIS_IDX_SVC]                       =   {GATT_DECL_PRIMARY_SERVICE,     PROP(RD), 0                 },

    // Manufacturer Name Characteristic Declaration
    [DIS_IDX_MANUFACTURER_NAME_CHAR]    =   {GATT_DECL_CHARACTERISTIC,      PROP(RD), 0                 },
    // Manufacturer Name Characteristic Value
    [DIS_IDX_MANUFACTURER_NAME_VAL]     =   {GATT_CHAR_MANUF_NAME,          PROP(RD), DIS_VAL_MAX_LEN   },

    // Model Number String Characteristic Declaration
    [DIS_IDX_MODEL_NB_STR_CHAR]         =   {GATT_DECL_CHARACTERISTIC,      PROP(RD), 0                 },
    // Model Number String Characteristic Value
    [DIS_IDX_MODEL_NB_STR_VAL]          =   {GATT_CHAR_MODEL_NB,            PROP(RD), DIS_VAL_MAX_LEN   },

    // Serial Number String Characteristic Declaration
    [DIS_IDX_SERIAL_NB_STR_CHAR]        =   {GATT_DECL_CHARACTERISTIC,      PROP(RD), 0                 },
    // Serial Number String Characteristic Value
    [DIS_IDX_SERIAL_NB_STR_VAL]         =   {GATT_CHAR_SERIAL_NB,           PROP(RD), DIS_VAL_MAX_LEN   },

    // Hardware Revision String Characteristic Declaration
    [DIS_IDX_HARD_REV_STR_CHAR]         =   {GATT_DECL_CHARACTERISTIC,      PROP(RD), 0                 },
    // Hardware Revision String Characteristic Value
    [DIS_IDX_HARD_REV_STR_VAL]          =   {GATT_CHAR_HW_REV,              PROP(RD), DIS_VAL_MAX_LEN   },

    // Firmware Revision String Characteristic Declaration
    [DIS_IDX_FIRM_REV_STR_CHAR]         =   {GATT_DECL_CHARACTERISTIC,      PROP(RD), 0                 },
    // Firmware Revision String Characteristic Value
    [DIS_IDX_FIRM_REV_STR_VAL]          =   {GATT_CHAR_FW_REV,              PROP(RD), DIS_VAL_MAX_LEN   },

    // Software Revision String Characteristic Declaration
    [DIS_IDX_SW_REV_STR_CHAR]           =   {GATT_DECL_CHARACTERISTIC,      PROP(RD), 0                 },
    // Software Revision String Characteristic Value
    [DIS_IDX_SW_REV_STR_VAL]            =   {GATT_CHAR_SW_REV,              PROP(RD), DIS_VAL_MAX_LEN   },

    // System ID Characteristic Declaration
    [DIS_IDX_SYSTEM_ID_CHAR]            =   {GATT_DECL_CHARACTERISTIC,      PROP(RD), 0                 },
    // System ID Characteristic Value
    [DIS_IDX_SYSTEM_ID_VAL]             =   {GATT_CHAR_SYS_ID,              PROP(RD), DIS_SYS_ID_LEN    },

    // IEEE 11073-20601 Regulatory Certification Data List Characteristic Declaration
    [DIS_IDX_IEEE_CHAR]                 =   {GATT_DECL_CHARACTERISTIC,      PROP(RD), 0                 },
    // IEEE 11073-20601 Regulatory Certification Data List Characteristic Value
    [DIS_IDX_IEEE_VAL]                  =   {GATT_CHAR_IEEE_CERTIF,         PROP(RD), DIS_SYS_ID_LEN    },

    // PnP ID Characteristic Declaration
    [DIS_IDX_PNP_ID_CHAR]               =   {GATT_DECL_CHARACTERISTIC,      PROP(RD), 0                 },
    // PnP ID Characteristic Value
    [DIS_IDX_PNP_ID_VAL]                =   {GATT_CHAR_PNP_ID,              PROP(RD), DIS_PNP_ID_LEN    },
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check if an attribute shall be added or not in the database
 *
 * @param features DIS features
 *
 * @return Feature config flag
 ****************************************************************************************
 */
__STATIC uint32_t diss_compute_cfg_flag(uint16_t features)
{
    //Service Declaration
    uint32_t cfg_flag = 1;

    for (uint8_t i = 0; i < DIS_VAL_MAX; i++)
    {
        if (((features >> i) & 1) == 1)
        {
            cfg_flag |= (3 << (i*2 + 1));
        }
    }

    return cfg_flag;
}

/**
 ****************************************************************************************
 * @brief Retrieve value from attribute handle
 *
 * @param[in] p_env  Service environment variable
 * @param[in] handle Attribute handle to search
 *
 * @return  Value Identifier from attribute handle
 ****************************************************************************************
 */
__STATIC uint8_t diss_hdl_to_val_id(diss_env_t *p_env, uint16_t hdl)
{
    uint8_t val_id = DIS_VAL_MAX;

    // handle cursor, start from first characteristic of service handle
    uint16_t cur_hdl = p_env->start_hdl + 1;

    for (uint8_t i = 0; i < DIS_VAL_MAX; i++)
    {
        if (((p_env->features >> i) & 1) == 1)
        {
            // check if value handle correspond to requested handle
            if ((cur_hdl + 1) == hdl)
            {
                val_id = i;
                break;
            }

            cur_hdl += 2;
        }
    }

    return val_id;
}

/**
 ****************************************************************************************
 * @brief Retrieve handle attribute from value identifier
 *
 * @param[in] p_env   Service environment variable
 * @param[in] val_id  Value identifier
 *
 * @return Handle attribute from value
 ****************************************************************************************
 */
__STATIC uint16_t diss_val_id_to_hdl(diss_env_t *p_env, uint8_t val_id)
{
    uint16_t hdl = p_env->start_hdl + 1;
    int8_t id;

    for (id = 0; id < DIS_VAL_MAX; id++)
    {
        if (((p_env->features >> id) & 1) == 1)
        {
            // requested value
            if (val_id == id)
            {
                hdl += 1;
                break;
            }

            hdl += 2;
        }
    }

    // check if handle found
    return ((id == DIS_VAL_MAX) ? GATT_INVALID_HDL : hdl);
}

/**
 ****************************************************************************************
 * @brief Check if the provided value length matches characteristic requirements
 *
 * @param char_code Characteristic Code
 * @param val_len   Length of the Characteristic value
 *
 * @return status if value length is ok or not
 ****************************************************************************************
 */
__STATIC uint8_t diss_check_val_len(uint8_t char_code, uint8_t val_len)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    // Check if length is upper than the general maximal length
    if (val_len > DIS_VAL_MAX_LEN)
    {
        status = PRF_ERR_UNEXPECTED_LEN;
    }
    else
    {
        // Check if length matches particular requirements
        switch (char_code)
        {
            case DIS_VAL_SYSTEM_ID:
            {
                if (val_len != DIS_SYS_ID_LEN)
                {
                    status = PRF_ERR_UNEXPECTED_LEN;
                }
            } break;

            case DIS_VAL_IEEE:
            {
                if (val_len < DIS_IEEE_CERTIF_MIN_LEN)
                {
                    status = PRF_ERR_UNEXPECTED_LEN;
                }
            } break;

            case DIS_VAL_PNP_ID:
            {
                if (val_len != DIS_PNP_ID_LEN)
                {
                    status = PRF_ERR_UNEXPECTED_LEN;
                }
            } break;

            default: { /* Nothing to do */ } break;
        }
    }

    return (status);
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
__STATIC void diss_cb_att_read_get(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
                                   uint16_t max_length)
{
    diss_env_t *p_diss_env = PRF_ENV_GET(DISS, diss);
    // retrieve value attribute
    uint8_t   val_id      = diss_hdl_to_val_id(p_diss_env, hdl);
    common_buf_t* p_buf       = NULL;
    uint16_t  status      = GAP_ERR_NO_ERROR;
    uint16_t  att_val_len = 0;
    bool      confirm_send     = true;

    if(offset > DIS_VAL_MAX_LEN)
    {
        status = ATT_ERR_INVALID_OFFSET;
    }
    // Check Characteristic Code
    else if (val_id < DIS_VAL_MAX)
    {
        // Check value in already present in service
        struct diss_val_elmt *p_val = (struct diss_val_elmt *) common_list_pick(&(p_diss_env->values));

        // loop until value found
        while (p_val != NULL)
        {
            // value is present in service
            if (p_val->value == val_id)
            {
                break;
            }

            p_val = (struct diss_val_elmt *)p_val->hdr.next;
        }

        if (p_val != NULL)
        {
            att_val_len = p_val->length;

            if(offset > att_val_len)
            {
                status = ATT_ERR_INVALID_OFFSET;
            }
            else
            {
                status = common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, att_val_len - offset, GATT_BUFFER_TAIL_LEN);

                if(status == GAP_ERR_NO_ERROR)
                {
                    common_buf_copy_data_from_mem(p_buf, &(p_val->data[offset]), att_val_len - offset);
                }
                else
                {
                    status = ATT_ERR_INSUFF_RESOURCE;
                }
            }
        }
        else
        {
            uint32_t diss_token   = 0;
            const diss_cb_t* p_cb = (const diss_cb_t*) p_diss_env->prf_env.p_cb;

            SETF(diss_token, DISS_TOKEN_GATT_TOKEN, token);
            SETF(diss_token, DISS_TOKEN_CONIDX,     conidx);
            SETF(diss_token, DISS_TOKEN_OFFSET,     offset);

            p_cb->cb_value_get(diss_token, val_id);
            confirm_send = false;
        }
    }
    else
    {
        // application error, value cannot be retrieved
        status = ATT_ERR_APP_ERROR;
    }

    // Immediately send confirmation message
    if(confirm_send)
    {
        gatt_srv_att_read_get_cfm(conidx, user_lid, token, status, att_val_len, p_buf);
        if(p_buf != NULL)
        {
            common_buf_release(p_buf);
        }
    }
}

/**
 ****************************************************************************************
 * @brief This function is called during a write procedure to get information about a
 *        specific attribute handle.
 *
 *        @see gatt_srv_att_info_get_cfm shall be called to provide attribute information
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 ****************************************************************************************
 */
__STATIC void diss_cb_att_info_get (uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl)
{
    gatt_srv_att_info_get_cfm(conidx, user_lid, token, ATT_ERR_APP_ERROR, 0);
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
__STATIC void diss_cb_att_val_set(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
                        common_buf_t* p_data)
{
    gatt_srv_att_val_set_cfm(conidx, user_lid, token, ATT_ERR_APP_ERROR);
}


/// Service callback hander
__STATIC const gatt_srv_cb_t diss_cb =
{
        .cb_event_sent    = NULL,
        .cb_att_read_get  = diss_cb_att_read_get,
        .cb_att_event_get = NULL,
        .cb_att_info_get  = diss_cb_att_info_get,
        .cb_att_val_set   = diss_cb_att_val_set,
};

/*
 * PROFILE NATIVE HANDLERS
 ****************************************************************************************
 */
uint16_t diss_value_set(uint8_t val_id, uint8_t length, const uint8_t* p_data)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    diss_env_t *p_diss_env = PRF_ENV_GET(DISS, diss);

    if (val_id >= DIS_VAL_MAX)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if(p_diss_env != NULL)
    {
        // Characteristic Declaration attribute handle
        uint16_t hdl = diss_val_id_to_hdl(p_diss_env, val_id);

        // Check if the Characteristic exists in the database
        if (hdl != GATT_INVALID_HDL)
        {
            // Check the value length
            status = diss_check_val_len(val_id, length);

            if (status == GAP_ERR_NO_ERROR)
            {
                // Check value in already present in service
                struct diss_val_elmt *p_val = (struct diss_val_elmt *) common_list_pick(&(p_diss_env->values));

                // loop until value found
                while (p_val != NULL)
                {
                    // if value already present, remove old one
                    if (p_val->value == val_id)
                    {
                        common_list_extract(&(p_diss_env->values), &(p_val->hdr));
                        kernel_free(p_val);
                        break;
                    }

                    p_val = (struct diss_val_elmt *)p_val->hdr.next;
                }

                // allocate value data
                p_val = (struct diss_val_elmt *) kernel_malloc(sizeof(struct diss_val_elmt) + length, KERNEL_MEM_ATT_DB);

                p_val->value  = val_id;
                p_val->length = length;
                memcpy(p_val->data, p_data, length);
                // insert value into the list
                common_list_push_back(&(p_diss_env->values), &(p_val->hdr));
            }
        }
        else
        {
            status = PRF_ERR_INEXISTENT_HDL;
        }
    }

    return (status);
}

uint16_t diss_value_cfm(uint32_t token, uint8_t length, const uint8_t* p_data)
{
    uint16_t  status     = GAP_ERR_COMMAND_DISALLOWED;

    diss_env_t *p_diss_env = PRF_ENV_GET(DISS, diss);
    if(p_diss_env != NULL)
    {
        common_buf_t* p_buf      = NULL;
        uint16_t  gatt_token = (uint16_t) GETF(token, DISS_TOKEN_GATT_TOKEN);
        uint8_t   conidx     = (uint8_t)  GETF(token, DISS_TOKEN_CONIDX);
        uint8_t   offset     = (uint8_t)  GETF(token, DISS_TOKEN_OFFSET);

        if(offset > length)
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
            status = common_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, length - offset, GATT_BUFFER_TAIL_LEN);

            if(status == GAP_ERR_NO_ERROR)
            {
                common_buf_copy_data_from_mem(p_buf, &(p_data[offset]), length - offset);
            }
            else
            {
                status = ATT_ERR_INSUFF_RESOURCE;
            }
        }

        status = gatt_srv_att_read_get_cfm(conidx, p_diss_env->user_lid, gatt_token, status, length, p_buf);
        if(p_buf != NULL)
        {
            common_buf_release(p_buf);
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
 * @brief Handles reception of the @ref DISS_SET_VALUE_REQ message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int diss_set_value_req_handler(kernel_msg_id_t const msgid,
                                        struct diss_set_value_req const *p_param,
                                        kernel_task_id_t const dest_id,
                                        kernel_task_id_t const src_id)
{

    struct diss_set_value_rsp *p_rsp;
    // Request status
    uint16_t status = diss_value_set(p_param->val_id, p_param->length, p_param->data);

    // send response to application
    p_rsp         = KERNEL_MSG_ALLOC(DISS_SET_VALUE_RSP, src_id, dest_id, diss_set_value_rsp);
    if(p_rsp)
    {
        p_rsp->val_id  = p_param->val_id;
        p_rsp->status = status;

        kernel_msg_send(p_rsp);
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the value confirmation from application
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int diss_value_cfm_handler(kernel_msg_id_t const msgid,
                                      struct diss_value_cfm const *p_param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    // handle value confirmation
    diss_value_cfm(p_param->token, p_param->length, p_param->data);
    return (KERNEL_MSG_CONSUMED);
}


/// Default State handlers definition
KERNEL_MSG_HANDLER_TAB(diss)
{
    // Note: all messages must be sorted in ID ascending order

    {DISS_SET_VALUE_REQ,      (kernel_msg_func_t)diss_set_value_req_handler},
    {DISS_VALUE_CFM,          (kernel_msg_func_t)diss_value_cfm_handler},
};

/**
 ****************************************************************************************
 * @brief This function is called when GATT server user has initiated event send to peer
 *        device or if an error occurs.
 *
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] val_id        Requested value identifier (@see enum diss_val_id)
 ****************************************************************************************
 */
void diss_value_get_handler(uint32_t token, uint8_t val_id)
{
    struct diss_value_req_ind *p_req_ind;

    // request value to application

    p_req_ind = KERNEL_MSG_ALLOC(DISS_VALUE_REQ_IND, PRF_DST_TASK(DISS), PRF_SRC_TASK(DISS), diss_value_req_ind);

    if(p_req_ind)
    {
        p_req_ind->val_id = val_id;
        p_req_ind->token = token;
        kernel_msg_send(p_req_ind);
    }
}

/// Default Message handle
__STATIC const diss_cb_t diss_msg_cb =
{
    .cb_value_get = diss_value_get_handler,
};
#endif // (BLE_HL_MSG_API)



/*
 * PROFILE DEFAULT HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the DISS module.
 * This function performs all the initializations of the Profile module.
 *  - Creation of database (if it's a service)
 *  - Allocation of profile required memory
 *  - Initialization of task descriptor to register application
 *      - Task State array
 *      - Number of tasks
 *      - Default task handler
 *
 * @param[out]    p_env        Collector or Service allocated environment data.
 * @param[in|out] p_start_hdl  Service start handle (0 - dynamically allocated), only applies for services.
 * @param[in]     sec_lvl      Security level (@see enum gatt_svc_info_bf)
 * @param[in]     user_prio    GATT User priority
 * @param[in]     p_param      Configuration parameters of profile collector or service (32 bits aligned)
 * @param[in]     p_cb         Callback structure that handles event from profile
 *
 * @return status code to know if profile initialization succeed or not.
 ****************************************************************************************
 */
__STATIC uint16_t diss_init(prf_data_t *p_env, uint16_t *p_start_hdl, uint8_t sec_lvl, uint8_t user_prio,
                          struct diss_db_cfg *p_params, const diss_cb_t* p_cb)
{
    //------------------ create the attribute database for the profile -------------------

    // DB Creation Status
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t user_lid = GATT_INVALID_USER_LID;

    do
    {
        diss_env_t* p_diss_env;
        // Service content flag
        uint32_t cfg_flag;

        #if (BLE_HL_MSG_API)
        if(p_cb == NULL)
        {
            p_cb = &(diss_msg_cb);
        }
        #endif // (BLE_HL_MSG_API)

        if((p_params == NULL) || (p_start_hdl == NULL) || (p_cb == NULL) || (p_cb->cb_value_get == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // register DISS user
        status = gatt_user_srv_register(DIS_VAL_MAX_LEN, user_prio, &diss_cb, &user_lid);
        if(status != GAP_ERR_NO_ERROR) break;

        // Compute Attribute Table and save it in environment
        cfg_flag = diss_compute_cfg_flag(p_params->features);

        // Add GAP service
        status = gatt_db_svc16_add(user_lid, sec_lvl, GATT_SVC_DEVICE_INFO, DIS_IDX_NB,
                                   (uint8_t *)&cfg_flag, &(diss_att_db[0]), DIS_IDX_NB, p_start_hdl);
        if(status != GAP_ERR_NO_ERROR) break;

        //-------------------- allocate memory required for the profile  ---------------------
        p_diss_env = (diss_env_t *) kernel_malloc(sizeof(diss_env_t), KERNEL_MEM_ATT_DB);

        if(p_diss_env != NULL)
        {
            // allocate DISS required environment variable
            p_env->p_env = (prf_hdr_t *) p_diss_env;
            p_diss_env->start_hdl = *p_start_hdl;
            p_diss_env->features  = p_params->features;
            p_diss_env->user_lid  = user_lid;
            common_list_init(&(p_diss_env->values));

            // initialize profile environment variable
            p_diss_env->prf_env.p_cb     = p_cb;
            #if (BLE_HL_MSG_API)
            p_env->desc.msg_handler_tab  = diss_msg_handler_tab;
            p_env->desc.msg_cnt          = ARRAY_LEN(diss_msg_handler_tab);
            #endif // (BLE_HL_MSG_API)
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
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
__STATIC uint16_t diss_destroy(prf_data_t *p_env, uint8_t reason)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    diss_env_t* p_diss_env = (diss_env_t*) p_env->p_env;

    if(reason != PRF_DESTROY_RESET)
    {
        status = gatt_user_unregister(p_diss_env->user_lid);
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        // remove all values present in list
        while (!common_list_is_empty(&(p_diss_env->values)))
        {
            struct common_list_hdr *p_hdr = common_list_pop_front(&(p_diss_env->values));

            kernel_free(p_hdr);
        }

        // free profile environment variables
        p_env->p_env = NULL;
        kernel_free(p_diss_env);
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
__STATIC void diss_con_create(prf_data_t *p_env, uint8_t conidx, const gap_con_param_t* p_con_param)
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
__STATIC void diss_con_cleanup(prf_data_t *p_env, uint8_t conidx, uint16_t reason)
{
    // Nothing to do
}



/// DISS Task interface required by profile manager
const prf_task_cbs_t diss_itf =
{
    .cb_init          = (prf_init_cb) diss_init,
    .cb_destroy       = diss_destroy,
    .cb_con_create    = diss_con_create,
    .cb_con_cleanup   = diss_con_cleanup,
    .cb_con_upd       = NULL,
};

/**
 ****************************************************************************************
 * @brief Retrieve DIS service profile interface
 *
 * @return DIS service profile interface
 ****************************************************************************************
 */
const prf_task_cbs_t *diss_prf_itf_get(void)
{
    return &diss_itf;
}


#endif //BLE_DIS_SERVER

/// @} DISS
