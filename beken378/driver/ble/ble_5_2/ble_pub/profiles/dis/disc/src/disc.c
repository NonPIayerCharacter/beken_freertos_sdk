/**
 ****************************************************************************************
 *
 * @file disc.c
 *
 * @brief Device Information Service Client Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup DISC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_DIS_CLIENT)
#include "prf.h"
#include "prf_utils.h"
#include "disc.h"
#include "gap.h"

#include "kernel_mem.h"
#include <string.h>


/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of Device Information Service Client task instances
#define DIS_VAL_MAX_LEN (128)

/// Possible states of the DISC task
enum
{
    /// Connection not present
    DISC_FREE,
    /// Idle state
    DISC_IDLE,
    /// Discovering State
    DISC_DISCOVERING,

    ///Number of defined states.
    DISC_STATE_MAX
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Environment variable for each Connections
typedef struct disc_cnx_env
{
    /// Peer database discovered handle mapping
    disc_dis_content_t dis;
    /// counter used to check service uniqueness
    uint8_t            nb_svc;
    /// Client activity state
    uint8_t            state;
} disc_cnx_env_t;

/// Device Information Service Client environment variable
typedef struct disc_env
{
    /// profile environment
    prf_hdr_t            prf_env;
    /// Environment variable pointer for each connections
    disc_cnx_env_t*      p_env[BLE_CONNECTION_MAX];
    /// GATT User local identifier
    uint8_t              user_lid;
} disc_env_t;



/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// State machine used to retrieve Device Information Service characteristics information
const prf_char_def_t disc_dis_char[DISC_VAL_MAX] =
{
    // Manufacturer Name
    [DISC_VAL_MANUFACTURER_NAME] = {GATT_CHAR_MANUF_NAME,  ATT_REQ(PRES, OPT), PROP(RD)},
    // Model Number String
    [DISC_VAL_MODEL_NB_STR]      = {GATT_CHAR_MODEL_NB,    ATT_REQ(PRES, OPT), PROP(RD)},
    // Serial Number String
    [DISC_VAL_SERIAL_NB_STR]     = {GATT_CHAR_SERIAL_NB,   ATT_REQ(PRES, OPT), PROP(RD)},
    // Hardware Revision String
    [DISC_VAL_HARD_REV_STR]      = {GATT_CHAR_HW_REV,      ATT_REQ(PRES, OPT), PROP(RD)},
    // Firmware Revision String
    [DISC_VAL_FIRM_REV_STR]      = {GATT_CHAR_FW_REV,      ATT_REQ(PRES, OPT), PROP(RD)},
    // TSoftware Revision String
    [DISC_VAL_SW_REV_STR]        = {GATT_CHAR_SW_REV,      ATT_REQ(PRES, OPT), PROP(RD)},
    // System ID
    [DISC_VAL_SYSTEM_ID]         = {GATT_CHAR_SYS_ID,      ATT_REQ(PRES, OPT), PROP(RD)},
    // IEEE
    [DISC_VAL_IEEE]              = {GATT_CHAR_IEEE_CERTIF, ATT_REQ(PRES, OPT), PROP(RD)},
    // PnP ID
    [DISC_VAL_PNP_ID]            = {GATT_CHAR_PNP_ID,      ATT_REQ(PRES, OPT), PROP(RD)},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send discovery results to application.
 *
 * @param p_disc_env    Client Role task environment
 * @param conidx        Connection index
 * @param status        Response status code
 *****************************************************************************************
 */
__STATIC void disc_enable_cmp(disc_env_t* p_disc_env, uint8_t conidx, uint16_t status)
{
    const disc_cb_t* p_cb = (const disc_cb_t*) p_disc_env->prf_env.p_cb;

    if(p_disc_env != NULL)
    {
        p_cb->cb_enable_cmp(conidx, status, &(p_disc_env->p_env[conidx]->dis));

        if (status != GAP_ERR_NO_ERROR)
        {
            // clean-up environment variable allocated for task instance
            kernel_free(p_disc_env->p_env[conidx]);
            p_disc_env->p_env[conidx] = NULL;
        }
        else
        {
             p_disc_env->p_env[conidx]->state = DISC_IDLE;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Send read result to application,.
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the request
 * @param[in] val_id        Value Identifier (@see enum disc_val_id)
 * @param[in] length        Length of data value)
 * @param[in] p_data        Pointer of data value
 ****************************************************************************************
 */
__STATIC void disc_read_val_cmp(uint8_t conidx, uint16_t status, uint8_t val_id, uint16_t length, const uint8_t* p_data)
{
    disc_env_t* p_disc_env = PRF_ENV_GET(DISC, disc);

    if(p_disc_env != NULL)
    {
        const disc_cb_t* p_cb = (const disc_cb_t*) p_disc_env->prf_env.p_cb;

        p_cb->cb_read_val_cmp(conidx, status, val_id, length, p_data);
    }
}

/*
 * GATT USER CLIENT HANDLERS
 ****************************************************************************************
 */

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
__STATIC void disc_discover_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    // Get the address of the environment
    disc_env_t* p_disc_env = PRF_ENV_GET(DISC, disc);

    if(p_disc_env != NULL)
    {
        if (p_disc_env->p_env[conidx]->nb_svc ==  1)
        {
            status = prf_check_svc_char_validity(DISC_VAL_MAX, p_disc_env->p_env[conidx]->dis.vals, disc_dis_char);
        }
        // too much services
        else if (p_disc_env->p_env[conidx]->nb_svc > 1)
        {
            status = PRF_ERR_MULTIPLE_SVC;
        }
        // no services found
        else
        {
            status = PRF_ERR_STOP_DISC_CHAR_MISSING;
        }

        disc_enable_cmp(p_disc_env, conidx, status);
    }
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
__STATIC void disc_read_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    if(status != GAP_ERR_NO_ERROR)
    {
        disc_read_val_cmp(conidx, status, (uint8_t) dummy, 0, NULL);
    }
}

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
__STATIC void disc_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint8_t disc_info,
                          uint8_t nb_att, const gatt_svc_att_t* p_atts)
{
    // Get the address of the environment
    disc_env_t* p_disc_env = PRF_ENV_GET(DISC, disc);

    if(p_disc_env != NULL)
    {
        BLE_ASSERT_INFO(p_disc_env->p_env[conidx] != NULL, conidx, user_lid);

        if (p_disc_env->p_env[conidx]->nb_svc == 0)
        {
            //Even if we get multiple responses we only store 1 range
            if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_START))
            {
                p_disc_env->p_env[conidx]->dis.svc.shdl = hdl;
            }

            if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_END))
            {
                p_disc_env->p_env[conidx]->dis.svc.ehdl = hdl + nb_att -1;
            }

            // Retrieve characteristics
            prf_extract_svc_info(hdl, nb_att, p_atts, DISC_VAL_MAX,
                                 &disc_dis_char[0], &(p_disc_env->p_env[conidx]->dis.vals[0]), 0, NULL, NULL);

        }

        if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_END))
        {
            p_disc_env->p_env[conidx]->nb_svc++;
        }
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
__STATIC void disc_att_val_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint16_t offset,
                              common_buf_t* p_data)
{
    disc_read_val_cmp(conidx, GAP_ERR_NO_ERROR, (uint8_t) dummy, common_buf_data_len(p_data), common_buf_data(p_data));
}


/// Client callback hander
__STATIC const gatt_cli_cb_t disc_cb =
{
    .cb_discover_cmp    = disc_discover_cmp_cb,
    .cb_read_cmp        = disc_read_cmp_cb,
    .cb_write_cmp       = NULL,
    .cb_att_val_get     = NULL,
    .cb_svc             = disc_svc_cb,
    .cb_svc_info        = NULL,
    .cb_inc_svc         = NULL,
    .cb_char            = NULL,
    .cb_desc            = NULL,
    .cb_att_val         = disc_att_val_cb,
    .cb_att_val_evt     = NULL,
    .cb_svc_changed     = NULL,
};

/*
 * PROFILE NATIVE API
 ****************************************************************************************
 */

uint16_t disc_enable(uint8_t conidx, uint8_t con_type, const disc_dis_content_t* p_dis)
{
    // Status
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Device Information Service Client Role Task Environment
    disc_env_t* p_disc_env = PRF_ENV_GET(DISC, disc);

    if(p_disc_env != NULL)
    {
        if ((conidx < BLE_CONNECTION_MAX) && (p_disc_env->p_env[conidx] == NULL))
        {
            // allocate environment variable for task instance
            p_disc_env->p_env[conidx] = (disc_cnx_env_t *) kernel_malloc(sizeof(disc_cnx_env_t), KERNEL_MEM_ATT_DB);

            if(p_disc_env->p_env[conidx] != NULL)
            {
                // Config connection, start discovering
                if (con_type == PRF_CON_DISCOVERY)
                {
                    uint16_t gatt_svc_uuid = GATT_SVC_DEVICE_INFO;
                    memset(p_disc_env->p_env[conidx], 0, sizeof(disc_cnx_env_t));

                    // start discovery
                    status = gatt_cli_discover_svc(conidx, p_disc_env->user_lid, 0, GATT_DISCOVER_SVC_PRIMARY_BY_UUID, true,
                                                   GATT_MIN_HDL, GATT_MAX_HDL, GATT_UUID_16, (uint8_t*) &gatt_svc_uuid);

                    // Go to DISCOVERING state
                    p_disc_env->p_env[conidx]->state = DISC_DISCOVERING;
                }
                // normal connection, get saved att details
                else
                {
                    memcpy(&(p_disc_env->p_env[conidx]->dis), p_dis, sizeof(disc_dis_content_t));
                    status = GAP_ERR_NO_ERROR;

                    // send APP confirmation that can start normal connection to TH
                    disc_enable_cmp(p_disc_env, conidx, GAP_ERR_NO_ERROR);
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

uint16_t disc_read_val(uint8_t conidx, uint8_t val_id)
{
    // Status
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Device Information Service Client Role Task Environment
    disc_env_t* p_disc_env = PRF_ENV_GET(DISC, disc);

    if(p_disc_env != NULL)
    {
        if ((conidx < BLE_CONNECTION_MAX) && (p_disc_env->p_env[conidx] != NULL) && (p_disc_env->p_env[conidx]->state == DISC_IDLE))
        {
            uint16_t search_hdl = GATT_INVALID_HDL;

            // retrieve search handle
            if (val_id < DISC_VAL_MAX)
            {
                search_hdl = p_disc_env->p_env[conidx]->dis.vals[val_id].val_hdl;
            }

            //Check if handle is viable
            if (search_hdl != GATT_INVALID_HDL)
            {
                // perform read request
                status = gatt_cli_read(conidx, p_disc_env->user_lid, val_id, search_hdl, 0, 0);
            }
            else
            {
                // invalid handle requested
                status = PRF_ERR_INEXISTENT_HDL;
            }
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
 * @brief Handles reception of the @ref DISC_ENABLE_REQ message.
 * The handler enables the Device Information Service Client Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int disc_enable_req_handler(kernel_msg_id_t const msgid, struct disc_enable_req const *p_param,
                                     kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint16_t status = disc_enable(p_param->conidx, p_param->con_type, &(p_param->dis));

    // send an error if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct disc_enable_rsp *p_rsp = KERNEL_MSG_ALLOC(DISC_ENABLE_RSP, src_id, dest_id, disc_enable_rsp);
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
 * @brief Handles reception of the @ref DISC_RD_CHAR_CMD message.
 * Check if the handle exists in profile(already discovered) and send request, otherwise
 * error to APP.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int disc_rd_val_cmd_handler(kernel_msg_id_t const msgid, struct disc_rd_val_cmd const *p_param,
                                      kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint16_t status = disc_read_val(p_param->conidx, p_param->val_id);

    // send error response if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct disc_cmp_evt *p_evt = KERNEL_MSG_ALLOC(DISC_CMP_EVT, src_id, dest_id, disc_cmp_evt);

        if(p_evt != NULL)
        {
            p_evt->conidx      = p_param->conidx;
            p_evt->operation   = DISC_RD_VAL_CMD_OP_CODE;
            p_evt->status      = status;

            kernel_msg_send(p_evt);
        }
    }

    return (KERNEL_MSG_CONSUMED);
}

/// Default State handlers definition
KERNEL_MSG_HANDLER_TAB(disc)
{
    // Note: all messages must be sorted in ID ascending order

    {DISC_ENABLE_REQ,        (kernel_msg_func_t)disc_enable_req_handler},
    {DISC_RD_VAL_CMD,        (kernel_msg_func_t)disc_rd_val_cmd_handler},
};

/**
 ****************************************************************************************
 * @brief This function is called when GATT server user has initiated event send to peer
 *        device or if an error occurs.
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Client Enable status (@see enum hl_err)
 * @param[in] p_dis         Pointer to bond data information that describe peer database
 ****************************************************************************************
 */
void disc_enable_cmp_handler(uint8_t conidx, uint16_t status, const disc_dis_content_t* p_dis)
{
    // Send APP the details of the discovered attributes on DISC
    struct disc_enable_rsp *p_rsp = KERNEL_MSG_ALLOC(DISC_ENABLE_RSP, PRF_DST_TASK(DISC), PRF_SRC_TASK(DISC),
                                                 disc_enable_rsp);
    if(p_rsp != NULL)
    {
        p_rsp->conidx = conidx;
        p_rsp->status = status;
        memcpy(&(p_rsp->dis), p_dis, sizeof(disc_dis_content_t));
        kernel_msg_send(p_rsp);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT server user has initiated event send to peer
 *        device or if an error occurs.
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Read status (@see enum hl_err)
 * @param[in] val_id        Value identifer read (@see enum disc_val_id)
 * @param[in] length        Value data length
 * @param[in] p_data        Pointer to value data
 ****************************************************************************************
 */
void disc_read_val_cmp_handler(uint8_t conidx, uint16_t status, uint8_t val_id, uint16_t length, const uint8_t* p_data)
{
    kernel_task_id_t src_id  = PRF_SRC_TASK(DISC);
    kernel_task_id_t dest_id = PRF_DST_TASK(DISC);

    if(status == GAP_ERR_NO_ERROR)
    {
        struct disc_rd_val_ind *p_ind = KERNEL_MSG_ALLOC_DYN(DISC_RD_VAL_IND, dest_id, src_id, disc_rd_val_ind, length);
        if(p_ind != NULL)
        {
            p_ind->conidx = conidx;
            p_ind->val_id = val_id;
            p_ind->length = length;
            memcpy(&(p_ind->value), p_data, length);
            kernel_msg_send(p_ind);
        }
    }

    struct disc_cmp_evt *p_evt = KERNEL_MSG_ALLOC(DISC_CMP_EVT, dest_id, src_id, disc_cmp_evt);
    if(p_evt != NULL)
    {
        p_evt->conidx      = conidx;
        p_evt->operation   = DISC_RD_VAL_CMD_OP_CODE;
        p_evt->status      = status;

        kernel_msg_send(p_evt);
    }
}

/// Default Message handle
__STATIC const disc_cb_t disc_msg_cb =
{
    .cb_enable_cmp   = disc_enable_cmp_handler,
    .cb_read_val_cmp = disc_read_val_cmp_handler,
};
#endif // (BLE_HL_MSG_API)


/*
 * PROFILE DEFAULT HANDLERS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialization of the DISC module.
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
__STATIC uint16_t disc_init(prf_data_t* p_env, uint16_t* p_start_hdl, uint8_t sec_lvl, uint8_t user_prio,
                          const void* p_params, const disc_cb_t* p_cb)
{
    uint8_t conidx;
    // DB Creation Status
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t user_lid = GATT_INVALID_USER_LID;

    do
    {
        disc_env_t* p_disc_env;
        #if (BLE_HL_MSG_API)
        if(p_cb == NULL)
        {
            p_cb = &(disc_msg_cb);
        }
        #endif // (BLE_HL_MSG_API)

        if((p_params == NULL) || (p_cb == NULL) || (p_cb->cb_enable_cmp == NULL) || (p_cb->cb_read_val_cmp == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // register DISC user
        status = gatt_user_cli_register(DIS_VAL_MAX_LEN, user_prio, &disc_cb, &user_lid);
        if(status != GAP_ERR_NO_ERROR) break;

        //-------------------- allocate memory required for the profile  ---------------------
        p_disc_env = (disc_env_t*) kernel_malloc(sizeof(disc_env_t), KERNEL_MEM_ATT_DB);

        if(p_disc_env != NULL)
        {
            // allocate DISC required environment variable
            p_env->p_env = (prf_hdr_t *) p_disc_env;

            // initialize environment variable
            p_disc_env->prf_env.p_cb    = p_cb;
            #if (BLE_HL_MSG_API)
            p_env->desc.msg_handler_tab = disc_msg_handler_tab;
            p_env->desc.msg_cnt         = ARRAY_LEN(disc_msg_handler_tab);
            #endif // (BLE_HL_MSG_API)

            p_disc_env->user_lid = user_lid;
            for (conidx = 0; conidx < BLE_CONNECTION_MAX; conidx++)
            {
                p_disc_env->p_env[conidx] = NULL;
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
__STATIC uint16_t disc_destroy(prf_data_t *p_env, uint8_t reason)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    disc_env_t* p_disc_env = (disc_env_t*) p_env->p_env;

    if(reason != PRF_DESTROY_RESET)
    {
        status = gatt_user_unregister(p_disc_env->user_lid);
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        uint8_t idx;
        // cleanup environment variable for each task instances
        for (idx = 0; idx < BLE_CONNECTION_MAX; idx++)
        {
            if (p_disc_env->p_env[idx] != NULL)
            {
                kernel_free(p_disc_env->p_env[idx]);
            }
        }

        // free profile environment variables
        p_env->p_env = NULL;
        kernel_free(p_disc_env);
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
__STATIC void disc_con_create(prf_data_t *p_env, uint8_t conidx, const gap_con_param_t* p_con_param)
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
__STATIC void disc_con_cleanup(prf_data_t *p_env, uint8_t conidx, uint16_t reason)
{
    disc_env_t* p_disc_env = (disc_env_t*) p_env->p_env;

    // clean-up environment variable allocated for task instance
    if (p_disc_env->p_env[conidx] != NULL)
    {
        kernel_free(p_disc_env->p_env[conidx]);
        p_disc_env->p_env[conidx] = NULL;
    }
}

/// DISC Task interface required by profile manager
const prf_task_cbs_t disc_itf =
{
    .cb_init          = (prf_init_cb) disc_init,
    .cb_destroy       = disc_destroy,
    .cb_con_create    = disc_con_create,
    .cb_con_cleanup   = disc_con_cleanup,
    .cb_con_upd       = NULL,
};

/**
 ****************************************************************************************
 * @brief Retrieve DIS client profile interface
 *
 * @return DIS client profile interface
 ****************************************************************************************
 */
const prf_task_cbs_t* disc_prf_itf_get(void)
{
    return &disc_itf;
}

#endif //BLE_DIS_CLIENT

/// @} DISC
