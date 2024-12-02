/**
 ****************************************************************************************
 *
 * @file app_mesh.c
 *
 * @brief mesh Application Module entry point
 *
 * @auth  gang.cheng
 *
 * @date  2018.07.09
 *
 * Copyright (C) Beken 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "mesh_general_api.h" 
#include "lld_adv_test.h"
#include "app_mesh.h"                // Bracese Application Module Definitions
#include "app_mm_msg.h"                // Bracese Application Module Definitions
#include "m_api.h"
#include "application.h"                     // Application Definitions
#include "app_task.h"                // application task definitions
#include "common_bt.h"
#include "prf_types.h"               // Profile common types definition
#include "architect.h"                    // Platform Definitions
#include "prf.h"
#include "lld_evt.h"
#include "ble_uart.h"
#include "m_api.h"
#include "mesh_api_msg.h"
#include "mal.h"
#include "m_bcn.h"
#include "m_prov_int.h"     // Mesh Provisioning Internal Defines
#include "mm_vendors.h"
#include "m_fnd_Scenes.h"
#include "m_fnd_int.h"
#include "mesh_param_int.h"
#include "m_fnd_Fw_Update.h"
#include "m_fnd_BLOB_Transfer.h"
#include "m_defines.h"
#include "ble_api.h"
#include "mal_int.h"

/*
 * LOCATION FUN DEFINES
 ****************************************************************************************
 */
static void app_mesh_adv_report_cb(const struct adv_report* p_report);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// braces Application Module Environment Structure
struct app_mesh_env_tag app_mesh_env;

uint8_t nvds_ctlset_value = 0;
uint8_t nvds_ctlset_value_len = 1;

ble_mesh_unprovisioned_data_t unprovisioned_data = 
{
    {{0xA8, 0x01, 0x71, 0x33, 0x02, 0x00, 0x00, 0x40, 0x43, 0xF8, 0x07, 0xDA, 0x78, 0x00, 0x00, 0x00}, 0x0000, 0x0},
    {3, 1, 1, 0, 0, 0, 0},
    {0xdd, 0x75, 0x50, 0x72, 0x3b, 0xbe, 0x09, 0xdd, 0xab, 0xc9, 0x60, 0xaa, 0x5e, 0x71, 0x72, 0xf7}
};
ble_mesh_composition_data_t composition_data = 
{
    .cid =  0x01A8,
    .feature = M_FEAT_RELAY_NODE_SUP | M_FEAT_PROXY_NODE_SUP | M_FEAT_FRIEND_NODE_SUP | M_FEAT_LOW_POWER_NODE_SUP | M_FEAT_MSG_API_SUP | M_FEAT_PB_GATT_SUP | M_FEAT_DYN_BCN_INTV_SUP,
    .pid = 0x0,
    .vid = 0x0001,
    .crpl = 2
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_mesh_init(void)
{
    // Reset the environment
    memset(&app_mesh_env, 0, sizeof(struct app_mesh_env_tag));

    mesh_stack_param_init();

    mal_adv_report_register(app_mesh_adv_report_cb);
#if (MESH_MEM_TB_BUF_DBG)
    mesh_mem_dbg_init();
#endif /* MESH_MEM_TB_BUF_DBG */
}

ble_err_t bk_ble_mesh_create_db(void)
{
    ble_err_t ret = ERR_SUCCESS;

    if (kernel_state_get(TASK_BLE_APP) == APPM_READY)
    {
        mesh_cfg_t *db_cfg;
        struct gapm_profile_task_add_cmd *req = KERNEL_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                TASK_BLE_GAPM, TASK_BLE_APP,
                                                gapm_profile_task_add_cmd, sizeof( mesh_cfg_t));
        // Fill message
        req->operation = GAPM_PROFILE_TASK_ADD;
        req->sec_lvl = 0;//PERM(SVC_AUTH, ENABLE);
        req->prf_task_id = TASK_BLE_ID_MESH;
        req->app_task = TASK_BLE_APP;
        req->start_hdl = 0; //req->start_hdl = 0; dynamically allocated

        // Set parameters
        db_cfg = ( mesh_cfg_t* ) req->param;

        db_cfg->prf_cfg.features = composition_data.feature;
        db_cfg->prf_cfg.cid = composition_data.cid;
        db_cfg->prf_cfg.pid = composition_data.pid;
        db_cfg->prf_cfg.vid = composition_data.vid;

#if (BLE_MESH_MDL)
        db_cfg->model_cfg.nb_replay = composition_data.crpl;
#endif // (BLE_MESH_MDL)

        kernel_state_set(TASK_BLE_APP, APPM_CREATE_DB);

        kernel_msg_send(req);
    }
    else
    {
        ret = ERR_CREATE_DB;
        MESH_APP_PRINT_ERR("MESH_DB CREATE FAILED\r\n");
    }

    return ret;
}

void bk_ble_mesh_set_cid(uint16_t cid)
{
    composition_data.cid = cid;
}

void bk_ble_mesh_set_crpl(uint16_t crpl)
{
    composition_data.crpl = crpl;
}

void bk_ble_mesh_set_feature(uint32_t feature)
{
    composition_data.feature = feature;
}

void bk_ble_mesh_set_pid(uint16_t pid)
{
    composition_data.pid = pid;
}

void bk_ble_mesh_set_vid(uint16_t vid)
{
    composition_data.vid = vid;
}

void bk_ble_mesh_set_dev_uuid(uint8_t *device_uuid)
{
    memcpy(unprovisioned_data.config.device_uuid, device_uuid, MESH_DEV_UUID_LEN);
}

void bk_ble_mesh_set_auth_data(uint8_t *auth_data)
{
    memcpy(unprovisioned_data.auth_value, auth_data, 16);
}

void bk_ble_mesh_set_capabilities(uint8_t number_of_elements,
                                             uint8_t public_key_type,
                                             uint8_t static_oob_type,
                                             uint8_t output_oob_size,
                                             uint16_t output_oob_action,
                                             uint8_t input_oob_size,
                                             uint16_t input_oob_action
                                             )
{
    unprovisioned_data.cap.number_of_elements = number_of_elements;
    unprovisioned_data.cap.public_key_type = public_key_type;
    unprovisioned_data.cap.static_oob_type = static_oob_type;
    unprovisioned_data.cap.output_oob_size = output_oob_size;
    unprovisioned_data.cap.output_oob_action = output_oob_action;
    unprovisioned_data.cap.input_oob_size = input_oob_size;
    unprovisioned_data.cap.input_oob_action = input_oob_action;
}

static void app_mesh_adv_report_cb(const struct adv_report* p_report)
{
    //MESH_APP_PRINT_INFO("evt_type = %x  :%s\n", p_report->evt_type, (p_report->evt_type == ADV_CONN_UNDIR) ? "ADV_CONN_UNDIR" :(p_report->evt_type == ADV_CONN_DIR)? "ADV_CONN_DIR" : (p_report->evt_type == ADV_DISC_UNDIR)? "ADV_DISC_UNDIR" : (p_report->evt_type == ADV_NONCONN_UNDIR)? "ADV_NONCONN_UNDIR": "Unknow");
    //MESH_APP_PRINT_INFO("adv_addr = %02x:%02x:%02x:%02x:%02x:%02x\n",
    //                    p_report->adv_addr.addr[0], p_report->adv_addr.addr[1],
    //                    p_report->adv_addr.addr[2], p_report->adv_addr.addr[3],
    //                    p_report->adv_addr.addr[4], p_report->adv_addr.addr[5]);
}

void user_models_bind_app_key(uint16_t app_key_id)
{
    m_lid_t app_key_lid;
    uint16_t status;

    status = m_tb_key_app_find(app_key_id, &app_key_lid); // 0 not change

    MESH_APP_PRINT_INFO("user_models_bind_app_key  app_key_lid = 0x%x,status:%x\n", app_key_lid, status);

    if (status == MESH_ERR_NO_ERROR)
    {
        for (int i = 2; i < m_tb_mio_get_nb_model(); i++)
        {
            status = m_tb_key_model_bind(app_key_lid, i);
            MESH_APP_PRINT_INFO("m_tb_key_model_bind  m_lid= 0x%x,status:%x\n", i, status);
            if (status == MESH_ERR_NO_ERROR)
            {
                m_tb_mio_bind(i);
            }
        }
    }
}


uint16_t user_models_subs_group_addr(m_lid_t m_lid, uint16_t addr)
{
    uint16_t status;

    status =  m_tb_mio_add_subscription(m_lid, addr);
    return status;
}

uint16_t user_models_publish_set(uint16_t app_key_id, m_lid_t m_lid, uint16_t addr)
{
    uint16_t status;
    m_lid_t app_key_lid;
    status = m_tb_key_app_find(app_key_id, &app_key_lid); // 0 not change
    status = m_tb_mio_set_publi_param(m_lid, addr, NULL,
                                      app_key_lid, M_TTL_DEFAULT, 54,
                                      0,
                                      0);
    return status;
}

void app_mesh_add_models_server(void)
{
    MESH_APP_PRINT_INFO("app_mesh_add_mesh_models_server\r\n");
}

extern void m_link_open_ack_dis(void);
__STATIC void app_unprov_adv_cb_timerout(void *p_env)
{
    MESH_APP_PRINT_INFO("%s end!!!\r\n", __func__);

    m_bcn_stop_tx_unprov_bcn();
#if (BLE_MESH_GATT_PROV)
    m_prov_bearer_gatt_stop();

    m_prov_bearer_scan_stop();
#endif
    rwip_prevent_sleep_clear(BK_MESH_ACTIVE);

#if (UNPROV_TIMEOUT_ADV)
    rwip_prevent_sleep_set(BK_MESH_ACTIVE);
    m_api_prov_param_cfm_t *cfm = KERNEL_MSG_ALLOC(MESH_API_PROV_PARAM_CFM, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_prov_param_cfm);

    cfm->nb_elt = unprovisioned_data.cap.number_of_elements;
    cfm->pub_key_oob = unprovisioned_data.cap.public_key_type;
    cfm->static_oob = unprovisioned_data.cap.static_oob_type;
    cfm->out_oob_size = unprovisioned_data.cap.output_oob_size;
    cfm->out_oob_action = unprovisioned_data.cap.output_oob_action;
    cfm->in_oob_size = unprovisioned_data.cap.input_oob_size;
    cfm->in_oob_action = unprovisioned_data.cap.input_oob_action;
    cfm->oob_info = unprovisioned_data.config.oob_information;
    cfm->uri_hash = unprovisioned_data.config.uri_hash;
    cfm->info = 0;
    memcpy(cfm->dev_uuid,unprovisioned_data.config.device_uuid,16);

    kernel_msg_send(cfm);
    m_link_open_ack_dis();
    m_stack_param.m_adv_interval = 200;
    m_stack_param.m_bcn_default_unprov_bcn_intv_ms = 60000;
#endif
}

void app_unprov_adv_timeout_set(uint32_t timer)
{
    MESH_APP_PRINT_INFO("app_unprov_adv_timeout_set %d\r\n", timer);
    if (timer)
    {
        app_mesh_env.timer_upd.cb = app_unprov_adv_cb_timerout;
        app_mesh_env.timer_upd.period = timer;
        mesh_tb_timer_set(&app_mesh_env.timer_upd, timer);
        rwip_prevent_sleep_set(BK_MESH_ACTIVE);
    }
    else
    {
        mesh_tb_timer_clear(&app_mesh_env.timer_upd);
    }
}

__STATIC void app_model_bind_success_cb(void *p_env)
{
    MESH_APP_PRINT_INFO("%s end!!!\r\n", __func__);
}

__STATIC void app_model_bind_fail_cb(void *p_env)
{
    MESH_APP_PRINT_INFO("%s end!!!\r\n", __func__);
}

/**
 ****************************************************************************************
 * @brief
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_BLE_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_mesh_msg_dflt_handler(kernel_msg_id_t const msgid,
                                     void const *param,
                                     kernel_task_id_t const dest_id,
                                     kernel_task_id_t const src_id)
{
    // Drop the message
    app_models_msg_pro_handler(msgid, param, dest_id, src_id);


    return (KERNEL_MSG_CONSUMED);
}

static int app_mesh_msg_model_app_bind_handler(kernel_msg_id_t const msgid,
        struct m_api_model_app_bind_ind const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    // Drop the message
    MESH_APP_PRINT_INFO("%s\n", __func__);

    MESH_APP_PRINT_INFO("param->status = 0x%x\n", param->status);
    MESH_APP_PRINT_INFO("model_id = 0x%x\r\n", param->model_id);

    return (KERNEL_MSG_CONSUMED);
}

extern uint16_t quick_onoff_count;
static int app_mesh_msg_node_reset_handler(kernel_msg_id_t const msgid,
        void const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_msg_node_reset_handler msgid = 0x%x\n", msgid);

    return (KERNEL_MSG_CONSUMED);
}

static int app_mesh_msg_key_ind_handler(kernel_msg_id_t const msgid,
                                        struct m_tb_key const *param,
                                        kernel_task_id_t const dest_id,
                                        kernel_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_msg_key_ind_handler msgid = 0x%x\n", msgid);

    m_tb_key_t *key  = (m_tb_key_t *)param;
    uint8_t r_key[MESH_KEY_LEN];
    mem_rcopy(r_key, key->key, MESH_KEY_LEN);
    switch (key->key_type)
    {
        case M_TB_KEY_DEVICE:
        {
            MESH_APP_PRINT_INFO("******************DEVICE key************************\n");
            MESH_APP_PRINT_INFO("%s\n", mesh_buffer_to_hex(r_key, MESH_KEY_LEN));
        } break;
        case M_TB_KEY_NETWORK:
        {
            MESH_APP_PRINT_INFO("******************NETWORK key************************\n");
            MESH_APP_PRINT_INFO("%s\n", mesh_buffer_to_hex(r_key, MESH_KEY_LEN));
        } break;
        case M_TB_KEY_APPLICATION:
        {
            MESH_APP_PRINT_INFO("******************APPLICATION key********************\n");
            m_tb_key_app_t *app_key = (m_tb_key_app_t *)param;
            MESH_APP_PRINT_INFO("%s\n", mesh_buffer_to_hex(r_key, MESH_KEY_LEN));

            if(ble_mesh_event_cb)
            {
                ble_mesh_event_cb(BLE_MESH_APP_KEY_ADD_DONE, NULL);
            }

	        m_tb_state_set_prov_state(M_TB_STATE_PROV_STATE_PROV);
	        m_tb_state_set_beacon_state(M_CONF_BCN_STATE_BROAD);
			m_bcn_stop_tx_unprov_bcn();
#if (BLE_MESH_GATT_PROV)
    		m_prov_bearer_gatt_stop();
#endif
            m_tb_store_config(3); 
            
        } break;

        default:
            break;
    }

    return (KERNEL_MSG_CONSUMED);
}

static int app_mesh_api_cmp_handler(kernel_msg_id_t const msgid,
                                    struct m_api_cmp_evt const *param,
                                    kernel_task_id_t const dest_id,
                                    kernel_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_api_cmp_handler,cmd_code:0x%x,stu:%x\n", param->cmd_code, param->status);
    switch (param->cmd_code)
    {

        case M_API_STORAGE_LOAD: //0x50
        {
            app_mesh_enable();
        } break;

        case M_API_ENABLE: //0x0
        {
            MESH_APP_PRINT_INFO("M_API_ENABLE param->status %x\n", param->status);
            if(ble_mesh_event_cb)
            {
                ble_mesh_event_cb(BLE_MESH_INIT_DONE, NULL);
            }
        } break;

        case M_API_DISABLE: //0x1
        {
            // Check that should store the key info to nvs or not.
            //m_tb_store_nvs_after_stop_scan();
            //app_mesh_enable();
            MESH_APP_PRINT_INFO("M_API_DISABLE param->status %x\n", param->status);
        }

        default:break;
    }

    return (KERNEL_MSG_CONSUMED);
}
static int app_mesh_model_api_cmp_handler(kernel_msg_id_t const msgid,
        struct mm_api_cmp_evt const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_model_api_cmp_handler,cmd_code:0x%x,stu:%x\n", param->cmd_code, param->status);
    switch (param->cmd_code)
    {
        case MM_API_REGISTER_SERVER://0x0
        {
            if (param->status == MESH_ERR_NO_ERROR)
            {
                MESH_APP_PRINT_INFO("model register success.\n");
            }

        } break;

        case MM_API_SRV_SET://200
        {

        } break;

        default:
            break;
    }

    return (KERNEL_MSG_CONSUMED);
}

static int app_mesh_api_prov_auth_data_req_ind_handler(kernel_msg_id_t const msgid,
        struct m_api_prov_auth_data_req_ind const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_api_prov_auth_data_req_ind_handler\n");
    MESH_APP_PRINT_INFO("auth_method:%x,auth_action:%x,auth_size:%x\n", param->auth_method, param->auth_action, param->auth_size);

    m_api_prov_auth_data_cfm_t *cfm = KERNEL_MSG_ALLOC_DYN(MESH_API_PROV_AUTH_DATA_CFM, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_prov_auth_data_cfm, 16);

    cfm->accept = 1;
    cfm->auth_size = param->auth_size;
    
    memcpy(cfm->auth_data, unprovisioned_data.auth_value, param->auth_size);

    for (int i =0 ; i < 16; i++)
    {
        MESH_APP_PRINT_INFO("cfm->auth_data[%d] = 0x%02x\r\n", i, cfm->auth_data[i]);
    }
    MESH_APP_PRINT_INFO("cfm->auth_size = 0x%02x\r\n",cfm->auth_size);

    kernel_msg_send(cfm);

    return (KERNEL_MSG_CONSUMED);
}

static int app_mesh_api_prov_param_req_ind_handler(kernel_msg_id_t const msgid,
        void const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_api_prov_param_req_ind_handler.\n");
    //sean add
    m_api_prov_param_cfm_t *cfm = KERNEL_MSG_ALLOC(MESH_API_PROV_PARAM_CFM, prf_get_task_from_id(TASK_BLE_ID_MESH), TASK_BLE_APP, m_api_prov_param_cfm);

    cfm->nb_elt = unprovisioned_data.cap.number_of_elements;
    cfm->pub_key_oob = unprovisioned_data.cap.public_key_type;
    cfm->static_oob = unprovisioned_data.cap.static_oob_type;
    cfm->out_oob_size = unprovisioned_data.cap.output_oob_size;
    cfm->out_oob_action = unprovisioned_data.cap.output_oob_action;
    cfm->in_oob_size = unprovisioned_data.cap.input_oob_size;
    cfm->in_oob_action = unprovisioned_data.cap.input_oob_action;
    cfm->oob_info = unprovisioned_data.config.oob_information;
    cfm->uri_hash = unprovisioned_data.config.uri_hash;
    cfm->info = 0;
    memcpy(cfm->dev_uuid,unprovisioned_data.config.device_uuid,16);

    kernel_msg_send(cfm);

    return (KERNEL_MSG_CONSUMED);
}

static int app_mesh_api_prov_attention_update_ind_handler(kernel_msg_id_t const msgid,
        struct m_api_attention_update_ind const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s\n", __func__);

    MESH_APP_PRINT_INFO("param->attention_state :%d\n", param->attention_state);

    return (KERNEL_MSG_CONSUMED);
}

extern mesh_stack_param_int_t m_stack_param;
static int app_mesh_api_prov_state_ind_handler(kernel_msg_id_t const msgid,
        struct m_api_prov_state_ind const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s\n", __func__);

    MESH_APP_PRINT_INFO("state :%d, status:%x\n", param->state, param->status);

    if (param->state == M_PROV_STARTED)
    {
        if(ble_mesh_event_cb)
        {
            ble_mesh_event_cb(BLE_MESH_PROV_INVITE, NULL);
        }
        MESH_APP_PRINT_INFO("test: notify midea ble mesh sdk msg prov-invi prov-start!\r\n");
    }
    else if (param->state == M_PROV_FAILED)
    {
        MESH_APP_PRINT_INFO("light_prov_fail\n");
    }
    else if (param->state == M_PROV_SUCCEED)
    {
        m_tb_store_config(14);
        m_tb_state_set_relay_state(1, 1);
        app_unprov_adv_timeout_set(0);

        if(ble_mesh_event_cb)
        {
            ble_mesh_event_cb(BLE_MESH_PROV_DONE, NULL);
        }
    }
    else if(param->state == M_PROV_ADVCTL) ///for config adv
    {

    }

    return (KERNEL_MSG_CONSUMED);
}

static int app_mesh_api_prov_start_ind_handler(kernel_msg_id_t const msgid,
		struct m_api_prov_start_ind const *param,
		kernel_task_id_t const dest_id,
		kernel_task_id_t const src_id)
{
    struct bk_prov_start_ind prov_ind;

    prov_ind.algorithm = param->algorithm;
    prov_ind.auth_action = param->auth_action;
    prov_ind.auth_method = param->auth_method;
    prov_ind.auth_size = param->auth_size;
    prov_ind.pub_key = param->pub_key;
    
    if(ble_mesh_event_cb)
    {
        ble_mesh_event_cb(BLE_MESH_PROV_START, (void *)(&prov_ind));
    }

    MESH_APP_PRINT_INFO("+++++%s: \n+++++", __func__);
	MESH_APP_PRINT_INFO("type= %d, algorithm = %d, pub_key = %d, auth_method= %d, auth_actionio= %d,auth_size =%d\n",
						param->type,param->algorithm, param->pub_key, 
					    param->auth_method, param->auth_action, param->auth_size);

    return (KERNEL_MSG_CONSUMED);
}

static int app_mesh_api_compo_data_ind_handler(
                    kernel_msg_id_t const msgid,
                    void const *param,
                    kernel_task_id_t const dest_id,
                    kernel_task_id_t const src_id)
{
    // m_tb_store_config(5);
    return (KERNEL_MSG_CONSUMED);
}

/// Default State handlers definition
const struct kernel_msg_handler app_mesh_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KERNEL_MSG_DEFAULT_HANDLER,                    (kernel_msg_func_t)app_mesh_msg_dflt_handler},

    {MESH_API_MODEL_APP_BIND_IND,               (kernel_msg_func_t)app_mesh_msg_model_app_bind_handler},

    {MESH_API_NODE_RESET_IND,                   (kernel_msg_func_t)app_mesh_msg_node_reset_handler},

    {MESH_API_KEY_IND,                          (kernel_msg_func_t)app_mesh_msg_key_ind_handler},

    {MESH_API_CMP_EVT,                          (kernel_msg_func_t)app_mesh_api_cmp_handler},

    {MESH_MDL_API_CMP_EVT,                      (kernel_msg_func_t)app_mesh_model_api_cmp_handler},

    {MESH_API_PROV_AUTH_DATA_REQ_IND,           (kernel_msg_func_t)app_mesh_api_prov_auth_data_req_ind_handler},

    {MESH_API_PROV_PARAM_REQ_IND,              (kernel_msg_func_t)app_mesh_api_prov_param_req_ind_handler},

    {MESH_API_ATTENTION_UPDATE_IND,            (kernel_msg_func_t)app_mesh_api_prov_attention_update_ind_handler},

    {MESH_API_PROV_STATE_IND,                   (kernel_msg_func_t)app_mesh_api_prov_state_ind_handler},

    {MESH_API_COMPO_DATA_REQ_IND,               (kernel_msg_func_t)app_mesh_api_compo_data_ind_handler},

	{MESH_API_PROV_START_IND,                   (kernel_msg_func_t)app_mesh_api_prov_start_ind_handler},
};

const struct kernel_state_handler app_mesh_table_handler =
{&app_mesh_msg_handler_list[0], (sizeof(app_mesh_msg_handler_list)/sizeof(struct kernel_msg_handler))};

