/**
 ****************************************************************************************
 *
 * @file app_sec.c
 *
 * @brief Application Security Entry Point
 *
 * Copyright (C) RivieraWaves 2009-2015
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

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_APP_SEC)

#include <string.h>
#include "common_math.h"
#include "app_ble.h"

#include "app_sec.h"        // Application Security API Definition
#include "app_task.h"       // Application Manager API Definitionde 
#include "kernel_timer.h"
#include "common_utils.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#define SEC_TK_PASSKEY      123456

/// Application Security Environment Structure
struct app_sec_env_tag app_sec_env;
extern struct app_env_tag app_ble_env;
extern int bk_rand();

bool app_sec_peer_pairing_recv(void);

bool app_sec_peer_encrypt_recv(void);


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_sec_init(void)
{
	#if (NVDS_SUPPORT)
	uint8_t length = NVDS_LEN_PERIPH_BONDED;
	if (nvds_get(NVDS_TAG_PERIPH_BONDED, &length, (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
	{
		// If read value is invalid, set status to not bonded
		if ((app_sec_env.bonded != true) && (app_sec_env.bonded != false))
		{
			app_sec_env.bonded = false;
		}
	}
	app_sec_env.peer_pairing_recv = false;
	app_sec_env.peer_encrypt_recv = false;
	#else
	app_sec_env.bonded = false;
	app_sec_env.peer_pairing_recv = false;
	app_sec_env.peer_encrypt_recv = false;
	app_sec_env.passkey = SEC_TK_PASSKEY;
	#endif

	app_sec_env.sec_notice_cb = NULL;
	app_sec_env.pairing_param.sec_req = GAP_NO_SEC;

	app_sec_env.pairing_param.iocap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
	app_sec_env.pairing_param.auth = GAP_AUTH_REQ_NO_MITM_NO_BOND;
	app_sec_env.pairing_param.ikey_dist = GAP_KDIST_NONE;
	app_sec_env.pairing_param.rkey_dist = GAP_KDIST_NONE;
}

uint8_t app_sec_config(struct app_pairing_cfg *param, sec_notice_cb_t func)
{
	uint8_t status = APP_SEC_ERROR_NO_ERROR;

	if (param->sec_req == GAP_NO_SEC) {
		status = APP_SEC_ERROR_PARAM_INVALID;
	} else if (param->sec_req == GAP_SEC1_NOAUTH_PAIR_ENC) {
		if (param->auth & GAP_AUTH_MITM) {
			status = APP_SEC_ERROR_PARAM_INVALID;
		}
	} else if (param->sec_req == GAP_SEC1_AUTH_PAIR_ENC) {
		if (!(param->auth & GAP_AUTH_MITM)) {
			status = APP_SEC_ERROR_PARAM_INVALID;
		}
	#if BLE_APP_SEC_CON
	} else if (param->sec_req == GAP_SEC1_SEC_CON_PAIR_ENC){
		if (!(param->auth & GAP_AUTH_MITM) || !(param->auth & GAP_AUTH_SEC_CON)) {
			status = APP_SEC_ERROR_PARAM_INVALID;
		}
	#endif
	} else {
		//TODO LE security mode 2
		status = APP_SEC_ERROR_PARAM_UNSUPPORT;
	}

	if (status) {
		BLE_ASSERT_WARN(0, param->sec_req, param->auth);
		return status;
	}

	app_sec_env.sec_notice_cb = func;
	app_sec_env.pairing_param.sec_req = param->sec_req;

	app_sec_env.pairing_param.iocap = param->iocap;
	app_sec_env.pairing_param.auth = param->auth;
	app_sec_env.pairing_param.ikey_dist = param->ikey_dist;
	app_sec_env.pairing_param.rkey_dist = param->rkey_dist;
	return status;
}

bool app_sec_get_bond_status(void)
{
	return app_sec_env.bonded;
}

bool app_sec_peer_pairing_recv(void)
{
	return app_sec_env.peer_pairing_recv;
}

bool app_sec_set_tk_passkey(uint32_t passkey)
{
	if( passkey > 999999) {
		BLE_ASSERT_WARN(0, passkey, 0);
		return false;
	}
	app_sec_env.passkey = passkey;
	return true;
}

bool app_sec_peer_encrypt_recv(void)
{
	return app_sec_env.peer_encrypt_recv;
}


void app_sec_remove_bond(void)
{
    #if (NVDS_SUPPORT)
    // Check if we are well bonded
    if (app_sec_env.bonded == true)
    {
        // Update the environment variable
        app_sec_env.bonded = false;

        if (nvds_put(NVDS_TAG_PERIPH_BONDED, NVDS_LEN_PERIPH_BONDED,
                     (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
        {
            BLE_ASSERT_ERR(0);
        }
    }
    #endif 
    app_sec_env.bonded = false;
    app_sec_env.peer_pairing_recv = false;
    app_sec_env.peer_encrypt_recv = false;
}

void app_sec_send_security_req(uint8_t conidx)
{
	uint8_t conhdl = app_ble_env.connections[conidx].conhdl;
	uint8_t role = app_ble_env.connections[conidx].role;

	if (role == 0) {
		/* command supported only by slave of the connection. */
		bk_printf("app_sec_send_security_req Failed\r\n");
		return;
	}

	if(!app_sec_peer_encrypt_recv() && !app_sec_peer_pairing_recv())
	{
		struct gapc_security_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_SECURITY_CMD,
													KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl), TASK_BLE_APP,
													gapc_security_cmd);

		cmd->operation = GAPC_SECURITY_REQ;

		cmd->auth      = app_sec_env.pairing_param.auth;

		// Send the message
		kernel_msg_send(cmd);
	}
}

void app_sec_send_bond_cmd(uint8_t conidx)
{
    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;
    uint8_t role = app_ble_env.connections[conidx].role;

    if (role == 1) {
        /* command supported only by master of the connection. */
        bk_printf("app_sec_send_bond_cmd Failed\r\n");
        return;
    }

    struct gapc_bond_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_BOND_CMD,
                                                    KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl), TASK_BLE_APP,
                                                    gapc_bond_cmd);

    cmd->operation = GAPC_BOND;

    cmd->pairing.auth      = app_sec_env.pairing_param.auth;
    cmd->pairing.iocap     = app_sec_env.pairing_param.iocap;
    cmd->pairing.oob       = GAP_OOB_AUTH_DATA_NOT_PRESENT;
    cmd->pairing.key_size  = 16;
    cmd->pairing.ikey_dist = app_sec_env.pairing_param.ikey_dist;
    cmd->pairing.rkey_dist = app_sec_env.pairing_param.rkey_dist;

    cmd->pairing.sec_req   = app_sec_env.pairing_param.sec_req;

    // Send the message
    kernel_msg_send(cmd);
}

void app_sec_send_encryption_cmd(uint8_t conidx)
{
    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;
    uint8_t role = app_ble_env.connections[conidx].role;

    if (role == 1) {
        /* command supported only by master of the connection. */
        bk_printf("app_sec_send_bond_cmd Failed\r\n");
        return;
    }
    struct gapc_encrypt_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_ENCRYPT_CMD,
                                                KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl), TASK_BLE_APP,
                                                gapc_encrypt_cmd);

    cmd->operation = GAPC_ENCRYPT;
    memcpy(&cmd->ltk,&app_sec_env.peer_ltk,sizeof(struct gapc_ltk));
    // Send the message
    kernel_msg_send(cmd);

    bk_printf("app_sec_send_encryption_cmd\r\n");
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

static int gapc_bond_req_ind_handler(kernel_msg_id_t const msgid,
                                     struct gapc_bond_req_ind const *param,
                                     kernel_task_id_t const dest_id,
                                     kernel_task_id_t const src_id)
{
    // Prepare the GAPC_BOND_CFM message
    struct gapc_bond_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_BOND_CFM,
                                             src_id, TASK_BLE_APP,
                                             gapc_bond_cfm);

    switch (param->request)
    {
        case (GAPC_PAIRING_REQ):
        {
            cfm->request = GAPC_PAIRING_RSP;

            cfm->accept  = false;

            // Check if we are already bonded (Only one bonded connection is supported)
            if (!app_sec_env.bonded && app_sec_env.pairing_param.sec_req != GAP_NO_SEC)
            {
                cfm->accept  = true;
                app_sec_env.peer_pairing_recv = true;

                cfm->data.pairing_feat.iocap     = app_sec_env.pairing_param.iocap;
                cfm->data.pairing_feat.oob       = GAP_OOB_AUTH_DATA_NOT_PRESENT;
                cfm->data.pairing_feat.auth      = app_sec_env.pairing_param.auth;
                cfm->data.pairing_feat.key_size  = 16;
                cfm->data.pairing_feat.ikey_dist = app_sec_env.pairing_param.ikey_dist;
                cfm->data.pairing_feat.rkey_dist = app_sec_env.pairing_param.rkey_dist;

                cfm->data.pairing_feat.sec_req   = app_sec_env.pairing_param.sec_req;
            }
        } break;

        case (GAPC_LTK_EXCH):
        {
            uint8_t counter;
            cfm->accept  = true;
            cfm->request = GAPC_LTK_EXCH;

            // Generate all the values
            cfm->data.ltk.ediv = (uint16_t)bk_rand();

            for (counter = 0; counter < RAND_NB_LEN; counter++)
            {
                cfm->data.ltk.ltk.key[counter]    = (uint8_t)bk_rand();
                cfm->data.ltk.randnb.nb[counter] = (uint8_t)bk_rand();
            }

            for (counter = RAND_NB_LEN; counter < KEY_LEN; counter++)
            {
                cfm->data.ltk.ltk.key[counter]    = (uint8_t)bk_rand();
            }

            memcpy(&app_sec_env.ltk,&cfm->data.ltk,sizeof(struct gapc_ltk));
            #if (NVDS_SUPPORT)
            // Store the generated value in NVDS
            if (nvds_put(NVDS_TAG_LTK, NVDS_LEN_LTK,
                         (uint8_t *)&cfm->data.ltk) != NVDS_OK)
            {
                BLE_ASSERT_ERR(0);
            }
            #endif// #if (NVDS_SUPPORT)
        } break;


        case (GAPC_IRK_EXCH):
        {
            cfm->accept  = true;
            cfm->request = GAPC_IRK_EXCH;

            // Load IRK
            memcpy(cfm->data.irk.irk.key,app_ble_env.loc_irk, KEY_LEN);
            // load device address
            cfm->data.irk.addr.addr_type = ADDR_PUBLIC;
            memcpy(cfm->data.irk.addr.addr,(uint8_t *)&common_default_bdaddr,BD_ADDR_LEN);
        } break;

        case (GAPC_TK_EXCH):
        {
            uint32_t passkey = 0;
            cfm->accept  = true;
            cfm->request = GAPC_TK_EXCH;
            passkey = app_sec_env.passkey;
            memset(&cfm->data.tk,0,sizeof(struct gap_sec_key));
            memcpy(&cfm->data.tk.key[0],(uint8_t *)&passkey,sizeof(uint32_t));
            bk_printf("SEC_TK_PASSKEY:%d\r\n",passkey);
        } break;
        case GAPC_NC_EXCH:
        {
        } break;
        default:
        {
            BLE_ASSERT_ERR(0);
        } break;
    }

    // Send the message
    kernel_msg_send(cfm);

    return (KERNEL_MSG_CONSUMED);
}


static int gapc_bond_ind_handler(kernel_msg_id_t const msgid,
                                 struct gapc_bond_ind const *param,
                                 kernel_task_id_t const dest_id,
                                 kernel_task_id_t const src_id)
{
    switch (param->info)
    {
        case (GAPC_PAIRING_SUCCEED):
        {
            // Update the bonding status in the environment
            if (param->data.pairing.level & GAP_PAIRING_BOND_PRESENT_BIT) {
                app_sec_env.bonded = true;
            }
            if (param->data.pairing.level == GAP_PAIRING_BOND_SECURE_CON) {
                memcpy(&app_sec_env.ltk, &app_sec_env.peer_ltk, sizeof(struct gapc_ltk));
            }
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_PAIRING_SUCCEED, NULL);
            }
        } break;

        case (GAPC_REPEATED_ATTEMPT):
        {
            bk_printf("[warning]GAPC_REPEATED_ATTEMPT\r\n");
        } break;

        case (GAPC_IRK_EXCH):
        {
            memcpy(&app_sec_env.peer_irk,&param->data.irk,sizeof(struct gapc_irk));
            for(int i = 0;i<sizeof(struct gap_sec_key);i++)
            {
                bk_printf("irk.key[%d]  = %x\r\n",i,param->data.irk.irk.key[i]);
            }

            for(int i = 0;i<sizeof(struct bd_addr);i++)
            {
                bk_printf("addr.addr[%d]  = %x\r\n",i,param->data.irk.addr.addr[i]);
            }

        } break;

        case (GAPC_PAIRING_FAILED):
        {
            app_sec_env.peer_pairing_recv = false;
            app_sec_env.peer_encrypt_recv = false;
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_PAIRING_FAILED, NULL);
            }
        } break;

        case (GAPC_LTK_EXCH):
        {
            bk_printf("Long Term Key exchange ok\r\n");
            bk_printf("Peer EDIV:0x%x\r\n",param->data.ltk.ediv);
            bk_printf("Peer key_size:%d\r\n",param->data.ltk.key_size);
            bk_printf("Peer randnb:%x:%x:%x:%x:%x:%x:%x:%x\r\n",param->data.ltk.randnb.nb[0],param->data.ltk.randnb.nb[1],param->data.ltk.randnb.nb[2],
            param->data.ltk.randnb.nb[3],param->data.ltk.randnb.nb[4],param->data.ltk.randnb.nb[5],param->data.ltk.randnb.nb[6],param->data.ltk.randnb.nb[7]);
            bk_printf("Peer LTK:");
            for(int i = 0;i<sizeof(struct gap_sec_key);i++)
            {
                bk_printf("%x:",param->data.ltk.ltk.key[i]);
            }
            bk_printf("\r\n");
            memcpy(&app_sec_env.peer_ltk,&param->data.ltk,sizeof(struct gapc_ltk));
        } break;
        case (GAPC_TK_EXCH):
        {
            //
        }break;
        default:
        {
            bk_printf("gapc_bond_ind_handler:%d\r\n",param->info);
            BLE_ASSERT_ERR(0);
        } break;
    }

    return (KERNEL_MSG_CONSUMED);

}

static int gapc_encrypt_req_ind_handler(kernel_msg_id_t const msgid,
                                        struct gapc_encrypt_req_ind const *param,
                                        kernel_task_id_t const dest_id,
                                        kernel_task_id_t const src_id)
{
    bk_printf("%s \r\n",__func__);
    app_sec_env.peer_encrypt_recv = true;

    // Prepare the GAPC_ENCRYPT_CFM message
    struct gapc_encrypt_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_ENCRYPT_CFM,
                                     src_id, TASK_BLE_APP,
                                     gapc_encrypt_cfm);

    cfm->found    = false;

    if (app_sec_env.bonded) {

        // Check if the provided EDIV and Rand Nb values match with the stored values
        if ((param->ediv == app_sec_env.ltk.ediv) &&
            !memcmp(&param->rand_nb.nb[0], &app_sec_env.ltk.randnb.nb[0], sizeof(struct rand_nb))) {
            cfm->found    = true;
            cfm->key_size = GAP_KEY_LEN;
            memcpy(&cfm->ltk, &app_sec_env.ltk.ltk, sizeof(struct gap_sec_key));
        }
    }
    // Send the message
    kernel_msg_send(cfm);

    return (KERNEL_MSG_CONSUMED);
}


static int gapc_encrypt_ind_handler(kernel_msg_id_t const msgid,
                                    struct gapc_encrypt_ind const *param,
                                    kernel_task_id_t const dest_id,
                                    kernel_task_id_t const src_id)
{
    // encryption/ re-encryption succeeded
    bk_printf("[%s]\r\n",__func__);

    return (KERNEL_MSG_CONSUMED);
}

static int gapc_security_ind_handler(kernel_msg_id_t const msgid,
                                    struct gapc_security_ind const *param,
                                    kernel_task_id_t const dest_id,
                                    kernel_task_id_t const src_id)
{
    bk_printf("[%s]Peer auth:%d\r\n",__func__,param->auth);
    uint8_t conn_idx = app_ble_find_conn_idx_handle(KERNEL_IDX_GET(src_id));
    app_sec_send_bond_cmd(conn_idx);

    return (KERNEL_MSG_CONSUMED);
}

static int app_sec_msg_dflt_handler(kernel_msg_id_t const msgid,
                                    void *param,
                                    kernel_task_id_t const dest_id,
                                    kernel_task_id_t const src_id)
{
    // Drop the message
    bk_printf("[%s]msgid:0x%x,src_id:0x%x\r\n",__func__,msgid,src_id);
    return (KERNEL_MSG_CONSUMED);
}

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct kernel_msg_handler app_sec_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KERNEL_MSG_DEFAULT_HANDLER,  (kernel_msg_func_t)app_sec_msg_dflt_handler},

    {GAPC_BOND_REQ_IND,       (kernel_msg_func_t)gapc_bond_req_ind_handler},
    {GAPC_BOND_IND,           (kernel_msg_func_t)gapc_bond_ind_handler},

    {GAPC_ENCRYPT_REQ_IND,    (kernel_msg_func_t)gapc_encrypt_req_ind_handler},
    {GAPC_ENCRYPT_IND,        (kernel_msg_func_t)gapc_encrypt_ind_handler},
    {GAPC_SECURITY_IND,       (kernel_msg_func_t)gapc_security_ind_handler},
};

const struct app_subtask_handlers app_sec_handlers =
    {&app_sec_msg_handler_list[0], (sizeof(app_sec_msg_handler_list)/sizeof(struct kernel_msg_handler))};

#endif //(BLE_APP_SEC)

/// @} APP
