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

/// Application Security Environment Structure
struct app_sec_env_tag app_sec_env;
extern struct app_env_tag app_ble_env;
extern int bk_rand();
static const uint32_t crc32_table[] =
{
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint32_t app_sec_crc32(uint32_t crc, const void *buf, size_t size)
{
    const uint8_t *p;

    p = (const uint8_t *)buf;
    crc = crc ^ ~0U;

    while (size--) {
        crc = crc32_table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
    }

    return crc ^ ~0U;
}

void app_sec_init(void)
{
    #if (NVDS_SUPPORT)
    uint8_t length = NVDS_LEN_PERIPH_BONDED;
    if (nvds_get(NVDS_TAG_PERIPH_BONDED, &length, (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
    {
        // If read value is invalid, set status to not bonded
        if (app_sec_env.bonded > MAX_BOND_NUM_MASK)
        {
            app_sec_env.bonded = 0;
        }
    }
    #elif (APP_SEC_BOND_STORE)
    uint32_t flash_addr_ptr;
    uint16_t bond_info_size = sizeof(bond_info_t);
    uint32_t crc;
    uint8_t crc_fail_count;
    uint8_t i;
    bool remove_flag = false;

    app_sec_env.flash_bond_ptr = bk_flash_get_info(BK_PARTITION_BLE_BONDING_FLASH);
    app_sec_env.bonded = 0;

    for (i = 0; i < MAX_BOND_NUM; i++) {
        crc_fail_count = 0;
        flash_addr_ptr = app_sec_env.flash_bond_ptr->partition_start_addr + bond_info_size * i;

        while(crc_fail_count < MAX_CRC_FAIL_TIMES) {
            flash_read((char *)(&app_sec_env.bond_info[i]), sizeof(bond_info_t), flash_addr_ptr);

            if (app_sec_env.bond_info[i].crc == CRC_DEFAULT_VALUE) {
                break;
            } else {
                crc = CRC_DEFAULT_VALUE;
                crc = app_sec_crc32(crc, &app_sec_env.bond_info[i], bond_info_size - 4);
                if (crc == app_sec_env.bond_info[i].crc) {
                    app_sec_env.bonded |= (1 << i);
                    break;
                }
                crc_fail_count++;
            }
        }

        if (crc_fail_count == MAX_CRC_FAIL_TIMES) {
            bk_printf("CRC fail, remove bonded info, idx=%d\r\n", i);
            remove_flag = true;
        }
    }

    if (remove_flag) {
        bk_flash_enable_security(FLASH_PROTECT_NONE);
        flash_ctrl(CMD_FLASH_ERASE_SECTOR, &app_sec_env.flash_bond_ptr->partition_start_addr);
        bk_flash_enable_security(FLASH_UNPROTECT_LAST_BLOCK);

        for (i = 0; i < MAX_BOND_NUM; i++) {
            if ((app_sec_env.bonded >> i) & 1) {
                app_sec_env.flash_write_idx[i] = true;
            } else {
                app_sec_env.flash_write_idx[i] = false;
            }
        }

        // update bonding info in flash
        app_sec_store_bond_info_in_flash();
    }
    #else
    app_sec_env.bonded = 0;
    #endif

    app_sec_env.sec_notice_cb = NULL;
    app_sec_env.pairing_param.sec_req = GAP_NO_SEC;

    app_sec_env.pairing_param.iocap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
    app_sec_env.pairing_param.auth = GAP_AUTH_REQ_NO_MITM_NO_BOND;
    app_sec_env.pairing_param.ikey_dist = GAP_KDIST_NONE;
    app_sec_env.pairing_param.rkey_dist = GAP_KDIST_NONE;
}

sec_err_t app_sec_config(struct app_pairing_cfg *param, sec_notice_cb_t func)
{
    sec_err_t status = APP_SEC_ERROR_NO_ERROR;

    app_sec_env.sec_notice_cb = func;
    app_sec_env.pairing_param.sec_req = param->sec_req;

    app_sec_env.pairing_param.iocap = param->iocap;
    app_sec_env.pairing_param.auth = param->auth;
    app_sec_env.pairing_param.ikey_dist = param->ikey_dist;
    app_sec_env.pairing_param.rkey_dist = param->rkey_dist;

    return status;
}

#if (APP_SEC_BOND_STORE)
sec_err_t app_sec_store_bond_info_in_flash(void)
{
    sec_err_t ret = APP_SEC_ERROR_NO_ERROR;
    bond_info_t check_info;
    uint32_t flash_addr_ptr;
    uint32_t crc;
    uint8_t crc_fail_count;
    uint8_t idx;

    for (idx = 0; idx < MAX_BOND_NUM; idx++) {
        if (app_sec_env.flash_write_idx[idx] == false) {
            continue;
        } else {
            app_sec_env.flash_write_idx[idx] = false;
        }

        crc_fail_count = 0;
        flash_addr_ptr = app_sec_env.flash_bond_ptr->partition_start_addr + idx * sizeof(bond_info_t);

        while(crc_fail_count < MAX_CRC_FAIL_TIMES) {
            bk_flash_enable_security(FLASH_PROTECT_NONE);
            flash_write((char *)(&app_sec_env.bond_info[idx]), sizeof(bond_info_t), flash_addr_ptr);
            bk_flash_enable_security(FLASH_UNPROTECT_LAST_BLOCK);

            //read & check crc
            flash_read((char *)(&check_info), sizeof(bond_info_t), flash_addr_ptr);
            crc = CRC_DEFAULT_VALUE;
            crc = app_sec_crc32(crc, &check_info, sizeof(bond_info_t) - 4);
            if (crc == check_info.crc) {
                break;
            }
            crc_fail_count++;
        }

        if (crc_fail_count == MAX_CRC_FAIL_TIMES) {
            ret = APP_SEC_ERROR_CRC_FAIL;
            app_sec_remove_bond(idx, true);
            bk_printf("CRC fail, remove bonded info idx=%d\r\n", idx);
            return ret;
        }
    }

    return ret;
}
#endif

uint8_t app_sec_get_bond_status(void)
{
    return app_sec_env.bonded;
}

uint8_t app_sec_get_free_bond_idx(void)
{
    uint8_t idx;

    for (idx = 0; idx < MAX_BOND_NUM; idx++) {
        if (((app_sec_env.bonded >> idx) & 1) == 0) {
            break;
        }
    }
    return idx;
}

sec_err_t app_sec_search_bond_list(uint8_t conidx)
{
    sec_err_t ret = APP_SEC_ERROR_NO_ERROR;
    uint8_t idx;
    uint8_t peer_addr_type = app_ble_env.connections[conidx].peer_addr_type;
    struct bd_addr *peer_addr = &(app_ble_env.connections[conidx].peer_addr);
    struct gapc_irk *peer_irk_ptr;

    app_sec_env.sec_info[conidx].matched_peer_idx = INVALID_IDX;

    if (app_sec_env.bonded == 0) {
        kernel_msg_send_basic(APP_PEER_ADDR_CMP_CMP, KERNEL_BUILD_ID(TASK_BLE_APP, conidx), KERNEL_BUILD_ID(TASK_BLE_APP, conidx));

        if (app_sec_env.sec_notice_cb) {
            app_sec_env.sec_notice_cb(APP_SEC_BONDLIST_COMPARISON_CMP_IND, &conidx);
        }
        return ret;
    }

    if (peer_addr_type == 0) {
        //public addr
        for (idx = 0; idx < MAX_BOND_NUM; idx++) {
            if ((app_sec_env.bonded >> idx) & 1) {
                peer_irk_ptr = &app_sec_env.bond_info[idx].peer_irk;

                if (peer_irk_ptr->addr.addr_type == 0){
                    if (common_bdaddr_compare(peer_addr, (struct bd_addr *)(&peer_irk_ptr->addr.addr))){
                        break;
                    }
                }
            }
        }
        app_sec_env.sec_info[conidx].matched_peer_idx = idx;
    } else if ((peer_addr->addr[BD_ADDR_LEN-1] & BD_ADDR_RND_ADDR_TYPE_MSK) == BD_ADDR_STATIC){
        //static addr
        for (idx = 0; idx < MAX_BOND_NUM; idx++) {
            if ((app_sec_env.bonded >> idx) & 1) {
                peer_irk_ptr = &app_sec_env.bond_info[idx].peer_irk;

                if (peer_irk_ptr->addr.addr_type == 1){
                    if (common_bdaddr_compare(peer_addr, (struct bd_addr *)(&peer_irk_ptr->addr.addr))){
                        break;
                    }
                }
            }
        }
        app_sec_env.sec_info[conidx].matched_peer_idx = idx;
    } else if ((peer_addr->addr[BD_ADDR_LEN-1] & BD_ADDR_RND_ADDR_TYPE_MSK) == BD_ADDR_RSLV){
        //RPA
        struct gapm_resolv_addr_cmd *cmd = KERNEL_MSG_ALLOC_DYN(GAPM_RESOLV_ADDR_CMD,
                                                        TASK_BLE_GAPM,
                                                        KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                                        gapm_resolv_addr_cmd,
                                                        sizeof(struct gap_sec_key) * MAX_BOND_NUM);

        if (!cmd) {
            BLE_ASSERT_ERR(cmd);
            return APP_SEC_ERROR_NO_MEM;
        }

        cmd->operation = GAPM_RESOLV_ADDR;
        cmd->nb_key = 0;
        memcpy(cmd->addr.addr, peer_addr->addr, sizeof(bd_addr_t));
        for (idx = 0; idx < MAX_BOND_NUM; idx++) {
            if ((app_sec_env.bonded >> idx) & 1) {
                peer_irk_ptr = &app_sec_env.bond_info[idx].peer_irk;
                memcpy(cmd->irk[cmd->nb_key++].key, peer_irk_ptr->irk.key, GAP_KEY_LEN);
            }
        }
        // Send the message
        kernel_msg_send(cmd);

        return ret;
    }

    kernel_msg_send_basic(APP_PEER_ADDR_CMP_CMP, KERNEL_BUILD_ID(TASK_BLE_APP, conidx), KERNEL_BUILD_ID(TASK_BLE_APP, conidx));
    if (app_sec_env.sec_notice_cb) {
        app_sec_env.sec_notice_cb(APP_SEC_BONDLIST_COMPARISON_CMP_IND, &conidx);
    }

    return ret;
}

bool app_sec_check_tk_passkey(uint32_t passkey)
{
    if( passkey > 999999) {
        BLE_ASSERT_WARN(0, passkey, 0);
        return false;
    }

    return true;
}

sec_err_t app_sec_remove_bond(uint8_t idx, bool disconnect)
{
    uint8_t conhdl;
    uint8_t i;

    if ((idx >= INVALID_IDX) || (!(app_sec_env.bonded & (1 << idx)))) {
        return APP_SEC_ERROR_PARAM_INVALID;
    }

    app_sec_env.bonded &= ~(1 << idx);

    for (i = 0; i < BLE_CONNECTION_MAX; i++) {
        if (app_sec_env.sec_info[i].matched_peer_idx == idx) {
            app_sec_env.sec_info[i].matched_peer_idx = INVALID_IDX;

            conhdl = app_ble_env.connections[i].conhdl;
            if (BLE_APP_CONHDL_IS_VALID(conhdl) && disconnect) {
                app_ble_disconnect(i, COMMON_ERROR_REMOTE_USER_TERM_CON);
            }

            break;
        }
    }

    #if (NVDS_SUPPORT)
    // Check if we are well bonded
    if (app_sec_env.bonded)
    {
        if (nvds_put(NVDS_TAG_PERIPH_BONDED, NVDS_LEN_PERIPH_BONDED,
                     (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
        {
            BLE_ASSERT_ERR(0);
        }
    }
    #elif (APP_SEC_BOND_STORE)

    bk_flash_enable_security(FLASH_PROTECT_NONE);
    flash_ctrl(CMD_FLASH_ERASE_SECTOR, &app_sec_env.flash_bond_ptr->partition_start_addr);
    bk_flash_enable_security(FLASH_UNPROTECT_LAST_BLOCK);

    for (i = 0; i < MAX_BOND_NUM; i++) {
        if ((app_sec_env.bonded >> i) & 1) {
            app_sec_env.flash_write_idx[i] = true;
        } else {
            app_sec_env.flash_write_idx[i] = false;
        }
    }

    // update bonding info in flash
    kernel_msg_send_basic(APP_SEC_BOND_SAVE_TIMER, TASK_BLE_APP, TASK_BLE_APP);
    #endif

    return APP_SEC_ERROR_NO_ERROR;
}

sec_err_t app_sec_send_security_req(uint8_t conidx)
{
    if (conidx >= BLE_ACTIVITY_MAX) {
        return APP_SEC_ERROR_UNKNOWN_CONN;
    }

    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;
    uint8_t role = app_ble_env.connections[conidx].role;

    if (role == APP_BLE_MASTER_ROLE) {
        /* command supported only by slave of the connection. */
        return APP_SEC_ERROR_ROLE;
    }

    if (BLE_APP_CONHDL_IS_VALID(conhdl)) {
        if (app_sec_env.bonded == MAX_BOND_NUM_MASK) {
            return APP_SEC_ERROR_MAX_BOND_NUM;
        }

        struct gapc_security_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_SECURITY_CMD,
                                                    KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
                                                    KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                                    gapc_security_cmd);

        cmd->operation = GAPC_SECURITY_REQ;
        cmd->auth      = app_sec_env.pairing_param.auth;

        // Send the message
        kernel_msg_send(cmd);
    }else {
        return APP_SEC_ERROR_UNKNOWN_CONN;
    }

    return APP_SEC_ERROR_NO_ERROR;
}

sec_err_t app_sec_send_pairing_req(uint8_t conidx)
{
    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;
    uint8_t role = app_ble_env.connections[conidx].role;
    uint8_t matched_peer_idx = app_sec_env.sec_info[conidx].matched_peer_idx;

    if (role == APP_BLE_SLAVE_ROLE) {
        /* command supported only by master of the connection. */
        return APP_SEC_ERROR_ROLE;
    }

    if (!BLE_APP_CONHDL_IS_VALID(conhdl)) {
        return APP_SEC_ERROR_UNKNOWN_CONN;
    }

    if (matched_peer_idx < INVALID_IDX) {
        return APP_SEC_ERROR_DEV_ALREADY_BOND;
    }

    struct gapc_bond_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_BOND_CMD,
                                                KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
                                                KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                                gapc_bond_cmd);

    cmd->operation = GAPC_BOND;

    cmd->pairing.iocap     = app_sec_env.pairing_param.iocap;
    cmd->pairing.oob       = GAP_OOB_AUTH_DATA_NOT_PRESENT;
    cmd->pairing.auth      = app_sec_env.pairing_param.auth;
    cmd->pairing.key_size  = 16;
    cmd->pairing.ikey_dist = app_sec_env.pairing_param.ikey_dist;
    cmd->pairing.rkey_dist = app_sec_env.pairing_param.rkey_dist;

    cmd->pairing.sec_req   = app_sec_env.pairing_param.sec_req;

    // Send the message
    kernel_msg_send(cmd);

    return APP_SEC_ERROR_NO_ERROR;
}

sec_err_t app_sec_send_pairing_rsp(uint8_t conidx, bool accept)
{
    uint8_t ret = APP_SEC_ERROR_NO_ERROR;
    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;
    uint8_t role = app_ble_env.connections[conidx].role;
    uint8_t matched_peer_idx = app_sec_env.sec_info[conidx].matched_peer_idx;

    if (role == APP_BLE_MASTER_ROLE) {
        /* command supported only by slave of the connection. */
        return APP_SEC_ERROR_ROLE;
    }
    if (!BLE_APP_CONHDL_IS_VALID(conhdl)) {
        return APP_SEC_ERROR_UNKNOWN_CONN;
    }

    // Prepare the GAPC_BOND_CFM message
    struct gapc_bond_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_BOND_CFM,
                                             KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
                                             KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                             gapc_bond_cfm);
    cfm->request = GAPC_PAIRING_RSP;
    cfm->accept  = accept;

    if (matched_peer_idx < INVALID_IDX) {
        app_sec_remove_bond(matched_peer_idx, false);
    }

    if (app_sec_env.bonded == MAX_BOND_NUM_MASK) {
        kernel_msg_send(cfm);
        return APP_SEC_ERROR_MAX_BOND_NUM;
    }

    cfm->data.pairing_feat.iocap     = app_sec_env.pairing_param.iocap;
    cfm->data.pairing_feat.oob       = GAP_OOB_AUTH_DATA_NOT_PRESENT;
    cfm->data.pairing_feat.auth      = app_sec_env.pairing_param.auth;
    cfm->data.pairing_feat.key_size  = 16;
    cfm->data.pairing_feat.ikey_dist = app_sec_env.pairing_param.ikey_dist;
    cfm->data.pairing_feat.rkey_dist = app_sec_env.pairing_param.rkey_dist;
    cfm->data.pairing_feat.sec_req   = app_sec_env.pairing_param.sec_req;

    // Send the message
    kernel_msg_send(cfm);

    return ret;
}

sec_err_t app_sec_tk_exchange_cfm(uint8_t conidx, uint32_t passkey, bool accept)
{
    uint8_t ret = APP_SEC_ERROR_NO_ERROR;
    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;

    if (!BLE_APP_CONHDL_IS_VALID(conhdl)) {
        return APP_SEC_ERROR_UNKNOWN_CONN;
    }

    // Prepare the GAPC_BOND_CFM message
    struct gapc_bond_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_BOND_CFM,
                                             KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
                                             KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                             gapc_bond_cfm);
    cfm->request = GAPC_TK_EXCH;
    cfm->accept = accept;

    if (app_sec_check_tk_passkey(passkey)) {
        memset(&cfm->data.tk, 0, sizeof(struct gap_sec_key));
        memcpy(&cfm->data.tk.key[0], (uint8_t *)&passkey, sizeof(uint32_t));
    } else {
        cfm->accept  = false;
        ret = APP_SEC_ERROR_TK;
    }

    // Send the message
    kernel_msg_send(cfm);

    return ret;
}

sec_err_t app_sec_nc_exchange_cfm(uint8_t conidx, bool accept)
{
    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;

    if (!BLE_APP_CONHDL_IS_VALID(conhdl)) {
        return APP_SEC_ERROR_UNKNOWN_CONN;
    }

    // Prepare the GAPC_BOND_CFM message
    struct gapc_bond_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_BOND_CFM,
                                             KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
                                             KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                             gapc_bond_cfm);
    cfm->request = GAPC_NC_EXCH;
    cfm->accept = accept;

    // Send the message
    kernel_msg_send(cfm);

    return APP_SEC_ERROR_NO_ERROR;
}

sec_err_t app_sec_send_encryption_cmd(uint8_t conidx)
{
    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;
    uint8_t role = app_ble_env.connections[conidx].role;
    uint8_t matched_peer_idx = app_sec_env.sec_info[conidx].matched_peer_idx;
    struct gapc_ltk *bonded_ltk;

    if (role == APP_BLE_SLAVE_ROLE) {
        /* command supported only by master of the connection. */
        return APP_SEC_ERROR_ROLE;
    }

    if (!BLE_APP_CONHDL_IS_VALID(conhdl)) {
        return APP_SEC_ERROR_UNKNOWN_CONN;
    }

    struct gapc_encrypt_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_ENCRYPT_CMD,
                                                KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl),
                                                KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                                gapc_encrypt_cmd);

    cmd->operation = GAPC_ENCRYPT;

    if (matched_peer_idx < INVALID_IDX) {
        // encryption in pairing procedure
        bonded_ltk = &app_sec_env.bond_info[matched_peer_idx].ltk;
        memcpy(&cmd->ltk, bonded_ltk, sizeof(struct gapc_ltk));
    } else {
        memcpy(&cmd->ltk, &app_sec_env.sec_info[conidx].peer_ltk, sizeof(struct gapc_ltk));
    }

    // Send the message
    kernel_msg_send(cmd);

    return APP_SEC_ERROR_NO_ERROR;
}

sec_err_t app_sec_master_security_start(uint8_t conidx)
{
    uint8_t status = APP_SEC_ERROR_NO_ERROR;

    if (app_sec_env.sec_info[conidx].auth.ltk_present) {
        status = app_sec_send_encryption_cmd(conidx);
    } else {
        status = app_sec_send_pairing_req(conidx);
    }

    return status;
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
    uint8_t conhdl = KERNEL_IDX_GET(src_id);
    uint8_t conidx = app_ble_find_conn_idx_handle(conhdl);

    switch (param->request)
    {
        case (GAPC_PAIRING_REQ):
        {
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_PAIRING_REQ_IND, &conidx);
            }
        } break;

        case (GAPC_LTK_EXCH):
        {
            // Prepare the GAPC_BOND_CFM message
            struct gapc_bond_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_BOND_CFM,
                                                     src_id, KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                                     gapc_bond_cfm);
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

            memcpy(&app_sec_env.sec_info[conidx].ltk, &cfm->data.ltk,sizeof(struct gapc_ltk));
            #if (NVDS_SUPPORT)
            // Store the generated value in NVDS
            if (nvds_put(NVDS_TAG_LTK, NVDS_LEN_LTK,
                         (uint8_t *)&cfm->data.ltk) != NVDS_OK)
            {
                BLE_ASSERT_ERR(0);
            }
            #endif// #if (NVDS_SUPPORT)

            // Send the message
            kernel_msg_send(cfm);
        } break;

        case (GAPC_IRK_EXCH):
        {
            // Prepare the GAPC_BOND_CFM message
            struct gapc_bond_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_BOND_CFM,
                                                     src_id, KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                                     gapc_bond_cfm);

            cfm->accept  = true;
            cfm->request = GAPC_IRK_EXCH;

            // Load IRK
            memcpy(cfm->data.irk.irk.key,app_ble_env.loc_irk, KEY_LEN);
            // load device address
            cfm->data.irk.addr.addr_type = ADDR_PUBLIC;
            memcpy(cfm->data.irk.addr.addr.addr,(uint8_t *)&common_default_bdaddr,BD_ADDR_LEN);
            // Send the message
            kernel_msg_send(cfm);
        } break;

        case (GAPC_TK_EXCH):
        {
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_PASSKEY_REPLY, &conidx);
            }
        } break;

        case GAPC_NC_EXCH:
        {
            numeric_cmp_t nc_par;

            nc_par.conn_idx = conidx;
            nc_par.num_value = 0;
            for (int i = 0; i < 3; i++) {
                nc_par.num_value += param->data.nc_data.value[i] << (i * 8);
            }
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_CONFIRM_REPLY, &nc_par);
            }
        } break;

        case GAPC_CSRK_EXCH:
        {
            #if BLE_APP_SIGN_WRITE
            // Prepare the GAPC_BOND_CFM message
            struct gapc_bond_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_BOND_CFM,
                                                     src_id, KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                                     gapc_bond_cfm);

            cfm->accept  = true;
            cfm->request = GAPC_CSRK_EXCH;

            // Generate CSRK
            for (uint8_t counter = 0; counter < GAP_KEY_LEN; counter++)
            {
                cfm->data.csrk.key[counter] = (uint8_t)bk_rand();
            }

            app_sec_env.sec_info[conidx].local_csrk_present = true;
            memcpy(&app_sec_env.sec_info[conidx].csrk, &cfm->data.csrk, sizeof(struct gap_sec_key));

            // Send the message
            kernel_msg_send(cfm);
            #endif
        } break;

        default:
        {
            BLE_ASSERT_INFO(0, param->request, 0);
        } break;
    }

    return (KERNEL_MSG_CONSUMED);
}


static int gapc_bond_ind_handler(kernel_msg_id_t const msgid,
                                 struct gapc_bond_ind const *param,
                                 kernel_task_id_t const dest_id,
                                 kernel_task_id_t const src_id)
{
    uint8_t conhdl = KERNEL_IDX_GET(src_id);
    uint8_t conidx = app_ble_find_conn_idx_handle(conhdl);
    uint8_t role = app_ble_env.connections[conidx].role;
    uint8_t peer_addr_type = app_ble_env.connections[conidx].peer_addr_type;
    struct bd_addr *peer_addr = &(app_ble_env.connections[conidx].peer_addr);
    sec_info_t *sec_p = &app_sec_env.sec_info[conidx];

    switch (param->info)
    {
        case (GAPC_PAIRING_SUCCEED):
        {
            // Update the bonding status in the environment
            uint8_t idx;
            idx = app_sec_get_free_bond_idx();

            if (idx < MAX_BOND_NUM && (param->data.auth.info & GAP_AUTH_BOND)) {
                app_sec_env.bonded |= (1 << idx);
                sec_p->matched_peer_idx = idx;
                bond_info_t *bond_info_ptr = &app_sec_env.bond_info[idx];

                // store pairing security level
                memcpy(&sec_p->auth, &param->data.auth, sizeof(struct gapc_bond_auth));
                memcpy(&bond_info_ptr->auth, &param->data.auth, sizeof(struct gapc_bond_auth));

                // store ltk
                if (param->data.auth.info & GAP_AUTH_SEC_CON) {
                    memcpy(&sec_p->ltk, &sec_p->peer_ltk, sizeof(struct gapc_ltk));
                }

                if (role == APP_BLE_MASTER_ROLE) {
                    memcpy(&(bond_info_ptr->ltk), &sec_p->peer_ltk, sizeof(struct gapc_ltk));
                } else {
                    memcpy(&(bond_info_ptr->ltk), &sec_p->ltk, sizeof(struct gapc_ltk));
                }

                // store irk
                memcpy(&(bond_info_ptr->peer_irk), &sec_p->peer_irk, sizeof(struct gapc_irk));
                if (common_bdaddr_compare((struct bd_addr *)bond_info_ptr->peer_irk.addr.addr.addr, &common_null_bdaddr)) {
                    // didn't distribute IRK, store peer address
                    memcpy(bond_info_ptr->peer_irk.addr.addr.addr, peer_addr, sizeof(bd_addr_t));
                    bond_info_ptr->peer_irk.addr.addr_type = peer_addr_type;
                }

                #if BLE_APP_SIGN_WRITE
                // store csrk & counter
                if (sec_p->local_csrk_present) {
                    bond_info_ptr->sign_counter = sec_p->sign_counter;
                    memcpy(&bond_info_ptr->csrk, &sec_p->csrk, sizeof(struct gap_sec_key));
                }

                if (sec_p->peer_csrk_present) {
                    bond_info_ptr->peer_sign_counter = sec_p->peer_sign_counter;
                    memcpy(&bond_info_ptr->peer_csrk, &sec_p->peer_csrk, sizeof(struct gap_sec_key));
                }
                #endif

                #if (APP_SEC_BOND_STORE)
                bond_info_ptr->crc = CRC_DEFAULT_VALUE;
                bond_info_ptr->crc = app_sec_crc32(bond_info_ptr->crc, bond_info_ptr, sizeof(bond_info_t) - 4);
                app_sec_env.flash_write_idx[idx] = true;

                kernel_msg_send_basic(APP_SEC_BOND_SAVE_TIMER, TASK_BLE_APP, TASK_BLE_APP);
                #endif
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
            memcpy(&sec_p->peer_irk,&param->data.irk,sizeof(struct gapc_irk));
        } break;

        case (GAPC_PAIRING_FAILED):
        {
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_PAIRING_FAIL, NULL);
            }
        } break;

        case (GAPC_LTK_EXCH):
        {
            memcpy(&sec_p->peer_ltk,&param->data.ltk,sizeof(struct gapc_ltk));
        } break;

        case (GAPC_CSRK_EXCH):
        {
            #if BLE_APP_SIGN_WRITE
            sec_p->peer_csrk_present = true;
            memcpy(&sec_p->peer_csrk, &param->data.csrk, sizeof(struct gap_sec_key));
            #endif
        } break;

        case (GAPC_TK_EXCH):
        {
        } break;

        default:
        {
            BLE_ASSERT_INFO(0, param->info, 0);
        } break;
    }

    return (KERNEL_MSG_CONSUMED);

}

static int gapc_encrypt_req_ind_handler(kernel_msg_id_t const msgid,
                                        struct gapc_encrypt_req_ind const *param,
                                        kernel_task_id_t const dest_id,
                                        kernel_task_id_t const src_id)
{
    struct gapc_ltk *bonded_ltk;
    uint8_t conhdl = KERNEL_IDX_GET(src_id);
    uint8_t conidx = app_ble_find_conn_idx_handle(conhdl);
    uint8_t matched_peer_idx = app_sec_env.sec_info[conidx].matched_peer_idx;

    // Prepare the GAPC_ENCRYPT_CFM message
    struct gapc_encrypt_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_ENCRYPT_CFM,
                                     src_id, KERNEL_BUILD_ID(TASK_BLE_APP, conidx),
                                     gapc_encrypt_cfm);

    cfm->found    = false;

    if (matched_peer_idx < MAX_BOND_NUM) {
        bonded_ltk = &app_sec_env.bond_info[matched_peer_idx].ltk;

        // Check if the provided EDIV and Rand Nb values match with the stored values
        if ((param->ediv == bonded_ltk->ediv) && !memcmp(param->rand_nb.nb, bonded_ltk->randnb.nb, sizeof(struct rand_nb))) {
            cfm->found    = true;
            cfm->key_size = GAP_KEY_LEN;
            memcpy(cfm->ltk.key, bonded_ltk->ltk.key, sizeof(struct gap_sec_key));
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
    uint8_t conn_idx = app_ble_find_conn_idx_handle(KERNEL_IDX_GET(src_id));

    if (app_sec_env.sec_notice_cb) {
        app_sec_env.sec_notice_cb(APP_SEC_ENCRYPT_SUCCEED, &conn_idx);
    }

    return (KERNEL_MSG_CONSUMED);
}

static int gapc_security_ind_handler(kernel_msg_id_t const msgid,
                                    struct gapc_security_ind const *param,
                                    kernel_task_id_t const dest_id,
                                    kernel_task_id_t const src_id)
{
    uint8_t conn_idx = app_ble_find_conn_idx_handle(KERNEL_IDX_GET(src_id));

    if (app_sec_env.sec_notice_cb) {
        app_sec_env.sec_notice_cb(APP_SEC_SECURITY_REQ_IND, &conn_idx);
    }

    return (KERNEL_MSG_CONSUMED);
}

#if BLE_APP_SIGN_WRITE
static int gapc_sign_counter_ind_handler(kernel_msg_id_t const msgid,
                                    struct gapc_sign_counter_ind const *param,
                                    kernel_task_id_t const dest_id,
                                    kernel_task_id_t const src_id)
{
    uint8_t conn_idx = app_ble_find_conn_idx_handle(KERNEL_IDX_GET(src_id));

    app_sec_env.sec_info[conn_idx].sign_counter = param->local_sign_counter;
    app_sec_env.sec_info[conn_idx].peer_sign_counter = param->peer_sign_counter;

    return (KERNEL_MSG_CONSUMED);
}
#endif

static int gapm_addr_solved_handler(kernel_msg_id_t const msgid,
                                    struct gapm_addr_solved_ind *param,
                                    kernel_task_id_t const dest_id,
                                    kernel_task_id_t const src_id)
{
    struct gap_sec_key *bonded_irk;
    uint8_t conidx = KERNEL_IDX_GET(dest_id);
    uint8_t idx;

    for (idx = 0; idx < MAX_BOND_NUM; idx++) {
        if ((app_sec_env.bonded >> idx) & 1) {
            bonded_irk = &app_sec_env.bond_info[idx].peer_irk.irk;
            if (!memcmp(&(param->irk), bonded_irk, sizeof(struct gap_sec_key))){
                app_sec_env.sec_info[conidx].matched_peer_idx = idx;
                break;
            }
        }
    }

    kernel_msg_send_basic(APP_PEER_ADDR_CMP_CMP, KERNEL_BUILD_ID(TASK_BLE_APP, conidx), KERNEL_BUILD_ID(TASK_BLE_APP, conidx));
    if (app_sec_env.sec_notice_cb) {
        app_sec_env.sec_notice_cb(APP_SEC_BONDLIST_COMPARISON_CMP_IND, &conidx);
    }

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
    #if BLE_APP_SIGN_WRITE
    {GAPC_SIGN_COUNTER_IND,   (kernel_msg_func_t)gapc_sign_counter_ind_handler},
    #endif

    {GAPM_ADDR_SOLVED_IND,    (kernel_msg_func_t)gapm_addr_solved_handler},
};

const struct app_subtask_handlers app_sec_handlers =
    {&app_sec_msg_handler_list[0], (sizeof(app_sec_msg_handler_list)/sizeof(struct kernel_msg_handler))};

#endif //(BLE_APP_SEC)

/// @} APP
