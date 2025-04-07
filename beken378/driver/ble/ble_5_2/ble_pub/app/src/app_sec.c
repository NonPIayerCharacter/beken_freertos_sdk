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

/**
 * Calculate the CRC32 value of a memory buffer.
 *
 * @param crc accumulated CRC32 value, must be 0 on first call
 * @param buf buffer to calculate CRC32 value for
 * @param size bytes in buffer
 *
 * @return calculated CRC32 value
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

    app_sec_env.flash_bond_ptr = bk_flash_get_info(BK_PARTITION_BLE_BONDING_FLASH);
    app_sec_env.bonded = 0;
    app_sec_env.passkey = SEC_TK_PASSKEY;

    for (uint8_t i = 0; i < MAX_BOND_NUM; i++) {
        crc_fail_count = 0;
        flash_addr_ptr = app_sec_env.flash_bond_ptr->partition_start_addr + bond_info_size * i;

        while(crc_fail_count < MAX_CRC_FAIL_TIMES) {
            flash_read((char *)(&app_sec_env.bond_info[i]), sizeof(bond_info_t), flash_addr_ptr);

            if (app_sec_env.bond_info[i].crc == CRC_DEFAULT_VALUE) {
                // no bonding info
                break;
            } else {
                // bonding info exits, check crc
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
            app_sec_remove_bond(i);
            bk_printf("CRC fail, remove bonded info idx=%d\r\n", i);
        }
    }
    #else
        app_sec_env.bonded = 0;
        app_sec_env.passkey = SEC_TK_PASSKEY;
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
    uint8_t cnt, idx;

    for (cnt = 0; cnt < app_sec_env.flash_write_num; cnt++) {
        idx = app_sec_env.flash_write_idx[cnt];
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
            app_sec_remove_bond(idx);
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

    app_sec_env.matched_peer_idx = INVAILD_IDX;

    if (app_sec_env.bonded == 0) {
        return APP_SEC_ERROR_NO_ERROR;
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
        app_sec_env.matched_peer_idx = idx;
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
        app_sec_env.matched_peer_idx = idx;
    } else if ((peer_addr->addr[BD_ADDR_LEN-1] & BD_ADDR_RND_ADDR_TYPE_MSK) == BD_ADDR_RSLV){
        //RPA
        struct gapm_resolv_addr_cmd *cmd = KERNEL_MSG_ALLOC_DYN(GAPM_RESOLV_ADDR_CMD,
                                                        TASK_BLE_GAPM, TASK_BLE_APP,
                                                        gapm_resolv_addr_cmd,
                                                        sizeof(gap_sec_key_t) * MAX_BOND_NUM);

        if (!cmd) {
            return APP_SEC_ERROR_NO_MEM;
        }

        cmd->operation = GAPM_RESOLV_ADDR;
        cmd->nb_key = 0;
        memcpy(cmd->addr.addr, peer_addr->addr, sizeof(struct bd_addr));
        for (idx = 0; idx < MAX_BOND_NUM; idx++) {
            if ((app_sec_env.bonded >> idx) & 1) {
                peer_irk_ptr = &app_sec_env.bond_info[idx].peer_irk;
                memcpy(cmd->irk[cmd->nb_key++].key, peer_irk_ptr->irk.key, GAP_KEY_LEN);
            }
        }
        // Send the message
        kernel_msg_send(cmd);
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

sec_err_t app_sec_remove_bond(uint8_t idx)
{
    if ((idx >= INVAILD_IDX) || (!(app_sec_env.bonded & (1 << idx)))) {
        return APP_SEC_ERROR_PARAM_INVALID;
    }

    app_sec_env.bonded &= ~(1 << idx);

    #if (NVDS_SUPPORT)
    // Check if we are well bonded
    if (app_sec_env.bonded)
    {
        // Update the environment variable
        if (nvds_put(NVDS_TAG_PERIPH_BONDED, NVDS_LEN_PERIPH_BONDED,
                     (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
        {
            BLE_ASSERT_ERR(0);
        }
    }
    #elif (APP_SEC_BOND_STORE)
    uint8_t i;

    bk_flash_enable_security(FLASH_PROTECT_NONE);
    flash_ctrl(CMD_FLASH_ERASE_SECTOR, &app_sec_env.flash_bond_ptr->partition_start_addr);
    bk_flash_enable_security(FLASH_UNPROTECT_LAST_BLOCK);

    app_sec_env.flash_write_num = 0;
    for (i = 0; i < MAX_BOND_NUM; i++) {
        if ((app_sec_env.bonded >> i) & 1) {
            app_sec_env.flash_write_idx[app_sec_env.flash_write_num++] = i;
        }
    }

    // set timer to update bonding info in flash
    kernel_timer_set(APP_SEC_BOND_SAVE_TIMER, TASK_BLE_APP, 20);
    #endif

    return APP_SEC_ERROR_NO_ERROR;
}

sec_err_t app_sec_send_security_req(uint8_t conidx)
{
    sec_err_t ret = APP_SEC_ERROR_NO_ERROR;

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

        // peer device is not in bonded list
        if(app_sec_env.matched_peer_idx == INVAILD_IDX)
        {
            if (app_sec_env.bonded == MAX_BOND_NUM_MASK) {
                return APP_SEC_ERROR_MAX_BOND_NUM;
            }

            struct gapc_security_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_SECURITY_CMD,
                                                        KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl), TASK_BLE_APP,
                                                        gapc_security_cmd);

            cmd->operation = GAPC_SECURITY_REQ;
            cmd->auth      = app_sec_env.pairing_param.auth;

            // Send the message
            kernel_msg_send(cmd);
        } else {
            return APP_SEC_ERROR_DEV_ALREADY_BOND;
        }
    }else {
        return APP_SEC_ERROR_UNKNOWN_CONN;
    }

    return ret;
}

sec_err_t app_sec_send_pairing_req(uint8_t conidx)
{
    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;
    uint8_t role = app_ble_env.connections[conidx].role;

    if (role == APP_BLE_SLAVE_ROLE) {
        /* command supported only by master of the connection. */
        return APP_SEC_ERROR_ROLE;
    }

    if (!BLE_APP_CONHDL_IS_VALID(conhdl)) {
        return APP_SEC_ERROR_UNKNOWN_CONN;
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

    return APP_SEC_ERROR_NO_ERROR;
}

sec_err_t app_sec_send_pairing_rsp(uint8_t conidx)
{
    uint8_t ret = APP_SEC_ERROR_NO_ERROR;
    uint8_t conhdl = app_ble_env.connections[conidx].conhdl;
    uint8_t role = app_ble_env.connections[conidx].role;

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
    cfm->accept  = false;

    if (app_sec_env.matched_peer_idx < INVAILD_IDX) {
        app_sec_remove_bond(app_sec_env.matched_peer_idx);
    }

    if (app_sec_env.bonded == MAX_BOND_NUM_MASK) {
        kernel_msg_send(cfm);
        return APP_SEC_ERROR_MAX_BOND_NUM;
    }

    cfm->accept  = true;
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

sec_err_t app_sec_tk_exchange_cfm(uint8_t conidx, uint32_t passkey)
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

    if (app_sec_check_tk_passkey(passkey)) {
        cfm->accept = true;
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

    if (role == APP_BLE_SLAVE_ROLE) {
        /* command supported only by master of the connection. */
        return APP_SEC_ERROR_ROLE;
    }

    if (!BLE_APP_CONHDL_IS_VALID(conhdl)) {
        return APP_SEC_ERROR_UNKNOWN_CONN;
    }

    struct gapc_encrypt_cmd *cmd = KERNEL_MSG_ALLOC(GAPC_ENCRYPT_CMD,
                                                KERNEL_BUILD_ID(TASK_BLE_GAPC, conhdl), TASK_BLE_APP,
                                                gapc_encrypt_cmd);
    cmd->operation = GAPC_ENCRYPT;
    memcpy(&cmd->ltk, &app_sec_env.peer_ltk, sizeof(struct gapc_ltk));

    // Send the message
    kernel_msg_send(cmd);

    return APP_SEC_ERROR_NO_ERROR;
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
    uint8_t conn_idx = app_ble_find_conn_idx_handle(KERNEL_IDX_GET(src_id));

    switch (param->request)
    {
        case (GAPC_PAIRING_REQ):
        {
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_PAIRING_REQ_IND, &conn_idx);
            }
        } break;
        case (GAPC_LTK_EXCH):
        {
            // Prepare the GAPC_BOND_CFM message
            struct gapc_bond_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_BOND_CFM,
                                                     src_id, TASK_BLE_APP,
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

            memcpy(&app_sec_env.ltk, &cfm->data.ltk, sizeof(struct gapc_ltk));
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
                                                     src_id, TASK_BLE_APP,
                                                     gapc_bond_cfm);

            cfm->accept  = true;
            cfm->request = GAPC_IRK_EXCH;

            // Load IRK
            memcpy(cfm->data.irk.irk.key,app_ble_env.loc_irk, KEY_LEN);
            // load device address
            cfm->data.irk.addr.addr_type = ADDR_PUBLIC;
            memcpy(cfm->data.irk.addr.addr,(uint8_t *)&common_default_bdaddr,BD_ADDR_LEN);
            // Send the message
            kernel_msg_send(cfm);
        } break;
        case (GAPC_TK_EXCH):
        {
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_PASSKEY_REPLY, &conn_idx);
            }
        } break;
        case GAPC_NC_EXCH:
        {
            numeric_cmp_t nc_par;

            nc_par.conn_idx = conn_idx;
            nc_par.num_value = 0;
            for (int i = 0; i < 3; i++) {
                nc_par.num_value += param->data.nc_data.value[i] << (i * 8);
            }
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_CONFIRM_REPLY, &nc_par);
            }
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
    switch (param->info)
    {
        case (GAPC_PAIRING_SUCCEED):
        {
            // Update the bonding status in the environment
            uint8_t idx;
            idx = app_sec_get_free_bond_idx();

            if (idx < MAX_BOND_NUM && (param->data.pairing.level & GAP_AUTH_BOND)) {
                app_sec_env.bonded |= (1 << idx);
                app_sec_env.matched_peer_idx = idx;
                bond_info_t *bond_info_ptr;
                bond_info_ptr = &app_sec_env.bond_info[idx];

                if (param->data.pairing.sc_pairing) {
                    memcpy(&app_sec_env.ltk, &app_sec_env.peer_ltk, sizeof(struct gapc_ltk));
                }
                memcpy(&(bond_info_ptr->ltk), &app_sec_env.ltk, sizeof(struct gapc_ltk));
                memcpy(&(bond_info_ptr->peer_irk), &app_sec_env.peer_irk, sizeof(struct gapc_irk));

                #if (APP_SEC_BOND_STORE)
                bond_info_ptr->crc = CRC_DEFAULT_VALUE;
                bond_info_ptr->crc = app_sec_crc32(bond_info_ptr->crc, bond_info_ptr, sizeof(bond_info_t) - 4);
                app_sec_env.flash_write_num = 1;
                app_sec_env.flash_write_idx[0] = idx;
                kernel_timer_set(APP_SEC_BOND_SAVE_TIMER, TASK_BLE_APP, 20);
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
            memcpy(&app_sec_env.peer_irk,&param->data.irk,sizeof(struct gapc_irk));
        } break;

        case (GAPC_PAIRING_FAILED):
        {
            if (app_sec_env.sec_notice_cb) {
                app_sec_env.sec_notice_cb(APP_SEC_PAIRING_FAILED, NULL);
            }
        } break;

        case (GAPC_LTK_EXCH):
        {
            memcpy(&app_sec_env.peer_ltk,&param->data.ltk,sizeof(struct gapc_ltk));
        } break;

        case (GAPC_TK_EXCH):
        {
        }break;

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

    // Prepare the GAPC_ENCRYPT_CFM message
    struct gapc_encrypt_cfm *cfm = KERNEL_MSG_ALLOC(GAPC_ENCRYPT_CFM,
                                     src_id, TASK_BLE_APP,
                                     gapc_encrypt_cfm);

    cfm->found    = false;

    if (app_sec_env.matched_peer_idx < INVAILD_IDX) {
        bonded_ltk = &app_sec_env.bond_info[app_sec_env.matched_peer_idx].ltk;

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

static int gapm_addr_solved_handler(kernel_msg_id_t const msgid,
                                    struct gapm_addr_solved_ind *param,
                                    kernel_task_id_t const dest_id,
                                    kernel_task_id_t const src_id)
{
    struct gap_sec_key *bonded_irk;
    uint8_t idx;

    for (idx = 0; idx < MAX_BOND_NUM; idx++) {
        if ((app_sec_env.bonded >> idx) & 1) {
            bonded_irk = &app_sec_env.bond_info[idx].peer_irk.irk;
            if (!memcmp(&(param->irk), bonded_irk, sizeof(struct gap_sec_key))){
                app_sec_env.matched_peer_idx = idx;
            }
        }
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

    {GAPM_ADDR_SOLVED_IND,    (kernel_msg_func_t)gapm_addr_solved_handler},
};

const struct app_subtask_handlers app_sec_handlers =
    {&app_sec_msg_handler_list[0], (sizeof(app_sec_msg_handler_list)/sizeof(struct kernel_msg_handler))};

#endif //(BLE_APP_SEC)

/// @} APP
