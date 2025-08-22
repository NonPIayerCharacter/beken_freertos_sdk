// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "string.h"
#include "ble_ui.h"
#include "app_ble.h"
#include "app_sdp.h"
#include "gatt_db.h"
#include "comm.h"
#if (BLE_APP_PRESENT && (BLE_CENTRAL))
#include "app_ble_init.h"
#include "app_sdp.h"
#endif

#if BLE_APP_SEC
#include "app_sec.h"
#endif

ble_notice_cb_t ble_event_notice = NULL;
extern struct app_env_tag app_ble_env;

void ble_set_notice_cb(ble_notice_cb_t func)
{
    ble_event_notice = func;
}

uint8_t ble_appm_get_dev_name(uint8_t* name, uint32_t buf_len)
{
    if (buf_len >= app_ble_env.dev_name_len) {
        // copy name to provided pointer
        memcpy(name, app_ble_env.dev_name, app_ble_env.dev_name_len);
        // return name length
        return app_ble_env.dev_name_len;
    }

    return 0;
}

uint8_t ble_appm_set_dev_name(uint8_t len, uint8_t* name)
{
    // copy name to provided pointer
    if (len <= APP_DEVICE_NAME_MAX_LEN) {
        app_ble_env.dev_name_len = len;
        memcpy(app_ble_env.dev_name, name, len);
        // return name length
        return app_ble_env.dev_name_len;
    }

    return 0;
}

ble_err_t bk_ble_adv_start(uint8_t actv_idx, struct adv_param *adv, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        if (actv_idx < BLE_ACTIVITY_MAX) {
            op_mask = (1 << BLE_OP_CREATE_ADV_POS) | (1 << BLE_OP_SET_ADV_DATA_POS)
                      | (1 << BLE_OP_SET_RSP_DATA_POS) | (1 << BLE_OP_START_ADV_POS);
            app_ble_run(actv_idx, BLE_INIT_ADV, op_mask, callback);
            memcpy(&(app_ble_env.actvs[actv_idx].param.adv), adv, sizeof(struct adv_param));
            ret = app_ble_create_advertising(actv_idx, adv);
            if (ret != ERR_SUCCESS) {
                app_ble_reset();
            }
        } else {
            bk_printf("Unknow actv idx[%d]\r\n", actv_idx);
            return ERR_UNKNOW_IDX;
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_adv_stop(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        if (actv_idx < BLE_ACTIVITY_MAX) {
            op_mask = (1 << BLE_OP_STOP_ADV_POS) | (1 << BLE_OP_DEL_ADV_POS);
            app_ble_run(actv_idx, BLE_DEINIT_ADV, op_mask, callback);
            ret = app_ble_stop_advertising(actv_idx);
            if (ret != ERR_SUCCESS) {
                app_ble_reset();
            }
        } else {
            bk_printf("Unknow actv idx[%d]\r\n", actv_idx);
            return ERR_UNKNOW_IDX;
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_scan_start(uint8_t actv_idx, struct scan_param *scan, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        if (actv_idx < BLE_ACTIVITY_MAX) {
            op_mask = (1 << BLE_OP_CREATE_SCAN_POS) | (1 << BLE_OP_START_SCAN_POS);
            app_ble_run(actv_idx, BLE_INIT_SCAN, op_mask, callback);
            memcpy(&(app_ble_env.actvs[actv_idx].param.scan), scan, sizeof(struct scan_param));
            ret = app_ble_create_scaning(actv_idx);
            if (ret != ERR_SUCCESS) {
                app_ble_reset();
            }
        } else {
            bk_printf("Unknow actv idx[%d]\r\n", actv_idx);
            return ERR_UNKNOW_IDX;
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_scan_stop(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        if (actv_idx < BLE_ACTIVITY_MAX) {
            op_mask = (1 << BLE_OP_STOP_SCAN_POS) | (1 << BLE_OP_DEL_SCAN_POS);
            app_ble_run(actv_idx, BLE_DEINIT_SCAN, op_mask, callback);
            ret = app_ble_stop_scaning(actv_idx);
            if (ret != ERR_SUCCESS) {
                app_ble_reset();
            }
        } else {
            bk_printf("Unknow actv idx[%d]\r\n", actv_idx);
            return ERR_UNKNOW_IDX;
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_create_advertising(uint8_t actv_idx,
                                    unsigned char chnl_map,
                                    uint32_t intv_min,
                                    uint32_t intv_max,
                                    ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    struct adv_param adv;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_CREATE_ADV_POS;
        app_ble_run(actv_idx, BLE_CREATE_ADV, op_mask, callback);
        adv.channel_map = chnl_map;
        adv.duration = 0;
        adv.interval_max = intv_max;
        adv.interval_min = intv_min;
        adv.prop = (1 << ADV_PROP_CONNECTABLE_POS) | (1 << ADV_PROP_SCANNABLE_POS);
        ret = app_ble_create_advertising(actv_idx, &adv);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_create_extended_advertising(uint8_t actv_idx,
        unsigned char chnl_map,
        uint32_t intv_min,
        uint32_t intv_max,
        uint8_t scannable,
        uint8_t connectable,
        ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ext_adv_param_cfg_t ext_adv_param;
    ble_err_t ret = ERR_SUCCESS;

    if (scannable && connectable) {
        bk_printf("extended adv cannot be both scannable and connectable!\r\n");
        return ERR_CMD_NOT_SUPPORT;
    }

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_CREATE_ADV_POS;
        app_ble_run(actv_idx, BLE_CREATE_ADV, op_mask, callback);
        ext_adv_param.channel_map = chnl_map;
        ext_adv_param.interval_min = intv_min;
        ext_adv_param.interval_max = intv_max;
        ext_adv_param.duration = 0;
        ext_adv_param.sid = actv_idx;
        ext_adv_param.prop = (scannable << ADV_PROP_SCANNABLE_POS) | (connectable << ADV_PROP_CONNECTABLE_POS);
        ret = app_ble_create_extended_advertising(actv_idx, &ext_adv_param);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}


ble_err_t bk_ble_start_advertising(uint8_t actv_idx, uint16 duration, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_START_ADV_POS;
        app_ble_run(actv_idx, BLE_START_ADV, op_mask, callback);
        ret = app_ble_start_advertising(actv_idx, duration);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_stop_advertising(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_STOP_ADV_POS;
        app_ble_run(actv_idx, BLE_STOP_ADV, op_mask, callback);
        ret = app_ble_stop_advertising(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_delete_advertising(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_DEL_ADV_POS;
        app_ble_run(actv_idx, BLE_DELETE_ADV, op_mask, callback);
        ret = app_ble_delete_advertising(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_set_adv_data(uint8_t actv_idx, unsigned char* adv_buff, unsigned char adv_len, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if(adv_len > ADV_DATA_LEN)
    {
        bk_printf("adv_data_len error:%d\r\n", adv_len);
        return ERR_ADV_DATA;
    }

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_SET_ADV_DATA_POS;
        app_ble_run(actv_idx, BLE_SET_ADV_DATA, op_mask, callback);
        ret = app_ble_set_adv_data(actv_idx, adv_buff, adv_len);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_set_ext_adv_data(uint8_t actv_idx, unsigned char* adv_buff, uint16_t adv_len, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (adv_len > BLE_CFG_MAX_ADV_DATA_LEN - 3)		//ADV_AD_TYPE_FLAGS_LENGTH
    {
        bk_printf("ext_adv_data_len error:%d\r\n", adv_len);
        return ERR_ADV_DATA;
    }

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_SET_ADV_DATA_POS;
        app_ble_run(actv_idx, BLE_SET_ADV_DATA, op_mask, callback);
        ret = app_ble_set_adv_data(actv_idx, adv_buff, adv_len);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}


ble_err_t bk_ble_set_scan_rsp_data(uint8_t actv_idx, unsigned char* scan_buff, unsigned char scan_len, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (scan_len > ADV_DATA_LEN)
    {
        bk_printf("scan_rsp_len error:%d\r\n", scan_len);
        return ERR_ADV_DATA;
    }

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_SET_RSP_DATA_POS;
        app_ble_run(actv_idx, BLE_SET_RSP_DATA, op_mask, callback);
        ret = app_ble_set_scan_rsp_data(actv_idx, scan_buff, scan_len);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_set_ext_scan_rsp_data(uint8_t actv_idx, unsigned char* scan_buff, uint16_t scan_len, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (scan_len > BLE_CFG_MAX_ADV_DATA_LEN - 3)	//ADV_AD_TYPE_FLAGS_LENGTH
    {
        bk_printf("ext_scan_rsp_len error:%d\r\n", scan_len);
        return ERR_ADV_DATA;
    }

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_SET_RSP_DATA_POS;
        app_ble_run(actv_idx, BLE_SET_RSP_DATA, op_mask, callback);
        ret = app_ble_set_scan_rsp_data(actv_idx, scan_buff, scan_len);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}


/////////////////////////////// ble connected API /////////////////////////////////////////////
ble_err_t bk_ble_update_param(uint8_t conn_idx, uint16_t intv_min, uint16_t intv_max,
                              uint16_t latency, uint16_t sup_to)
{
    struct gapc_conn_param conn_param;

    conn_param.intv_min = intv_min;
    conn_param.intv_max = intv_max;
    conn_param.latency = latency;
    conn_param.time_out = sup_to;

    return app_ble_update_param(conn_idx, &conn_param);
}

ble_err_t bk_ble_disconnect(uint8_t conn_idx)
{
    return app_ble_disconnect(conn_idx, COMMON_ERROR_REMOTE_USER_TERM_CON);
}

/////////////////////////////// ble scan API /////////////////////////////////////////////
ble_err_t bk_ble_create_scaning(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_CREATE_SCAN_POS;
        app_ble_run(actv_idx, BLE_CREATE_SCAN, op_mask, callback);
        ret = app_ble_create_scaning(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_start_scaning(uint8_t actv_idx, uint16_t scan_intv, uint16_t scan_wd, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_START_SCAN_POS;
        app_ble_run(actv_idx, BLE_START_SCAN, op_mask, callback);
        ret = app_ble_start_scaning(actv_idx, scan_intv, scan_wd);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_stop_scaning(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_STOP_SCAN_POS;
        app_ble_run(actv_idx, BLE_STOP_SCAN, op_mask, callback);
        ret = app_ble_stop_scaning(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_delete_scaning(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_DEL_SCAN_POS;
        app_ble_run(actv_idx, BLE_DELETE_SCAN, op_mask, callback);
        ret = app_ble_delete_scaning(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

#if (CFG_BLE_PER_ADV)
ble_err_t bk_ble_create_periodic_advertising(uint8_t actv_idx,
        struct per_adv_param *per_adv,
        ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_CREATE_ADV_POS;
        app_ble_run(actv_idx, BLE_CREATE_ADV, op_mask, callback);
        ret = app_ble_create_periodic_advertising(actv_idx, per_adv);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_set_periodic_adv_data(uint8_t actv_idx, unsigned char *adv_buff, uint16_t adv_len, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_SET_ADV_DATA_POS;
        app_ble_run(actv_idx, BLE_SET_ADV_DATA, op_mask, callback);
        ret = app_ble_set_periodic_adv_data(actv_idx, adv_buff, adv_len);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_start_periodic_advertising(uint8_t actv_idx, uint16 duration, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        if (actv_idx < BLE_ACTIVITY_MAX) {
            op_mask = 1 << BLE_OP_START_ADV_POS;
            app_ble_run(actv_idx, BLE_START_ADV, op_mask, callback);
            ret = app_ble_start_advertising(actv_idx, duration);
            if (ret != ERR_SUCCESS) {
                app_ble_reset();
            }
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_stop_periodic_advertising(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_STOP_ADV_POS;
        app_ble_run(actv_idx, BLE_STOP_ADV, op_mask, callback);
        ret = app_ble_stop_advertising(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_delete_periodic_advertising(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_DEL_ADV_POS;
        app_ble_run(actv_idx, BLE_DELETE_ADV, op_mask, callback);
        ret = app_ble_delete_advertising(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}
#endif

#if (CFG_BLE_PER_SYNC)
ble_err_t bk_ble_create_periodic_sync(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_CREATE_PERIODIC_SYNC_POS;
        app_ble_run(actv_idx, BLE_CREATE_PERIODIC_SYNC, op_mask, callback);
        ret = app_ble_create_periodic_sync(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_start_periodic_sync(uint8_t actv_idx, ble_periodic_sync_param_t *param, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_START_PERIODIC_SYNC_POS;
        app_ble_run(actv_idx, BLE_START_PERIODIC_SYNC, op_mask, callback);
        ret = app_ble_start_periodic_sync(actv_idx, param);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_stop_periodic_sync(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_STOP_PERIODIC_SYNC_POS;
        app_ble_run(actv_idx, BLE_STOP_PERIODIC_SYNC, op_mask, callback);
        ret = app_ble_stop_periodic_sync(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_delete_periodic_sync(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_DEL_PERIODIC_SYNC_POS;
        app_ble_run(actv_idx, BLE_DELETE_PERIODIC_SYNC, op_mask, callback);
        ret = app_ble_delete_periodic_sync(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}
#endif

#if (CFG_BLE_PER_ADV) | (CFG_BLE_PER_SYNC)
ble_err_t bk_ble_periodic_adv_sync_transf(uint8_t actv_idx, uint16_t service_data)
{
    return app_ble_periodic_adv_sync_transf(actv_idx, service_data);
}
#endif

ble_err_t bk_ble_gap_config_local_icon(uint16_t appearance)
{
    app_ble_env.dev_appearance = appearance;
    return ERR_SUCCESS;
}

ble_err_t bk_ble_gap_set_channels(bk_ble_channels_t *channels)
{
    return app_ble_set_channels(channels);
}

ble_err_t bk_ble_gap_clear_whitelist(void)
{
    return app_ble_list_clear_wl_cmd();
}

ble_err_t bk_ble_gap_update_whitelist(uint8_t add_remove, struct bd_addr *addr, uint8_t addr_type)
{
    return app_ble_update_wl_cmd(add_remove, addr, addr_type);
}

ble_err_t bk_ble_gap_get_whitelist_size(uint8_t *wl_size)
{
    return app_ble_get_wl_size_cmd(wl_size);
}

ble_err_t bk_ble_gap_update_per_adv_list(uint8_t add_remove, struct bd_addr *addr, uint8_t addr_type, uint8_t adv_sid)
{
    gap_per_adv_bdaddr_t p_pal_info;

    p_pal_info.addr_type = addr_type;
    p_pal_info.adv_sid = adv_sid;
    memcpy(&p_pal_info.addr[0], addr, BD_ADDR_LEN);

    return app_ble_update_per_adv_list_cmd(add_remove, &p_pal_info);
}

ble_err_t bk_ble_gap_clear_per_adv_list(void)
{
    return app_ble_clear_per_adv_list_cmd();
}

#if (BLE_CENTRAL)
ble_err_t bk_ble_gap_prefer_ext_connect_params_set(uint8_t phy_mask, struct appm_create_conn_param *phy_1m_conn_params,
        struct appm_create_conn_param *phy_2m_conn_params, struct appm_create_conn_param *phy_coded_conn_params)
{
    ble_err_t ret = ERR_SUCCESS;
    ext_conn_param_t pref_param;
    memset(&pref_param, 0, sizeof(pref_param));
    pref_param.phy_mask = phy_mask;

    if (phy_mask & GAPM_INIT_PROP_1M_BIT) {
        memcpy(&pref_param.conn_param_1m, phy_1m_conn_params, sizeof(struct appm_create_conn_param));
    }

    if (phy_mask & GAPM_INIT_PROP_2M_BIT) {
        memcpy(&pref_param.conn_param_2m, phy_2m_conn_params, sizeof(struct appm_create_conn_param));
    }

    if (phy_mask & GAPM_INIT_PROP_CODED_BIT) {
        memcpy(&pref_param.conn_param_coded, phy_coded_conn_params, sizeof(struct appm_create_conn_param));
    }

    appm_set_gap_prefer_ext_connect_params(&pref_param);

    return ret;
}

ble_err_t bk_ble_create_init(uint8_t con_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_CREATE_INIT_POS;
        app_ble_run(con_idx, BLE_INIT_CREATE, op_mask, callback);
        ret = appm_create_initing(con_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
            ret = ERR_INIT_CREATE;
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_init_set_connect_dev_addr(unsigned char connidx,struct bd_addr *bdaddr,unsigned char addr_type)
{
    struct bd_addr peer_bdaddr;

    memcpy(peer_bdaddr.addr,bdaddr->addr,6);
    appm_set_connect_dev_addr(connidx,bdaddr,addr_type);

    return ERR_SUCCESS;
}

ble_err_t bk_ble_init_start_conn(uint8_t con_idx,uint16_t con_dev_time, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_INIT_START_POS;
        app_ble_run(con_idx, BLE_INIT_START_CONN, op_mask, callback);
        ret = appm_start_connecting(con_idx,con_dev_time);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
            ret = ERR_INIT_CREATE;
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_init_stop_conn(uint8_t con_idx,ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_INIT_STOP_POS;
        app_ble_run(con_idx, BLE_INIT_STOP_CONN, op_mask, callback);
        ret = appm_stop_connencting(con_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
            ret = ERR_INIT_CREATE;
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

ble_err_t bk_ble_delete_init(uint8_t actv_idx, ble_cmd_cb_t callback)
{
    uint32_t op_mask;
    ble_err_t ret = ERR_SUCCESS;

    if (app_ble_env_state_get() == APP_BLE_READY) {
        op_mask = 1 << BLE_OP_INIT_DEL_POS;
        app_ble_run(actv_idx, BLE_INIT_DELETE, op_mask, callback);
        ret = appm_delete_initing(actv_idx);
        if (ret != ERR_SUCCESS) {
            app_ble_reset();
        }
    } else {
        bk_printf("ble is not ready\r\n");
        ret = ERR_BLE_STATUS;
    }

    return ret;
}

void bk_ble_sdp_register_filt_service_tab(unsigned char service_tab_nb,app_sdp_service_uuid *service_tab)
{
    //app_sdp_env.service_tab_nb = service_tab_nb;
    //app_sdp_env.service_tab = service_tab;
}
#endif

#if (BLE_GATT_CLI)
ble_err_t bk_ble_read_service_data_by_handle_req(uint8_t conidx, uint16_t handle)
{
    return sdp_svc_read_characteristic(conidx, handle, 0, 0);
}

ble_err_t bk_ble_write_service_data_req(uint8_t conidx, uint16_t handle, uint16_t data_len, uint8_t *data)
{
    return sdp_svc_write_characteristic(conidx, handle, data_len, data);
}

ble_err_t bk_ble_gattc_read_by_type(uint8_t conn_idx, uint16_t start_handle,
                                    uint16_t end_handle, uint8_t uuid_type, uint8_t *uuid)
{
    charac_uuid_t p_uuid;

    p_uuid.uuid_type = uuid_type;
    memcpy(p_uuid.uuid, uuid, GATT_UUID_128_LEN);

    return app_ble_gattc_read_by_type(conn_idx, start_handle, end_handle, &p_uuid);
}

ble_err_t bk_ble_gattc_read_multiple(uint8_t conn_idx, app_gattc_multi_t *read_multi)
{
    return app_ble_gattc_read_multiple(conn_idx, read_multi);
}

ble_err_t bk_ble_gattc_register_for_notify(uint8_t conidx, uint16_t handle)
{
    ble_err_t ret = ERR_SUCCESS;

    uint8_t data[2] = {0x01, 0x00};

    if (sdp_svc_write_characteristic(conidx, handle, 2, data)) {
        ret = ERR_CMD_RUN;
    }
    return ret;
}

ble_err_t bk_ble_gattc_register_for_indicate(uint8_t conidx, uint16_t handle)
{
    ble_err_t ret = ERR_SUCCESS;

    uint8_t data[2] = {0x02, 0x00};

    if (sdp_svc_write_characteristic(conidx, handle, 2, data)) {
        ret = ERR_CMD_RUN;
    }
    return ret;
}

ble_err_t bk_ble_gattc_unregister_for_notify_or_indicate(uint8_t conidx, uint16_t handle)
{
    ble_err_t ret = ERR_SUCCESS;

    uint8_t data[2] = {0};

    if (sdp_svc_write_characteristic(conidx, handle, 2, data)) {
        ret = ERR_CMD_RUN;
    }
    return ret;
}
#endif

#if (BLE_APP_SEC)
ble_err_t bk_ble_get_bond_device_num(uint8_t *dev_num)
{
    return app_ble_get_bonded_device_num(dev_num);
}

ble_err_t bk_ble_get_bonded_device_list(uint8_t *dev_num, bond_device_addr_t *dev_list)
{
    ble_err_t ret = ERR_SUCCESS;
    ret = app_ble_get_bonded_device_list(dev_num, dev_list);
    return ret;
}

sec_err_t bk_ble_remove_bond_device(uint8_t idx, bool disconnect)
{
    return app_sec_remove_bond(idx, disconnect);
}

sec_err_t bk_ble_gap_set_security_param(struct app_pairing_cfg *param, sec_notice_cb_t func)
{
    return app_sec_config(param, func);
}

sec_err_t bk_ble_security_req(uint8_t conn_idx)
{
    return app_sec_send_security_req(conn_idx);
}

sec_err_t bk_ble_security_start(uint8_t conn_idx)
{
    return app_sec_master_security_start(conn_idx);
}

sec_err_t bk_ble_pairing_rsp(uint8_t conn_idx, bool accept)
{
    return app_sec_send_pairing_rsp(conn_idx, accept);
}

sec_err_t bk_ble_passkey_reply(uint8_t conn_idx, bool accept, uint32_t passkey)
{
    return app_sec_tk_exchange_cfm(conn_idx, passkey, accept);
}

sec_err_t bk_ble_confirm_reply(uint8_t conn_idx, bool accept)
{
    return app_sec_nc_exchange_cfm(conn_idx, accept);
}
#endif

ble_err_t bk_ble_get_sendable_packets_num(uint16_t *pkt_total)
{
    return app_ble_get_sendable_packets_num(pkt_total);
}

ble_err_t bk_ble_get_cur_sendable_packets_num(uint16_t *pkt_curr)
{
    return app_ble_get_cur_sendable_packets_num(pkt_curr);
}

ble_err_t bk_ble_gatt_mtu_change(uint8_t conn_idx)
{
    ble_err_t ret = ERR_SUCCESS;
    ret = app_ble_mtu_exchange(conn_idx);
    return ret;
}

ble_err_t bk_ble_set_pref_mtu(uint8_t conn_idx,uint16_t pref_mtu)
{
    ble_err_t ret = ERR_SUCCESS;
    ret = app_ble_set_pref_mtu(conn_idx,pref_mtu);
    return ret;
}

ble_err_t bk_ble_get_con_mtu(uint8_t con_idx,uint16_t *mtu)
{
    if(con_idx >= BLE_CONNECTION_MAX)
        return ERR_BLE_STATUS;

    uint8_t conhdl = app_ble_env.connections[con_idx].conhdl;

    if ((conhdl == UNKNOW_CONN_HDL) || (conhdl == USED_CONN_HDL)) {
        return ERR_BLE_STATUS;
    }
    app_ble_mtu_get(con_idx,mtu);

    return ERR_SUCCESS;
}

ble_err_t bk_ble_gap_read_phy(uint8_t conn_idx, ble_read_phy_t *phy)
{
    ble_err_t ret = ERR_SUCCESS;
    ret = app_ble_gap_read_phy(conn_idx, phy);
    return ret;
}

ble_err_t bk_ble_gap_set_phy(uint8_t conn_idx, ble_set_phy_t *phy)
{
    ble_err_t ret = ERR_SUCCESS;
    ret = app_ble_gap_set_phy(conn_idx, phy);
    return ret;
}

ble_err_t bk_ble_gatts_app_unregister(uint16_t service_handle)
{
    ble_err_t ret = ERR_SUCCESS;
    gatt_db_svc_t *p_svc = NULL;

    ret = gatt_db_svc_find(service_handle, &p_svc);
    if (ret == ERR_SUCCESS) {
        ret = app_ble_gatts_app_unregister(p_svc->user_lid, service_handle);
    }

    return ret;
}

ble_err_t bk_ble_gatts_remove_service(uint16_t start_handle)
{
    ble_err_t ret = ERR_SUCCESS;
    gatt_db_svc_t *p_svc = NULL;

    ret = gatt_db_svc_find(start_handle, &p_svc);
    if (ret == ERR_SUCCESS) {
        ret = app_ble_gatts_remove_service(p_svc->user_lid, start_handle);
    }

    return ret;
}

ble_err_t bk_ble_gatts_remove_service_by_prf_id(uint16_t prf_task_id)
{
    ble_err_t ret = ERR_SUCCESS;
    gatt_db_svc_t *p_svc = NULL;

    prf_data_t *ble_prf = (prf_data_t*)prf_data_get_by_task_id(prf_task_id + TASK_BLE_ID_COMMON);

    struct bk_ble_env_tag *ble_env = (struct bk_ble_env_tag*)ble_prf->p_env;

    ret = gatt_db_svc_find(ble_env->start_hdl, &p_svc);
    if (ret == ERR_SUCCESS) {
        ret = app_ble_gatts_remove_service(p_svc->user_lid, ble_env->start_hdl);
    }

    return ret;
}

ble_err_t bk_ble_gatts_read_response(app_gatts_rsp_t *rsp)
{
    return app_ble_gatts_read_response(rsp);
}

ble_err_t bk_ble_gatts_set_attr_value(uint16_t attr_handle, uint16_t length, uint8_t *value)
{
    return app_ble_gatts_set_attr_value(attr_handle, length, value);
}

ble_err_t bk_ble_gatts_get_attr_value(uint16_t attr_handle, uint16_t *length, uint8_t **value)
{
    return app_ble_gatts_get_attr_value(attr_handle, length, value);
}

ble_err_t bk_ble_gatts_send_service_change_indication(uint16_t start_handle, uint16_t end_handle)
{
    return app_ble_gatts_svc_chg_ind_send(start_handle, end_handle);
}

ble_err_t bk_ble_gatts_stop_service(uint16_t service_handle)
{
    return ERR_CMD_NOT_SUPPORT;
}

