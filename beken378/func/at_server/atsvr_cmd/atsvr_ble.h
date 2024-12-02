#ifndef _ATSVR_BLE_H_
#define _ATSVR_BLE_H_

#include "ble_api_5_x.h"

#define MAX_NAME_SIZE                    16
#define BK_ATT_DECL_PRIMARY_SERVICE     {0x00,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_ATT_DECL_CHARACTERISTIC      {0x03,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_ATT_DESC_CLIENT_CHAR_CFG     {0x02,0x29,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BLE_DEVICE_NAME_SIZE                    18

extern struct bd_addr common_default_bdaddr;

typedef struct
{
    uint8_t  scan_idx;
    uint8_t  scan_type;
    uint8_t  own_addr_type;
    uint8_t  filter_policy;
    uint16_t scan_intvl;
    uint16_t scan_wd;
    uint16_t timeout;
    uint8_t  filter_type;
    uint16_t filter_param[32];
    uint8_t scan_state;

} ble_scan_param_t;

typedef struct
{
    uint8_t  adv_idx;
    uint8_t  channel_map;
    uint8_t  adv_type;
    uint16_t interval_min;
    uint16_t interval_max;
    uint16_t advDataLen;
    uint8_t  advData[MAX_ADV_DATA_LEN];
    uint16_t respDataLen;
    uint8_t  respData[MAX_ADV_DATA_LEN];
    char adv_name[MAX_NAME_SIZE];
    uint8_t uuid_len;
    uint8_t uuid[16];
    uint8_t manufacturer_data_len;
    uint8_t manufacturer_data[16];
    uint8_t include_power;
} ble_adv_param_t;

typedef struct
{
    /// Connection interval
    uint16_t con_interval;
    /// Connection latency
    uint16_t con_latency;
    /// Link supervision timeout
    uint16_t sup_to;
    uint8_t con_state;
} ble_conn_param_t;

typedef struct
{
    uint8_t ble_device_name[BLE_DEVICE_NAME_SIZE];
    uint8_t mode;
    uint8_t connidx;        //slave conn_idx
    uint8_t sdp_conn_idx;    //master conn_indx
    uint8_t sdp_svr_ing;
    uint8_t sdp_char_ing;
    uint8_t remote_address[6];
    ble_scan_param_t scan;
    ble_adv_param_t adv;
    ble_conn_param_t conn;
} hal_ble_env_t;

typedef struct Bt_Server
{
    uint8_t *uuid;
    unsigned char uuidLen;
    unsigned char service_id;
    unsigned char start;
    unsigned char max_att_handle;
    bk_attm_desc_t *att_db;
    struct Bt_Server *next;
} BtServer;

typedef struct {
    uint8_t srv_num;
    BtServer *srv;
} ServerList_t;

typedef struct {
    char  ble_device_name[BLE_DEVICE_NAME_SIZE];
    uint8_t  adv_type;
    uint16_t interval_min;
    uint16_t interval_max;
    uint8_t  channel_map;
    uint8_t  uuid_len;
    uint8_t  uuid[16];
    uint8_t  manufacturer_data_len;
    uint8_t  manufacturer_data[16];
    uint8_t  include_power;
    uint16_t advDataLen;
    uint8_t  advData[MAX_ADV_DATA_LEN];
    uint16_t respDataLen;
    uint8_t  respData[MAX_ADV_DATA_LEN];
    uint8_t  scan_type;
    uint8_t  own_addr_type;
    uint8_t  filter_policy;
    uint16_t scan_intvl;
    uint16_t scan_wd;
    uint16_t con_interval;
    uint16_t con_latency;
    uint16_t sup_to;
} BLE_PARAM_T;

enum
{
    BK_Attr_IDX_SVC,
    BK_Attr_IDX_CHAR,
    BK_Attr_IDX_CLIENT_CHAR_CFG,
    BK_Attr_IDX_NB,
};

void at_ble_cmd_init(void);

#endif
