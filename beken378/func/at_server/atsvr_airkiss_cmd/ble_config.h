#ifndef __ATSVR_BLE_CONFIG_H_
#define __ATSVR_BLE_CONFIG_H_

#include "app_ble.h"
#include "app_sdp.h"
#include "app_ble_init.h"
#if CFG_USE_DISTRIBUTION_NETWORK
typedef struct ble_data {
    int type;
    int16_t length;
    char buffer[128];
}ble_data_t;
enum
{
    TYPE_MSG_DATA,			//send data
    MSG_EXIT_NET_CONFIG,	//eixt ble network
};

typedef enum {
    E_DEV_MSG_SET_WIFI_INFO,		//set wifi ssid and pwsswd
} e_dev_info_msg_type;

uint8_t ble_adv_init(void);
void bk_ble_notice_cb(ble_notice_t notice, void *param);
ble_err_t bk_ble_db_create(void);
void ble_adv_deinit(uint8_t actv_idx);
void ble_msg_cmd_push_to_queue(int type,char *data,int len);
#endif
#endif