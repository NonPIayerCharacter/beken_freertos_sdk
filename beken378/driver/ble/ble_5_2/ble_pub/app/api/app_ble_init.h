#ifndef _APP_BLE_INIT_H_
#define _APP_BLE_INIT_H_

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT && BLE_CENTRAL)

///#include "ble_api.h"
#include "gap.h"
#include "ble_api_5_x.h"
#include "common_bt_defines.h"

/// Scan interval
#define APP_CONN_SCAN_INTV                 (60)
/// Scan window
#define APP_CONN_SCAN_WD                   (30)
/// connection interval
#define APP_CONN_INTV                      (50)
/// connection latency
#define APP_CONN_LATENCY                   (0)
/// connection timeout
#define APP_CONN_SUP_TO                    (500)
/// duration of connection event
#define APP_CONN_CE_LEN_MIN                (10)
#define APP_CONN_CE_LEN_MAX                (20)

#define APP_INIT_REUSE_ACTV_IDX            0
#define APP_INIT_SET_STOP_CONN_TIMER       1
#define APP_INIT_STOP_CONN_TIMER_EVENT     1

//////////////////////////////////////////////////////
typedef struct app_ble_initing_env_tag
{
	struct gap_bdaddr g_bdaddr;
	struct{
		unsigned char actv_idx;
		#define BLE_INIT_IDX_NONE     0
		#define BLE_INIT_IDX_USED     1
		#define BLE_INIT_IDX_STOPED   2
		unsigned char state;   ///0:none,1.used,2:stoped
	}init_idx[BLE_CONNECTION_MAX];
}app_ble_initing_env_t;

extern int app_ble_master_appm_disconnect(uint8_t conidx);
extern void app_ble_initing_init(void);
#if APP_INIT_REUSE_ACTV_IDX
extern void appm_set_initing_actv_idx(unsigned char conidx,unsigned char actv_idx,unsigned char state);
#endif
#endif  ////BLE_CENTRAL

extern void appm_set_gap_prefer_ext_connect_params(ext_conn_param_t *pref_par);
extern ble_err_t appm_start_connecting(uint8_t con_idx,uint16_t con_dev_time);
extern ble_err_t appm_stop_connencting(uint8_t con_idx);
extern ble_err_t appm_delete_initing(uint8_t con_idx);
extern int appm_set_connect_dev_addr(unsigned char connidx,struct bd_addr *bdaddr,unsigned char addr_type);
extern ble_err_t appm_create_initing(uint8_t con_idx);

#endif  ///_APP_BLE_INIT_H_

