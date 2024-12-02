#ifndef _SDP_COMM_H_
#define _SDP_COMM_H_

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT && BLE_GATT_CLI)
#include "rwprf_config.h"
#include "rwble_hl_config.h"
#include "prf_types.h"
#include "kernel_task.h"
#include "ble_api_5_x.h"
#include "prf_utils.h"

#define SDP_INVALID_CONIDX                (0xFF)
#define SDP_INVALID_USER_LID              (0xFF)
#define SDP_CHARACTERISTIC_VALUE_TYPE     (0x01)
#define SDP_CHARACTERISTIC_CCCD_TYPE      (0x02)

#define BLE_APP_SDP_LOG                1

#define BLE_APP_SDP_ALL                0x00
#define BLE_APP_SDP_WARN               0x01
#define BLE_APP_SDP_IMPO               0x02   ///important
#define BLE_APP_SDP_NONE               0xFF

#define BLE_APP_SDP_DBG_LEVEL          BLE_APP_SDP_IMPO

#define BLE_APP_SDP_DBG_CHECK(LEVEL)   (BLE_APP_SDP_LOG && ((LEVEL) >= BLE_APP_SDP_DBG_LEVEL))

struct db
{
	struct bk_prf_svc svc;
	uint8_t chars_nb;
	struct bk_prf_char_def *chars;
	uint8_t descs_nb;
	struct bk_prf_desc_def *descs;
};

struct sdp_db
{
	struct common_list_hdr hdr;
	struct db svr;
};

struct sdp_env_tag
{
	///Connection index
	uint8_t con_idx;
	/// GATT User local identifier
	uint8_t user_lib;
	///mtu size
	uint16_t mtu;
	/// server list
	struct common_list svr_list;
	/// State of db
	kernel_state_t state;
};

extern void sdp_common_init(void);
extern uint8_t sdp_common_create(uint8_t con_idx,uint16_t mtu);
extern uint8_t sdp_common_cleanup(uint8_t con_idx);
extern uint8_t sdp_discover_all_service(uint8_t con_idx);
extern struct sdp_env_tag * sdp_get_free_env(void);
extern struct sdp_env_tag * sdp_get_env_use_conidx(uint8_t con_idx);
extern void * sdp_get_db_use_handle(uint8_t con_idx,uint16_t handle,uint8_t *charac_type);
#endif  ////BLE_GATT_CLI
#endif  ///_SDP_COMM_H_


