#ifndef _APP_SDP_H_
#define _APP_SDP_H_

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT && BLE_GATT_CLI)
#include "sdp_comm.h"

typedef enum{
	CHARAC_NOTIFY,
	CHARAC_INDICATE,
	CHARAC_READ,
	CHARAC_READ_DONE,
	CHARAC_WRITE_DONE,
}CHAR_TYPE;

typedef void (*app_sdp_callback)(uint8_t conidx,uint16_t chars_val_hdl,uint8_t uuid_len,uint8_t *uuid);
typedef void (*app_sdp_charac_callback)(CHAR_TYPE type,uint8 conidx,uint16_t hdl,uint16_t len,uint8 *data);

typedef struct{
	/// Service UUID Length
	uint8_t  uuid_len;
	/// Service UUID
	uint8_t  uuid[16];
}app_sdp_service_uuid;

typedef struct{
	/////Filter non-selected service tables
	uint8_t filtration;
	///////service tables number
	uint8_t service_tab_nb;
	////service tables:Only this table will be registered.But you need to set "filtration" flag
	app_sdp_service_uuid *service_tab;
	////Tell the properties of the registration
	app_sdp_callback sdp_cb;
	////Attribute data callback
	app_sdp_charac_callback charac_cb;
	app_sdp_comm_callback comm_cb;
}app_sdp_env_tag;

typedef enum{
	SDP_CHARAC_NOTIFY_EVENT,
	SDP_CHARAC_INDICATE_EVENT,
	SDP_CHARAC_READ,
	SDP_CHARAC_READ_DONE,
	SDP_CHARAC_WRITE_DONE,
	SDP_DISCOVER_SVR,
	SDP_DISCOVER_SVR_DONE,
	SDP_MAX,
}sdp_notice_t;

typedef enum sdp_att_type{
	SDP_ATT_GET_SVR_UUID_ALL,  ////Gets all the services for this connection
	SDP_ATT_GET_SVR_UUID_BY_SVR_UUID,
	SDP_ATT_GET_ATT_UUID_ALL,  ////Gets all the ATT's for this connection
	SDP_ATT_GET_ATT_DESC_UUID_ALL,  ////Gets all the ATT-DESC's for this connection
	SDP_ATT_SVR_ATT_BY_SVR_UUID, ////Gets all ATT's for this SVR-UUID for this connection
	SDP_ATT_SVR_ATT_DESC_BY_SVR_UUID, ///Gets all ATT-DESC's for this SVR-UUID for this connection
	SDP_ATT_SVR_ATT_AND_DESC_BY_SVR_UUID, ///Gets all ATT and ATT-DESC's for this SVR-UUID for this connection
	SDP_ATT_COMPLETE,
	SDP_ATT_ERROR,
}sdp_att_type_t;

struct sdp_att_event_t
{
	/// Event Type
	enum sdp_att_type type;
	uint8_t svr_id;
	uint8_t uuid_len;
	uint8_t uuid[16];
	///if start_hdl = end_hdl = 0,it is invaild
	uint16_t start_hdl;
	uint16_t end_hdl;
};

typedef struct{
	/// Connection index
	uint32_t con_idx;
	/// Attribute handle
	uint16_t hdl;
	/// Value length
	uint16_t value_length;
	///Value
	uint8_t *value;
	///status
	uint8_t status;
	///others
	uint16_t dummy;
}sdp_event_t;

typedef void (*sdp_notice_cb_t)(sdp_notice_t notice, void *param);
typedef void (*sdp_discovery_cb_t)(uint8_t con_idx,sdp_att_type_t notice, void *param);

extern void app_sdp_characteristic_callback_handler(uint8_t conidx,uint16_t chars_val_hdl,uint8_t uuid_len,uint8_t *uuid);
extern void app_sdp_charac_callback_handler(CHAR_TYPE type,uint8 conidx,uint16_t hdl,uint16_t len,uint8 *data);
extern void register_app_sdp_characteristic_callback(app_sdp_callback cb);
extern void register_app_sdp_charac_callback(app_sdp_charac_callback cb);
extern uint8_t sdp_svc_write_characteristic(uint8_t con_idx,uint16_t handle,uint16_t data_len,uint8_t *data);
extern uint8_t sdp_svc_read_characteristic(uint8_t con_idx,uint16_t handle,uint16_t offset,uint16_t length);
extern void register_app_sdp_service_tab(uint8_t service_tab_nb,app_sdp_service_uuid *service_tab);
extern void app_sdp_service_filtration(uint8_t en);
extern void sdp_set_notice_cb(sdp_notice_cb_t func);
extern void sdp_set_discovery_svc_cb(sdp_discovery_cb_t func);
extern uint8_t sdp_update_gatt_mtu(uint8_t con_idx);
extern uint8_t sdp_get_all_service(uint8_t con_idx);
extern uint8_t sdp_get_all_char(uint8_t con_idx, uint16_t start_hdl, uint16_t end_hdl);
extern uint8_t sdp_get_all_desc(uint8_t con_idx, uint16_t start_hdl, uint16_t end_hdl);
extern uint8_t sdp_get_att_table(uint8_t con_idx,struct sdp_att_event_t const *param);
extern uint16_t sdp_get_att_hdl(uint8_t con_idx,uint8_t srv_index,uint8_t char_index,uint8_t desc_index);
extern ble_err_t sdp_get_att_infor(uint8_t con_idx,struct sdp_att_event_t *param);

#endif  ////BLE_CENTRAL
#endif  ///_APP_SDP_H_

