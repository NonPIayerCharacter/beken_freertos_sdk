#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "rtos_pub.h"
#include "error.h"
#include "uart_pub.h"
#include "str_pub.h"
#include "drv_model_pub.h"
#if EDDYSTONE_DEMO
#include "eddystone_core.h"
#include "ble_api_5_x.h"
#include "app_ble.h"
#include "gatt.h"

typedef struct {
	ble_ecs_brdcst_cap_t    brdcst_cap;
	ble_ecs_active_slot_t   active_slot;
	ble_ecs_adv_intrvl_t    adv_intrvl;
	ble_ecs_radio_tx_pwr_t  radio_tx_pwr;
	ble_ecs_adv_tx_pwr_t    adv_tx_pwr;
	ble_ecs_lock_state_t    lock_state;
	ble_ecs_unlock_t        unlock;
	ble_ecs_rw_adv_slot_t   rw_adv_slot;
	ble_ecs_fac_reset_t     fac_reset;
	ble_ecs_remain_conn_t   remain_conn;
} ble_ecs_init_params_t;

ble_ecs_init_params_t init_params;
uint8_t data_adv[CHAR_LENGTH_MAX] = {0};
uint8_t data_length = 0;
uint8_t url = 0;
uint8_t tx_power = 0xFC;
uint8_t eddystone_default_data[] = DEFAULT_FRAME_DATA;
uint8_t tlm[] = DEFAULT_TLM_FRAME_DATA;
uint8_t APP_CONFIG_LOCK_CODE[ECS_KEY_SIZE]= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} ;
uint8_t eddystone_uuid[16] = {0x95, 0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf, 0x4b, 0xd3, 0x8e, 0x00, 0x75, 0xc8, 0xa3};

bk_attm_desc_t eddystone_att_db[EDDYSTONE_IDX_NB] =
{
	[EDDYSTONE_SVC]                     = {BLE_GATT_DECL_PRIMARY_SERVICE, PROP(RD), 0},

	[EDDYSTONE_BRDCST_CAP_CHAR]         = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_BRDCST_CAP_VALUE]        = {BLE_UUID_ECS_BRDCST_CAP_CHAR,       PROP(RD)|SEC_LVL_VAL(UUID_TYPE,2), 7|OPT(NO_OFFSET)},

	[EDDYSTONE_ACTIVE_SLOT_CHAR]        = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_ACTIVE_SLOT_VALUE]       = {BLE_UUID_ECS_ACTIVE_SLOT_CHAR,      PROP(RD)|PROP(WR)|SEC_LVL_VAL(UUID_TYPE,2), 1|OPT(NO_OFFSET)},

	[EDDYSTONE_ADV_INTRVL_CHAR]         = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_ADV_INTRVL_VALUE]        = {BLE_UUID_ECS_ADV_INTRVL_CHAR,       PROP(RD)|PROP(WR)|SEC_LVL_VAL(UUID_TYPE,2), 2|OPT(NO_OFFSET)},

	[EDDYSTONE_RADIO_TX_PWR_CHAR]       = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_RADIO_TX_PWR_VALUE]      = {BLE_UUID_ECS_RADIO_TX_PWR_CHAR,     PROP(RD)|PROP(WR)|SEC_LVL_VAL(UUID_TYPE,2), 1|OPT(NO_OFFSET)},

	[EDDYSTONE_ADV_TX_PWR_CHAR]         = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_ADV_TX_PWR_VALUE]        = {BLE_UUID_ECS_ADV_TX_PWR_CHAR,       PROP(RD)|PROP(WR)|SEC_LVL_VAL(UUID_TYPE,2), 1|OPT(NO_OFFSET)},

	[EDDYSTONE_LOCK_STATE_CHAR]         = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_LOCK_STATE_VALUE]        = {BLE_UUID_ECS_LOCK_STATE_CHAR,       PROP(RD)|PROP(WR)|SEC_LVL_VAL(UUID_TYPE,2), 17|OPT(NO_OFFSET)},

	[EDDYSTONE_UNLOCK_CHAR]             = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_UNLOCK_VALUE]            = {BLE_UUID_ECS_UNLOCK_CHAR,           PROP(RD)|PROP(WR)|SEC_LVL_VAL(UUID_TYPE,2), 16|OPT(NO_OFFSET)},

	[EDDYSTONE_PUBLIC_ECDH_KEY_CHAR]    = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_PUBLIC_ECDH_KEY_VALUE]   = {BLE_UUID_ECS_PUBLIC_ECDH_KEY_CHAR,  PROP(RD)|SEC_LVL_VAL(UUID_TYPE,2), 32|OPT(NO_OFFSET)},

	[EDDYSTONE_EID_ID_KEY_CHAR]         = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_EID_ID_KEY_VALUE]        = {BLE_UUID_ECS_EID_ID_KEY_CHAR,       PROP(RD)|SEC_LVL_VAL(UUID_TYPE,2), 16|OPT(NO_OFFSET)},

	[EDDYSTONE_RW_ADV_SLOT_CHAR]        = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_RW_ADV_SLOT_VALUE]       = {BLE_UUID_ECS_RW_ADV_SLOT_CHAR,      PROP(RD)|PROP(WR)|SEC_LVL_VAL(UUID_TYPE,2), 34|OPT(NO_OFFSET)},

	[EDDYSTONE_FAC_RESET_CHAR]          = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_FAC_RESET_VALUE]         = {BLE_UUID_FAC_RESET_CHAR,            PROP(WR)|SEC_LVL_VAL(UUID_TYPE,2), 1|OPT(NO_OFFSET)},

	[EDDYSTONE_REMAIN_CONN_CHAR]        = {BLE_GATT_DECL_CHARACTERISTIC,  PROP(RD), 0},
	[EDDYSTONE_REMAIN_CONN_VALUE]       = {BLE_UUID_REMAIN_CONN_CHAR,          PROP(RD)|PROP(WR)|SEC_LVL_VAL(UUID_TYPE,2), 1|OPT(NO_OFFSET)},
};

void eddystone_services_init(void)
{
	int8_t tx_powers[ECS_NUM_OF_SUPORTED_TX_POWER] = ECS_SUPPORTED_TX_POWER;
	/*Init the broadcast capabilities characteristic*/
	memset(&init_params.brdcst_cap, 0, sizeof(init_params.brdcst_cap));
	init_params.brdcst_cap.max_supp_total_slots = APP_MAX_ADV_SLOTS;
	init_params.brdcst_cap.max_supp_eid_slots   = APP_MAX_EID_SLOTS;
	init_params.brdcst_cap.cap_bitfield         = ( (APP_IS_VARIABLE_ADV_SUPPORTED << ECS_BRDCST_VAR_ADV_SUPPORTED_Pos)
													| (APP_IS_VARIABLE_TX_POWER_SUPPORTED << ECS_BRDCST_VAR_TX_POWER_SUPPORTED_Pos))
													& (ECS_BRDCST_VAR_RFU_MASK);
	init_params.brdcst_cap.supp_frame_types     = ( (APP_IS_UID_SUPPORTED << ECS_FRAME_TYPE_UID_SUPPORTED_Pos)
													| (APP_IS_URL_SUPPORTED << ECS_FRAME_TYPE_URL_SUPPORTED_Pos)
													| (APP_IS_TLM_SUPPORTED << ECS_FRAME_TYPE_TLM_SUPPORTED_Pos)
													| (APP_IS_EID_SUPPORTED << ECS_FRAME_TYPE_EID_SUPPORTED_Pos))
													& (ECS_FRAME_TYPE_RFU_MASK);
	init_params.brdcst_cap.supp_frame_types = ((init_params.brdcst_cap.supp_frame_types << 8) & 0xFF00) | ((init_params.brdcst_cap.supp_frame_types >> 8) & 0xFF);
	memcpy(init_params.brdcst_cap.supp_radio_tx_power, tx_powers, ECS_NUM_OF_SUPORTED_TX_POWER);
	/*Init the active slots characteristic*/
	init_params.active_slot = 0;
	/*Init the advertising intervals characteristic*/
	init_params.adv_intrvl = ((APP_CFG_NON_CONN_ADV_INTERVAL_MS << 8) & 0xFF00) | ((APP_CFG_NON_CONN_ADV_INTERVAL_MS >> 8) & 0xFF);
	/*Init the radio tx power characteristic*/
	init_params.radio_tx_pwr = APP_CFG_DEFAULT_RADIO_TX_POWER;
	init_params.adv_tx_pwr = APP_CFG_DEFAULT_RADIO_TX_POWER;
	/*Init the lock state characteristic*/
	init_params.lock_state.read = BLE_ECS_LOCK_STATE_LOCKED;
	memcpy(init_params.unlock.lock, APP_CONFIG_LOCK_CODE, ECS_KEY_SIZE);
	init_params.rw_adv_slot.frame_type = (eddystone_frame_type_t)(DEFAULT_FRAME_TYPE);
	init_params.rw_adv_slot.p_data = eddystone_default_data;
	init_params.rw_adv_slot.char_length = sizeof(eddystone_default_data) + 1; // plus the frame_type
	init_params.fac_reset = 0;
	init_params.remain_conn = 0x01;
}

void ble_eddystone_adv_data_get(void)
{
	uint8_t temp[CHAR_LENGTH_MAX] = {0};
	memcpy(temp, &(init_params.rw_adv_slot.frame_type), EDDYSTONE_FRAME_TYPE_LENGTH);
	if (init_params.rw_adv_slot.frame_type == EDDYSTONE_FRAME_TYPE_TLM) {
		data_length = EDDYSTONE_TLM_FRAME_LENGTH;
		memcpy(init_params.rw_adv_slot.p_data, tlm, data_length - EDDYSTONE_FRAME_TYPE_LENGTH);
		memcpy(&(temp[1]), init_params.rw_adv_slot.p_data, (data_length - EDDYSTONE_FRAME_TYPE_LENGTH));
	} else if (init_params.rw_adv_slot.frame_type == EDDYSTONE_FRAME_TYPE_UID) {
		data_length = init_params.rw_adv_slot.char_length + 1;
		memcpy(&(temp[1]), &(tx_power), EDDYSTONE_TX_POWER_LENGTH);
		memcpy(&(temp[2]), init_params.rw_adv_slot.p_data, (data_length - EDDYSTONE_FRAME_TYPE_LENGTH));
	} else if (init_params.rw_adv_slot.frame_type == EDDYSTONE_FRAME_TYPE_URL) {
		if (url == 0) {
			data_length = init_params.rw_adv_slot.char_length;
			memcpy(&(temp[1]), init_params.rw_adv_slot.p_data, (data_length - EDDYSTONE_FRAME_TYPE_LENGTH));
			url = 1;
		} else {
			data_length = init_params.rw_adv_slot.char_length + 1;
			memcpy(&(temp[1]), &(tx_power), EDDYSTONE_TX_POWER_LENGTH);
			memcpy(&(temp[2]), init_params.rw_adv_slot.p_data, (data_length - EDDYSTONE_FRAME_TYPE_LENGTH));
		}
	}
	memcpy(data_adv, temp, data_length);
}

ble_err_t bk_eddystone_init(void)
{
	ble_err_t status = ERR_SUCCESS;
	struct bk_ble_db_cfg ble_db_cfg;
	ble_db_cfg.att_db = eddystone_att_db;
	ble_db_cfg.att_db_nb = EDDYSTONE_IDX_NB;
	ble_db_cfg.prf_task_id = 0;
	ble_db_cfg.start_hdl = 0;
	ble_db_cfg.svc_perm = BK_PERM_SET(SVC_UUID_LEN, UUID_128);
	memcpy(&(ble_db_cfg.uuid[0]), &eddystone_uuid, 16);
	status = bk_ble_create_db(&ble_db_cfg);
	return status;
}

ble_err_t bk_ble_adv_start_first(uint8_t actv_idx, struct adv_param *adv, ble_cmd_cb_t callback)
{
	uint32_t op_mask;
	ble_err_t ret = ERR_SUCCESS;
	if (app_ble_env_state_get() == APP_BLE_READY) {
		if (actv_idx < BLE_ACTIVITY_MAX) {
			op_mask = (1 << BLE_OP_CREATE_ADV_POS) | (1 << BLE_OP_SET_ADV_DATA_POS) | (1 << BLE_OP_START_ADV_POS);
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

ble_err_t bk_ble_write_event_private(void *param)
{
	uint16_t retval = BK_ERR_BLE_SUCCESS;
	write_req_t *w_req = (write_req_t *)param;
	uint8_t temp[CHAR_LENGTH_MAX] = {0};
	bk_printf("write:conn_idx:%d, prf_id:%d, add_id:%d, len:%d, data[0]:%02x\r\n",
			w_req->conn_idx, w_req->prf_id, w_req->att_idx, w_req->len, w_req->value[0]);

	if ((w_req->att_idx == EDDYSTONE_UNLOCK_VALUE) && (init_params.lock_state.read == BLE_ECS_LOCK_STATE_LOCKED)) {
		memcpy(&(init_params.unlock.lock), &(w_req->value[0]), w_req->len);
		init_params.lock_state.read = BLE_ECS_LOCK_STATE_UNLOCKED;
	} else if (init_params.lock_state.read == BLE_ECS_LOCK_STATE_UNLOCKED) {
		switch (w_req->att_idx) {
			case EDDYSTONE_ACTIVE_SLOT_VALUE:
			{
				memcpy(&(init_params.active_slot), &(w_req->value[0]), w_req->len);
				break;
			}
			case EDDYSTONE_ADV_INTRVL_VALUE:
			{
				memcpy(&(init_params.adv_intrvl), &(w_req->value[0]), w_req->len);
				break;
			}
			case EDDYSTONE_RADIO_TX_PWR_VALUE:
			{
				memcpy(&(init_params.radio_tx_pwr), &(w_req->value[0]), w_req->len);
				break;
			}
			case EDDYSTONE_ADV_TX_PWR_VALUE:
			{
				memcpy(&(init_params.adv_tx_pwr), &(w_req->value[0]), w_req->len);
				break;
			}
			case EDDYSTONE_LOCK_STATE_VALUE:
			{
				memcpy(&(init_params.lock_state.write), &(w_req->value[0]), w_req->len);
				init_params.lock_state.read = init_params.lock_state.write.lock_byte;
				break;
			}
			case EDDYSTONE_RW_ADV_SLOT_VALUE:
			{
				init_params.rw_adv_slot.char_length = w_req->len;
				init_params.rw_adv_slot.frame_type = w_req->value[0];
				memcpy(&(temp[0]), &(w_req->value[1]), (w_req->len - EDDYSTONE_FRAME_TYPE_LENGTH));
				memcpy(init_params.rw_adv_slot.p_data, temp, (w_req->len - EDDYSTONE_FRAME_TYPE_LENGTH));
				break;
			}
			case EDDYSTONE_FAC_RESET_VALUE:
			{
				init_params.fac_reset= w_req->value[0];
				if (init_params.fac_reset == 0x0B) {
					bk_printf("Advanced Implementation,Not In Place Yet.\r\n");
					init_params.fac_reset = 0;
				}
				break;
			}
			case EDDYSTONE_REMAIN_CONN_VALUE:
			{
				memcpy(&(init_params.remain_conn), &(w_req->value[0]), w_req->len);
				break;
			}
			default:
				bk_printf("unknown write\r\n");
				break;
		}
	}
	return retval;
}

ble_err_t bk_ble_read_event_private(void *param)
{
	uint16_t retval = BK_ERR_BLE_SUCCESS;
	read_req_t *r_req = (read_req_t *)param;
	bk_printf("read:conn_idx:%d, prf_id:%d, add_id:%d\r\n",r_req->conn_idx, r_req->prf_id, r_req->att_idx);

	if (r_req->att_idx == EDDYSTONE_LOCK_STATE_VALUE) {
		r_req->length = sizeof(init_params.lock_state.read);
		memcpy(&(r_req->value[0]), &(init_params.lock_state.read), r_req->length);
	} else if ((r_req->att_idx == EDDYSTONE_UNLOCK_VALUE) && (init_params.lock_state.read == BLE_ECS_LOCK_STATE_LOCKED)) {
		r_req->length = ECS_KEY_SIZE;
		for (int i = 0;i < ECS_KEY_SIZE;i++) {
			extern int bk_rand();
			init_params.unlock.lock[i] = (uint8_t)bk_rand();
		}
		memcpy(&(r_req->value[0]), &(init_params.unlock.lock), r_req->length);
	} else if (init_params.lock_state.read == BLE_ECS_LOCK_STATE_UNLOCKED) {
		switch (r_req->att_idx) {
			case EDDYSTONE_BRDCST_CAP_VALUE:
			{
				r_req->length = sizeof(init_params.brdcst_cap);
				memcpy(&(r_req->value[0]), &(init_params.brdcst_cap), r_req->length);
				break;
			}
			case EDDYSTONE_ACTIVE_SLOT_VALUE:
			{
				r_req->length = sizeof(init_params.active_slot);
				memcpy(&(r_req->value[0]), &(init_params.active_slot), r_req->length);
				break;
			}
			case EDDYSTONE_ADV_INTRVL_VALUE:
			{
				r_req->length = sizeof(init_params.adv_intrvl);
				memcpy(&(r_req->value[0]), &(init_params.adv_intrvl), r_req->length);
				break;
			}
			case EDDYSTONE_RADIO_TX_PWR_VALUE:
			{
				r_req->length = sizeof(init_params.radio_tx_pwr);
				memcpy(&(r_req->value[0]), &(init_params.radio_tx_pwr), r_req->length);
				break;
			}
			case EDDYSTONE_ADV_TX_PWR_VALUE:
			{
				r_req->length = sizeof(init_params.adv_tx_pwr);
				memcpy(&(r_req->value[0]), &(init_params.adv_tx_pwr), r_req->length);
				break;
			}
			case EDDYSTONE_PUBLIC_ECDH_KEY_VALUE:
			{
				for (uint8_t i = 0 ; i < 32; i++) {
					extern int bk_rand();
					r_req->value[i] = (uint8_t)bk_rand();
				}
				r_req->length = 32;
				break;
			}
			case EDDYSTONE_EID_ID_KEY_VALUE:
			{
				for (int i = 0;i < ECS_KEY_SIZE;i++) {
					extern int bk_rand();
					r_req->value[i] = (uint8_t)bk_rand();
				}
				r_req->length = ECS_KEY_SIZE;
				break;
			}
			case EDDYSTONE_RW_ADV_SLOT_VALUE:
			{
				if (init_params.active_slot == 0) {
					ble_eddystone_adv_data_get();
					memcpy(r_req->value, data_adv, data_length);
					r_req->length = data_length;
				} else {
					r_req->length = 0;
				}
				break;
			}
			case EDDYSTONE_REMAIN_CONN_VALUE:
			{
				r_req->length = sizeof(init_params.remain_conn);
				memcpy(&(r_req->value[0]), &(init_params.remain_conn), r_req->length);
				if (init_params.remain_conn == 0x1) {
					ble_eddystone_post_msg(EDDYSTONE_ADV_EVENT, NULL, 0);
				}
				break;
			}
			default:
				bk_printf("unknown read\r\n");
				break;
		}
	} else {
		r_req->length = 0;
	}
	return retval;
}

ble_err_t bk_ble_adv_event_private(void *param)
{
	uint16_t retval = BK_ERR_BLE_SUCCESS;
	uint8_t adv_data[31];
	uint8_t data_len;
	adv_data[0] = 0x03;
	adv_data[1] = 0x03;
	adv_data[2] = 0xaa;
	adv_data[3] = 0xfe;
	adv_data[4] = data_length + 3;
	adv_data[5] = 0x16;
	adv_data[6] = 0xaa;
	adv_data[7] = 0xfe;
	memcpy(&adv_data[8], data_adv, data_length);
	data_len = data_length + 8;
	bk_ble_set_adv_data(0, adv_data, data_len, ble_eddystone_cmd_cb);
	return retval;
}

ble_err_t bk_ble_connect_event_private(void *param)
{
	uint16_t retval = BK_ERR_BLE_SUCCESS;
	conn_ind_t *c_ind = (conn_ind_t *)param;
	bk_printf("connect_ind:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
				c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
				c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);
	return retval;
}

ble_err_t bk_ble_disconnect_event_private(void *param)
{
	uint16_t retval = BK_ERR_BLE_SUCCESS;
	discon_ind_t *d_ind = (discon_ind_t *)param;
	bk_printf("disconnect_ind:conn_idx:%d,reason:%d\r\n", d_ind->conn_idx,d_ind->reason);
	return retval;
}

ble_err_t bk_ble_init_conn_papam_update_data_req_event_private(void *param)
{
	uint16_t retval = BK_ERR_BLE_SUCCESS;
	conn_param_t *d_ind = (conn_param_t *)param;
	bk_printf("CONN_PARAM_UPDATE:conn_idx:%d,intv_min:%d,intv_max:%d,time_out:%d\r\n",d_ind->conn_idx,
				d_ind->intv_min,d_ind->intv_max,d_ind->time_out);
	app_ble_send_conn_param_update_cfm(d_ind->conn_idx, true);
	return retval;
}

void ble_eddystone_cmd_cb(ble_cmd_t cmd, ble_cmd_param_t *param)
{
	bk_printf("cmd:%d idx:%d status:%d\r\n", cmd, param->cmd_idx, param->status);
}
#endif
