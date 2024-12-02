#ifndef _EDDYSTONE_CORE_H_
#define _EDDYSTONE_CORE_H_
#include "bk_err.h"
#include "include.h"
#include "error.h"
#include "uart_pub.h"
#include "drv_model_pub.h"
#include "gatt.h"
#include "ble_api_5_x.h"
#include "string.h"
#include "app_ble.h"
#include "rtos_pub.h"
#include "mem_pub.h"
#include "gatt.h"
#include "kernel_mem.h"
#include "eddystone.h"

//EDDYSTONE SERVICE
enum
{
	EDDYSTONE_SVC,
	EDDYSTONE_BRDCST_CAP_CHAR,
	EDDYSTONE_BRDCST_CAP_VALUE,
	EDDYSTONE_ACTIVE_SLOT_CHAR,
	EDDYSTONE_ACTIVE_SLOT_VALUE,
	EDDYSTONE_ADV_INTRVL_CHAR,
	EDDYSTONE_ADV_INTRVL_VALUE,
	EDDYSTONE_RADIO_TX_PWR_CHAR,
	EDDYSTONE_RADIO_TX_PWR_VALUE,
	EDDYSTONE_ADV_TX_PWR_CHAR,
	EDDYSTONE_ADV_TX_PWR_VALUE,
	EDDYSTONE_LOCK_STATE_CHAR,
	EDDYSTONE_LOCK_STATE_VALUE,
	EDDYSTONE_UNLOCK_CHAR,
	EDDYSTONE_UNLOCK_VALUE,
	EDDYSTONE_PUBLIC_ECDH_KEY_CHAR,
	EDDYSTONE_PUBLIC_ECDH_KEY_VALUE,
	EDDYSTONE_EID_ID_KEY_CHAR,
	EDDYSTONE_EID_ID_KEY_VALUE,
	EDDYSTONE_RW_ADV_SLOT_CHAR,
	EDDYSTONE_RW_ADV_SLOT_VALUE,
	EDDYSTONE_FAC_RESET_CHAR,
	EDDYSTONE_FAC_RESET_VALUE,
	EDDYSTONE_REMAIN_CONN_CHAR,
	EDDYSTONE_REMAIN_CONN_VALUE,
	EDDYSTONE_IDX_NB,
};

//EDDYSTONE CONFIGS
#define BLE_UUID_ECS_BRDCST_CAP_CHAR          {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x01,0x75,0xc8,0xa3}
#define BLE_UUID_ECS_ACTIVE_SLOT_CHAR         {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x02,0x75,0xc8,0xa3}
#define BLE_UUID_ECS_ADV_INTRVL_CHAR          {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x03,0x75,0xc8,0xa3}
#define BLE_UUID_ECS_RADIO_TX_PWR_CHAR        {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x04,0x75,0xc8,0xa3}
#define BLE_UUID_ECS_ADV_TX_PWR_CHAR          {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x05,0x75,0xc8,0xa3}
#define BLE_UUID_ECS_LOCK_STATE_CHAR          {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x06,0x75,0xc8,0xa3}
#define BLE_UUID_ECS_UNLOCK_CHAR              {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x07,0x75,0xc8,0xa3}
#define BLE_UUID_ECS_PUBLIC_ECDH_KEY_CHAR     {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x08,0x75,0xc8,0xa3}
#define BLE_UUID_ECS_EID_ID_KEY_CHAR          {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x09,0x75,0xc8,0xa3}
#define BLE_UUID_ECS_RW_ADV_SLOT_CHAR         {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x0A,0x75,0xc8,0xa3}
#define BLE_UUID_FAC_RESET_CHAR               {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x0B,0x75,0xc8,0xa3}
#define BLE_UUID_REMAIN_CONN_CHAR             {0x95,0xe2,0xed,0xeb,0x1b,0xa0,0x39,0x8a,0xdf,0x4b,0xd3,0x8e,0x0C,0x75,0xc8,0xa3}
#define BLE_GATT_DECL_PRIMARY_SERVICE         {0x00,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BLE_GATT_DECL_CHARACTERISTIC          {0x03,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

#define APP_MAX_ADV_SLOTS                                   5
#define APP_MAX_EID_SLOTS                                   APP_MAX_ADV_SLOTS
#define ECS_NUM_OF_SUPORTED_TX_POWER                        (9)
#define ECS_SUPPORTED_TX_POWER                              {0xD8, 0xEC, 0xF0, 0xF4, 0xF8, 0xFC, 0x00, 0x03, 0x04}
/*Characteristic: Broadcast Capabilities*/
/* Field: ble_ecs_init_params_t.brdcst_cap.cap_bitfield*/
#define ECS_BRDCST_VAR_ADV_SUPPORTED_Yes                    (1)
#define ECS_BRDCST_VAR_ADV_SUPPORTED_No                     (0)
#define ECS_BRDCST_VAR_ADV_SUPPORTED_Pos                    (0)
#define ECS_BRDCST_VAR_ADV_SUPPORTED_Msk                    (1 << ECS_BRDCST_VAR_ADV_SUPPORTED_Pos)
#define ECS_BRDCST_VAR_TX_POWER_SUPPORTED_Yes               (1)
#define ECS_BRDCST_VAR_TX_POWER_SUPPORTED_No                (0)
#define ECS_BRDCST_VAR_TX_POWER_SUPPORTED_Pos               (1)
#define ECS_BRDCST_VAR_TX_POWER_SUPPORTED_Msk               (1 << ECS_BRDCST_VAR_TX_POWER_SUPPORTED_Pos)
#define ECS_BRDCST_VAR_RFU_MASK                             (0x03)

/* Field: ble_ecs_init_params_t.brdcst_cap.supp_frame_types*/
#define ECS_FRAME_TYPE_UID_SUPPORTED_Yes                    (1)
#define ECS_FRAME_TYPE_UID_SUPPORTED_No                     (0)
#define ECS_FRAME_TYPE_UID_SUPPORTED_Pos                    (0)
#define ECS_FRAME_TYPE_UID_SUPPORTED_Msk                    (1 << ECS_FRAME_TYPE_UID_SUPPORTED_Pos)

#define ECS_FRAME_TYPE_URL_SUPPORTED_Yes                    (1)
#define ECS_FRAME_TYPE_URL_SUPPORTED_No                     (0)
#define ECS_FRAME_TYPE_URL_SUPPORTED_Pos                    (1)
#define ECS_FRAME_TYPE_URL_SUPPORTED_Msk                    (1 << ECS_FRAME_TYPE_URL_SUPPORTED_Pos)

#define ECS_FRAME_TYPE_TLM_SUPPORTED_Yes                    (1)
#define ECS_FRAME_TYPE_TLM_SUPPORTED_No                     (0)
#define ECS_FRAME_TYPE_TLM_SUPPORTED_Pos                    (2)
#define ECS_FRAME_TYPE_TLM_SUPPORTED_Msk                    (1 << ECS_FRAME_TYPE_TLM_SUPPORTED_Pos)

#define ECS_FRAME_TYPE_EID_SUPPORTED_Yes                    (1)
#define ECS_FRAME_TYPE_EID_SUPPORTED_No                     (0)
#define ECS_FRAME_TYPE_EID_SUPPORTED_Pos                    (3)
#define ECS_FRAME_TYPE_EID_SUPPORTED_Msk                    (1 << ECS_FRAME_TYPE_EID_SUPPORTED_Pos)
#define ECS_FRAME_TYPE_RFU_MASK                             (0x000F)


//Broadcast Capabilities
#define APP_IS_VARIABLE_ADV_SUPPORTED                   ECS_BRDCST_VAR_ADV_SUPPORTED_No
#define APP_IS_VARIABLE_TX_POWER_SUPPORTED              ECS_BRDCST_VAR_TX_POWER_SUPPORTED_Yes

#define APP_IS_UID_SUPPORTED                            ECS_FRAME_TYPE_UID_SUPPORTED_Yes
#define APP_IS_URL_SUPPORTED                            ECS_FRAME_TYPE_URL_SUPPORTED_Yes
#define APP_IS_TLM_SUPPORTED                            ECS_FRAME_TYPE_TLM_SUPPORTED_Yes
#define APP_IS_EID_SUPPORTED                            ECS_FRAME_TYPE_EID_SUPPORTED_Yes
#define APP_CFG_NON_CONN_ADV_INTERVAL_MS                1000
#define APP_CFG_DEFAULT_RADIO_TX_POWER                  0x0
// Eddystone data
#define EDDYSTONE_FRAME_TYPE_LENGTH                     1
#define EDDYSTONE_TX_POWER_LENGTH                       1
#define EDDYSTONE_TLM_FRAME_LENGTH                      14
#define CHAR_LENGTH_MAX                                 34
#define ECS_KEY_SIZE                                    16
#define DEFAULT_FRAME_TYPE                              EDDYSTONE_FRAME_TYPE_URL
#define DEFAULT_FRAME_DATA                              {0x00,0x01,0x6e,0x6f, 0x72, 0x64, 0x69, 0x63, 0x73, 0x65,0x6d,0x69,0x00}
#define DEFAULT_TLM_FRAME_DATA                          {0x00, 0x0C, 0xBA, 0x1C, 0x00, 0x00, 0x00, 0x00, 0X15, 0x00, 0x00, 0x01, 0xFB}
extern uint8_t eddystone_uuid[16];
extern uint8_t eddystone_default_data[];

typedef enum {
	EDDYSTONE_FRAME_TYPE_UID = 0x00,
	EDDYSTONE_FRAME_TYPE_URL = 0x10,
	EDDYSTONE_FRAME_TYPE_TLM = 0x20,
	EDDYSTONE_FRAME_TYPE_EID = 0x30,
} eddystone_frame_type_t;

enum lock_state_read {
	BLE_ECS_LOCK_STATE_LOCKED     = 0x00,
	BLE_ECS_LOCK_STATE_UNLOCKED     = 0x01,
	BLE_ECS_LOCK_STATE_UNLOCKED_AUTO_RELOCK_DISABLED     = 0x02,
};

typedef struct ble_ecs_brdcst_cap_t {
	uint8_t version_byte;
	uint8_t max_supp_total_slots;
	uint8_t max_supp_eid_slots;
	uint8_t cap_bitfield;
	uint16_t supp_frame_types;
	int8_t supp_radio_tx_power[ECS_NUM_OF_SUPORTED_TX_POWER];
} ble_ecs_brdcst_cap_t;

typedef uint8_t ble_ecs_active_slot_t;
typedef uint16_t ble_ecs_adv_intrvl_t;
typedef uint8_t ble_ecs_radio_tx_pwr_t;
typedef uint8_t ble_ecs_adv_tx_pwr_t;
typedef uint8_t ble_ecs_lock_state_read_t;

typedef struct ble_ecs_lock_state_write_t {
	uint8_t lock_byte;
	uint8_t lock_key[ECS_KEY_SIZE];
} ble_ecs_lock_state_write_t;

typedef struct ble_ecs_lock_state_t {
	ble_ecs_lock_state_read_t read;
	ble_ecs_lock_state_write_t write;
} ble_ecs_lock_state_t;

typedef struct ble_ecs_unlock_t {
	uint8_t lock[ECS_KEY_SIZE];
} ble_ecs_unlock_t;

typedef struct ble_ecs_rw_adv_slot_t {
	uint8_t frame_type;
	uint8_t *p_data;
	uint8_t char_length;
} ble_ecs_rw_adv_slot_t;

typedef uint8_t ble_ecs_fac_reset_t;
typedef uint8_t ble_ecs_remain_conn_t;

typedef struct {
	uint16_t msg_id;
	void *data;
} EDDYSTONE_MSG_T;
typedef bk_err_t (*eddystone_api_handle_t)(void *param, uint32_t len);
typedef struct {
	uint32_t msg;
	eddystone_api_handle_t handle;
} ble_eddystone_msg_api_handle_t;

#define BK_ERR_BLE_SUCCESS               BK_OK                      /**< success */
#define BK_ERR_BLE_FAIL                  BK_FAIL                    /**< fail        */

typedef enum {
	EDDYSTONE_WRITE_EVENT,
	EDDYSTONE_CONNECT_EVENT,
	EDDYSTONE_DISCONNECT_EVENT,
	EDDYSTONE_INIT_CONN_PARAM_UPDATE_REQ_EVENT,
	EDDYSTONE_ADV_EVENT,
} eddystone_notice_t;

extern void eddystone_services_init(void);
extern ble_err_t bk_eddystone_init(void);
extern void ble_eddystone_cmd_cb(ble_cmd_t cmd, ble_cmd_param_t *param);
extern ble_err_t bk_ble_adv_start_first(uint8_t actv_idx, struct adv_param *adv, ble_cmd_cb_t callback);
extern ble_err_t bk_ble_write_event_private(void *param);
extern ble_err_t bk_ble_read_event_private(void *param);
extern ble_err_t bk_ble_connect_event_private(void *param);
extern ble_err_t bk_ble_disconnect_event_private(void *param);
extern ble_err_t bk_ble_init_conn_papam_update_data_req_event_private(void *param);
extern ble_err_t bk_ble_adv_event_private(void *param);
#endif // _EDDYSTONE_CORE_H_
// EOF
