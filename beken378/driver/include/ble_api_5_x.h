#ifndef _BLE_API_5_X_H_
#define _BLE_API_5_X_H_
#include "typedef.h"

#if (CFG_BLE_VERSION == BLE_VERSION_5_2)
#include "gatt.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_ADV_DATA_LEN           (0x1F)
#define MAX_SCAN_NUM               (15)

/// BD address length
#define GAP_BD_ADDR_LEN       (6)
/// Maximal length of the Device Name value
#define APP_DEVICE_NAME_MAX_LEN      (18)

#define ABIT(n) (1 << n)

#define BK_PERM_SET(access, right) \
    (((BK_PERM_RIGHT_ ## right) << (access ## _POS)) & (access ## _MASK))

#define BK_PERM_GET(perm, access)\
    (((perm) & (access ## _MASK)) >> (access ## _POS))

typedef enum
{
    /// Stop notification/indication
    PRF_STOP_NTFIND = 0x0000,
    /// Start notification
    PRF_START_NTF,
    /// Start indication
    PRF_START_IND,
} prf_conf;

/**
 *   15   14   13   12   11   10    9    8    7    6    5    4    3    2    1    0
 * +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 * |EXT | WS | I  | N  | WR | WC | RD | B  |    NP   |    IP   |   WP    |    RP   |
 * +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 *
 * Bit [0-1]  : Read Permission         (0 = NO_AUTH, 1 = UNAUTH, 2 = AUTH, 3 = SEC_CON)
 * Bit [2-3]  : Write Permission        (0 = NO_AUTH, 1 = UNAUTH, 2 = AUTH, 3 = SEC_CON)
 * Bit [4-5]  : Indication Permission   (0 = NO_AUTH, 1 = UNAUTH, 2 = AUTH, 3 = SEC_CON)
 * Bit [6-7]  : Notification Permission (0 = NO_AUTH, 1 = UNAUTH, 2 = AUTH, 3 = SEC_CON)
 *
 * Bit [8]    : Broadcast permission
 * Bit [9]    : Read Command accepted
 * Bit [10]   : Write Command accepted
 * Bit [11]   : Write Request accepted
 * Bit [12]   : Send Notification
 * Bit [13]   : Send Indication
 * Bit [14]   : Write Signed accepted
 * Bit [15]   : Extended properties present
 */

typedef enum
{
    /// Read Permission Mask
    RP_MASK             = 0x0003,
    RP_POS              = 0,
    /// Write Permission Mask
    WP_MASK             = 0x000C,
    WP_POS              = 2,
    /// Indication Access Mask
    IP_MASK            = 0x0030,
    IP_POS             = 4,
    /// Notification Access Mask
    NP_MASK            = 0x00C0,
    NP_POS             = 6,
    /// Broadcast descriptor present
    BROADCAST_MASK     = 0x0100,
    BROADCAST_POS      = 8,
    /// Read Access Mask
    RD_MASK            = 0x0200,
    RD_POS             = 9,
    /// Write Command Enabled attribute Mask
    WRITE_COMMAND_MASK = 0x0400,
    WRITE_COMMAND_POS  = 10,
    /// Write Request Enabled attribute Mask
    WRITE_REQ_MASK     = 0x0800,
    WRITE_REQ_POS      = 11,
    /// Notification Access Mask
    NTF_MASK           = 0x1000,
    NTF_POS            = 12,
    /// Indication Access Mask
    IND_MASK           = 0x2000,
    IND_POS            = 13,
    /// Write Signed Enabled attribute Mask
    WRITE_SIGNED_MASK  = 0x4000,
    WRITE_SIGNED_POS   = 14,
    /// Extended properties descriptor present
    EXT_MASK           = 0x8000,
    EXT_POS            = 15,
} bk_perm_mask;

/**
 * Attribute Extended permissions
 *
 * Extended Value permission bit field
 *
 *   15   14   13   12   11   10    9    8    7    6    5    4    3    2    1    0
 * +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 * | RI |UUID_LEN |EKS |                       Reserved                            |
 * +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 *
 * Bit [0-11] : Reserved
 * Bit [12]   : Encryption key Size must be 16 bytes
 * Bit [13-14]: UUID Length             (0 = 16 bits, 1 = 32 bits, 2 = 128 bits, 3 = RFU)
 * Bit [15]   : Trigger Read Indication (0 = Value present in Database, 1 = Value not present in Database)
 */
typedef enum
{
    /// Check Encryption key size Mask
    EKS_MASK         = 0x1000,
    EKS_POS          = 12,
    /// UUID Length
    UUID_LEN_MASK    = 0x6000,
    UUID_LEN_POS     = 13,
    /// Read trigger Indication
    RI_MASK          = 0x8000,
    RI_POS           = 15,
}bk_ext_perm_mask;

/**
 * Service permissions
 *
 *    7    6    5    4    3    2    1    0
 * +----+----+----+----+----+----+----+----+
 * |SEC |UUID_LEN |DIS |  AUTH   |EKS | MI |
 * +----+----+----+----+----+----+----+----+
 *
 * Bit [0]  : Task that manage service is multi-instantiated (Connection index is conveyed)
 * Bit [1]  : Encryption key Size must be 16 bytes
 * Bit [2-3]: Service Permission      (0 = NO_AUTH, 1 = UNAUTH, 2 = AUTH, 3 = Secure Connect)
 * Bit [4]  : Disable the service
 * Bit [5-6]: UUID Length             (0 = 16 bits, 1 = 32 bits, 2 = 128 bits, 3 = RFU)
 * Bit [7]  : Secondary Service       (0 = Primary Service, 1 = Secondary Service)
 */
typedef enum
{
    /// Task that manage service is multi-instantiated
    SVC_MI_MASK        = 0x01,
    SVC_MI_POS         = 0,
    /// Check Encryption key size for service Access
    SVC_EKS_MASK       = 0x02,
    SVC_EKS_POS        = 1,
    /// Service Permission authentication
    SVC_AUTH_MASK      = 0x0C,
    SVC_AUTH_POS       = 2,
    /// Disable the service
    SVC_DIS_MASK       = 0x10,
    SVC_DIS_POS        = 4,
    /// Service UUID Length
    SVC_UUID_LEN_MASK  = 0x60,
    SVC_UUID_LEN_POS   = 5,
    /// Service type Secondary
    SVC_SECONDARY_MASK = 0x80,
    SVC_SECONDARY_POS  = 7,
}bk_svc_perm_mask;

/// Attribute & Service access mode
enum
{
    /// Disable access
    BK_PERM_RIGHT_DISABLE   = 0,
    /// Enable access
    BK_PERM_RIGHT_ENABLE   = 1,
};

/// Attribute & Service access rights
enum
{
    /// No Authentication
    BK_PERM_RIGHT_NO_AUTH  = 0,
    /// Access Requires Unauthenticated link
    BK_PERM_RIGHT_UNAUTH   = 1,
    /// Access Requires Authenticated link
    BK_PERM_RIGHT_AUTH     = 2,
    /// Access Requires Secure Connection link
    BK_PERM_RIGHT_SEC_CON  = 3,
};

/// Attribute & Service UUID Length
enum
{
    /// 16  bits UUID
    BK_PERM_RIGHT_UUID_16         = 0,
    /// 32  bits UUID
    BK_PERM_RIGHT_UUID_32         = 1,
    /// 128 bits UUID
    BK_PERM_RIGHT_UUID_128        = 2,
    /// Invalid
    BK_PERM_RIGHT_UUID_RFU        = 3,
};

typedef enum  {
	BLE_CMD_NONE,
	// ADV_CMD:FOR BLE 5.1
	BLE_CREATE_ADV,
	BLE_SET_ADV_DATA,
	BLE_SET_RSP_DATA,
	BLE_START_ADV,
	BLE_STOP_ADV,
	BLE_DELETE_ADV,
	// ADV_CMD:FOR BLE 4.2
	BLE_INIT_ADV,
	BLE_DEINIT_ADV,
	// SCAN_CMD:FOR BLE 5.1
	BLE_CREATE_SCAN,
	BLE_START_SCAN,
	BLE_STOP_SCAN,
	BLE_DELETE_SCAN,
	// SCAN_CMD:FOR BLE 4.2
	BLE_INIT_SCAN,
	BLE_DEINIT_SCAN,
	/////conn
	BLE_CONN_UPDATE_MTU,
	BLE_CONN_UPDATE_PARAM,
	BLE_CONN_DIS_CONN,

	////init
	BLE_INIT_CREATE,
	BLE_INIT_START_CONN,
	BLE_INIT_STOP_CONN,
	BLE_INIT_DIS_CONN,
	BLE_INIT_READ_CHAR,
	BLE_INIT_WRITE_CHAR,
	BLE_CMD_MAX,
} ble_cmd_t;

typedef enum  {
	BLE_5_STACK_OK,
	BLE_5_WRITE_EVENT,
	BLE_5_READ_EVENT,
	BLE_5_REPORT_ADV,
	BLE_5_MTU_CHANGE,
	BLE_5_PHY_IND_EVENT,
	BLE_5_CONNECT_EVENT,
	BLE_5_DISCONNECT_EVENT,
	BLE_5_ATT_INFO_REQ,
	BLE_5_CREATE_DB,

	BLE_5_TX_DONE,

	BLE_5_INIT_CONNECT_EVENT,
	BLE_5_INIT_DISCONNECT_EVENT,
	BLE_5_INIT_CONNECT_FAILED_EVENT,
	BLE_5_INIT_CONN_PARAM_UPDATE_REQ_EVENT,
	BLE_5_INIT_CONN_PARAM_UPDATE_IND_EVENT,
	BLE_5_SDP_REGISTER_FAILED,
} ble_notice_t;

typedef enum {
	ERR_SUCCESS = 0,
	ERR_PROFILE,
	ERR_CREATE_DB,
	ERR_CMD_NOT_SUPPORT,
	ERR_UNKNOW_IDX,
	ERR_BLE_STATUS,
	ERR_ADV_DATA,
	ERR_CMD_RUN,
	ERR_NO_MEM,
	ERR_INIT_CREATE,
	ERR_INIT_STATE,

	ERR_ATTC_WRITE,
	ERR_ATTC_WRITE_UNREGISTER,
} ble_err_t;

typedef enum {
    /// Indicate that advertising is connectable, reception of CONNECT_REQ or AUX_CONNECT_REQ
    /// PDUs is accepted. Not applicable for periodic advertising.
    ADV_PROP_CONNECTABLE_POS     = 0,
    ADV_PROP_CONNECTABLE_BIT     = 0x01,

    /// Indicate that advertising is scannable, reception of SCAN_REQ or AUX_SCAN_REQ PDUs is
    /// accepted
    ADV_PROP_SCANNABLE_POS       = 1,
    ADV_PROP_SCANNABLE_BIT       = 0x02,

    /// Indicate that advertising targets a specific device. Only apply in following cases:
    ///   - Legacy advertising: if connectable
    ///   - Extended advertising: connectable or (non connectable and non discoverable)
    ADV_PROP_DIRECTED_POS        = 2,
    ADV_PROP_DIRECTED_BIT        = 0x04,

    /// Indicate that High Duty Cycle has to be used for advertising on primary channel
    /// Apply only if created advertising is not an extended advertising
    ADV_PROP_HDC_POS             = 3,
    ADV_PROP_HDC_BIT             = 0x08,

    /// Bit 4 is reserved
    ADV_PROP_RESERVED_4_POS      = 4,
    ADV_PROP_RESERVED_4_BIT      = 0x10,

    /// Enable anonymous mode. Device address won't appear in send PDUs
    /// Valid only if created advertising is an extended advertising
    ADV_PROP_ANONYMOUS_POS       = 5,
    ADV_PROP_ANONYMOUS_BIT       = 0x20,
}adv_prop_bf_t;

typedef struct{
	uint8_t cmd_idx;
	ble_err_t status;
}ble_cmd_param_t;

typedef struct
{
	uint8_t conn_idx;
	uint16_t prf_id;
	uint16_t att_idx;
	uint16_t length;
	uint8_t status;
} att_info_req_t;

typedef struct
{
	uint8_t conn_idx;
	uint16_t prf_id;
	uint16_t att_idx;
	uint8_t *value;
	uint16_t len;
} write_req_t;

typedef struct
{
	uint8_t conn_idx;
	uint16_t prf_id;
	uint16_t att_idx;
	uint8_t *value;
	uint16_t size;
	uint16_t length;
} read_req_t;

typedef struct
{
	uint8_t actv_idx;
	uint8_t evt_type;
	uint8_t adv_addr_type;
	uint8_t adv_addr[6];
	uint8_t data_len;
	uint8_t *data;
	uint8_t rssi;
} recv_adv_t;

typedef struct
{
	uint8_t conn_idx;
	uint16_t mtu_size;
} mtu_change_t;

typedef struct
{
	uint8_t tx_phy;                       /**< The transmitter PHY */
	uint8_t rx_phy;                       /**< The receiver PHY */
	uint8_t phy_opt;                      /**< PHY options  */
} ble_set_phy_t;

typedef struct
{
	uint8_t conn_idx;
	/// Peer address type
	uint8_t peer_addr_type;
	/// Peer BT address
	uint8_t peer_addr[6];
} conn_ind_t;

typedef struct
{
	uint8_t conn_idx;
	/// Reason of disconnection
    uint8_t reason;
} discon_ind_t;

typedef struct
{
	uint8_t conn_idx;
	/// Connection interval minimum
	uint16_t intv_min;
	/// Connection interval maximum
	uint16_t intv_max;
	/// Latency
	uint16_t latency;
	/// Supervision timeout
	uint16_t time_out;
} conn_param_t;

typedef struct
{
	uint8_t conn_idx;
	uint16_t interval;
	/// Latency
	uint16_t latency;
	/// Supervision timeout
	uint16_t time_out;
} conn_update_ind_t;

typedef struct
{
    uint8_t conn_idx;
    /// LE PHY for data transmission (@see enum gap_phy)
    uint8_t tx_phy;
    /// LE PHY for data reception (@see enum gap_phy)
    uint8_t rx_phy;
} conn_phy_ind_t;

typedef struct
{
	uint8_t status;
	uint8_t prf_id;
} create_db_t;

typedef struct{
	uint8_t status;
	uint8_t operation;  ///0:INDICATE; 1:notify
	uint8_t prf_id;
	uint16_t att_id;
	uint16_t up_con_idx;
}ble_slave_con_ind_nty_t;

#if (CFG_BLE_VERSION == BLE_VERSION_5_1)
typedef struct
{
	/// 16 bits UUID LSB First
	uint8_t uuid[16];
	/// Attribute Permissions (@see enum attm_perm_mask)
	uint16_t perm;
	/// Attribute Extended Permissions (@see enum attm_value_perm_mask)
	uint16_t ext_perm;
	/// Attribute Max Size
	/// note: for characteristic declaration contains handle offset
	/// note: for included service, contains target service handle
	uint16_t max_size;
}bk_attm_desc_t;

#elif (CFG_BLE_VERSION == BLE_VERSION_5_2)
typedef struct
{
	/// Attribute UUID (LSB First)
	uint8_t  uuid[16];
	/// Attribute information bit field (@see enum gatt_att_info_bf)
	uint16_t info;
	/// Attribute extended information bit field (@see enum gatt_att_ext_info_bf)
	/// Note:
	///   - For Included Services and Characteristic Declarations, this field contains targeted handle.
	///   - For Characteristic Extended Properties, this field contains 2 byte value
	///   - For Client Characteristic Configuration and Server Characteristic Configuration, this field is not used.
	uint16_t ext_info;
}bk_attm_desc_t;
#endif //(CFG_BLE_VERSION == BLE_VERSION_5_2)

struct bk_ble_db_cfg
{
	uint16_t prf_task_id;
	///Service uuid
	uint8_t uuid[16];
	///Number of db
	uint8_t att_db_nb;
	///Start handler, 0 means autoalloc
	uint16_t start_hdl;
	///Attribute database
	bk_attm_desc_t* att_db;
	///Service config
	uint8_t svc_perm;
};

struct adv_param {
	uint8_t  advData[MAX_ADV_DATA_LEN];
	uint8_t  advDataLen;
	uint8_t  respData[MAX_ADV_DATA_LEN];
	uint8_t  respDataLen;
	uint8_t  channel_map;
	uint16_t interval_min;
	uint16_t interval_max;
	uint16_t duration;
	uint8_t prop;
};

typedef struct ext_adv_param_cfg {
	uint8_t  channel_map;
	uint16_t interval_min;
	uint16_t interval_max;
	uint16_t duration;
	uint8_t sid;
	uint8_t prop;
}ext_adv_param_cfg_t;

struct scan_param {
	uint8_t  filter_en;
	uint8_t  channel_map;
	uint16_t interval;
	uint16_t window;
	uint8_t filter_type;
	uint8_t filter_param[32];
};

#define MST_ATT_UUID_128_LEN                        0x0010
struct mst_sdp_svc_ind
{
	uint8_t  svr_id;
	/// Service UUID Length
	uint8_t  uuid_len;
	/// Service UUID
	uint8_t  uuid[MST_ATT_UUID_128_LEN];
	/// Service start handle
	uint16_t start_hdl;
	/// Service end handle
	uint16_t end_hdl;
};

/// characteristic info
struct mst_sdp_char_inf
{
	uint8_t  svr_id;
	uint8_t uuid_len;
	uint8_t uuid[MST_ATT_UUID_128_LEN]; // add by sean 2016.11.15 refer 3431n Master
	/// Characteristic handle
	uint16_t char_hdl;
	/// Value handle
	uint16_t val_hdl;
	/// Characteristic properties
	uint8_t prop;
	/// End of characteristic offset
	uint8_t char_ehdl_off;
};

/// characteristic description
struct mst_sdp_char_desc_inf
{
	uint8_t  svr_id;
	/// UUID length
	uint8_t uuid_len;
	/// UUID
	uint8_t uuid[MST_ATT_UUID_128_LEN];

	uint8_t char_code;
	/// Descriptor handle
	uint16_t desc_hdl;
};

typedef enum{
	MST_TYPE_SVR_UUID = 0,
	MST_TYPE_ATT_UUID,
	MST_TYPE_ATT_DESC,
	MST_TYPE_SDP_END,

	MST_TYPE_ATTC_SVR_UUID,  ///Service the UUID
	MST_TYPE_ATTC_ATT_UUID,  ///ATT of a service
	MST_TYPE_ATTC_ATT_DESC,  ///ATT DESC of a service
	MST_TYPS_ATTC_PARAM_ERR,  ///The delivered parameter is abnormal or unknown
	MST_TYPE_ATTC_ERR,	 ///if appm_get_init_attc_info return is ok && ble is disconnect,so update the event
	MST_TYPE_ATTC_END,	 ///End of the operation
	MST_TYPE_ATTC_WRITE_RSP,
	MST_TYPE_ATTC_WRITE_NO_RESPONSE,
	MST_TYPE_ATTC_CHARAC_READ_DONE,

	MST_TYPE_MTU_EXC = 0x10,
	MST_TYPE_MTU_EXC_DONE,

	MST_TYPE_UPP_ASK = 0x20,   ///Ask if you agree to update the parameter
	MST_TYPE_UPDATA_STATUS,    ////updata param status


	MST_TYPE_INIT_CREATE_OK = 0x80,
}MASTER_COMMON_TYPE;

typedef void (*app_sdp_comm_callback)(MASTER_COMMON_TYPE type,uint8 conidx,void *param);
typedef void (*ble_cmd_cb_t)(ble_cmd_t cmd, ble_cmd_param_t *param);
typedef void (*ble_notice_cb_t)(ble_notice_t notice, void *param);

uint8_t app_ble_get_idle_actv_idx_handle(void);

ble_err_t bk_ble_create_db (struct bk_ble_db_cfg* ble_db_cfg);

void ble_set_notice_cb(ble_notice_cb_t func);
uint8_t ble_appm_get_dev_name(uint8_t* name, uint32_t buf_len);
uint8_t ble_appm_set_dev_name(uint8_t len, uint8_t* name);
ble_err_t bk_ble_adv_start(uint8_t actv_idx, struct adv_param *adv, ble_cmd_cb_t callback);
ble_err_t bk_ble_adv_stop(uint8_t actv_idx, ble_cmd_cb_t callback);
ble_err_t bk_ble_scan_start(uint8_t actv_idx, struct scan_param *scan, ble_cmd_cb_t callback);
ble_err_t bk_ble_scan_stop(uint8_t actv_idx, ble_cmd_cb_t callback);
ble_err_t bk_ble_create_advertising(uint8_t actv_idx, unsigned char chnl_map, uint32_t intv_min, uint32_t intv_max, ble_cmd_cb_t callback);
ble_err_t bk_ble_create_extended_advertising(uint8_t actv_idx, unsigned char chnl_map, uint32_t intv_min, uint32_t intv_max, uint8_t scannable, uint8_t connectable, ble_cmd_cb_t callback);
ble_err_t bk_ble_start_advertising(uint8_t actv_idx, uint16 duration, ble_cmd_cb_t callback);
ble_err_t bk_ble_stop_advertising(uint8_t actv_idx, ble_cmd_cb_t callback);
ble_err_t bk_ble_delete_advertising(uint8_t actv_idx, ble_cmd_cb_t callback);
ble_err_t bk_ble_set_adv_data(uint8_t actv_idx, unsigned char* adv_buff, unsigned char adv_len, ble_cmd_cb_t callback);
ble_err_t bk_ble_set_ext_adv_data(uint8_t actv_idx, unsigned char * adv_buff, uint16_t adv_len, ble_cmd_cb_t callback);
ble_err_t bk_ble_set_scan_rsp_data(uint8_t actv_idx, unsigned char* scan_buff, unsigned char scan_len, ble_cmd_cb_t callback);
ble_err_t bk_ble_set_ext_scan_rsp_data(uint8_t actv_idx, unsigned char * scan_buff, uint16_t scan_len, ble_cmd_cb_t callback);
ble_err_t bk_ble_update_param(uint8_t conn_idx, uint16_t intv_min, uint16_t intv_max,
					uint16_t latency, uint16_t sup_to, ble_cmd_cb_t callback);
ble_err_t bk_ble_disconnect(uint8_t conn_idx, ble_cmd_cb_t callback);
ble_err_t bk_ble_gatt_mtu_change(uint8_t conn_idx,ble_cmd_cb_t callback);
ble_err_t bk_ble_create_scaning(uint8_t actv_idx, ble_cmd_cb_t callback);
ble_err_t bk_ble_start_scaning(uint8_t actv_idx, uint16_t scan_intv, uint16_t scan_wd, ble_cmd_cb_t callback);
ble_err_t bk_ble_stop_scaning(uint8_t actv_idx, ble_cmd_cb_t callback);
ble_err_t bk_ble_delete_scaning(uint8_t actv_idx, ble_cmd_cb_t callback);
ble_err_t bk_ble_send_ind_value(uint32_t len, uint8_t *buf, uint16_t prf_id, uint16_t att_idx);
ble_err_t bk_ble_send_ntf_value( uint32_t len, uint8_t *buf, uint16_t prf_id, uint16_t att_idx);
int bk_ble_get_con_mtu(unsigned char con_idx,uint16_t *mtu);
extern void ble_ps_enable_set(void);
extern void ble_ps_enable_clear(void);
extern UINT32 ble_ps_enabled(void );

extern ble_err_t bk_ble_conidx_send_ntf(uint8_t conidx,uint32_t len, uint8_t *buf, uint16_t prf_id, uint16_t att_idx);
extern ble_err_t bk_ble_conidx_send_ind(uint8_t conidx,uint32_t len, uint8_t *buf, uint16_t prf_id, uint16_t att_idx);

#ifdef __cplusplus
}
#endif

#endif

