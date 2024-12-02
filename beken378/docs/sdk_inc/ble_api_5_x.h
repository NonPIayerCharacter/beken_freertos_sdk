#ifndef _BLE_API_5_X_H_
#define _BLE_API_5_X_H_

/**
 * @brief     	Get an idle activity
 *
 * @return 		the idle activity's index
 */
uint8_t app_ble_get_idle_actv_idx_handle(void);


/**
 *
 * example:
 *     First we must build test_att_db
 *     test_att_db is a database for att, which used in ble discovery. reading writing and other operation is used on a att database.
 *
 *
 * @code
 *	#define BK_ATT_DECL_PRIMARY_SERVICE_128     {0x00,0x28,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
 *	#define BK_ATT_DECL_CHARACTERISTIC_128      {0x03,0x28,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
 *	#define BK_ATT_DESC_CLIENT_CHAR_CFG_128     {0x02,0x29,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
 *	
 *	#define WRITE_REQ_CHARACTERISTIC_128        {0x01,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
 *	#define INDICATE_CHARACTERISTIC_128         {0x02,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
 *	#define NOTIFY_CHARACTERISTIC_128           {0x03,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
 *	
 *	static const uint8_t test_svc_uuid[16] = {0xFF,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0};
 *	
 *	enum
 *	{
 *		TEST_IDX_SVC,
 *		TEST_IDX_FF01_VAL_CHAR,
 *		TEST_IDX_FF01_VAL_VALUE,
 *		TEST_IDX_FF02_VAL_CHAR,
 *		TEST_IDX_FF02_VAL_VALUE,
 *		TEST_IDX_FF02_VAL_IND_CFG,
 *		TEST_IDX_FF03_VAL_CHAR,
 *		TEST_IDX_FF03_VAL_VALUE,
 *		TEST_IDX_FF03_VAL_NTF_CFG,
 *		TEST_IDX_NB,
 *	};
 *
 *  //att records database.
 *	bk_attm_desc_t test_att_db[TEST_IDX_NB] =
 *	{
 *		//  Service Declaration
 *		[TEST_IDX_SVC]              = {BK_ATT_DECL_PRIMARY_SERVICE_128, PROP(RD), 0},
 *	
 *		//  Level Characteristic Declaration
 *		[TEST_IDX_FF01_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
 *		//  Level Characteristic Value
 *		[TEST_IDX_FF01_VAL_VALUE]   = {WRITE_REQ_CHARACTERISTIC_128,    PROP(WR)|ATT_UUID(128), 128|OPT(NO_OFFSET)},
 *	
 *		[TEST_IDX_FF02_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
 *		//  Level Characteristic Value
 *		[TEST_IDX_FF02_VAL_VALUE]   = {INDICATE_CHARACTERISTIC_128,     PROP(I), 128|OPT(NO_OFFSET)},
 *	
 *		//  Level Characteristic - Client Characteristic Configuration Descriptor
 *	
 *		[TEST_IDX_FF02_VAL_IND_CFG] = {BK_ATT_DESC_CLIENT_CHAR_CFG_128, PROP(RD)|PROP(WR),OPT(NO_OFFSET)},
 *	
 *		[TEST_IDX_FF03_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
 *		//  Level Characteristic Value
 *		[TEST_IDX_FF03_VAL_VALUE]   = {NOTIFY_CHARACTERISTIC_128,       PROP(N), 128|OPT(NO_OFFSET)},
 *	
 *		//  Level Characteristic - Client Characteristic Configuration Descriptor
 *	
 *		[TEST_IDX_FF03_VAL_NTF_CFG] = {BK_ATT_DESC_CLIENT_CHAR_CFG_128, PROP(RD)|PROP(WR), OPT(NO_OFFSET)},
 *	};
 *
 *   
 *
 * @endcode
 * TEST_IDX_SVC is nessecery, is declare a primary att service. The macro define is:
 * @endcode
 * which is an UUID say it is a "primary service"
 *
 * TEST_IDX_FF01_VAL_CHAR declare a characteristic as a element in service, it must be PROP(RD)
 *
 * TEST_IDX_FF01_VAL_VALUE is the real value of TEST_IDX_FF01_VAL_CHAR,
 *
 * PROP(N)  Notification Access
 *
 * PROP(I)  Indication Access 
 *
 * PROP(RD) Read Access
 *
 * PROP(WR) Write Request Enabled
 *
 * PROP(WC) Write Command Enabled
 *
 * ATT_UUID(128)  set att uuid len
 *
 * Secondlly, we build ble_db_cfg
 * @code
 *     struct bk_ble_db_cfg ble_db_cfg;
 *
 *     ble_db_cfg.att_db = (ble_attm_desc_t *)test_att_db;
 *     ble_db_cfg.att_db_nb = TEST_IDX_NB;
 *     ble_db_cfg.prf_task_id = g_test_prf_task_id;
 *     ble_db_cfg.start_hdl = 0;
 *     ble_db_cfg.svc_perm = BK_BLE_PERM_SET(SVC_UUID_LEN, UUID_16);
 * @endcode
 * prf_task_id is app handle. If you have multi att service, used prf_task_id to distinguish it.
 * svc_perm show TEST_IDX_SVC UUID type's len.
 *
 * @brief     Register a gatt service
 * @param
 *     - ble_db_cfg: service param
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_create_db (struct bk_ble_db_cfg* ble_db_cfg);


/**
 * @brief     Register ble event notification callback
 *
 * @param
 *    - func: event callback
 *
 * @attention 
 *	1. you must regist it, otherwise you cant get any event !
 * 
 *  2. you must regist it before bk_ble_create_db, otherwise you cant get BLE_5_CREATE_DB event
 *
 * User example:
 * @code
 * void ble_notice_cb(ble_notice_t notice, void *param)
 * {
 *    switch (notice) {
 *    case BLE_5_STACK_OK:
 *    case BLE_5_WRITE_EVENT: 
 *    case BLE_5_READ_EVENT:
 *    case  BLE_5_TX_DONE
 *      break;
 *    case BLE_5_CREATE_DB:
 *    //bk_ble_create_db success here
 *      break;
 *    }
 * }

ble_set_notice_cb(ble_notice_cb);
 * @endcode
 * @return
 *    - void
 */
void ble_set_notice_cb(ble_notice_cb_t func);


/**
 * @brief     Get device name
 *
 * @param
 *    - name: store the device name
 *    - buf_len: the length of buf to store the device name
 *
 * @return
 *    - length: the length of device name
 */
uint8_t ble_appm_get_dev_name(uint8_t* name, uint32_t buf_len);


/**
 * @brief     Set device name
 *
 * @param
 *    - len: the length of device name
 *    - name: the device name to be set
 *
 * @return
 *    - length: the length of device name
 */
uint8_t ble_appm_set_dev_name(uint8_t len, uint8_t* name);

/**
 * @brief     Create and start a ble advertising activity
 *
 * @param
 *    - actv_idx: the index of activity
 *    - adv: the advertising parameter
 *    - callback: register a callback for this action, ble_cmd_t: BLE_INIT_ADV
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *
 * User example:
 * @code
 *		struct adv_param adv_info;
 *		adv_info.channel_map = 7;
 *		adv_info.duration = 0;
 *		adv_info.prop = (1 << ADV_PROP_CONNECTABLE_POS) | (1 << ADV_PROP_SCANNABLE_POS);
 *		adv_info.interval_min = 160;
 *		adv_info.interval_max = 160;
 *		adv_info.advData[0] = 0x09;
 *		adv_info.advData[1] = 0x09;
 *		memcpy(&adv_info.advData[2], "7238_BLE", 8);
 *		adv_info.advDataLen = 10;
 *		adv_info.respData[0] = 0x05;
 *		adv_info.respData[1] = 0x08;
 *		memcpy(&adv_info.respData[2], "7238", 4);
 *		adv_info.respDataLen = 6;
 *		actv_idx = app_ble_get_idle_actv_idx_handle();
 *		bk_ble_adv_start(actv_idx, &adv_info, ble_cmd_cb);
 * @endcode
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_adv_start(uint8_t actv_idx, struct adv_param *adv, ble_cmd_cb_t callback);


/**
 * @brief     Stop and delete the advertising that has been created
 *
 * @param
 *    - actv_idx: the index of activity
 *    - callback: register a callback for this action, ble_cmd_t: BLE_DEINIT_ADV
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_adv_start
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_adv_stop(uint8_t actv_idx, ble_cmd_cb_t callback);



/**
 * @brief     Create and start a ble scan activity
 *
 * @param
 *    - actv_idx: the index of activity
 *    - scan: the scan parameter
 *    - callback: register a callback for this action, ble_cmd_t: BLE_INIT_SCAN
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *
 * User example:
 * @code
 *		struct scan_param scan_info;
 *		scan_info.channel_map = 7;
 *		scan_info.interval = 100;
 *		scan_info.window = 30;
 *		actv_idx = app_ble_get_idle_actv_idx_handle();
 *		bk_ble_scan_start(actv_idx, &scan_info, ble_cmd_cb);
 * @endcode
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_scan_start(uint8_t actv_idx, struct scan_param *scan, ble_cmd_cb_t callback);

/**
 * @brief     Stop and delete the scan that has been created
 *
 * @param
 *    - actv_idx: the index of activity
 *    - callback: register a callback for this action, ble_cmd_t: BLE_DEINIT_SCAN
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_scan_start
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_scan_stop(uint8_t actv_idx, ble_cmd_cb_t callback);



/**
 * @brief     Create a ble advertising activity
 *
 * @param
 *    - actv_idx: the index of activity
 *    - chnl_map: the advertising channel map
 *    - intv_min: the advertising min interval
 *    - intv_max: the advertising max interval
 *    - callback: register a callback for this action, ble_cmd_t: BLE_CREATE_ADV
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *
 * User example:
 * @code
 *     actv_idx = app_ble_get_idle_actv_idx_handle();
 *     if (actv_idx != UNKNOW_ACT_IDX) {
 *         bk_ble_create_advertising(actv_idx,7,160,160, ble_cmd_cb);
 *     }
 * @endcode
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_create_advertising(uint8_t actv_idx, unsigned char chnl_map, uint32_t intv_min, uint32_t intv_max, ble_cmd_cb_t callback);


/**
 * @brief     Create a ble advertising activity
 *
 * @param
 *    - actv_idx: 	 the index of activity
 *    - chnl_map:	 the advertising channel map
 *    - intv_min:	 the advertising min interval
 *    - intv_max:    the advertising max interval
 *    - scannable:   the advertising whether be scanned
 *    - connectable: the advertising whether be connected
 *    - callback: 	 register a callback for this action, ble_cmd_t: BLE_CREATE_ADV
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *
 * User example:
 * @code
 *     actv_idx = app_ble_get_idle_actv_idx_handle();
 *     if (actv_idx != UNKNOW_ACT_IDX) {
 *         bk_ble_create_extended_advertising(actv_idx,7,160,160,1,0,ble_cmd_cb);
 *     }
 * @endcode
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_create_extended_advertising(uint8_t actv_idx, unsigned char chnl_map, uint32_t intv_min, uint32_t intv_max, uint8_t scannable, uint8_t connectable, ble_cmd_cb_t callback);


/**
 * @brief     Start a ble advertising
 *
 *  @attention 
 * 	1. you must wait callback status, 0 mean success.
 * 	2. must used after bk_ble_create_advertising
 * 
 * @param
 *    - actv_idx: the index of activity
 *    - duration: Advertising duration (in unit of 10ms). 0 means that advertising continues
 *    - callback: register a callback for this action, ble_cmd_t: BLE_START_ADV
 *
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_start_advertising(uint8_t actv_idx, uint16 duration, ble_cmd_cb_t callback);


/**
 * @brief     Stop the advertising that has been started
 *
 * @param
 *    - actv_idx: the index of activity
 *    - callback: register a callback for this action, ble_cmd_t: BLE_STOP_ADV
 * @attention 
 * 1. you must wait callback status, 0 mean success.
 * 2. must used after bk_ble_start_advertising
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_stop_advertising(uint8_t actv_idx, ble_cmd_cb_t callback);


/**
 * @brief     Delete the advertising that has been created
 *
 * @param
 *    - actv_idx: the index of activity
 *    - callback: register a callback for this action, ble_cmd_t: BLE_DELETE_ADV
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_create_advertising
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_delete_advertising(uint8_t actv_idx, ble_cmd_cb_t callback);


/**
 * @brief     Set the advertising data
 *
 * @param
 *    - actv_idx: the index of activity
 *    - adv_buff: advertising data
 *    - adv_len: the length of advertising data
 *    - callback: register a callback for this action, ble_cmd_t: BLE_SET_ADV_DATA
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_create_advertising
 *
 *
 * User example:
 * @code
 *     const uint8_t adv_data[] = {0x0A, 0x09, 0x37 0x32, 0x33, 0x31, 0x4e, 0x5f, 0x42, 0x4c, 0x45};
 *     bk_ble_set_adv_data(actv_idx, adv_data, sizeof(adv_data), ble_cmd_cb);
 * @endcode
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_set_adv_data(uint8_t actv_idx, unsigned char* adv_buff, unsigned char adv_len, ble_cmd_cb_t callback);


/**
 * @brief     Set the ext advertising data
 *
 * @param
 *    - actv_idx: the index of activity
 *    - adv_buff: advertising data
 *    - adv_len: the length of advertising data
 *    - callback: register a callback for this action, ble_cmd_t: BLE_SET_ADV_DATA
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_create_extended_advertising
 *
 *
 * User example:
 * @code
 *     const uint8_t adv_data[] = {0x0A, 0x09, 0x37 0x32, 0x33, 0x31, 0x4e, 0x5f, 0x42, 0x4c, 0x45};
 *     bk_ble_set_ext_adv_data(actv_idx, adv_data, sizeof(adv_data), ble_cmd_cb);
 * @endcode
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_set_ext_adv_data(uint8_t actv_idx, unsigned char * adv_buff, uint16_t adv_len, ble_cmd_cb_t callback);

/**
 * @brief     Set the scan response data
 *
 * @param
 *    - actv_idx: the index of activity
 *    - scan_buff: scan response data
 *    - scan_len: the length of scan response data
 *    - callback: register a callback for this action, ble_cmd_t: BLE_SET_RSP_DATA
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *	2. scan rsp data similaly to adv data
 *  3. must used after bk_ble_create_advertising
 *
 *
 * User example:
 * @code
 *     const uint8_t scan_data[] = {0x0A, 0x09, 0x37 0x32, 0x33, 0x31, 0x4e, 0x5f, 0x42, 0x4c, 0x45};
 *     bk_ble_set_scan_rsp_data(actv_idx, scan_data, sizeof(scan_data), ble_cmd_cb);
 * @endcode
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_set_scan_rsp_data(uint8_t actv_idx, unsigned char* scan_buff, unsigned char scan_len, ble_cmd_cb_t callback);


/**
 * @brief     Set the ext adv scan response data
 *
 * @param
 *    - actv_idx: the index of activity
 *    - scan_buff: scan response data
 *    - scan_len: the length of scan response data
 *    - callback: register a callback for this action, ble_cmd_t: BLE_SET_RSP_DATA
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *	2. scan rsp data similaly to adv data
 *  3. must used after bk_ble_create_extended_advertising
 *
 *
 * User example:
 * @code
 *     const uint8_t scan_data[] = {0x0A, 0x09, 0x37 0x32, 0x33, 0x31, 0x4e, 0x5f, 0x42, 0x4c, 0x45};
 *     bk_ble_set_ext_scan_rsp_data(actv_idx, scan_data, sizeof(scan_data), ble_cmd_cb);
 * @endcode
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_set_ext_scan_rsp_data(uint8_t actv_idx, unsigned char * scan_buff, uint16_t scan_len, ble_cmd_cb_t callback);

/**
 * @brief     Update connection parameters
 *
 * @param
 *    - conn_idx: the index of connection
 *    - intv_min: connection min interval
 *    - intv_max: connection max interval
 *    - latency:  connection latency
 *    - sup_to:   connection timeout
 *    - callback: register a callback for this action, ble_cmd_t: BLE_CONN_UPDATE_PARAM
 * @attention 
 * 1. you must wait callback status, 0 mean success.
 * 2. must used after connected
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_update_param(uint8_t conn_idx, uint16_t intv_min, uint16_t intv_max,uint16_t latency, uint16_t sup_to, ble_cmd_cb_t callback);

/**
 * @brief     Disconnect a ble connection
 *
 * @param
 *    - conn_idx: the index of connection
 *    - callback: register a callback for this action, ble_cmd_t: BLE_CONN_DIS_CONN
 *
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after connected
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */

ble_err_t bk_ble_disconnect(uint8_t conn_idx, ble_cmd_cb_t callback);

/**
 * @brief     Exchange MTU
 *
 * @param
 *    - conn_idx: the index of connection
 *    - callback: register a callback for this action, ble_cmd_t: BLE_CONN_UPDATE_MTU
 * @attention 
 * 1. you must wait callback status, 0 mean success.
 * 2. must used after connected
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_gatt_mtu_change(uint8_t conn_idx,ble_cmd_cb_t callback);

/**
 * @brief     Create a ble scan activity
 *
 * @param
 *    - actv_idx: the index of activity
 *    - callback: register a callback for this action, ble_cmd_t: BLE_CREATE_SCAN
 *
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *
 * User exzample:
 * @code
 *	actv_idx = app_ble_get_idle_actv_idx_handle();
 *  bk_ble_create_scaning(actv_idx, ble_at_cmd);
 *
 * @endcode
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_create_scaning(uint8_t actv_idx, ble_cmd_cb_t callback);

/**
 * @brief     Start a ble scan
 *
 * @param
 *    - actv_idx:  the index of activity
 *    - scan_intv: scan interval
 *    - scan_wd:   scan window
 *    - callback:  register a callback for this action, ble_cmd_t: BLE_START_SCAN
 *
 * @attention 
 * 1. you must wait callback status, 0 mean success.
 * 2. must used after bk_ble_create_scaning
 * 3. adv will report in ble_notice_cb_t as BLE_5_REPORT_ADV
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_start_scaning(uint8_t actv_idx, uint16_t scan_intv, uint16_t scan_wd, ble_cmd_cb_t callback);

/**
 * @brief     Stop the scan that has been started
 *
 * @param
 *    - actv_idx: the index of activity
 *    - callback: register a callback for this action, ble_cmd_t: BLE_STOP_SCAN
 *
 * @attention 
 * 1. you must wait callback status, 0 mean success.
 * 2. must used after bk_ble_start_scaning
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_stop_scaning(uint8_t actv_idx, ble_cmd_cb_t callback);

/**
 * @brief     Delete the scan that has been created
 *
 * @param
 *    - actv_idx: the index of activity
 *    - callback: register a callback for this action, ble_cmd_t: BLE_DELETE_SCAN
 *
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_create_scaning
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_delete_scaning(uint8_t actv_idx, ble_cmd_cb_t callback);

/**
 * @brief As slaver, send a notification of an attribute's value
 *
 * @param
 *    - conidx: the index of connection
 *    - len: the length of attribute's value
 *    - buf: attribute's value
 *    - prf_id: The id of the profile
 *    - att_idx: The index of the attribute
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_conidx_send_ntf(uint8_t conidx,uint32_t len, uint8_t *buf, uint16_t prf_id, uint16_t att_idx);

/**
 * @brief As slaver, send an indication of an attribute's value
 *
 * @param
 *    - conidx: the index of connection
 *    - len: the length of attribute's value
 *    - buf: attribute's value
 *    - prf_id: The id of the profile
 *    - att_idx: The index of the attribute
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_conidx_send_ind(uint8_t conidx,uint32_t len, uint8_t *buf, uint16_t prf_id, uint16_t att_idx);

/**
 * @brief     Register ble master event notification callback
 *
 * @param
 *    - func: event callback
 *
 * @attention 
 *	1. you must regist it, otherwise you cant get any master event !
 * 
 *  2. you must regist it before bk_ble_create_init
 *
 * User example:
 * @code
 *void sdp_event_cb(sdp_notice_t notice, void *param)
 *{
 *	switch (notice) {
 *		case SDP_CHARAC_NOTIFY_EVENT:
 *			{
 *				sdp_event_t *g_sdp = (sdp_event_t *)param;
 *				bk_printf("[SDP_CHARAC_NOTIFY_EVENT]con_idx:%d,hdl:0x%x,value_length:%d\r\n",g_sdp->con_idx,g_sdp->hdl,g_sdp->value_length);
 *			}
 *			break;
 *		case SDP_CHARAC_INDICATE_EVENT:
 *			{
 *				sdp_event_t *g_sdp = (sdp_event_t *)param;
 *				bk_printf("[SDP_CHARAC_INDICATE_EVENT]con_idx:%d,hdl:0x%x,value_length:%d\r\n",g_sdp->con_idx,g_sdp->hdl,g_sdp->value_length);
 *			}
 *			break;
 *		case SDP_CHARAC_READ:
 *			{
 *				sdp_event_t *g_sdp = (sdp_event_t *)param;
 *				bk_printf("[SDP_CHARAC_READ]con_idx:%d,hdl:0x%x,value_length:%d\r\n",g_sdp->con_idx,g_sdp->hdl,g_sdp->value_length);
 *			}
 *			break;
 *		case SDP_DISCOVER_SVR_DONE:
 *			{
 *				bk_printf("[SDP_DISCOVER_SVR_DONE]\r\n");
 *			}
 *			break;
 *		case SDP_CHARAC_WRITE_DONE:
 *			{
 *				bk_printf("[SDP_CHARAC_WRITE_DONE]\r\n");
 *			}
 *			break;
 *		default:
 *			bk_printf("[%s]Event:%d\r\n",__func__,notice);
 *			break;
 *	}
 *}
 * sdp_set_notice_cb(sdp_event_cb);
 * @endcode
 * @return
 *    - void
 */
void sdp_set_notice_cb(sdp_notice_cb_t func);


/**
 * @brief     Create a activity for initiating a connection
 *
 * @param
 *    - con_idx: 	  the index of connection
 *    - con_interval: the connection parameter
 *    - con_latency:  the connection parameter
 *    - sup_to: 	  the connection parameter
 *    - callback:     register a callback for this action, ble_cmd_t: BLE_INIT_CREATE
 *
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *
 * User example:
 * @code
 *   con_interval = 0x40; //interval
 *   con_latency = 0;
 *   sup_to = 0x200;//supervision timeout
 *   bk_ble_create_init(con_idx, con_interval, con_latency,sup_to,ble_at_cmd);
 * @endcode
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_create_init(uint8_t con_idx,unsigned short con_interval,unsigned short con_latency,unsigned short sup_to,ble_cmd_cb_t callback);


/**
 * @brief     Set the address of the device to be connected
 *
 * @param
 *    - connidx: the index of connection
 *    - bdaddr: the address of the device to be connected
 *    - addr_type: the address type of the device to be connected, 1: public 0: random
 *
 * @attention 
 *	1. must used before bk_ble_init_start_conn and used after bk_ble_create_init
 *	2. addr_type must right, if wrong, cant connect
 *
 * User example:
 * @code
 *		struct bd_addr bdaddr;
 * 		uint8_t mac[6]={0xc8,0x47,0x8c,0x11,0x22,0x33};
 *		memcpy(bdaddr.addr,mac,6);
 *		addr_type = ADDR_PUBLIC;
 *		bk_ble_init_set_connect_dev_addr(actv_idx,&bdaddr,addr_type);
 * @endcode
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_init_set_connect_dev_addr(unsigned char connidx,struct bd_addr *bdaddr,unsigned char addr_type);


/**
 * @brief     start a connection
 *
 * @param
 *    - con_idx: the index of connection
 *    - callback: register a callback for this action, ble_cmd_t: BLE_INIT_START_CONN
 *
 * @attention 
 *	1. you must wait callback status,0 mean success
 *	2. must used after bk_ble_create_init and bk_ble_init_set_connect_dev_addr
 *	3. when connect result, will recv BLE_5_INIT_CONNECT_EVENT in ble_notice_cb_t
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_init_start_conn(uint8_t con_idx,ble_cmd_cb_t callback);


/**
 * @brief     Stop a connection
 *
 * @param
 *    - con_idx: the index of connection
 *    - callback: register a callback for this action, ble_cmd_t: BLE_INIT_STOP_CONN
 *
 * @attention 
 *	1. you must wait callback status,0 mean success
 *	2. must used after bk_ble_init_start_conn
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_init_stop_conn(uint8_t con_idx,ble_cmd_cb_t callback);


/**
 * @brief As master, read attribute value, the result is reported in the callback registered through bk_ble_register_app_sdp_charac_callback
 *
 * @param
 *    - conidx: the index of connection
 *    - handle: the handle of attribute value
 *    - callback: register a callback for this action, ble_cmd_t: BLE_INIT_READ_CHAR
 *
 * @return
 * - ERR_SUCCESS: succeed
 * - others: fail
 */
ble_err_t bk_ble_read_service_data_by_handle_req(uint8_t conidx,uint16_t handle,ble_cmd_cb_t callback);


/**
 * @brief As master, write attribute value
 *
 * @param
 *	- conidx: the index of connection
 *	- handle: the handle of attribute value
 *	- data: value data
 *	- data_len: the length of attribute value
 *	- callback: register a callback for this action, ble_cmd_t: BLE_INIT_WRITE_CHAR
 * @return
 * - ERR_SUCCESS: succeed
 * - others: fail
 */
ble_err_t bk_ble_write_service_data_req(uint8_t conidx,uint16_t handle,uint16_t data_len,uint8_t *data,ble_cmd_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif

