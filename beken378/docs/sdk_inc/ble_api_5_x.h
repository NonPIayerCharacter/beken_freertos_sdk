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
 *
 * @param     ble_db_cfg: service param
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_create_db (struct bk_ble_db_cfg* ble_db_cfg);


/**
 * @brief     Register ble event notification callback
 *
 * @param     func: event callback
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
 *
 * @return     void
 */
void ble_set_notice_cb(ble_notice_cb_t func);


/**
 * @brief     Get device name
 *
 * @param	name: store the device name
 * @param	buf_len: the length of buf to store the device name
 *
 * @return
 *   		- length: the length of device name
 */
uint8_t ble_appm_get_dev_name(uint8_t* name, uint32_t buf_len);


/**
 * @brief     Set device name
 *
 * @param    len: the length of device name
 * @param    name: the device name to be set
 *
 * @return
 *    - length: the length of device name
 */
uint8_t ble_appm_set_dev_name(uint8_t len, uint8_t* name);

/**
 * @brief     Create and start a ble advertising activity
 *
 * @param   actv_idx: the index of activity
 * @param   adv: the advertising parameter
 * @param   callback: register a callback for this action, ble_cmd_t: BLE_INIT_ADV
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
 * @param    actv_idx: the index of activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_DEINIT_ADV
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
 * @param    actv_idx: the index of activity
 * @param    scan: the scan parameter
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_INIT_SCAN
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
 * @param    actv_idx: the index of activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_DEINIT_SCAN
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
 * @param    actv_idx: the index of activity
 * @param    chnl_map: the advertising channel map
 * @param    intv_min: the advertising min interval
 * @param    intv_max: the advertising max interval
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_CREATE_ADV
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
 * @param    actv_idx: 	 the index of activity
 * @param    chnl_map:	 the advertising channel map
 * @param    intv_min:	 the advertising min interval
 * @param    intv_max:    the advertising max interval
 * @param    scannable:   the advertising whether be scanned
 * @param    connectable: the advertising whether be connected
 * @param    callback: 	 register a callback for this action, ble_cmd_t: BLE_CREATE_ADV
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
 * @param    actv_idx: the index of activity
 * @param    duration: Advertising duration (in unit of 10ms). 0 means that advertising continues
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_START_ADV
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
 * @param    actv_idx: the index of activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_STOP_ADV
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
 * @param    actv_idx: the index of activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_DELETE_ADV
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
 * @param    actv_idx: the index of activity
 * @param    adv_buff: advertising data
 * @param    adv_len: the length of advertising data
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_SET_ADV_DATA
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
 * @param    actv_idx: the index of activity
 * @param    adv_buff: advertising data
 * @param    adv_len: the length of advertising data
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_SET_ADV_DATA
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
 * @param    actv_idx: the index of activity
 * @param    scan_buff: scan response data
 * @param    scan_len: the length of scan response data
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_SET_RSP_DATA
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
 * @param    actv_idx: the index of activity
 * @param    scan_buff: scan response data
 * @param    scan_len: the length of scan response data
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_SET_RSP_DATA
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
 * @brief     Create a ble periodic advertising activity.
 *
 * @param    actv_idx : the index of periodic advertising activity
 * @param    per_adv: the periodic advertising parameter
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_CREATE_ADV
 *
 * @attention
 *	1. you must wait callback status, 0 mean success.
 *
 * User example:
 * @code
 *		struct per_adv_param per_adv_param;
 *		per_adv_param.adv_intv_min = 120;
 *		per_adv_param.adv_intv_max = 160;
 *		per_adv_param.chnl_map = 7;
 *		per_adv_param.adv_prop = (0 << ADV_PROP_CONNECTABLE_POS) | (0 << ADV_PROP_SCANNABLE_POS);;
 *		per_adv_param.prim_phy = 1;
 *		per_adv_param.second_phy = 1;
 *		per_adv_param.own_addr_type = GAPM_STATIC_ADDR;
 *
 *		actv_idx = app_ble_get_idle_actv_idx_handle();
 *		bk_ble_create_periodic_advertising(actv_idx, &per_adv_param, ble_cmd_cb);
 * @endcode
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_create_periodic_advertising(uint8_t actv_idx, struct per_adv_param *per_adv, ble_cmd_cb_t callback);


/**
 * @brief     Set the periodic advertising data.
 *
 * @param    actv_idx : the index of periodic advertising activity
 * @param    adv_buff: periodic advertising data
 * @param    adv_len: the length of periodic advertising data
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_SET_ADV_DATA
 *
 * @attention
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_create_periodic_advertising.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_set_periodic_adv_data(uint8_t actv_idx, unsigned char *adv_buff, uint16_t adv_len, ble_cmd_cb_t callback);


/**
 * @brief     Start a ble periodic advertising.
 *
 * @param    actv_idx : the index of periodic advertising activity
 * @param    duration: periodic advertising (in unit of 10ms). 0 means that periodic advertising continues
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_START_ADV
 *
 *  @attention
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_create_periodic_advertising.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_start_periodic_advertising(uint8_t actv_idx, uint16 duration, ble_cmd_cb_t callback);


/**
 * @brief     Stop the periodic advertising that has been started.
 *
 * @param    actv_idx : the index of periodic advertising activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_STOP_ADV
 *
 * @attention
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_start_periodic_advertising.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_stop_periodic_advertising(uint8_t actv_idx, ble_cmd_cb_t callback);


/**
 * @brief     Delete the periodic advertising that has been created.
 *
 * @param    actv_idx : the index of periodic advertising activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_DELETE_ADV
 *
 * @attention
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_create_periodic_advertising.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_delete_periodic_advertising(uint8_t actv_idx, ble_cmd_cb_t callback);


/**
 * @brief     Create a ble periodic synchronize activity.
 *
 * @param    actv_idx : the index of periodic synchronize activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_CREATE_PERIODIC_SYNC
 *
 * @attention
 *	1. you must wait callback status, 0 mean success.
 *
 * User example:
 * @code
 *		actv_idx = app_ble_get_idle_actv_idx_handle();
 *		bk_ble_create_periodic_sync(actv_idx, ble_cmd_cb);
 * @endcode
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_create_periodic_sync(uint8_t actv_idx, ble_cmd_cb_t callback);


/**
 * @brief     Start a ble periodic synchronize.
 *
 * @param    actv_idx : the index of periodic synchronize activity
 * @param    param: the periodic synchronization parameters
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_START_PERIODIC_SYNC
 *
 * @attention
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_create_periodic_sync.
 *
 * User example:
 * @code
 *		ble_periodic_sync_param_t periodic_param;
 *		uint8_t mac[6]={0xc8,0x47,0x8c,0x11,0x22,0x33};
 *		memcpy(periodic_param.adv_addr.addr, mac, GAP_BD_ADDR_LEN);
 *		periodic_param.report_en_bf = 1;
 *		periodic_param.adv_sid = 0;
 *		periodic_param.adv_addr_type = 0;
 *		periodic_param.skip = 0;
 *		periodic_param.sync_to = 150;
 *		periodic_param.cte_type = 0;
 *		periodic_param.per_sync_type = GAPM_PER_SYNC_TYPE_GENERAL;
 *		bk_ble_start_periodic_sync(actv_idx, &periodic_param, ble_cmd_cb);
 * @endcode
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_start_periodic_sync(uint8_t actv_idx, ble_periodic_sync_param_t *param, ble_cmd_cb_t callback);


/**
 * @brief     Stop the periodic synchronize that has been started.
 *
 * @param    actv_idx : the index of periodic synchronize activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_STOP_PERIODIC_SYNC
 *
 * @attention
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_start_periodic_sync.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_stop_periodic_sync(uint8_t actv_idx, ble_cmd_cb_t callback);


/**
 * @brief     Delete the periodic synchronize that has been created.
 *
 * @param    actv_idx : the index of periodic synchronize activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_DELETE_PERIODIC_SYNC
 *
 * @attention
 *	1. you must wait callback status, 0 mean success.
 *	2. must used after bk_ble_create_periodic_sync.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_delete_periodic_sync(uint8_t actv_idx, ble_cmd_cb_t callback);


/**
 * @brief     Transfer periodic advertising sync information to peer device.
 *
 * @param    actv_idx : periodic advertising or periodic sync activity index
 * @param    service_data: a value provided by application
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_periodic_adv_sync_transf(uint8_t actv_idx, uint16_t service_data);


/**
 * @brief     Update connection parameters
 *
 * @param    conn_idx: the index of connection
 * @param    intv_min: connection min interval
 * @param    intv_max: connection max interval
 * @param    latency:  connection latency
 * @param    sup_to:   connection timeout
 * @attention
 *    1. must used after connected
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_update_param(uint8_t conn_idx, uint16_t intv_min, uint16_t intv_max,uint16_t latency, uint16_t sup_to);


/**
 * @brief     Disconnect a ble connection
 *
 * @param    conn_idx: the index of connection
 *
 * @attention
 *    1. must used after connected
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_disconnect(uint8_t conn_idx);


/**
 * @brief     Create a ble scan activity
 *
 * @param    actv_idx: the index of activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_CREATE_SCAN
 *
 * @attention 
 *	1. you must wait callback status, 0 mean success.
 *
 * User example:
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
 * @param    actv_idx:  the index of activity
 * @param    scan_intv: scan interval
 * @param    scan_wd:   scan window
 * @param    callback:  register a callback for this action, ble_cmd_t: BLE_START_SCAN
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
 * @param    actv_idx: the index of activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_STOP_SCAN
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
 * @param    actv_idx: the index of activity
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_DELETE_SCAN
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
 * @brief     Read the current transmitter PHY and receiver PHY on the connection identified by remote address.
 *
 * @param    conn_idx : the index of connection
 * @param    phy: store the tx_phy and rx_phy
 *
 * @attention
 *	1. The command is used after the connection is established.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_read_phy(uint8_t conn_idx, ble_read_phy_t *phy);


/**
 * @brief     Set the PHY preferences for the connection identified by the remote address.
 *
 * @param    conn_idx : the index of connection
 * @param    phy: the phy to be set
 *
 * @attention
 *	1. The Controller might not be able to make the change (e.g. because the peer does not support the requested PHY) or may decide that the current PHY is preferable.
 *	2. The command is used after the connection is established.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_set_phy(uint8_t conn_idx, ble_set_phy_t *phy);


/**
 * @brief     Add or Remove a single device to the Periodic Advertiser list stored in the Controller.
 *
 * @param    addr_remove : 0=remove/1=add
 * @param    addr: device address
 * @param    addr_type: address type of device 0=public/1=random
 * @param    adv_sid: advertising sid subfield in the ADI field used to identify the periodic advertising
 *
 * @attention
 *	1. The size of the per_adv list is 6.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_update_per_adv_list(uint8_t add_remove, struct bd_addr *addr, uint8_t addr_type, uint8_t adv_sid);


/**
 * @brief     Remove all devices from the list of Periodic Advertisers in the Controller.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_clear_per_adv_list(void);


/**
 * @brief     Get the total size of Whitelist.
 *
 * @param    wl_size : point to whitelist size
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_get_whitelist_size(uint8_t *wl_size);


/**
 * @brief     Add or Remove a single device to the whitelist.
 *
 * @param    addr_remove : 0=remove/1=add
 * @param    addr: point to device address
 * @param    addr_type: address type of device 0=public/1=random
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_update_whitelist(uint8_t add_remove, struct bd_addr *addr, uint8_t addr_type);


/**
 * @brief     Remove all devices from the Whitelist.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_clear_whitelist(void);


/**
 * @brief     Set aux connection parameters.
 *
 * @param    phy_mask : bit 0: 1M; bit 1: 2M; bit 2: coded
 * @param    phy_1m_conn_params: point to the params used by 1M phy
 * @param    phy_2m_conn_params: point to the params used by 2M phy
 * @param    phy_coded_conn_params: point to the params used by coded phy
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_prefer_ext_connect_params_set(uint8_t phy_mask, struct appm_create_conn_param *phy_1m_conn_params, struct appm_create_conn_param *phy_2m_conn_params, struct appm_create_conn_param *phy_coded_conn_params);


/**
 * @brief     Set local gap appearance icon.
 *
 * @param    appearance : External appearance value, these values are defined by the Bluetooth SIG, please refer to https://specificationrefs.bluetooth.com/assigned-values/Appearance%20Values.pdf
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_config_local_icon(uint16_t appearance);


/**
 * @brief     BLE set host channels classification.
 *
 * @param    channels : The nth such field (in the range 0 to 36) contains the value for the link layer channel index n. 0 means channel n is bad. 1 means channel n is unknown. The most significant bits are reserved and shall be set to 0. At least one channel shall be marked as unknown.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gap_set_channels(bk_ble_channels_t *channels);


/**
 * @brief     Get the device number of bonded peer devices.
 *
 * @param    dev_num : point to bonede device number
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_get_bond_device_num(uint8_t *dev_num);


/**
 * @brief     Get the device list of bonded peer devices.
 *
 * @param    dev_num : point to the size of device list. If the number of bonded devices is larger than dev_num, this function will return fail.
 * @param    dev_list: point to the list of bonded device.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_get_bonded_device_list(uint8_t *dev_num, bk_ble_bond_dev_t *dev_list);


/**
 * @brief     Set the params of pairing, include IO capability, key distribution, auth and secure level.
 *
 * @param    param : point to the configuration of pairing params.
 * @param    func: the notice callback function of secure module.
 *
 * User example:
 *     Suppose peer device is a phone, so that it's IO capability is KeyboardDisplay and it supports Secure conncetion pairing.
 * @code
 *     1. Legacy Pairing Use Pass Key Entry.
 *      param.sec_req = GAP_SEC1_AUTH_PAIR_ENC;
 *      param.auth = GAP_AUTH_REQ_MITM_BOND;
 *      param.ikey_dist = GAP_KDIST_ENCKEY | GAP_KDIST_LINKKEY | GAP_KDIST_IDKEY;
 *      param.rkey_dist = GAP_KDIST_ENCKEY | GAP_KDIST_LINKKEY;
 *      param.iocap = GAP_IO_CAP_DISPLAY_ONLY;
 * @endcode
 * @code
 *     2. Secure Connection Pairing Use Pass Key Entry.
 *      param.sec_req = GAP_SEC1_SEC_CON_PAIR_ENC;
 *      param.auth = GAP_AUTH_REQ_SEC_CON_BOND;
 *      param.ikey_dist = GAP_KDIST_IDKEY;
 *      param.rkey_dist = GAP_KDIST_NONE;
 *      param.iocap = GAP_IO_CAP_DISPLAY_ONLY;
 * @endcode
 * @code
 *     3. Secure Connection Pairing Use Just Work.
 *      param.sec_req = GAP_SEC1_NOAUTH_PAIR_ENC;
 *      param.auth = GAP_AUTH_REQ_SEC_CON_BOND;
 *      param.ikey_dist = GAP_KDIST_IDKEY;
 *      param.rkey_dist = GAP_KDIST_NONE;
 *      param.iocap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
 * @endcode
 *
 * @return
 *    - APP_SEC_ERROR_NO_ERROR: succeed
 *    - others: fail
 */
sec_err_t bk_ble_gap_set_security_param(struct app_pairing_cfg *param, sec_notice_cb_t func);


/**
 * @brief     Grant security request access.
 *
 * @param    conn_idx : connection idx of app
 * @param    accept: accept the security request or not
 *
 * @return
 *    - APP_SEC_ERROR_NO_ERROR: succeed
 *    - others: fail
 */
sec_err_t bk_ble_gap_security_rsp(uint8_t conn_idx, bool accept);


/**
 * @brief     Grant pairing request access.
 *
 * @param    conn_idx : connection idx of app
 * @param    accept: accept the pairing request or not
 *
 * @return
 *    - APP_SEC_ERROR_NO_ERROR: succeed
 *    - others: fail
 */
sec_err_t bk_ble_gap_pairing_rsp(uint8_t conn_idx, bool accept);


/**
 * @brief     Reply the temporary key value to the peer device.
 *
 * @param    conn_idx : connection idx of app
 * @param    accept: accept to process tk exchange
 * @param    passkey: passkey value, must be less than 999999
 *
 * @return
 *    - APP_SEC_ERROR_NO_ERROR: succeed
 *    - others: fail
 */
sec_err_t bk_ble_passkey_reply(uint8_t conn_idx, bool accept, uint32_t passkey);


/**
 * @brief     Reply the confirm value to the peer device in the secure connection using numeric comparison.
 *
 * @param    conn_idx : connection idx of app
 * @param    accept: numbers to compare are the same or different.
 *
 * @return
 *    - APP_SEC_ERROR_NO_ERROR: succeed
 *    - others: fail
 */
sec_err_t bk_ble_confirm_reply(uint8_t conn_idx, bool accept);


/**
 * @brief     Get the total packet number that can be sent to controller
 *
 * @param    dev_num : point to the total packet number
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_get_sendable_packets_num(uint16_t *pkt_total);


/**
 * @brief     Get the available packet number that can be sent to controller
 *
 * @param    dev_num : point to the available packet number
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_get_cur_sendable_packets_num(uint16_t *pkt_curr);


/**
 * @brief     Register ble master event notification callback
 *
 * @param    func: event callback
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
 * @param    con_idx: 	  the index of connection
 * @param    con_interval: the connection parameter
 * @param    con_latency:  the connection parameter
 * @param    sup_to: 	  the connection parameter
 * @param    callback:     register a callback for this action, ble_cmd_t: BLE_INIT_CREATE
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
 * @param    connidx: the index of connection
 * @param    bdaddr: the address of the device to be connected
 * @param    addr_type: the address type of the device to be connected, 1: public 0: random
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
 * @param    con_idx: the index of connection
 * @param    con_dev_time(in ms): supervision timeout for create connect.0 means keep scan target device
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_INIT_START_CONN
 *
 * @attention 
 *	1. you must wait callback status,0 mean success
 *	2. must used after bk_ble_create_init and bk_ble_init_set_connect_dev_addr
 *	3. when connect result, will recv BLE_5_INIT_CONNECT_EVENT in ble_notice_cb_t
 *	4. when connect timeout,will recv BLE_5_INIT_CONNECT_FAILED_EVENT
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_init_start_conn(uint8_t con_idx,uint16_t con_dev_time,ble_cmd_cb_t callback);


/**
 * @brief     Stop a connection
 *
 * @param    con_idx: the index of connection
 * @param    callback: register a callback for this action, ble_cmd_t: BLE_INIT_STOP_CONN
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
 * @brief As slaver, send a notification of an attribute's value
 *
 * @param    conidx: the index of connection
 * @param    len: the length of attribute's value
 * @param    buf: attribute's value
 * @param    prf_id: The id of the profile
 * @param    att_idx: The index of the attribute
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_conidx_send_ntf(uint8_t conidx,uint32_t len, uint8_t *buf, uint16_t prf_id, uint16_t att_idx);


/**
 * @brief As slaver, send an indication of an attribute's value
 *
 * @param    conidx: the index of connection
 * @param    len: the length of attribute's value
 * @param    buf: attribute's value
 * @param    prf_id: The id of the profile
 * @param    att_idx: The index of the attribute
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_conidx_send_ind(uint8_t conidx,uint32_t len, uint8_t *buf, uint16_t prf_id, uint16_t att_idx);


/**
 * @brief As master, read attribute value, the result is reported in the callback registered through bk_ble_register_app_sdp_charac_callback
 *
 * @param    conidx: the index of connection
 * @param    handle: the handle of attribute value
 *
 * @return
 * - ERR_SUCCESS: succeed
 * - others: fail
 */
ble_err_t bk_ble_read_service_data_by_handle_req(uint8_t conidx,uint16_t handle);


/**
 * @brief As master, write attribute value
 *
 * @param	conidx: the index of connection
 * @param	handle: the handle of attribute value
 * @param	data: value data
 * @param	data_len: the length of attribute value
 * @return
 * - ERR_SUCCESS: succeed
 * - others: fail
 */
ble_err_t bk_ble_write_service_data_req(uint8_t conidx,uint16_t handle,uint16_t data_len,uint8_t *data);


/**
 * @brief     Exchange MTU
 *
 * @param    conn_idx: the index of connection
 * @attention
 *    1. must used after connected
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: other errors.
 */
ble_err_t bk_ble_gatt_mtu_change(uint8_t conn_idx);


/**
 * @brief     Send a response to a read request.
 *
 * @param    rsp : response data.
 *
 * @attention
 *	1. The role of GATT is server.
 *
 * User example:
 * @code
 *		uint16_t length = 3;
 *		uint8_t value[3] = {0};
 *		value[0] = 0x12;
 *		value[1] = 0x34;
 *		value[2] = 0x56;
 *
 *		app_gatts_rsp_t rsp;
 *		rsp.token = token; // Token provided by GATT module into the GATT_READ_REQ_IND message.
 *		rsp.con_idx = conn_idx; // Connection index.
 *		rsp.attr_handle = hdl; // Attribute handle.
 *		rsp.status = GAP_ERR_NO_ERROR; // Status of the request by GATT user.
 *		rsp.att_length = length;
 *		rsp.value_length = length;
 *		memcpy(rsp.value, value, length);
 *
 *		bk_ble_gatts_read_response(&rsp);
 * @endcode
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gatts_read_response(app_gatts_rsp_t *rsp);


/**
 * @brief     Unregister with GATT Server.
 *
 * @param    service_handle : the service_handle of GATT Server access handle
 *
 * @attention
 *	1. The service_handle is any handle between start_handle and end_handle of this unregister with GATT Server.
 *	2. The role of GATT is server.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gatts_app_unregister(uint16_t service_handle);


/**
 * @brief     Remove a service.
 *
 * @param    start_handle : attribute start_handle of the service to remove
 *
 * @attention
 *	1. The role of GATT is server.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gatts_remove_service(uint16_t start_handle);


/**
 * @brief     Set the attribute value by the application.
 *
 * @param    attr_handle : the attr_handle which to be set
 * @param    length: the value length
 * @param    value: the pointer to the attribute value
 *
 * @attention
 *	1. The attr_handle corresponding attribute UUID cannot be an attribute declaration.
 *	2. The role of GATT is server.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gatts_set_attr_value(uint16_t attr_handle, uint16_t length, uint8_t *value);


/**
 * @brief     Retrieve attribute value.
 *
 * @param    attr_handle : attribute handle
 * @param    length: the pointer to the attribute value length
 * @param    value:  the pointer to attribute value payload, the value cannot be modified by user
 *
 * @attention
 *	1. The role of GATT is server.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gatts_get_attr_value(uint16_t attr_handle, uint16_t *length, uint8_t **value);


/**
 * @brief     Send service change indication.
 *
 * @param    start_handle : Service changed start handle
 * @param    end_handle: Service changed end handle
 *
 * @attention
 *	1. The role of GATT is server.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gatts_send_service_change_indication(uint16_t start_handle, uint16_t end_handle);


/**
 * @brief     Read a service's characteristics of the given characteristic UUID.
 *
 * @param    conn_idx : the index of connection
 * @param    start_handle: Service changed start handle
 * @param    end_handle: Service changed end handle
 * @param    uuid_type: the uuid type
 * @param    uuid: the uuid of attribute which will be read
 *
 * @attention
 *	1. The input of uuid is little endian.
 *	2. The attribute characteristics property to uuid supports the read property.
 *	3. The role of GATT is client.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gattc_read_by_type(uint8_t conn_idx, uint16_t start_handle, uint16_t end_handle, uint8_t uuid_type, uint8_t *uuid);


/**
 * @brief     Read multiple characteristic or characteristic descriptors.
 *
 * @param    conn_idx : the index of connection
 * @param    read_multi : pointer to the read multiple parameter
 *
 * @attention
 *	1. The role of GATT is client.
 *
 * User example:
 * @code
 *		app_gattc_multi_t read_multi;
 *		gatt_att_t atts[2];
 *		read_multi.nb_att = 2;
 *		atts[0].length = 3;
 *		atts[0].hdl = 0x1b;
 *		atts[1].length = 3;
 *		atts[1].hdl = 0x1e;
 *		read_multi.p_atts = atts;
 *		bk_ble_gattc_read_multiple(conn_idx, &read_multi);
 * @endcode
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gattc_read_multiple(uint8_t conn_idx, app_gattc_multi_t *read_multi);


/**
 * @brief     Register for notification of a service.
 *
 * @param    conn_idx : the index of connection
 * @param    handle: GATT client characteristic configuration descriptor handle
 *
 * @attention
 *	1. The input of handle supports the notify property.
 *	2. The role of GATT is client.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gattc_register_for_notify(uint8_t conidx, uint16_t handle);


/**
 * @brief     Register for indication of a service.
 *
 * @param    conn_idx : the index of connection
 * @param    handle: GATT client characteristic configuration descriptor handle
 *
 * @attention
 *	1. The input of handle supports the indicate property.
 *	2. The role of GATT is client.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gattc_register_for_indicate(uint8_t conidx, uint16_t handle);


/**
 * @brief     Unregister for notification or indication of a service.
 *
 * @param    conn_idx : the index of connection
 * @param    handle: GATT client characteristic configuration descriptor handle
 *
 * @attention
 *	1. The input of handle supports the notify or indicate property.
 *	2. The role of GATT is client.
 *
 * @return
 *    - ERR_SUCCESS: succeed
 *    - others: fail
 */
ble_err_t bk_ble_gattc_unregister_for_notify_or_indicate(uint8_t conidx, uint16_t handle);


#ifdef __cplusplus
}
#endif

#endif

