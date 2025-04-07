
:link_to_translation:`zh_CN:[中文]`

==========================
Bluetooth Low Energy (LE)
==========================


Overview
========================


The Bluetooth module provides users with interface functions such as scanning, connection, broadcast, and data transmission for short-distance communication.
Bluetooth is composed of one or more task execution bodies and relies on Bluetooth interrupt driver to run.
Bluetooth has multiple events and callbacks, which constitute feedback called by the user.


Role
-------------------------------------------------------
Generally speaking, the actively connecting device is called central/master/client, and the device being connected is called peripheral/slaver/server.
Once the connection relationship between the two ends is determined, it will basically not change.




Notes on API calls
-------------------------------------------------------

Most APIs have callback parameters, and you should wait for the callback execution to complete before proceeding to the next step.
The processing of callback and event callback should not have blocking operations, and tasks that are too complex and time-consuming should not be processed.
The call stack of callback cannot be too deep.

.. important::
    It is strongly advised to avoid blocking the Bluetooth task, as it can result in abnormal occurrences such as disconnections, failure to scan, or failure to establish connections.
	
	
Common usage scenarios
========================

Slave mode, create ATT database for peer browsing
-------------------------------------------------------
Bluetooth LE uses the ATT database as a double-ended operating entity, and all read and write notification and other operations are performed on the ATT database.
In order to build a standard-compliant database, you need to understand the concepts of services, characteristics, and UUIDs.

- Record: A piece of data in the database is called a record, which consists of handle, type, and value.
- Service: Each ATT database has one or more services, such as HID, HeartRate.
- Characteristic: Each service contains one or more characteristics. For example, HID includes HID map and HID report. The former is a key mapping table and the latter is a key reporting table. The specific operation is to first read the HID map and then parse the HID report according to the map, so that the specific value of the button can be known.
- UUID: The above all exist in the form of records in the ATT database. In order to know these special records, the UUID value specified by the Bluetooth standard must be used to assign the type of the record. For example, DECL_PRIMARY_SERVICE_128 (0x2800) indicates that this record is a service declaration.


The following are specific examples:
::

	//Service declaration
	#define BK_ATT_DECL_PRIMARY_SERVICE_128     {0x00,0x28,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	//Characteristic declaration
	#define BK_ATT_DECL_CHARACTERISTIC_128      {0x03,0x28,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	//Characteristic client configuration declaration. This is a special UUID, indicating that this record is used to configure the described characteristics, generally including notify and indicate.
	#define BK_ATT_DESC_CLIENT_CHAR_CFG_128     {0x02,0x29,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	
	#define WRITE_REQ_CHARACTERISTIC_128        {0x01,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	#define INDICATE_CHARACTERISTIC_128         {0x02,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	#define NOTIFY_CHARACTERISTIC_128           {0x03,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	
	//Service UUID
	static const uint8_t test_svc_uuid[16] = {0xFF,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0};
	
	//Database IDX:
	enum
	{
		TEST_IDX_SVC,
		TEST_IDX_FF01_VAL_CHAR,
		TEST_IDX_FF01_VAL_VALUE,
		TEST_IDX_FF02_VAL_CHAR,
		TEST_IDX_FF02_VAL_VALUE,
		TEST_IDX_FF02_VAL_IND_CFG,
		TEST_IDX_FF03_VAL_CHAR,
		TEST_IDX_FF03_VAL_VALUE,
		TEST_IDX_FF03_VAL_NTF_CFG,
		TEST_IDX_NB,
	};	
	//Database.	bk_attm_desc_t test_att_db[TEST_IDX_NB] =
	{
		//  Service Declaration
		[TEST_IDX_SVC]              = {BK_ATT_DECL_PRIMARY_SERVICE_128, PROP(RD), 0},
	
		//  Level Characteristic Declaration
		[TEST_IDX_FF01_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
		//  Level Characteristic Value
		[TEST_IDX_FF01_VAL_VALUE]   = {WRITE_REQ_CHARACTERISTIC_128,    PROP(WR)|ATT_UUID(128), 128|OPT(NO_OFFSET)},
	
		[TEST_IDX_FF02_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
		//  Level Characteristic Value
		[TEST_IDX_FF02_VAL_VALUE]   = {INDICATE_CHARACTERISTIC_128,     PROP(I), 128|OPT(NO_OFFSET)},
		//  Level Characteristic - Client Characteristic Configuration Descriptor
		[TEST_IDX_FF02_VAL_IND_CFG] = {BK_ATT_DESC_CLIENT_CHAR_CFG_128, PROP(RD)|PROP(WR),OPT(NO_OFFSET)},
	
		[TEST_IDX_FF03_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
		//  Level Characteristic Value
		[TEST_IDX_FF03_VAL_VALUE]   = {NOTIFY_CHARACTERISTIC_128,       PROP(N), 128|OPT(NO_OFFSET)},
		//  Level Characteristic - Client Characteristic Configuration Descriptor
		[TEST_IDX_FF03_VAL_NTF_CFG] = {BK_ATT_DESC_CLIENT_CHAR_CFG_128, PROP(RD)|PROP(WR), OPT(NO_OFFSET)},
	};
	
	void ble_notice_cb(ble_notice_t notice, void *param)
	{
		switch (notice) {
			case BLE_5_STACK_OK: //Protocol stack initialization successful
			case BLE_5_WRITE_EVENT: //Write event
			case BLE_5_READ_EVENT:  //Read event
			case BLE_5_TX_DONE：   //notify/indicate transmission done event
			case BLE_5_REPORT_ADV, //scan result report event	
			case BLE_5_MTU_CHANGE,  //MTU change event	
			case BLE_5_CONNECT_EVENT,   //Slave mode connection event	
			case BLE_5_DISCONNECT_EVENT, //slave disconnection event
			case BLE_5_CREATE_DB:  //Service creation completion event
				break;
		}
	}

	struct bk_ble_db_cfg ble_db_cfg;
	ble_db_cfg.att_db = (ble_attm_desc_t *)test_service_db;
	ble_db_cfg.att_db_nb = TEST_IDX_NB;
	//The server handle should be different every time the database is created.
	ble_db_cfg.prf_task_id = g_test_prf_task_id;
	ble_db_cfg.start_hdl = 0;
	//The type of UUID of the service record, here is 128 bit
	ble_db_cfg.svc_perm = BK_BLE_PERM_SET(SVC_UUID_LEN, UUID_128);
	//Copy the specific value of the service
	os_memcpy(&(ble_db_cfg.uuid[0]), &test_svc_uuid, 16);
	//Register event callback
	ble_set_notice_cb(ble_at_notice_cb);
	//Create database
	bk_ble_create_db(&ble_db_cfg);

At this point we get a service with a UUID of {0xFF,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}. This service contains a characteristic with a UUID of {0x01,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0} that has write permission, a characteristic with a UUID of {0x02,0xFF} that has the indicate attribute, and a characteristic with a UUID of{0x03,0xFF} that has the notify attribute.

.. important::
   1. Server UUID LEN can be configured through ble_db_cfg.svc_perm = BK_BLE_PERM_SET(SVC_UUID_LEN, UUID_128). UUID LEN can be UUID_16 or UUID_128.
   2. Characteristic UUID LEN is equal to adding ATT_UUID(uuid_len) to the corresponding Characteristic info variable in the database db. uuid_len is generally 16 or 128.
   3. When the event callback function ble_notice_cb receives the BLE_5_CREATE_DB event, the service creation is successful. If you need to create multiple services, you can continue to create other services after this event, in which ble_db_cfg.prf_task_id needs to be increased by 1.


Enable advertising in slave mode
-------------------------------------------------------

After setting up the service, you need to enable advertising so that the peer can scan us.

::

	void ble_cmd_cb(ble_cmd_t cmd, ble_cmd_param_t *param)
	{
		bk_printf("cmd:%d idx:%d status:%d\r\n", cmd, param->cmd_idx, param->status);
	}

	chnl_map = 7;
	adv_intv_min = 0x120; //min
	adv_intv_max = 0x160; //max
	//Get the currently idle active index to start advertising.
	actv_idx = app_ble_get_idle_actv_idx_handle();
	if (actv_idx != UNKNOW_ACT_IDX) {
		bk_ble_create_advertising(actv_idx,chnl_map,adv_intv_min,adv_intv_max, ble_cmd_cb);
	}
	
	//In ble_at_cmd_cb, wait for the BLE_CREATE_ADV event.
	...
	//

	//Bluetooth advertising data, please refer to Bluetooth LE standard format.
	const uint8_t adv_data[] = {0x0A, 0x09, 0x37 0x32, 0x33, 0x31, 0x4e, 0x5f, 0x42, 0x4c, 0x45};
	bk_ble_set_adv_data(actv_idx, adv_data, sizeof(adv_data), ble_cmd_cb);

	//In ble_at_cmd_cb, wait for the BLE_SET_ADV_DATA event
	...
	//

	//Scan response data, please refer to Bluetooth LE standard format.
	const uint8_t adv_data[] = {0x0A, 0x09, 0x37 0x32, 0x33, 0x31, 0x4e, 0x5f, 0x42, 0x4c, 0x45};
	bk_ble_set_ext_adv_data(actv_idx, adv_data, sizeof(adv_data), ble_cmd_cb);


	//In ble_at_cmd_cb, wait for the BLE_SET_RSP_DATA event
	...
	//

	//Start advertising.
	bk_ble_start_advertising(actv_idx, 0, ble_cmd_cb);

	//In ble_at_cmd_cb, wait for the BLE_START_ADV event to complete the advertising.
	...
	//In ble_notice_cb



Enable scanning in master mode
-------------------------------------------------------

::

	actv_idx = app_ble_get_idle_actv_idx_handle();
	bk_ble_create_scaning(actv_idx, ble_cmd_cb);

	//In ble_at_cmd_cb, wait for the BLE_CREATE_SCAN event.
	...
	//
	
	scan_intv=100;
	scan_wd=30;
	bk_ble_start_scaning(actv_idx,scan_intv, scan_wd,ble_cmd_cb);
	
	//In ble_at_cmd_cb, wait for the BLE_START_SCAN event.
	...
	//
	
	//Get the scan result advertising data from the BLE_5_REPORT_ADV event in ble_notice_cb_t.


	
Establish connection in master mode
-------------------------------------------------------

::

	//Get the currently idle active index for establishing a connection.
	con_idx = bk_ble_get_idle_conn_idx_handle();
	con_interval = 0x40; //interval
	con_latency = 0;
	sup_to = 0x200;//supervision timeout
	bk_ble_create_init(con_idx, con_interval, con_latency,sup_to,ble_cmd_cb);

	//In ble_at_cmd_cb, wait for the BLE_INIT_CREATE event.
	...
	//

	//Set the peer address type. Mismatch will cause the connection to fail.
	struct bd_addr bdaddr;
	uint8_t mac[6]={0xc8,0x47,0x8c,0x11,0x22,0x33};
	memcpy(bdaddr.addr,mac,6);
	addr_type = ADDR_PUBLIC;
	bk_ble_init_set_connect_dev_addr(actv_idx,&bdaddr,addr_type);


	bk_ble_init_start_conn(con_idx, ble_cmd_cb)

	//In ble_at_cmd_cb, wait for the BLE_INIT_START_CONN event
	...
	//In ble_notice_cb, wait for the BLE_5_INIT_CONNECT_EVENT. The slave connection is successful.


Introduction to CLI commands
=============================


Slave mode
-------------------------------------------------------

- Start/stop general advertising

::

	Method 1:
	Start advertising
	ble active 
	ble create_adv
	ble set_adv_data
	ble set_adv_data 0
	ble set_rsp_data 0
	ble start_adv 0

	Stop advertising
	ble stop_adv 0

	Method 2:
	Disable advertising
	ble active 
	ble init_adv

	Stop advertising
	ble deinit_adv


- Start/stop extended advertising

::

	Start extended advertising
	ble active 
	ble create_ext_adv 1 0
	ble set_ext_adv_data 0
	ble set_ext_rsp_data 0
	ble start_adv 0

	Stop extended advertising
	ble stop_adv 0


- After being connected by the master, notify/indicate transmission

::

	Notify transmission
	ble notify 0

	Indicate transmission
	ble indicate 0


- After being connected by the master, update the connection parameters.

::

	ble update_conn 0


- After being connected by the master, disconnect automatically

::

	ble dis_conn 0

- Start SMP encrypted pairing

::

	//legacy pairing
	ble smp_init 0

	// secure connection pairiing
	ble smp_init 1

	//ble send security request
	ble sec_req 0


Master mode
-------------------------------------------------------

- Start/stop scanning

::

	Method 1:
	Start scanning
	ble active 
	ble create_scan
	ble start_scan 0

	Stop scanning
	ble stop_scan 0

	Method 2:
	Start scanning
	ble active 
	ble init_scan

	Stop scanning
	ble deinit_scan



- Master initiates connection

::

	ble con_create

	//4900428c47c8 represents the slave MAC (little endian). The first 0 represents the address type: 0 for Public BD address and 1 for Random BD Address, usually 0.
	ble con_start 4900428c47c8  0  0 


- Read and write operations after successful connection

::
	
	//17 needs to be changed to a handler corresponding to the writable attribute of the slave service.
	ble con_write 17 0

	//17 needs to be changed to a handler corresponding to the readable attributes of the slave service.
	ble con_read 17 0


- After successful connection, disconnect automatically

::
	
	ble con_dis 0