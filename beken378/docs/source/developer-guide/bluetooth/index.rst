========================
低功耗蓝牙(BLE)
========================


概述
========================


蓝牙模块向用户提供扫描、连接、广播、传输数据等接口功能，用于短距通讯。
蓝牙由一个或多个task执行体组成，依靠蓝牙中断驱动运行。
蓝牙有多个event和callback，这些构成了用户调用的反馈。


角色
-------------------------------------------------------
一般来讲，主动连接的设备称之为central/master/client，被连接的设备称之为peripheral/slaver/server。
一旦两端连接关系确定下来，则基本不会变化。




API调用注意事项
-------------------------------------------------------

大部分API具有callback参数，应当等待callback执行完成后再进行下一步。
callback、event callback的处理不应有阻塞操作，不能处理太复杂太耗时的任务。
callback的调用栈不能太深。

.. important::
    应极力避免蓝牙task被阻塞，否则会出现断连、扫不到、连不上等异常现象。
	
	
常用使用场景
========================

slave模式，创建ATT数据库供对端浏览
-------------------------------------------------------
ble通过ATT数据库作为双端的操作实体，所有的读写通知等操作都是对ATT数据库进行的。
为了建立一个符合标准的数据库，需要了解服务、特征、UUID的概念。

- 记录：数据库的一条数据称之为记录，由handle，类型、值组成。
- 服务：每个ATT数据库具有一个或多个服务，例如HID、HeartRate。
- 特征：每个服务包含一个或多个特征，例如HID包括HID map、HID report，前者是按键映射表，后者是按键上报，具体操作是先读取HID map，再根据map解析HID report就能知道按键具体值。
- UUID：以上几个均以记录的形式存在于ATT数据库中，为了知晓这些特殊记录，要用蓝牙标准规定的UUID值赋予记录的type。例如，DECL_PRIMARY_SERVICE_128(0x2800)表示这条记录为服务声明。


以下为具体示例
::

	//服务声明
	#define BK_ATT_DECL_PRIMARY_SERVICE_128     {0x00,0x28,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	//特征声明
	#define BK_ATT_DECL_CHARACTERISTIC_128      {0x03,0x28,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	//特征client配置声明。这是一个特殊的UUID，表示这条记录用于配置被描述的特征，一般有notify、indicate
	#define BK_ATT_DESC_CLIENT_CHAR_CFG_128     {0x02,0x29,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	
	#define WRITE_REQ_CHARACTERISTIC_128        {0x01,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	#define INDICATE_CHARACTERISTIC_128         {0x02,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	#define NOTIFY_CHARACTERISTIC_128           {0x03,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
	
	//服务UUID
	static const uint8_t test_svc_uuid[16] = {0xFF,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0};
	
	//数据库下标:
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
	//数据库.	bk_attm_desc_t test_att_db[TEST_IDX_NB] =
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
			case BLE_5_STACK_OK: //协议栈初始化成功
			case BLE_5_WRITE_EVENT: //写事件
			case BLE_5_READ_EVENT:  //读事件
			case BLE_5_TX_DONE：   // notify/indicate 发送成功事件
			case	BLE_5_REPORT_ADV, / scan 结果上报事件	
			case BLE_5_MTU_CHANGE,  // mtu 改变事件	
			case BLE_5_CONNECT_EVENT,   // slave 模式连接事件	
			case BLE_5_DISCONNECT_EVENT, // slave 断开事件
			case BLE_5_CREATE_DB:  // 服务创建完成事件
				break;
		}
	}

	struct bk_ble_db_cfg ble_db_cfg;
	ble_db_cfg.att_db = (ble_attm_desc_t *)test_service_db;
	ble_db_cfg.att_db_nb = TEST_IDX_NB;
	//server handle，每次创建数据库，应当不同。
	ble_db_cfg.prf_task_id = g_test_prf_task_id;
	ble_db_cfg.start_hdl = 0;
	//服务记录的UUID的类型，这里为128bit
	ble_db_cfg.svc_perm = BK_BLE_PERM_SET(SVC_UUID_LEN, UUID_128);
	//给服务具体值复制
	os_memcpy(&(ble_db_cfg.uuid[0]), &test_svc_uuid, 16);
	// 注册事件回调
	ble_set_notice_cb(ble_at_notice_cb);
	//创建数据库
	bk_ble_create_db(&ble_db_cfg);

此时我们得到一个UUID 为{0xFF,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0} 的服务，该服务包含一个UUID 为
{0x01,0xFF,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}的特征，该特征具有写权限；
UUID 为{0x02,0xFF}的特征，具有indicate 属性;
UUID 为{0x03,0xFF}的特征，具有notify 属性；

.. important::
   1. Server UUID len 可通过 ble_db_cfg.svc_perm = BK_BLE_PERM_SET(SVC_UUID_LEN, UUID_128)配置，UUID_LEN 有 UUID_16、UUID_128。
   2. Characteristic UUID len 同通过数据库db 中对应Characteristic 的info 变量加上ATT_UUID(uuid_len),uuid_len 一般为16或者128。
   3. 当事件回调函数ble_notice_cb收到BLE_5_CREATE_DB事件，才说明服务创建成功，如果需要创建多个服务，可以在此事件后继续创建其他服务，其中ble_db_cfg.prf_task_id 需要加1；


slave 模式开启广播
-------------------------------------------------------

设定好服务后，需要开启广播以让对端扫描到我们。

::

	void ble_cmd_cb(ble_cmd_t cmd, ble_cmd_param_t *param)
	{
		bk_printf("cmd:%d idx:%d status:%d\r\n", cmd, param->cmd_idx, param->status);
	}

	chnl_map = 7;
	adv_intv_min = 0x120; //min
	adv_intv_max = 0x160; //max
	//获取当前空闲的active index，用于开启广播
	actv_idx = app_ble_get_idle_actv_idx_handle();
	if (actv_idx != UNKNOW_ACT_IDX) {
		bk_ble_create_advertising(actv_idx,chnl_map,adv_intv_min,adv_intv_max, ble_cmd_cb);
	}
	
	//在ble_at_cmd_cb中，等待BLE_CREATE_ADV事件
	...
	//

	//蓝牙广播数据，请参考ble标准格式
	const uint8_t adv_data[] = {0x0A, 0x09, 0x37 0x32, 0x33, 0x31, 0x4e, 0x5f, 0x42, 0x4c, 0x45};
	bk_ble_set_adv_data(actv_idx, adv_data, sizeof(adv_data), ble_cmd_cb);

	//在ble_at_cmd_cb中，等待BLE_SET_ADV_DATA事件
	...
	//

	//扫描响应数据，请参考ble标准格式
	const uint8_t adv_data[] = {0x0A, 0x09, 0x37 0x32, 0x33, 0x31, 0x4e, 0x5f, 0x42, 0x4c, 0x45};
	bk_ble_set_ext_adv_data(actv_idx, adv_data, sizeof(adv_data), ble_cmd_cb);


	//在ble_at_cmd_cb中，等待BLE_SET_RSP_DATA事件
	...
	//

	//开启广播
	bk_ble_start_advertising(actv_idx, 0, ble_cmd_cb);

	//在ble_at_cmd_cb中，等待BLE_START_ADV事件,开启广播完成
	...
	//在ble_notice_cb



master 模式开启扫描
-------------------------------------------------------

::

	actv_idx = app_ble_get_idle_actv_idx_handle();
	bk_ble_create_scaning(actv_idx, ble_cmd_cb);

	//在ble_at_cmd_cb中，等待BLE_CREATE_SCAN
	...
	//
	
	scan_intv=100;
	scan_wd=30;
	bk_ble_start_scaning(actv_idx,scan_intv, scan_wd,ble_cmd_cb);
	
	//在ble_at_cmd_cb中，等待BLE_START_SCAN
	...
	//
	
	//在ble_notice_cb_t中BLE_5_REPORT_ADV事件 获取scan结果广播数据


	
master 模式建立连接
-------------------------------------------------------

::

	//获取当前空闲的active index，用于建立连接
	con_idx = bk_ble_get_idle_conn_idx_handle();
	con_interval = 0x40; //interval
	con_latency = 0;
	sup_to = 0x200;//supervision timeout
	bk_ble_create_init(con_idx, con_interval, con_latency,sup_to,ble_cmd_cb);

	//在ble_at_cmd_cb中，等待BLE_INIT_CREATE
	...
	//

	//设置对端地址类型，不匹配会导致连接不上
	struct bd_addr bdaddr;
	uint8_t mac[6]={0xc8,0x47,0x8c,0x11,0x22,0x33};
	memcpy(bdaddr.addr,mac,6);
	addr_type = ADDR_PUBLIC;
	bk_ble_init_set_connect_dev_addr(actv_idx,&bdaddr,addr_type);


	bk_ble_init_start_conn(con_idx, ble_cmd_cb)

	//在ble_at_cmd_cb中，等待BLE_INIT_START_CONN
	...
	//在ble_notice_cb中等待 BLE_5_INIT_CONNECT_EVENT，连接从机成功


CLI 命令介绍
========================


slave 模式
-------------------------------------------------------

- 开启/停止普通广播

::

	方式一：
	开启广播
	ble active 
	ble create_adv
	ble set_adv_data
	ble set_adv_data 0
	ble set_rsp_data 0
	ble start_adv 0

	停止广播
	ble stop_adv 0

	方式二：
	关闭广播
	ble active 
	ble init_adv

	停止广播
	ble deinit_adv


- 开启/停止拓展普通广播

::

	开启拓展广播
	ble active 
	ble create_ext_adv 1 0
	ble set_ext_adv_data 0
	ble set_ext_rsp_data 0
	ble start_adv 0

	停止拓展广播
	ble stop_adv 0


- 被master 连接上后，notify/indicate发送数据

::

	notify 发送
	ble notify 0

	indicate 发送
	ble indicate 0


- 被master 连接上后，更新连接参数

::

	ble update_conn 0


- 被master 连接上后，主动断开

::

	ble dis_conn 0

- 启动smp加密配对

::

	//legacy pairing
	ble smp_init 0

	// secure connection pairiing
	ble smp_init 1

	//ble send security request
	ble sec_req 0


master 模式
-------------------------------------------------------

- 开启/停止扫描

::

	方式一：
	开启扫描
	ble active 
	ble create_scan
	ble start_scan 0

	停止扫描
	ble stop_scan 0

	方式二：
	开启扫描
	ble active 
	ble init_scan

	停止扫描
	ble deinit_scan



- master发起连接

::

	ble con_create

	//4900428c47c8代表从机mac，小端, 第一个0代表的是地址类型，0：Public BD address  1: Random BD Address,一般为0)
	ble con_start 4900428c47c8  0  0 


- 连接成功后读写操作

::
	
	//17 需要改成对应从机服务的可写属性的handler
	ble con_write 17 0

	//17 需要改成对应从机服务的可读属性的handler
	ble con_read 17 0


- 连接成功后，主动断开

::
	
	ble con_dis 0