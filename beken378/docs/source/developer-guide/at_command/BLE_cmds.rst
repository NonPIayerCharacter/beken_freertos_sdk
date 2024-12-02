BLE指令集
=================


--------------------------------------------
AT+BLEINIT：Bluetooth LE 初始化
--------------------------------------------

**查询命令**

命令：
::

	AT+BLEINIT?

若已初始化，AT 返回：
::

	+BLEINIT:<role>
	OK

若未初始化，AT 返回：
::

	+BLEINIT:0
	OK

**执行命令**

命令：
::

	AT+BLEINIT=<init>

返回：
::

	OK

**参数**

- <init>:

	- 0: 注销 Bluetooth LE
	- 1: client 角色
	- 2: server 角色

**说明**

- 使用其它 Bluetooth LE 命令之前，请先调用本命令，初始化 Bluetooth LE 角色。
- Bluetooth LE 角色初始化后，不能直接切换。如需切换角色，需要先调用 AT+RST 命令重启系统，再重新初始化 Bluetooth LE 角色。
- 若使用BK7238 作为 Bluetooth LE server，需烧录 service bin 到 flash：0x1F8000对于如何生成 service bin 文件，请参考tools/at_ble_service/readme.md。
- 建议注销 Bluetooth LE 之前，停止正在进行的广播、扫描并断开所有的连接。

**示例**
::

	AT+BLEINIT=1


-----------------------------------------------
AT+BLENAME：betooth LE 设备名称
-----------------------------------------------

**查询命令**

命令：
::

	AT+BLENAME?

返回：
::

	+BLENAME:<device_name>
	OK

**执行命令**

命令：
::

	AT+BLENAME=<device_name>

返回：
::

	OK

**参数**

- <device_name>：Bluetooth LE 设备名称，最大长度：18。

**示例**
::

	AT+BLENAME="ble_demo"


---------------------------------------------------------
AT+BLESCANPARAM：查询/设置 Bluetooth LE 扫描参数
---------------------------------------------------------

**查询命令**

命令：
::

	AT+BLESCANPARAM?

返回：
::

	+BLESCANPARAM:<scan_type>,<own_addr_type>,<filter_policy>,<scan_interval>,<scan_window>
	OK

**执行命令**

命令：
::

	AT+BLESCANPARAM=<scan_type>,<own_addr_type>,<filter_policy>,<scan_interval>,<scan_window>

返回：
::

	OK

**参数**

- <scan_type>：扫描类型

	- 0: 被动扫描
	- 1: 主动扫描

- <own_addr_type>：地址类型

	- 0: 公共地址
	- 1: 随机地址
	- 2: RPA 公共地址
	- 3: RPA 随机地址

- <filter_policy>：扫描过滤方式，暂不支持设置
- <scan_interval>：扫描间隔。本参数值应大于等于 <scan_window> 参数值。参数范围：[0x0004,0x4000]。扫描间隔是该参数乘以 0.625 毫秒，所以实际的扫描间隔范围为 [2.5,10240] 毫秒。
- <scan_window>：扫描窗口。本参数值应小于等于 <scan_interval> 参数值。参数范围：[0x0004,0x4000]。扫描窗口是该参数乘以 0.625 毫秒，所以实际的扫描窗口范围为 [2.5,10240] 毫秒。


**示例**
::

	AT+BLEINIT=1		//角色：客户端
	AT+BLESCANPARAM=0,0,0,100,50


------------------------------------------------
AT+BLESCAN：使能 Bluetooth LE 扫描
------------------------------------------------

**执行命令**

命令：
::

	AT+BLESCAN=<enable>[,<interval>][,<filter_type>,<filter_param>]

返回：
::

	+BLESCAN:<addr>,<rssi>,<adv_data>,<scan_rsp_data>,<addr_type>
	OK

**参数**

- <enable>：

	- 1: 开始持续扫描
	- 0: 停止持续扫描

- [<interval>]：扫描持续时间，单位：秒。暂不支持设置
- [<filter_type>]：过滤选项

	- 1: "MAC"
	- 2: "NAME"

- [<filter_param>]：过滤参数，表示对方设备 MAC 地址或名称
- <addr>：Bluetooth LE 地址
- <rssi>：信号强度
- <adv_data>：广播数据
- <scan_rsp_data>：扫描响应数据
- <addr_type>：广播设备地址类型

**说明**

- 目前filter_type和filter_param 过滤项暂不支持。
- 响应中的 OK 和 +BLESCAN:<addr>,<rssi>,<adv_data>,<scan_rsp_data>,<addr_type>在输出顺序上没有严格意义上的先后顺序。

**示例**
::

	AT+BLEINIT=1		//角色：客户端
	AT+BLESCAN=1		//开始扫描
	AT+BLESCAN=0		//停止扫描
	AT+BLESCAN=1,3,1,"24:0A:C4:96:E6:88"	//开始扫描，过滤类型为 MAC 地址
	AT+BLESCAN=1,3,2,"BK-AT"		//开始扫描，过滤类型为设备名称


----------------------------------------------------
AT+BLEADDR：设置 Bluetooth LE 设备地址
----------------------------------------------------

**查询命令**

命令：
::

	AT+BLEADDR?

返回：
::

	+BLEADDR:<BLE_public_addr>
	OK

**说明**

- 设置Bluetooth LE设备的地址类型不支持修改，ble地址和wifi地址相关，如需更改请设置wifi地址。


----------------------------------------------------
AT+BLESCANRSPDATA：设置 Bluetooth LE 扫描响应
----------------------------------------------------

**执行命令**

命令：
::

	AT+BLESCANRSPDATA=<scan_rsp_data>

返回：
::

	OK

**参数**

- <scan_rsp_data>：扫描响应数据，为 HEX 字符串。例如，若想设置扫描响应数据为 “0x11 0x22 0x33 0x44 0x55”，则命令为 AT+BLESCANRSPDATA=”1122334455”。

**示例**
::

	AT+BLEINIT=2				//角色：服务器
	AT+BLESCANRSPDATA=050837323338		//short_name 7238


----------------------------------------------------
AT+BLEADVPARAM：查询/设置 Bluetooth LE 广播参数
----------------------------------------------------

**查询命令**

命令：
::

	AT+BLEADVPARAM?

返回：
::

	+BLEADVPARAM:<adv_int_min>,<adv_int_max>,<adv_type>,<channel_map>
	OK

**执行命令**

命令：
::

	AT+BLEADVPARAM=<adv_int_min>,<adv_int_max>,<adv_type>[,<own_addr_type>],<channel_map>[,<adv_filter_policy>][,<peer_addr_type>,<peer_addr>]

返回：
::

	OK

**参数**

- <adv_int_min>：最小广播间隔。参数范围：[0x0020,0x4000]。广播间隔等于该参数乘以0.625毫秒，所以实际的最小广播间隔范围为[20,10240]毫秒。本参数值应小于等于<adv_int_max> 参数值。
- <adv_int_max>：最大广播间隔。参数范围：[0x0020,0x4000]。广播间隔等于该参数乘以0.625毫秒，所以实际的最大广播间隔范围为[20,10240]毫秒。本参数值应大于等于<adv_int_min> 参数值。
- <adv_type>:

	- 0: ADV_TYPE_IND
	- 1: ADV_TYPE_DIRECT_IND_HIGH
	- 2: ADV_TYPE_SCAN_IND
	- 3: ADV_TYPE_NONCONN_IND
	- 4: ADV_TYPE_DIRECT_IND_LOW

- [<own_addr_type>]：暂不支持设置
- <channel_map>：广播信道

	- 1: ADV_CHNL_37
	- 2: ADV_CHNL_38
	- 4: ADV_CHNL_39
	- 7: ADV_CHNL_ALL

- [<adv_filter_policy>]：暂不支持设置
- [<peer_addr_type>]：暂不支持设置
- [<peer_addr>]：暂不支持设置
- [<primary_phy>]：暂不支持设置
- [<secondary_phy>]：暂不支持设置

**示例**
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEADVPARAM=160,160,0,7


--------------------------------------------------------------------
AT+BLEADVDATA：设置 Bluetooth LE 广播数据
--------------------------------------------------------------------

**执行命令**

命令：
::

	AT+BLEADVDATA=<adv_data>

返回：
::

	OK

**参数**

- <adv_data>：广播数据，为 HEX 字符串。例如，若想设置广播数据"Local Name"为 "7238_BLE"，则命令为 AT+BLEADVDATA="0909373233385F424C45"。

**说明**

- 如果之前已经使用命令AT+BLEADVDATAEX=<dev_name>,<uuid>,<manufacturer_data>,<include_power> 设置了广播数据，则会被本命令设置的广播数据覆盖。
- 如果您想使用本命令修改设备名称，则建议在执行完该命令之后执行 AT+BLENAME 命令将设备名称设置为同样的名称。


---------------------------------------------------------
AT+BLEADVDATAEX：自动设置 Bluetooth LE 广播数据
---------------------------------------------------------

**查询命令**

命令：
::

	AT+BLEADVDATAEX?

返回：
::

	+BLEADVDATAEX:<dev_name>,<uuid>,<manufacturer_data>,<include_power>
	OK

**执行命令**

命令：
::

	AT+BLEADVDATAEX=<dev_name>,<uuid>,<manufacturer_data>,<include_power>

返回：
::

	OK

**参数**

- <dev_name>：字符串参数，表示设备名称。例如，若设置设备名称为 “just-test”，则命令为AT+BLEADVSTARTEX="just-test",<uuid>,<manufacturer_data>,<include_power>。
- <uuid>：字符串参数。例如，若想设置 UUID 为 “0xA002”，则命令为AT+BLEADVSTARTEX=<dev_name>,"A002",<manufacturer_data>,<include_power>。
- <manufacturer_data>：制造商数据，为 HEX 字符串。例如，若想设置制造商数据为 “0x11 0x22 0x33 0x44 0x55”，则命令为AT+BLEADVSTARTEX=<dev_name>,<uuid>,"1122334455",<include_power>。
- <include_power>：暂不支持设置。

**说明**

- 如果之前已经使用命令 AT+BLEADVDATA=<adv_data> 设置了广播数据，则会被本命令设置的广播数据覆盖。

**示例**
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEADVDATAEX="AT_DEMO","A002","0102030405",1


-------------------------------------------------
AT+BLEADVSTART：开始 Bluetooth LE 广播
-------------------------------------------------

**执行命令**

命令：
::

	AT+BLEADVSTART

返回：
::

	OK

**说明**

- 若未使用命令 AT+BLEADVPARAM=<adv_parameter> 设置广播参数，则使用默认广播参数。
- 若未使用命令 AT+BLEADVDATA=<adv_data> 设置广播数据，则发送全 0 数据包。若之前已经使用命令 AT+BLEADVDATA=<adv_data> 设置过广播数据，则会被 AT+BLEADVDATAEX=<dev_name>,<uuid>,<manufacturer_data>,<include_power> 设置的广播数据覆盖，相反，如果先使用 AT+BLEADVDATAEX，则会被 AT+BLEADVDATA 设置的广播数据覆盖。
- 开启 Bluetooth LE 广播后，如果没有建立 Bluetooth LE 连接，那么将会一直保持广播；如果建立了连接，则会自动结束广播。

**示例**
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEADVSTART


---------------------------------------------------
AT+BLEADVSTOP：停止 Bluetooth LE 广播
---------------------------------------------------

**执行命令**

命令：
::

	AT+BLEADVSTOP

返回：
::

	OK

**说明**

- 若开始广播后，成功建立 Bluetooth LE 连接，则会自动结束 Bluetooth LE 广播，无需调用本命令。

**示例**
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEADVSTART
	AT+BLEADVSTOP


-----------------------------------------------
AT+BLECONN：建立 Bluetooth LE 连接
-----------------------------------------------

**查询命令**

命令：
::

	AT+BLECONN?

返回：
::

	+BLECONN:<conn_index>,<remote_address>
	OK

若未建立连接，则响应不显示 <conn_index> 和 <remote_address> 参数。

**执行命令**

命令：
::

	AT+BLECONN=<conn_index>,<remote_address>[,<addr_type>,<timeout>]

返回：
若建立连接成功，则提示：
::

	+BLECONN:<conn_index>,<remote_address>
	OK

若建立连接失败，则提示：
::

	+BLECONN:<conn_index>,-1
	ERROR

若是因为参数错误或者其它的一些原因导致连接失败，则提示：
::

	ERROR

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。
- <remote_address>：对方 Bluetooth LE 设备地址。
- [<addr_type>]：广播设备地址类型，默认值0：

	- 0: 公共地址 (Public Address)
	- 1: 随机地址 (Random Address)

- [<timeout>]：暂不支持设置，默认5s

**说明**

- 建议在建立新连接之前，先运行 AT+BLESCAN 命令扫描设备，确保目标设备处于广播状态。
- 如果 Bluetooth LE server 已初始化且连接已成功建立，则可以使用此命令在对等设备 (GATTC) 中发现服务。

**示例**

.. note::

	此处测试需要先用另外一块板开启服务端打开广播。

::

	AT+BLEINIT=1		//角色：客户端
	AT+BLECONN=0,"4988428C47C8",0,10


---------------------------------------------------------
AT+BLECONNPARAM：查询/更新 Bluetooth LE 连接参数
---------------------------------------------------------

**查询命令**

命令：
::

	AT+BLECONNPARAM?

返回：
::

	+BLECONNPARAM:<conn_index>,<min_interval>,<max_interval>,<cur_interval>,<latency>,<timeout>
	OK

**执行命令**

命令：
::

	AT+BLECONNPARAM=<conn_index>,<min_interval>,<max_interval>,<latency>,<timeout>

返回：
::

	OK

若设置失败，则提示以下信息：
::

	+BLECONNPARAM: <conn_index>,-1

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。
- <min_interval>：最小连接间隔。本参数值应小于等于 <max_interval> 参数值。参数范围：[0x0006,0x0C80]。连接间隔等于该参数乘以 1.25 毫秒，所以实际的最小连接间隔范围为 [7.5,4000] 毫秒。
- <max_interval>：最大连接间隔。本参数值应大于等于 <min_interval> 参数值。参数范围：[0x0006,0x0C80]。连接间隔等于该参数乘以 1.25 毫秒，所以实际的最大连接间隔范围为 [7.5,4000] 毫秒。
- <cur_interval>：当前连接间隔。
- <latency>：延迟。参数范围：[0x0000,0x01F3]。
- <timeout>：超时。参数范围：[0x000A,0x0C80]。超时等于该参数乘以 10 毫秒，所以实际的超时范围为 [100,32000] 毫秒。

**说明**

- 本命令要求先建立连接，并且仅支持 client 角色更新连接参数。

**示例**
::

	AT+BLEINIT=1		//角色：客户端
	AT+BLECONN=0,"24:0a:c4:09:34:23"
	AT+BLECONNPARAM=0,160,160,0,500


-----------------------------------------------------
AT+BLEDISCONN：断开 Bluetooth LE 连接
-----------------------------------------------------

**执行命令**

命令：
::

	AT+BLEDISCONN=<conn_index>

返回：
::

	OK						//收到 AT+BLEDISCONN 命令
	+BLEDISCONN:<conn_index>,<remote_address>	//运行命令成功

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。
- <remote_address>：对方 Bluetooth LE 设备地址。

**说明**

- 仅支持客户端运行本命令断开连接。

**示例**
::

	AT+BLEINIT=1		//角色：客户端
	AT+BLECONN=0,"24:0a:c4:09:34:23"
	AT+BLEDISCONN=0


-----------------------------------------------------
AT+BLEDATALEN：设置 Bluetooth LE 数据包长度
-----------------------------------------------------

**执行命令**

命令：
::

	AT+BLEDATALEN=<conn_index>,<pkt_data_len>

返回：
::

	OK

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。
- <pkt_data_len>：数据包长度，范围：[0x001B, 0x00FB]。

**说明**

- 需要先建立 Bluetooth LE 连接，才能设置数据包长度。

**示例**
::

	AT+BLEINIT=1		//角色：客户端
	AT+BLECONN=0,"24:0a:c4:09:34:23"
	AT+BLEDATALEN=0,30


-------------------------------------------------
AT+BLECFGMTU：设置 Bluetooth LE MTU 长度
-------------------------------------------------

**查询命令**

命令：
::

	AT+BLECFGMTU?

返回：
::

	+BLECFGMTU:<conn_index>,<mtu_size>
	OK

**执行命令**

命令：
::

	AT+BLECFGMTU=<conn_index>,<mtu_size>

返回：
::

	OK		//收到本命令

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。
- <mtu_size>：暂不支持设置。

**说明**

- 本命令要求先建立 Bluetooth LE 连接。
- 仅支持客户端运行本命令设置 MTU 的长度。
- MTU 的实际长度需要协商，响应 OK 只表示尝试协商 MTU 长度，因此设置长度不一定生效，建议调用 AT+BLECFGMTU? 查询实际 MTU 长度。

**示例**
::

	AT+BLEINIT=1		//角色：客户端
	AT+BLECONN=0,"24:0a:c4:09:34:23"
	AT+BLECFGMTU=0,64


------------------------------------------
AT+BLEGATTSSRVCRE：GATTS 创建服务
------------------------------------------

**执行命令**

命令：
::

	AT+BLEGATTSSRVCRE

返回：
::

	OK

**说明**

- 使用 bk7238作为 Bluetooth LE server 创建服务，需烧录 service bin 文件到 flash 中。如何生成 service bin 文件，请参考 tools/at_ble_service/readme.md。
- Bluetooth LE server 初始化后，请及时调用本命令创建服务；如果先建立 Bluetooth LE 连接，则无法创建服务。
- 如果 Bluetooth LE client 已初始化成功，可以使用此命令创建服务；也可以使用其他一些相应的 GATTS 命令，例如启动和停止服务、设置服务特征值和 notification/indication，具体命令如下：

	- AT+BLEGATTSSRVCRE (建议在 Bluetooth LE 连接建立之前使用)
	- AT+BLEGATTSSRVSTART (建议在 Bluetooth LE 连接建立之前使用)
	- AT+BLEGATTSSRV
	- AT+BLEGATTSCHAR
	- AT+BLEGATTSNTFY
	- AT+BLEGATTSIND
	- AT+BLEGATTSSETATTR

**示例**
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEGATTSSRVCRE


---------------------------------------------------
AT+BLEGATTSSRVSTART：GATTS 开启服务
---------------------------------------------------

**执行命令**

功能：

- GATTS 开启全部服务

命令：
::

	AT+BLEGATTSSRVSTART

响应：
::

	OK

**执行命令**

功能：

- GATTS 开启某指定服务

命令：
::

	AT+BLEGATTSSRVSTART=<srv_index>

响应：
::

	OK

**参数**

- <srv_index>：服务序号，从 1 开始递增。

**示例**

开启全部服务：
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART

开启2号服务指定服务：
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART=2


-------------------------------------------
AT+BLEGATTSSRVSTOP：GATTS 停止服务
-------------------------------------------

**说明**

- 暂不支持该命令。


----------------------------------------
AT+BLEGATTSSRV：GATTS 发现服务
----------------------------------------

**查询命令**

命令：
::

	AT+BLEGATTSSRV?

返回：
::

	+BLEGATTSSRV:<srv_index>,<start>,<srv_uuid>,<srv_type>
	OK

**参数**

- <srv_index>：服务序号，从 1 开始递增。
- <start>：

	- 0: 服务未开始；
	- 1: 服务已开始。

- <srv_uuid>：服务的 UUID。
- <srv_type>：服务的类型：

	- 0: 次要服务；
	- 1: 首要服务。

**示例**
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRV?


--------------------------------------------
AT+BLEGATTSCHAR：GATTS 发现服务特征
--------------------------------------------

**查询命令**

命令：
::

	AT+BLEGATTSCHAR?

返回：
对于服务特征信息，响应如下：
::

	+BLEGATTSCHAR:"char",<srv_index>,<char_index>,<char_uuid>,<char_prop>

对于描述符信息，响应如下：
::

	+BLEGATTSCHAR:"desc",<srv_index>,<char_index>,<desc_index>
	OK

**参数**

- <srv_index>：服务序号，从 1开始递增。
- <char_index>：服务特征的序号，从 1起始递增。
- <char_uuid>：服务特征的 UUID。
- <char_prop>：服务特征的属性。
- <desc_index>：特征描述符序号。
- <desc_uuid>：特征描述符的 UUID。

**示例**
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART
	AT+BLEGATTSCHAR?


------------------------------------------------------------
AT+BLEGATTSNTFY：服务器 notify服务特征值给客户端
------------------------------------------------------------

**执行命令**

功能：

- 服务器 notify 服务特征值给客户端

命令：
::

	AT+BLEGATTSNTFY=<conn_index>,<srv_index>,<char_index>,<length>

返回：
::

	>

符号 > 表示 AT 准备好接收串口数据，此时您可以输入数据，当数据长度达到参数 <length> 的值或者收到回车换行（”\r\n”）时，执行 notify 操作。
若数据传输成功，则提示：
::

	OK

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。（无效）
- <srv_index>：服务序号，可运行 AT+BLEGATTSCHAR? 查询。
- <char_index>：服务特征的序号，可运行 AT+BLEGATTSCHAR? 查询。
- <length>：数据长度。

**示例**
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART
	AT+BLEADVSTART		//开始广播，当client连接后，必须配置接收notify
	AT+BLEGATTSCHAR?	//查询允许notify客户端的特征（char_prop & 0x0010）
	//例如，使用1号服务的2号特征notify长度为4字节的数据，使用如下命令：
	AT+BLEGATTSNTFY=0,1,2,4
	//提示 ">" 符号后，输入4字节的数据，如"1234"，然后数据自动传输


---------------------------------------------------------------
AT+BLEGATTSIND：服务器 indicate 服务特征值给客户端
---------------------------------------------------------------

**执行命令**

命令：
::

	AT+BLEGATTSIND=<conn_index>,<srv_index>,<char_index>,<length>

返回：
::

	>

符号 > 表示 AT 准备好接收串口数据，此时您可以输入数据，当数据长度达到参数 <length> 的值或者收到回车换行（”\r\n”）时，执行 indicate 操作。
若数据传输成功，则提示：
::

	OK

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0, 2]。
- <srv_index>：服务序号，可运行 AT+BLEGATTSCHAR? 查询。
- <char_index>：服务特征的序号，可运行 AT+BLEGATTSCHAR? 查询。
- <length>：数据长度。

**示例**
::

	AT+BLEINIT=2		//角色：服务器
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART
	AT+BLEADVSTART		//开始广播，当client连接后，必须配置接收indication
	AT+BLEGATTSCHAR?	//查询客户端可以接收indication的特征（char_prop & 0x0020）
	//例如，使用2号服务的1号特征indicate长度为5字节的数据，命令如下：
	AT+BLEGATTSIND=0,2,1,5
	//提示 ">" 符号后，输入5字节的数据，如"54123"，然后数据自动传输


----------------------------------------------------
AT+BLEGATTSSETATTR：GATTS 设置服务特征值
----------------------------------------------------

**说明**

- 暂不支持该命令。


----------------------------------------------------
AT+BLEGATTCPRIMSRV：GATTC 发现基本服务
----------------------------------------------------

**查询命令**

命令：
::

	AT+BLEGATTCPRIMSRV=<conn_index>

返回：
::

	+BLEGATTCPRIMSRV:<conn_index>,<srv_index>,<srv_uuid>,<srv_type>
	OK

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。(自动获取，设置值不生效)
- <srv_index>：服务序号，从 1 开始递增。
- <srv_uuid>：服务的 UUID。
- <srv_type>：服务的类型：

	- 0: 次要服务；
	- 1: 首要服务。

**说明**

- 使用本命令，需要先建立 Bluetooth LE 连接。

**示例**
::

	AT+BLEINIT=1		//角色：客户端
	AT+BLECONN=0,"24:12:5f:9d:91:98"
	AT+BLEGATTCPRIMSRV=1


-----------------------------------------------
AT+BLEGATTCCHAR：GATTC 发现服务特征
-----------------------------------------------

**执行命令**

命令
::

	AT+BLEGATTCCHAR=<conn_index>,<srv_index>

返回：
对于服务特征信息，响应如下：
::

	+BLEGATTCCHAR:"char",<conn_index>,<srv_index>,<char_index>,<char_uuid>,<char_prop>

对于描述符信息，响应如下：
::

	+BLEGATTCCHAR:"desc",<conn_index>,<srv_index>,<char_index>,<desc_index>,<desc_uuid>
	OK

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。(自动获取，设置值不生效)
- <srv_index>：服务序号，可运行 AT+BLEGATTCPRIMSRV=<conn_index> 查询。
- <char_index>：服务特征的序号，从 0 开始递增。
- <char_uuid>：服务特征的 UUID。
- <char_prop>：服务特征的属性。
- <desc_index>：特征描述符序号。
- <desc_uuid>：特征描述符的 UUID。

**说明**

- 使用本命令，需要先建立 Bluetooth LE 连接。

**示例**
::

	AT+BLEINIT=1			//角色：客户端
	AT+BLECONN=0,"24:12:5f:9d:91:98"
	AT+BLEGATTCPRIMSRV=0
	AT+BLEGATTCCHAR=0,1		//根据前一条命令的查询结果，指定index查询


-----------------------------------------------
AT+BLEGATTCRD：GATTC 读取服务特征值
-----------------------------------------------

**执行命令**

命令：
::

	AT+BLEGATTCRD=<conn_index>,<srv_index>,<char_index>[,<desc_index>]

返回：
::

	+BLEGATTCRD:<conn_index>,<len>,<value>
	OK

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。
- <srv_index>：服务序号，可运行 AT+BLEGATTCPRIMSRV=<conn_index> 查询。
- <char_index>：服务特征序号，可运行 AT+BLEGATTCCHAR=<conn_index>, <srv_index> 查询。
- [<desc_index>]：特征描述符序号：

	- 若设置，读取目标描述符的值；
	- 若未设置，读取目标特征的值。

- <len>：数据长度。
- <value>：<char_value> 或者 <desc_value>。
- <char_value>：服务特征值，字符串格式，运行AT+BLEGATTCRD=<conn_index>, <srv_index>,<char_index>读取。例如，若响应为+BLEGATTCRD:0,1,0，则表示数据长度为1，内容为“0”。
- <desc_value>：服务特征描述符的值，字符串格式，运行 AT+BLEGATTCRD= <conn_index>,<srv_index>,<char_index>,<desc_index> 读取。例如，若响应为 +BLEGATTCRD:0,4,0123，则表示数据长度为 4，内容为 “0123”。

**说明**

- 使用本命令，需要先建立 Bluetooth LE 连接。
- 若目标服务特征不支持读操作，则返回 “ERROR”。

**示例**
::

	AT+BLEINIT=1		//角色：客户端
	AT+BLECONN=0,"24:12:5f:9d:91:98"
	AT+BLEGATTCPRIMSRV=0
	AT+BLEGATTCCHAR=0,3
	//根据前一条命令的查询结果，指定index查询（可读char_prop & 0x2）
	//例如，读取第2号服务的第3号特征，命令如下：
	AT+BLEGATTCRD=0,2,3


--------------------------------------------
AT+BLEGATTCWR：GATTC 写服务特征值
--------------------------------------------

**执行命令**

命令：
::

	AT+BLEGATTCWR=<conn_index>,<srv_index>,<char_index>[,<desc_index>],<length>

返回：
::

	>

符号 > 表示 AT 准备好接收串口数据，此时您可以输入数据，当数据长度达到参数 <length> 的值时，执行写入操作。
若数据传输成功，则提示：
::

	OK

**参数**

- <conn_index>：Bluetooth LE 连接号，范围：[0,2]。
- <srv_index>：服务序号，可运行 AT+BLEGATTCPRIMSRV=<conn_index> 查询。
- <char_index>：服务特征序号，可运行 AT+BLEGATTCCHAR=<conn_index>, <srv_index> 查询。
- [<desc_index>]：特征描述符序号：

	- 若设置，则写目标描述符的值；
	- 若未设置，则写目标特征的值。

- <length>：数据长度。

**说明**

- 使用本命令，需要先建立 Bluetooth LE 连接。
- 若目标服务特征不支持写操作，则返回 “ERROR”。

**示例**
::

	AT+BLEINIT=1		//角色：客户端
	AT+BLECONN=0,"24:12:5f:9d:91:98"
	AT+BLEGATTCPRIMSRV=0
	AT+BLEGATTCCHAR=0,3
	//根据前一条命令的查询结果，指定index查询（可写char_prop & 0x8）
	//例如，向第3号服务的第1号特征，写入长度为6字节的数据，命令如下：
	AT+BLEGATTCWR=0,3,1,6
	//提示 ">" 符号后，输入6字节的数据即可，如"123456"，然后开始写入