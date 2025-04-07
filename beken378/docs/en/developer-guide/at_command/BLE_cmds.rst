
:link_to_translation:`zh_CN:[中文]`

Bluetooth LE instruction set
==============================


--------------------------------------------
AT+BLEINIT: Bluetooth LE initialization
--------------------------------------------

**Query command**

Command:
::

	AT+BLEINIT?

If initialized, AT returns:
::

	+BLEINIT:<role>
	OK

If not initialized, AT returns:
::

	+BLEINIT:0
	OK

**Execution command**

Command:
::

	AT+BLEINIT=<init>

Return:
::

	OK

**Parameter**

- <init>:

	- 0: Log out Bluetooth LE
	- 1: client role
	- 2: server role

**Note**

- Before using other Bluetooth LE commands, please call this command to initialize the Bluetooth LE role.
- After the Bluetooth LE role is initialized, the role cannot be switched directly. If you need to switch roles, you need to call the AT+RST command to restart the system and then reinitialize the Bluetooth LE role.
- If you use BK7238 as Bluetooth LE server, you need to burn the service bin to Flash 0x1F8000. For how to generate the service bin file, please refer to tools/at_ble_service/readme.md.
- It is recommended to stop ongoing broadcasts, scans and disconnect all connections before logging out of Bluetooth LE.

**Example**
::

	AT+BLEINIT=1


-----------------------------------------------
AT+BLENAME: Betooth LE device name
-----------------------------------------------

**Query command**

Command:
::

	AT+BLENAME?

Return:
::

	+BLENAME:<device_name>
	OK

**Execution command**

Command:
::

	AT+BLENAME=<device_name>

Return:
::

	OK

**Parameter**

- <device_name>: Bluetooth LE device name, maximum length: 18.

**Example**
::

	AT+BLENAME="ble_demo"


---------------------------------------------------------
AT+BLESCANPARAM: query/set Bluetooth LE scan parameters
---------------------------------------------------------

**Query command**

Command:
::

	AT+BLESCANPARAM?

Return:
::

	+BLESCANPARAM:<scan_type>,<own_addr_type>,<filter_policy>,<scan_interval>,<scan_window>
	OK

**Execution command**

Command:
::

	AT+BLESCANPARAM=<scan_type>,<own_addr_type>,<filter_policy>,<scan_interval>,<scan_window>

Return:
::

	OK

**Parameters**

- <scan_type>: scanning type

	- 0: passive scanning
	- 1: active scanning

- <own_addr_type>: address type

	- 0: public address
	- 1: random address
	- 2: RPA public address
	- 3: RPA random address

- <filter_policy>: scan filtering method, setting is not supported yet.
- <scan_interval>: scan interval. The value of this parameter should be greater than or equal to the value of the parameter <scan_window> . Parameter range: [0x0004,0x4000]. The scan interval is this parameter multiplied by 0.625 milliseconds, so the actual scan interval range is [2.5,10240] milliseconds.
- <scan_window>: scan window. The value of this parameter should be less than or equal to the value of the parameter <scan_interval>. Parameter range: [0x0004,0x4000]. The scan window is this parameter multiplied by 0.625 milliseconds, so the actual scan window range is [2.5,10240] milliseconds.


**Example**
::

	AT+BLEINIT=1		//Role: client
	AT+BLESCANPARAM=0,0,0,100,50


------------------------------------------------
AT+BLESCAN: enable Bluetooth LE scanning
------------------------------------------------

**Execution command**

Command:
::

	AT+BLESCAN=<enable>[,<interval>][,<filter_type>,<filter_param>]

Return:
::

	+BLESCAN:<addr>,<rssi>,<adv_data>,<scan_rsp_data>,<addr_type>
	OK

**Parameters**

- <enable>：

	- 1: Start continuous scanning
	- 0: Stop continuous scanning

- [<interval>]: scanning interval, unit: seconds, not supported yet.
- [<filter_type>]: filtering method

	- 1: "MAC"
	- 2: "NAME"

- [<filter_param>]: filter parameter, indicating the MAC address or name of the other device.
- <addr>: Bluetooth LE address
- <rssi>: RSSI
- <adv_data>: advertising data
- <scan_rsp_data>: scan response data
- <addr_type>: advertising device address type

**Notes**

- Currently <filter_type> and <filter_param> are not supported yet.
- OK and +BLESCAN:<addr>,<rssi>,<adv_data>,<scan_rsp_data>,<addr_type> in the response have no strict order of output.

**Examples**
::

	AT+BLEINIT=1		//Role: client
	AT+BLESCAN=1		//Start scanning
	AT+BLESCAN=0		//Stop scanning
	AT+BLESCAN=1,3,1,"24:0A:C4:96:E6:88"	//Start scanning, filtering by MAC address
	AT+BLESCAN=1,3,2,"BK-AT"		//Start scanning, filtering by device name


----------------------------------------------------
AT+BLEADDR: set Bluetooth LE device address
----------------------------------------------------

**Query command**

Command:
::

	AT+BLEADDR?

Return:
::

	+BLEADDR:<BLE_public_addr>
	OK

**Note**

- Modifying the address type settings of Bluetooth LE devices is not supported. The BLE address is related to the Wi-Fi address. If you need to change it, please set the Wi-Fi address.


----------------------------------------------------
AT+BLESCANRSPDATA: set Bluetooth LE scan response
----------------------------------------------------

**Execution command**

Command:
::

	AT+BLESCANRSPDATA=<scan_rsp_data>

Return:
::

	OK

**Parameter**

- <scan_rsp_data>: Scan response data, which is a HEX string. For example, if you want to set the scan response data to "0x11 0x22 0x33 0x44 0x55", the command is AT+BLESCANRSPDATA="1122334455".

**Examples**
::

	AT+BLEINIT=2				//Role: server
	AT+BLESCANRSPDATA=050837323338		//short_name 7238


----------------------------------------------------------------
AT+BLEADVPARAM: query/set Bluetooth LE advertising parameters
----------------------------------------------------------------

**Query command**

Command:
::

	AT+BLEADVPARAM?

Return:
::

	+BLEADVPARAM:<adv_int_min>,<adv_int_max>,<adv_type>,<channel_map>
	OK

**Execution command**

Command:
::

	AT+BLEADVPARAM=<adv_int_min>,<adv_int_max>,<adv_type>[,<own_addr_type>],<channel_map>[,<adv_filter_policy>][,<peer_addr_type>,<peer_addr>]

Return:
::

	OK

**Parameters**

- <adv_int_min>: Minimum advertising interval. Parameter range: [0x0020,0x4000]. The advertising interval is equal to this parameter multiplied by 0.625 milliseconds, so the actual minimum advertising interval range is [20,10240] milliseconds. The value of this parameter should be less than or equal to the value of the parameter <adv_int_max>.
- <adv_int_max>: Maximum advertising interval. Parameter range: [0x0020,0x4000]. The advertising interval is equal to this parameter multiplied by 0.625 milliseconds, so the actual maximum advertising interval range is [20,10240] milliseconds. The value of this parameter should be greater than or equal to the value of the parameter <adv_int_min>.
- <adv_type>:

	- 0: ADV_TYPE_IND
	- 1: ADV_TYPE_DIRECT_IND_HIGH
	- 2: ADV_TYPE_SCAN_IND
	- 3: ADV_TYPE_NONCONN_IND
	- 4: ADV_TYPE_DIRECT_IND_LOW

- [<own_addr_type>]: not supported yet
- <channel_map>: advertising channel

	- 1: ADV_CHNL_37
	- 2: ADV_CHNL_38
	- 4: ADV_CHNL_39
	- 7: ADV_CHNL_ALL

- [<adv_filter_policy>]: Setting is not supported yet.
- [<peer_addr_type>]: Setting is not supported yet.
- [<peer_addr>]: Setting is not supported yet.
- [<primary_phy>]: Setting is not supported yet.
- [<secondary_phy>]: Setting is not supported yet.

**Example**
::

	AT+BLEINIT=2		//Role: server
	AT+BLEADVPARAM=160,160,0,7


--------------------------------------------------------------------
AT+BLEADVDATA: set Bluetooth LE advertising data
--------------------------------------------------------------------

**Execution command**

Command:
::

	AT+BLEADVDATA=<adv_data>

Return:
::

	OK

**Parameter**

- <adv_data>: Advertising data, which is a HEX string. For example, if you want to set the advertising data "Local Name" to "7238_BLE", the command is AT+BLEADVDATA="0909373233385F424C45".

**Note**

- If the advertising data has been set using the command AT+BLEADVDATAEX=<dev_name>,<uuid>,<manufacturer_data>,<include_power> before, it will be overwritten by the advertising data set by this command.
- If you want to use this command to modify the device name, it is recommended to execute the AT+BLENAME command after executing this command to set the device name to the same name.


-----------------------------------------------------------------
AT+BLEADVDATAEX: automatically set Bluetooth LE advertising data
-----------------------------------------------------------------

**Query command**

Command:
::

	AT+BLEADVDATAEX?

Return:
::

	+BLEADVDATAEX:<dev_name>,<uuid>,<manufacturer_data>,<include_power>
	OK

**Execution command**

Command:
::

	AT+BLEADVDATAEX=<dev_name>,<uuid>,<manufacturer_data>,<include_power>

Return:
::

	OK

**Parameters**

- <dev_name>: string parameter, indicating the device name. For example, if you want to set the device name to  "just-test" , the command is AT+BLEADVSTARTEX="just-test",<uuid>,<manufacturer_data>,<include_power>.
- <uuid>: string parameter. For example, if you want to set the UUID to "0xA002", the command is AT+BLEADVSTARTEX=<dev_name>,"A002",<manufacturer_data>,<include_power>.
- <manufacturer_data>: manufacturer data, which is a HEX string. For example, if you want to set the manufacturer data to "0x11 0x22 0x33 0x44 0x55", the command is AT+BLEADVSTARTEX=<dev_name>,<uuid>,"1122334455",<include_power>.
- <include_power>: Setting is not supported yet.

**Note**

- If the advertising data has been set using the command AT+BLEADVDATA=<adv_data> before, it will be overwritten by the advertising data set by this command.

**Example**
::

	AT+BLEINIT=2		//Role: server
	AT+BLEADVDATAEX="AT_DEMO","A002","0102030405",1


-------------------------------------------------
AT+BLEADVSTART: start Bluetooth LE advertising
-------------------------------------------------

**Execution command**

Command:
::

	AT+BLEADVSTART

Return:
::

	OK

**Notes**

- If the advertising parameters are not set using the command AT+BLEADVPARAM=<adv_parameter>, the default advertising parameters are used.
- If the advertising data is not set using the command AT+BLEADVDATA=<adv_data>, all-zero data packets are sent. If the advertising data has been set using the command AT+BLEADVDATA=<adv_data> before, it will be overwritten by the advertising data set by AT+BLEADVDATAEX=<dev_name>,<uuid>,<manufacturer_data>,<include_power>. On the contrary, if the advertising data has been set using the command AT+BLEADVDATAEX before, it will be overwritten by the advertising data set by AT+BLEADVDATA.
- After enabling Bluetooth LE advertising, if a Bluetooth LE connection is not established, the advertising will continue; if a connection is established, the advertising will automatically end.

**Example**
::

	AT+BLEINIT=2		//Role: server
	AT+BLEADVSTART


---------------------------------------------------
AT+BLEADVSTOP: stop Bluetooth LE advertising
---------------------------------------------------

**Execution command**

Command:
::

	AT+BLEADVSTOP

Return:
::

	OK

**Note**

- If a Bluetooth LE connection is successfully established after starting advertising, the Bluetooth LE advertising will automatically end without calling this command.

**Example**
::

	AT+BLEINIT=2		//Role: server
	AT+BLEADVSTART
	AT+BLEADVSTOP


-----------------------------------------------
AT+BLECONN: establish Bluetooth LE connection
-----------------------------------------------

**Query command**

Command:
::

	AT+BLECONN?

Return:
::

	+BLECONN:<conn_index>,<remote_address>
	OK

If the connection is not established, the response does not display the parameters <conn_index> and <remote_address>.

**Execution command**

Command:
::

	AT+BLECONN=<conn_index>,<remote_address>[,<addr_type>,<timeout>]

Return:
If the connection is established successfully, it will prompt:
::

	+BLECONN:<conn_index>,<remote_address>
	OK

If the connection fails to be established, it will prompt:
::

	+BLECONN:<conn_index>,-1
	ERROR

If the connection fails due to parameter errors or other reasons, it will prompt:
::

	ERROR

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2].
- <remote_address>: the other party’s Bluetooth LE device address.
- [<addr_type>]: advertising device address type. The default value is 0.

	- 0: public address
	- 1: aandom address

- [<timeout>]: Setting is not supported yet. The default value is 5 s.

**Notes**

- It is recommended to run the AT+BLESCAN command to scan the device before establishing a new connection to ensure that the target device is in the broadcast state.
- If the Bluetooth LE server has been initialized and the connection has been established successfully, you can use this command to discover services among peer devices (GATTC).

**Example**

.. note::

	For testing, you need to first use another board to start the server to enable advertising.

::

	AT+BLEINIT=1		//Role: client
	AT+BLECONN=0,"4988428C47C8",0,10


-----------------------------------------------------------------
AT+BLECONNPARAM: query/update Bluetooth LE connection parameters
-----------------------------------------------------------------

**Query command**

Command:
::

	AT+BLECONNPARAM?

Return:
::

	+BLECONNPARAM:<conn_index>,<min_interval>,<max_interval>,<cur_interval>,<latency>,<timeout>
	OK

**Execution command**

Command:
::

	AT+BLECONNPARAM=<conn_index>,<min_interval>,<max_interval>,<latency>,<timeout>

Return:
::

	OK

If the setting fails, it will prompt:
::

	+BLECONNPARAM: <conn_index>,-1

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2].
- <min_interval>: minimum connection interval. The value of this parameter should be less than or equal to the value of the parameter <max_interval>. Parameter range: [0x0006,0x0C80]. The connection interval is equal to this parameter multiplied by 1.25 milliseconds, so the actual minimum connection interval range is [7.5,4000] milliseconds.
- <max_interval>: maximum connection interval. The value of this parameter should be greater than or equal to the value of the parameter <min_interval>. Parameter range: [0x0006,0x0C80]. The connection interval is equal to this parameter multiplied by 1.25 milliseconds, so the actual maximum connection interval range is [7.5,4000] milliseconds.
- <cur_interval>: current connection interval.
- <latency>: delay. Parameter range: [0x0000,0x01F3].
- <timeout>: timeout. Parameter range: [0x000A,0x0C80]. The timeout is equal to this parameter multiplied by 10 milliseconds, so the actual timeout range is [100,32000] milliseconds.

**Note**

- This command requires establishing a connection first, and only supports updating connection parameters for the client role.

**Example**
::

	AT+BLEINIT=1		//Role: client
	AT+BLECONN=0,"24:0a:c4:09:34:23"
	AT+BLECONNPARAM=0,160,160,0,500


-----------------------------------------------------
AT+BLEDISCONN: disconnect Bluetooth LE connection
-----------------------------------------------------

**Execution command**

Command:
::

	AT+BLEDISCONN=<conn_index>

Return:
::

	OK						//Command received
	+BLEDISCONN:<conn_index>,<remote_address>	//Run command successfully

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2].
- <remote_address>: The other party’s Bluetooth LE device address.

**Note**

- Only the client is supported to run this command to disconnect.

**Example**
::

	AT+BLEINIT=1		//Role: client
	AT+BLECONN=0,"24:0a:c4:09:34:23"
	AT+BLEDISCONN=0


-----------------------------------------------------
AT+BLEDATALEN: set Bluetooth LE data packet length
-----------------------------------------------------

**Execution command**

Command:
::

	AT+BLEDATALEN=<conn_index>,<pkt_data_len>

Return:
::

	OK

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2].
- <pkt_data_len>: data packet length, range: [0x001B, 0x00FB].

**Note**

-  A Bluetooth LE connection needs to be established before the packet length can be set.

**Example**
::

	AT+BLEINIT=1		//Role: client
	AT+BLECONN=0,"24:0a:c4:09:34:23"
	AT+BLEDATALEN=0,30


-------------------------------------------------
AT+BLECFGMTU: set Bluetooth LE MTU length
-------------------------------------------------

**Query command**

Command:
::

	AT+BLECFGMTU?

Return:
::

	+BLECFGMTU:<conn_index>,<mtu_size>
	OK

**Execution command**

Command:
::

	AT+BLECFGMTU=<conn_index>,<mtu_size>

Return:
::

	OK		//Command received

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2].
- <mtu_size>: Setting is not supported yet.

**Notes**

- This command requires a Bluetooth LE connection to be established first.
- Only the client can run this command to set the MTU length.
- The actual length of the MTU needs to be negotiated. The OK response only indicates an attempt to negotiate the MTU length, so the set length may not take effect. It is recommended to call AT+BLECFGMTU? to query the actual MTU length.

**Example**
::

	AT+BLEINIT=1		//Role: client
	AT+BLECONN=0,"24:0a:c4:09:34:23"
	AT+BLECFGMTU=0,64


----------------------------------------------
AT+BLEGATTSSRVCRE: (GATTS) creat GATT service
----------------------------------------------

**Execution command**

Command:
::

	AT+BLEGATTSSRVCRE

Return:
::

	OK

**Notes**

- To use BK7238 as Bluetooth LE server to create a service, you need to burn the service bin file into Flash. How to generate service bin file, please refer to tools/at_ble_service/readme.md.
- After the Bluetooth LE server is initialized, please call this command promptly to create the service. If a Bluetooth LE connection is established first, the service cannot be created.
- If the Bluetooth LE client has been initialized successfully, you can use this command to create a service; you can also use other corresponding GATTS commands, such as starting and stopping services, setting service characteristic values and notification/indication. The specific commands are as follows:

	- AT+BLEGATTSSRVCRE (It is recommended to use this command before the Bluetooth LE connection is established)
	- AT+BLEGATTSSRVSTART (It is recommended to use this command before the Bluetooth LE connection is established)
	- AT+BLEGATTSSRV
	- AT+BLEGATTSCHAR
	- AT+BLEGATTSNTFY
	- AT+BLEGATTSIND
	- AT+BLEGATTSSETATTR

**Example**
::

	AT+BLEINIT=2		//Role: server
	AT+BLEGATTSSRVCRE


---------------------------------------------------
AT+BLEGATTSSRVSTART：(GATTS) start GATT service
---------------------------------------------------

**Execution command**

Function:

-  GATTS (GATT server) enables all services.

Command:
::

	AT+BLEGATTSSRVSTART

Return:
::

	OK

**Execution command**

Function:

- GATTS (GATT server) enables a specified service.

Command:
::

	AT+BLEGATTSSRVSTART=<srv_index>

Return:
::

	OK

**Parameter**

- <srv_index>: service index, increasing from 1

**Examples**

Enable all services:
::

	AT+BLEINIT=2		//Role: server
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART

Enable service No. 2:
::

	AT+BLEINIT=2		//Role: server
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART=2


-----------------------------------------------
AT+BLEGATTSSRVSTOP: (GATTS) stop GATT service
-----------------------------------------------

**Note**

- This command is not supported yet.


----------------------------------------------
AT+BLEGATTSSRV: (GATTS) discover GATT service 
----------------------------------------------

**Query command**

Command:
::

	AT+BLEGATTSSRV?

Return:
::

	+BLEGATTSSRV:<srv_index>,<start>,<srv_uuid>,<srv_type>
	OK

**Parameters**

- <srv_index>: service index, increasing from 1
- <start>：

	- 0: The service has not started.
	- 1: The service has started.

- <srv_uuid>: service UUID
- <srv_type>: service type

	- 0: secondary service
	- 1: primary service

**Example**
::

	AT+BLEINIT=2		//Role: server
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRV?


-------------------------------------------------------
AT+BLEGATTSCHAR: (GATTS) discover GATT characteristic 
-------------------------------------------------------

**Query command**

Command:
::

	AT+BLEGATTSCHAR?

Return:
For characteristic information, the response is as follows:
::

	+BLEGATTSCHAR:"char",<srv_index>,<char_index>,<char_uuid>,<char_prop>

For descriptor information, the response is as follows:
::

	+BLEGATTSCHAR:"desc",<srv_index>,<char_index>,<desc_index>
	OK

**Parameters**

- <srv_index>: service index, increasing from 1
- <char_index>: characteristic index, increasing from 1
- <char_uuid>: characteristic UUID
- <char_prop>: characteristic property
- <desc_index>: characteristic descriptor index
- <desc_uuid>: characteristic descriptor UUID

**Example**
::

	AT+BLEINIT=2		//Role: server
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART
	AT+BLEGATTSCHAR?


------------------------------------------------------------
AT+BLEGATTSNTFY: (Server) notify characteristic
------------------------------------------------------------

**Execution command**

Function:

- The server notifies the characteristic value to the client.

Command:
::

	AT+BLEGATTSNTFY=<conn_index>,<srv_index>,<char_index>,<length>

Return:
::

	>

The symbol > indicates that AT is ready to receive serial port data. At this time, you can input data. When the data length reaches the value of the parameter <length> or a carriage return or line feed ("\r\n") is received, perform the notify operation. If the data transfer is successful, it will prompt:
::

	OK

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2]. (invalid)
- <srv_index>: service index, which can be queried by running AT+BLEGATTSCHAR?.
- <char_index>: characteristic index, which can be queried by running AT+BLEGATTSCHAR?.
- <length>: data length.

**Examples**
::

	AT+BLEINIT=2		//Role: server
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART
	AT+BLEADVSTART		//Start advertising. When the client is connected, it must be configured to receive notifications.
	AT+BLEGATTSCHAR?	//Query the characteristics subscribed by the client for notifications (char_prop & 0x0010).
	//For example, to use characteristic No. 2 of service No. 1 to notify data with a length of 4 bytes, use the following command:
	AT+BLEGATTSNTFY=0,1,2,4
	//Enter 4-byte data after the symbol >, such as "1234", and then the data will be automatically transmitted.


---------------------------------------------------------------
AT+BLEGATTSIND: (Server) indicate characteristic
---------------------------------------------------------------

**Execution command**

Function:

- The server indicates the characteristic value to the client.

Command:
::

	AT+BLEGATTSIND=<conn_index>,<srv_index>,<char_index>,<length>

Return:
::

	>

The symbol > indicates that AT is ready to receive serial port data. At this time, you can input data. When the data length reaches the value of the parameter <length> or a carriage return or line feed ("\r\n") is received, perform the indicate operation. If the data transfer is successful, it will prompt:
::

	OK

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0, 2].
- <srv_index>: service index, which can be queried by running AT+BLEGATTSCHAR?.
- <char_index>: characteristic index, which can be queried by running AT+BLEGATTSCHAR?.
- <length>: data length.

**Examples**
::

	AT+BLEINIT=2		//Role: server
	AT+BLEGATTSSRVCRE
	AT+BLEGATTSSRVSTART
	AT+BLEADVSTART		//Start advertising. When the client is connected, it must be configured to receive indications.
	AT+BLEGATTSCHAR?	//Query the characteristics subscribed by the client for indications (char_prop & 0x0020).
	//For example, to use characteristic No. 1 of service No. 2 to indicate data with a length of 5 bytes, the command is as follows:
	AT+BLEGATTSIND=0,2,1,5
	//enter 5 bytes of data after the symbol >, such as "54123", and then the data will be automatically transmitted.


----------------------------------------------------
AT+BLEGATTSSETATTR: (GATTS) set characteristic value 
----------------------------------------------------

**Note**

- This command is not supported yet.


----------------------------------------------------
AT+BLEGATTCPRIMSRV: (GATTC) discover primary service
----------------------------------------------------

**Query command**

Command:
::

	AT+BLEGATTCPRIMSRV=<conn_index>

Return:
::

	+BLEGATTCPRIMSRV:<conn_index>,<srv_index>,<srv_uuid>,<srv_type>
	OK

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2]. (The index is obtained automatically, and the setting value does not take effect.)
- <srv_index>: service index, starting from 1 and increasing.
- <srv_uuid>: service UUID
- <srv_type>: service type:

	- 0: secondary service
	- 1: primary service.

**Note**

- To use this command, you need to establish a Bluetooth LE connection first.

**Example**
::

	AT+BLEINIT=1		//Role: client
	AT+BLECONN=0,"24:12:5f:9d:91:98"
	AT+BLEGATTCPRIMSRV=1


-------------------------------------------------
AT+BLEGATTCCHAR: (GATTC) discover characteristic 
-------------------------------------------------

**Execution command**

Command:
::

	AT+BLEGATTCCHAR=<conn_index>,<srv_index>

Return:
For characteristic information, the response is as follows:
::

	+BLEGATTCCHAR:"char",<conn_index>,<srv_index>,<char_index>,<char_uuid>,<char_prop>

For descriptor information, the response is as follows:
::

	+BLEGATTCCHAR:"desc",<conn_index>,<srv_index>,<char_index>,<desc_index>,<desc_uuid>
	OK

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2]. (The index is obtained automatically, the setting value does not take effect.)
- <srv_index>: service index, which can be queried by running AT+BLEGATTCPRIMSRV=<conn_index>.
- <char_index>: characteristic index, increasing from 0.
- <char_uuid>: characteristic UUID.
- <char_prop>: characteristic property.
- <desc_index>: characteristic descriptor index.
- <desc_uuid>: characteristic descriptor UUID.

**Note**

- To use this command, you need to establish a Bluetooth LE connection first.

**Example**
::

	AT+BLEINIT=1			//Role: client
	AT+BLECONN=0,"24:12:5f:9d:91:98"
	AT+BLEGATTCPRIMSRV=0
	AT+BLEGATTCCHAR=0,1		//Query the specified index based on the query result of the previous command.


-------------------------------------------------
AT+BLEGATTCRD: (GATTC) read characteristic value
-------------------------------------------------

**Execution command**

Command:
::

	AT+BLEGATTCRD=<conn_index>,<srv_index>,<char_index>[,<desc_index>]

Return:
::

	+BLEGATTCRD:<conn_index>,<len>,<value>
	OK

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2].
- <srv_index>: service index, which can be queried by running AT+BLEGATTCPRIMSRV=<conn_index>.
- <char_index>: characteristic index, which can be queried by running AT+BLEGATTCCHAR=<conn_index>,<srv_index>.
- [<desc_index>]: characteristic descriptor index:

	- If set, read the value of the target descriptor
	- If not set, read the value of the target characteristic.

- <len>: data length.
- <value>: characteristic value  or characteristic descriptor value.
- <char_value>: characteristic value, string, which can be read by running AT+BLEGATTCRD=<conn_index>,<srv_index>,<char_index>. For example, if the response is +BLEGATTCRD:0,1,0, it means that the data length is 1 and the content is "0".
- <desc_value>: characteristic descriptor value, string, which can be read by running AT+BLEGATTCRD=<conn_index>,<srv_index>,<char_index>,<desc_index>. For example, if the response is +BLEGATTCRD:0,4,0123, it means that the data length is 4 and the content is "0123".

**Notes**

- To use this command, you need to establish a Bluetooth LE connection first.
- If the target characteristic does not support read operations, "ERROR" will be returned.

**Example**
::

	AT+BLEINIT=1		//Role: client
	AT+BLECONN=0,"24:12:5f:9d:91:98"
	AT+BLEGATTCPRIMSRV=0
	AT+BLEGATTCCHAR=0,3
	//Based on the query result of the previous command, query the specified index (char_prop & 0x2 can be written).
	//For example, to read characteristic No. 3 of service No. 2, the command is as follows:
	AT+BLEGATTCRD=0,2,3
	AT+BLEGATTCRD=0,2,3


---------------------------------------------------
AT+BLEGATTCWR: (GATTC) write characteristic value 
---------------------------------------------------

**Execution command**

Command:
::

	AT+BLEGATTCWR=<conn_index>,<srv_index>,<char_index>[,<desc_index>],<length>

Return:
::

	>

The symbol > indicates that AT is ready to receive serial port data. At this time, you can input data. When the data length reaches the value of the parameter <length>, perform the write operation . If the data transfer is successful, it will prompt:
::

	OK

**Parameters**

- <conn_index>: Bluetooth LE connection index, range: [0,2].
- <srv_index>: service index, which can be queried by running AT+BLEGATTCPRIMSRV=<conn_index>.
- <char_index>: characteristic index, which can be queried by running AT+BLEGATTCCHAR=<conn_index> ,<srv_index> Inquire.
- [<desc_index>]: characteristic descriptor index:

	- If set, write the value of the target descriptor.
	- If not set, write the value of the target characteristic.

- <length>: data length.


**Notes**

- To use this command, you need to establish a Bluetooth LE connection first.
- If the target service characteristic does not support write operations, "ERROR" will be returned.

**Example**
::

	AT+BLEINIT=1		//Role: client
	AT+BLECONN=0,"24:12:5f:9d:91:98"
	AT+BLEGATTCPRIMSRV=0
	AT+BLEGATTCCHAR=0,3
	//Based on the query result of the previous command, query the specified index  (char_prop & 0x8 can be written).
	//For example, to write 6-byte data to characteristic No. 1 of service No. 3, the command is as follows:
	AT+BLEGATTCWR=0,3,1,6
	//After the prompt ">" symbol, enter 6-byte data, such as "123456", and then write begins.