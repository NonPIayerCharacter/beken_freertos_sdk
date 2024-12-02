wifi指令集
=================


--------------------------------------------
AT+CWMODE：设置/查询wifi工作模式
--------------------------------------------

**查询命令**

命令：
::

	AT+CCWMODE?

返回：
::

	+CWMODE: <mode>,<auto_connect>
	OK

**执行命令**

命令：
::

	AT+CWMODE=<mode>[,<auto_connect>]

返回：
::

	OK

**参数**

- <mode>：模式

	- 0: 无 Wi-Fi 模式，并且关闭 Wi-Fi RF
	- 1: Station 模式
	- 2: SoftAP 模式
	- 3: SoftAP+Station 模式

- <auto_connect>：

	- 0: 禁用自动连接 AP 的功能
	- 1: 启用自动连接 AP 的功能，若之前已经将自动连接 AP 的配置保存到 flash 中，重启会自动连接


-----------------------------------------------
AT+CIPSTAMAC：设置/查询当前WIFI MAC地址
-----------------------------------------------

**查询命令**

命令：
::

	AT+CIPSTAMAC?

返回：
::

	+WLMAC:<mac>
	OK

**执行命令**

命令：
::

	AT+CIPSTAMAC=<"mac">

返回：
::

	OK

**参数**

- <"mac">:支持"MM:MM:MM:MM:MM:MM"和"MM-MM-MM-MM-MM-MM"两种格式


------------------------
AT+CWSAP：SoftAP配置参数
------------------------

**执行命令**

命令：
::

	AT+CWSAP=<"ssid">,<"pwd">,<channel>,<ecn>[,max conn>][,<hidden>]

返回：
::

	OK

**参数**

- <SSID>：字符串参数，接入点名称
- <PWD>：字符串参数，密码，范围：8 ~ 64 字节 ASCII
- <CHANNELl>：信道号
- <proto>:

	- 0: 802.11bgn 协议标准
	- 1: 802.11bg 协议标准
	- 2: 802.11b 协议标准

- <ECN>：加密方式，不支持 WEP

	- 0 OPEN
	- 1  WEP
	- 2  WPA_TKIP
	- 3 WPA_AES
	- 4 WPA2_TKIP
	- 5 WPA2_AES
	- 6  AES_TKIP_WPA2_MIXED
	- 7 WPA3_SAE
	- 8  WPA3_WPA2    (默认)
	- 9 EAP
	- 10 OWE
	- 11 AUTO 

- [<max conn>]：允许连入 SoftAP 的最多 station 数目，取值范围：[1,3]
- [<ssid hidden>]：

	- 0: 广播 SSID（默认）
	- 1: 不广播 SSID

**说明**

- SoftAP 与WLMODE配置相关，只有当WLMODE中的mode参数配置为2，3时SoftAP才开启，默认SSID为bk7238。


----------------------------
AT+CWQIF：停止SoftAP
----------------------------

**执行命令**

命令：
::

	AT+CWQIF

返回：
::

	OK


----------------------------------------------------
AT+CIPAP：配置SoftAP静态IP及网关
----------------------------------------------------

**查询命令**

命令：
::

	AT+CIPAP?

返回：
::

	+CIPAP: <”ip”>,<”netmask”>,<”gateway”>
	OK

**执行命令**

命令：
::

	AT+CIPAP=<”ip”>,<”netmask”>,<”gateway”>

返回：
::

	OK

**参数**

- <”ip”>：字符串参数，表示IPv4地址
- <”gateway”>：网关
- <”netmask”>：子网掩码


----------------------------------------------------
AT+ CWLIF：查询连接到softap 上设备信息
----------------------------------------------------

**查询命令**

命令：
::

	AT+CWLIF

返回：
::

	+CWLIF：<index>,<ip>,<mac>,<rssi>
	OK

**参数**

- <index>：连接SoftAP序号
- <ip>：网关
- <mac>：子网掩码
- <rssi>：dns地址


----------------------------------------------------
AT+WSCAN：扫描可用的AP
----------------------------------------------------

**执行命令**

命令：
::

	AT+WSCAN

返回：
::

	-CMDRSP: +WSCAN <ssid>,<bssid>,<channel>,<enc>,<rssi>

**参数**

- <ssid>：字符串参数，AP的SSID
- <mac>：字符串参数，AP的MAC地址
- <channel>：子网掩码
- <ecn>：加密方式

	- 0 OPEN
	- 1 WEP
	- 2 WPA_TKIP
	- 3 WPA_AES
	- 4 WPA2_TKIP
	- 5 WPA2_AES
	- 6 AES_TKIP_WPA2_MIXED
	- 7 WPA3_SAE
	- 8 WPA3_WPA2(默认)
	- 9 EAP
	- 10 OWE
	- 11 AUTO

- <rssi>：信号强度


--------------------------------------------------------------------
AT+CWDHCP：启用/禁用 DHCP
--------------------------------------------------------------------

**查询命令**

命令：
::

	AT+CWDHCP?

返回：
::

	<state>

**执行命令**

命令：
::

	AT+CWDHCP=<operate>,<mode>

返回：
::

	OK

**参数**

- <operate>：

	- 0: 禁用
	- 1: 启用

- <mode>：

	- Bit0: Station 的 DHCP
	- Bit1: SoftAP 的 DHCP

- <state>：DHCP 的状态

	- Bit0:

		- 0: 禁用 Station 的 DHCP
		- 1: 启用 Station 的 DHCP

	- Bit1:

		- 0: 禁用 SoftAP 的 DHCP
		- 1: 启用 SoftAP 的 DHCP

**说明**

- 若AT+SYSSTORE=1，配置更改将保存到NVS分区

**示例**

启用：
::

	AT+CWDHCP=1,1		//启用sta的DHCP
	AT+CWDHCP=1,2		//启用ap的DHCP
	AT+CWDHCP=1,3		//启用sta+ap的DHCP

禁用：
::

	AT+CWDHCP=0,1		//禁用sta的DHCP
	AT+CWDHCP=0,2		//禁用ap的DHCP
	AT+CWDHCP=0,3		//禁用sta+ap的DHCP


--------------------------------------------------------------------
AT+CWJAP：Station连接AP
--------------------------------------------------------------------

**执行命令**

命令：
::

	AT+CWJAP=[<ssid>],[<pwd>][,<bssid>][,<pci_en>][,<reconn_interval>][,<listen_interval>][,<scan_mode>][,<jap_timeout>][,<pmf>]

返回：
::

	OK
	WIFI CONNECTED
	WIFI GOT IP

**说明**

- 若AT+SYSSTORE=1，配置更改将保存到NVS分区。


--------------------------------------------------------------------
AT+CWQAP：断开与 AP 的连接
--------------------------------------------------------------------

**执行命令**

命令：
::

	AT+CWQAP

返回：
::

	OK


--------------------------------------------------------------------
AT+CIPSTA：查询/设置 Station 的 IP 信息
--------------------------------------------------------------------

**查询命令**

命令：
::

	AT+CIPSTA?

返回：
::

	+CIPSTA:<ip>,<netmask>,<gateway>

**执行命令**

命令：
::

	AT+STASTATIC=<ip>,<netmask>,<gateway>

返回：
::

	OK

**参数**

- <ip>：字符串参数，表示 IPv4 地址
- <gateway>：网关
- <netmask>：子网掩码

