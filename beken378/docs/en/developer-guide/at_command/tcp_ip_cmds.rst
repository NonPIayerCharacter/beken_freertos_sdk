
:link_to_translation:`zh_CN:[中文]`

TCP/IP instruction set
=========================


-------------------------------------------------------------------
AT+ CIPSTATUS: query TCP/UDP/SSL connection status and information
-------------------------------------------------------------------

**Query command**

Command:
::

	AT+CIPSTATUS?

Return:
::

	+BIPSTATUS:<type>,<remoteip>,<port>,<localport>,<tetype>

**Parameters**

- <type>: connection type

	- 0: TCP
	- 1: UDP
	- 2: TLS
	- 3: DTLS

- <remoteip>: remote IP
- <port>: remote port
- <localport>: local port
- <tetype>：

	- 0: device as client
	- 1: device as server


-----------------------------------------------------------------------
AT+CIPSTART: establish TCP connection, UDP transfer, or TLS connection
-----------------------------------------------------------------------

**Execution command**

Command:
::

	AT+CIPSTART=<linkID>,<"type">,<remotehost>,<remoteport>[,<keepalive>,"localport"],<"localip">

Return:
::

	OK

**Parameters**

- <linkID>: network connection ID (0 to 5), supports up to 6 connections
- <"type">: connection type

	- 0: TCP
	- 1: UDP
	- 2: TLS
	- 3: DTLS

- <remotehost>: remote IP
- <remoteport>: remote port
- <keepalive>: TCP keep-alive interval, default value: 0

	- 0: Disable TCP keep alive
	- 1 to 7200: detection interval, unit: seconds

**Notes**

- Only TCP is valid, UDP does not need to be set. This parameter will eventually be configured to the socket option TCP_KEEPIDLE, keep alive and other socket options.
- TCP_KEEPINTVL uses 1 by default, TCP_KEEPCNT uses 3 by default.
- <"localport">: local port, this port needs to be set when the device needs to be set as a server.
- <"localip">: local IP, this IP needs to be configured when the device needs to be configured as a server.

**Examples**
::

	//1. Establish a TCP connection with connection ID 0 and keep-alive every 1800 seconds.
	AT+CIPSTART=0,"tcp","192.168.0.103",20108,1800

	//2. Establish a TCP Server connection with connection ID 1.
	AT+CIPSTART=2,"tcp","192.168.0.103",20108,1800, 5637, "192.168.0.107"

	//3. Establish a UDP client connection with connection ID 1.
	AT+CIPSTART=1,"udp","192.168.0.103",20108

	//4.Establish a UDP Server connection with connection ID 1 and local port 5637.
	AT+CIPSTART=1,"udp","192.168.0.103",20108,5637, "192.168.0.107"


------------------------------------------------------------------------
AT+CIPSEND: send data in normal transfer mode or Wi-Fi passthrough mode
------------------------------------------------------------------------

**Execution command**

Command:
::

	//Send TCP data
	AT+CIPSEND=<linkID>,<length> 

	//Send UDP data
	AT+CIPSEND=<linkID>,<length>[,<"remotehost">,<remote port>]

Return:
::

	>

The above response indicates that AT is ready to receive serial data. You can enter data at this time. After the length of data received by AT reaches <length>, data transfer begins.
If the connection is not established or the connection is disconnected during data transfer, return:
::

	CMDRSP:ERROR

If the data transfer is successful, return:
::

	SEND OK

**Execution command**

Function:

- Enter Wi-Fi passthrough mode

Command:
::

	AT+CIPSEND

Return:
::

	>

Or

::

	CMDRSP:ERROR

**Note**

- Wi-Fi passthrough mode: The device can receive a maximum of 1024 bytes each time and send a maximum of 1460 bytes. If the length of the currently received data is greater than the maximum number of bytes sent, AT will send it immediately; otherwise, the received data will be send within 20 ms. When a single packet +++ is input, exit the data sending mode in passthrough mode. Please wait at least 1 second before sending the next AT command. This command must be used in passthrough mode and single connection.

**Parameters**

- <linkID>: network connection ID (0 to 4), used for multiple connections.
- <length>: data length, maximum value: 2048 bytes.
- <”remote host”>: UDP transfer can specify the peer host: IPv4 address or domain name.
- <remote port>: UDP transfer can specify the peer port.


-------------------------------------------------
AT+CIPCLOSE: close TCP/UDP/SSL connection
-------------------------------------------------

**Execution command**

Command:
::

	AT+CIPSEND=<linkID>

Return:
::

	OK

**Parameter**

- <linkID>: The ID of the network connection that needs to be closed. If set to 5, it means closing all connections.


----------------------------------------------------
AT+CIPMUX: enable/disable multi-connection mode
----------------------------------------------------

**Query command**

Command:
::

	AT+CIPMUX?

Return:
::

	+CIPMUX:<mode>
	OK

**Execution command**

Command:
::

	AT+CIPMUX=<mode>

Return:
::

	OK

**Parameter**

- <mode>: connection mode, default value: 0.

	- 0: single connection
	- 1: Multiple connections

**Notes**

- Connection mode can only be changed when all connections are disconnected.
- Only the normal transfer mode (AT+CIPMODE=0) can be set to multiple connections


----------------------------------------------------
AT+CIPMODE: query/set transfer mode
----------------------------------------------------

**Query command**

Command:
::

	AT+CIPMODE?

Return:
::

	+CIPMODE:<mode>
	OK

**Execution command**

Command:
::

	AT+CIPMODE=<mode>

Return:
::

	OK

**Parameter**

- <mode>：

	- 0: normal transfer mode.
	- 1: Wi-Fi passthrough reception mode, only supports TCP single connection, UDP fixed communication peer, and SSL single connection.


----------------------------------------------------
AT+CIPSNTPCFG: query/set NTP server
----------------------------------------------------

**Query command**

Command:
::

	AT+CIPSNTPCFG?

Return:
::

	+CIPSNTPCFG:<enable>,<timezone>,<SNTP server>
	OK

**Execution command**

Command:
::

	AT+CIPSNTPCFG=<enable>,<timezone>,<SNTP server>

Return:
::

	OK

**Parameters**

- <enable>: set up SNTP server

	- 1: Set up SNTP server.
	- 0: Do not set SNTP server.

- <timezone>：

	- Value range: [-12,12], which is measured in hours and marks most time zones by offset from Coordinated Universal Time (UTC) (UTC−12:00 to UTC+12:00)

- [<SNTP server>]: SNTP server address or domain name

**Examples**
::

	//Enable SNTP server and set China time zone (UTC+08:00).
	AT+CIPSNTPCFG=1,8,"cn.ntp.org.cn"

	//Enable SNTP server and set the time zone of New York, USA (UTC−05:00).
	AT+CIPSNTPCFG=1,-5,"0.pool.ntp.org"


--------------------------------------------------------------------
AT+CIPSNTPTIME: query SNTP time
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+CIPSNTPTIME?

Return:
::

	+CIPSNTPTIME:<asctime style time>
	OK

**Example**
::

	AT+CIPSNTPTIME?
	
	+CIPSNTPTIME:2022-10-21 19:20:39
	OK


--------------------------------------------------------------------
AT+PING: ping the peer host
--------------------------------------------------------------------

**Execution command**

Command:
::

	AT+PING=<"host">

Return:
::

	+PING:<time>

Or

::

+PING:TIMEOUT

**Parameters**

- <”host”>: string parameter, indicating the IPv4 address or domain name of the peer host.
- <time>: ping response time, unit: milliseconds.


--------------------------------------------------------------------
AT+CIPDOMAIN: domain name resolution
--------------------------------------------------------------------

**Execution command**

Command:
::

	AT+CIPDOMAIN=<"domain name">

Return:
::

	+CIPDOMAIN:<"IP address">
	OK

**Parameters**

- <”domain name”>: domain name to be resolved
- <”IP address”>: the resolved IP address

**Note**

- Currently only supports resolution to IPv4 address


--------------------------------------------------------------------
AT+CIPDNS: query/set DNS server information
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+CIPDNS?

Return:
::

	+CIPDNS:<enable>[,<"DNS IP1">,<"DNS IP2">,<"DNS IP3">]
	OK

**Execution command**

Command:
::

	AT+CIPDNS=<enable>[,<"DNS IP1">,<"DNS IP2">,<"DNS IP3">]

Return:
::

	OK

Or

::

	ERROR

**Parameters**

- <enable>: Set DNS

	- 0: Enable automatic acquisition of DNS settings. DNS will revert to 208.67.222.222, and will only take effect when DHCP is updated.
	- 1: Enable manual setting of DNS information. If the values of parameters <DNS IPx> are not set, the default value 208.67.222.222 is used.

- <DNS IP1>: The first DNS IP address. For executing commands, this parameter is only valid when the <enable> parameter is 1, that is, manual DNS settings are enabled.
- <DNS IP2>: The second DNS IP address. For executing commands, this parameter is only valid when the <enable> parameter is 1, that is, manual DNS settings are enabled.
- <DNS IP3>: The third DNS IP address. For executing commands, this parameter is only valid when the <enable> parameter is 1, that is, manual DNS settings are enabled.

**Note**

- If AT+SYSSTORE=1, configuration changes will be saved in the NVS partition.


--------------------------------------------------------------------
AT+CIPSSLCPSK: query/set the PSK of the SSL client
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+CIPSSLCPSK?

Return:
::

	+CIPSSLCPSK:<linkID>,<"psk">,<"hint">
	OK

**Execution command**

Command:
::

	//Single connection: (AT+CIPMUX=0)
	AT+CIPSSLCPSK=<"psk">,<"hint">

	//Multiple connections: (AT+CIPMUX=1)
	AT+CIPSSLCPSK=<linkID>,<"psk">,<"hint">

Return:
::

	OK

**Parameters**

- <linkID>: Network connection ID (0 to max). In the case of a single connection, the value of this parameter is 0; in the case of multiple connections, if the parameter value is set to max, it means all connections. The default value of this parameter is 5.
- <”psk”>: PSK identity, maximum length: 48.
- <”hint”>: PSK hint, maximum length: 48.

**Note**

- If you want this configuration to take effect immediately, please run this command before establishing an SSL connection.


--------------------------------------------------------------------
AT+CIPDNS: query/set DNS server information
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+CIPDNS?

Return:
::

	+CIPDNS:<enable>[,<"DNS IP1">,<"DNS IP2">,<"DNS IP3">]
	OK

**Execution command**

Command:
::

	AT+CIPDNS=<enable>[,<"DNS IP1">,<"DNS IP2">,<"DNS IP3">]

Return:
::

	OK

Or

::

	ERROR

**Parameters**

- <enable>: Set DNS

	- 0: Enable automatic acquisition of DNS settings. DNS will revert to 208.67.222.222, which will only take effect when DHCP is updated.
	- 1: Enable manual setting of DNS information. If the values of parameters <DNS IPx> are not set, the default value 208.67.222.222 is used.

- <DNS IP1>: The first DNS IP address.
- <DNS IP2>: The second DNS IP address.
- <DNS IP3>: The third DNS IP address.

**Notes**

- If AT+SYSSTORE=1, configuration changes will be saved in the NVS partition.
- These three parameters cannot be set on the same server.
- When <enable> is 0, the DNS servers may change depending on the configuration of the router to which the device is connected.


