
:link_to_translation:`zh_CN:[中文]`

Wi-Fi instruction set
======================


--------------------------------------------
AT+CWMODE: set/query Wi-Fi operating mode
--------------------------------------------

**Query command**

Command:
::

	AT+CCWMODE?

Return:
::

	+CWMODE: <mode>,<auto_connect>
	OK

**Execution command**

Command:
::

	AT+CWMODE=<mode>[,<auto_connect>]

Return:
::

	OK

**Parameters**

- <mode>: mode

	- 0: No Wi-Fi mode, and Wi-Fi RF is turned off
	- 1: Station mode
	- 2: SoftAP mode
	- 3: SoftAP+Station mode

- <auto_connect>：

	- 0: Disable automatic connection to AP.
	- 1: Enable automatic connection to AP. If the configuration of automatic connection to AP has been saved to Flash before, it will automatically connect after restarting.


------------------------------------------------------
AT+CIPSTAMAC: set/query the current Wi-Fi MAC address
------------------------------------------------------

**Query command**

Command:
::

	AT+CIPSTAMAC?

Return:
::

	+WLMAC:<mac>
	OK

**Execution command**

Command:
::

	AT+CIPSTAMAC=<"mac">

Return:
::

	OK

**Parameter**

- <"mac">: Supports two formats: "MM:MM:MM:MM:MM:MM" and "MM-MM-MM-MM-MM-MM"


------------------------------------------
AT+CWSAP: SoftAP configuration parameters
------------------------------------------

**Execution command**

Command:
::

	AT+CWSAP=<"ssid">,<"pwd">,<channel>,<ecn>[,max conn>][,<hidden>]

Return:
::

	OK

**Parameters**

- <SSID>: string parameter, access point name
- <PWD>: string parameter, password, range: 8 to 64 bytes in ASCII
- <CHANNELl>: channel number
- <proto>:

	- 0: 802.11b/g/n standards
	- 1: 802.11b/g standards
	- 2: 802.11b standard

- <ECN>: Encryption method, does not support WEP

	- 0: OPEN
	- 1: WEP
	- 2: WPA_TKIP
	- 3: WPA_AES
	- 4: WPA2_TKIP
	- 5: WPA2_AES
	- 6: AES_TKIP_WPA2_MIXED
	- 7: WPA3_SAE
	- 8: WPA3_WPA2 (default)
	- 9: EAP
	- 10: OWE
	- 11: AUTO 

- [<max conn>]: The maximum number of stations allowed to connect to SoftAP, value range: [1,3]
- [<ssid hidden>]：

	- 0: Broadcast SSID (default)
	- 1: Do not broadcast SSID

**Note**

- SoftAP is related to WLMODE configuration. SoftAP is enabled only when the mode parameter in WLMODE is configured to 2 or 3. The default SSID is bk7238.


----------------------------
AT+CWQIF: stop SoftAP
----------------------------

**Execution command**

Command:
::

	AT+CWQIF

Return:
::

	OK


----------------------------------------------------
AT+CIPAP: configure SoftAP static IP and gateway
----------------------------------------------------

**Query command**

Command:
::

	AT+CIPAP?

Return:
::

	+CIPAP: <”ip”>,<”netmask”>,<”gateway”>
	OK

**Execution command**

Command:
::

	AT+CIPAP=<”ip”>,<”netmask”>,<”gateway”>

Return:
::

	OK

**Parameters**

- <”ip”>: string parameter, indicating IPv4 address
- <”gateway”>: gateway
- <”netmask”>: subnet mask


--------------------------------------------------------
AT+ CWLIF: query device information connected to SoftAP
--------------------------------------------------------

**Query command**

Command:
::

	AT+CWLIF

Return:
::

	+CWLIF：<index>,<ip>,<mac>,<rssi>
	OK

**Parameters**

- <index>: SoftAP serial number
- <ip>: gateway
- <mac>: subnet mask
- <rssi>: DNS address


----------------------------------------------------
AT+WSCAN: scan available APs
----------------------------------------------------

**Execution command**

Command:
::

	AT+WSCAN

Return:
::

	-CMDRSP: +WSCAN <ssid>,<bssid>,<channel>,<enc>,<rssi>

**Parameters**

- <ssid>: string parameter, SSID of AP
- <mac>: string parameter, MAC address of AP
- <channel>: subnet mask
- <ecn>: encryption method

	- 0: OPEN
	- 1: WEP
	- 2: WPA_TKIP
	- 3: WPA_AES
	- 4: WPA2_TKIP
	- 5: WPA2_AES
	- 6: AES_TKIP_WPA2_MIXED
	- 7: WPA3_SAE
	- 8: WPA3_WPA2 (default)
	- 9: EAP
	- 10: OWE
	- 11: AUTO

- <rssi>: RSSI


--------------------------------------------------------------------
AT+CWDHCP: enable/disable DHCP
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+CWDHCP?

Return:
::

	<state>

**Execution command**

Command:
::

	AT+CWDHCP=<operate>,<mode>

Return:
::

	OK

**Parameters**

- <operate>：

	- 0: disable
	- 1: enable

- <mode>：

	- Bit 0: DHCP of Station
	- Bit 1: DHCP of SoftAP

- <state>: DHCP status

	- Bit 0:

		- 0: Disable DHCP of Station
		- 1: Enable DHCP of Station 

	- Bit 1:

		- 0: Disable DHCP of SoftAP 
		- 1: Enable DHCP of SoftAP 

**Note**

- If AT+SYSSTORE=1, configuration changes will be saved to the NVS partition.

**Examples**

Enable:
::

	AT+CWDHCP=1,1		//Enable DHCP of STA
	AT+CWDHCP=1,2		//Enable DHCP of AP
	AT+CWDHCP=1,3		//Enable DHCP of STA+AP

Disable:
::

	AT+CWDHCP=0,1		//Disable DHCP of STA
	AT+CWDHCP=0,2		//Disable DHCP of AP
	AT+CWDHCP=0,3		//Disable DHCP of STA+AP


--------------------------------------------------------------------
AT+CWJAP: Station connects to AP
--------------------------------------------------------------------

**Execution command**

Command:
::

	AT+CWJAP=[<ssid>],[<pwd>][,<bssid>][,<pci_en>][,<reconn_interval>][,<listen_interval>][,<scan_mode>][,<jap_timeout>][,<pmf>]

Return:
::

	OK
	WIFI CONNECTED
	WIFI GOT IP

**Note**

- If AT+SYSSTORE=1, configuration changes will be saved to the NVS partition.


--------------------------------------------------------------------
AT+CWQAP: Disconnect from AP
--------------------------------------------------------------------

**Execution command**

Command:
::

	AT+CWQAP

Return:
::

	OK


--------------------------------------------------------------------
AT+CIPSTA: query/set IP information of Station
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+CIPSTA?

Return:
::

	+CIPSTA:<ip>,<netmask>,<gateway>

**Execution command**

Command:
::

	AT+STASTATIC=<ip>,<netmask>,<gateway>

Return:
::

	OK

**Parameters**

- <ip>: string parameter, indicating IPv4 address
- <gateway>: gateway
- <netmask>: subnet mask

