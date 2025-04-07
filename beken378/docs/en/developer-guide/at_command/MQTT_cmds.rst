
:link_to_translation:`zh_CN:[中文]`

MQTT instruction set
======================


--------------------------------------------
AT+MQTTUSERCFG: set MQTT user attributes
--------------------------------------------

**Execution command**

Command:
::

	AT+MQTTUSERCFG=<LinkID>,<scheme>,<"client_id">,<"username">,<"password">,<cert_key_ID>,<CA_ID>,<"path">

Return:
::

	OK

**Parameters**

- <LinkID>: Currently only link ID 0 is supported.
- <scheme>:

	- 1: MQTT over TCP
	- 2: MQTT over TLS (PSK encryption)

- <client_id>: MQTT client ID, maximum length: 256 bytes.
- <username>: username, used to log in to MQTT broker, maximum length: 64 bytes.
- <password>: password, used to log in to MQTT broker, maximum length: 64 bytes.
- <cert_key_ID>: certificate ID. Currently, cert certificates are not supported, and the parameter is 0.
- <CA_ID>: CA ID. Currently, CA certificates are not supported. The parameter is 0.
- <path>: resource path, maximum length: 32 bytes.

**Note**

- The total length of each AT command cannot exceed 256 bytes.


-----------------------------------------------
AT+ MQTTCONNCFG: set MQTT connection attributes
-----------------------------------------------

**Execution command**

Command:
::

	AT+MQTTCONNCFG=<LinkID>,<keepalive>,<disable_clean_session>,<"lwt_topic">,<"lwt_msg">,<lwt_qos>,<lwt_retain>

Return:
::

	OK

**Parameters**

- <LinkID>: Currently only link ID 0 is supported.
- <keepalive>: MQTT ping timeout, unit: seconds. Range: [0,7200]. Default value: 0, which means the timeout will be forced to 120 seconds.
- <disable_clean_session>: set the MQTT clean session flag.

	- 0: enable clean session
	- 1: disable clean session

- <lwt_topic>: will topic, maximum length: 128 bytes.
- <lwt_msg>: will message, maximum length: 64 bytes.
- <lwt_qos>: will QoS, the parameter can be 0, 1, 2, default value: 0.
- <lwt_retain>: will retain, the parameter can be 0 or 1, default value: 0.

**Example**
::

	AT+MQTTCONNCFG=0,180,0,"test/aaa","good bye",1,0


---------------------------------------------
AT+MQTTCONN: set MQTT connection attributes
---------------------------------------------

**Query command**

Command:
::

	AT+MQTTCONN?

Return:
::

	+MQTTCONN:<LinkID>,<state>,<scheme>,<"host">,<port>,<reconnect>
	OK

**Execution command**

Command:
::

	AT+MQTTCONN=<LinkID>,<host>,<port>,<reconnect>

Return:
::

	OK

**Parameters**

- <LinkID>: Currently only link ID 0 is supported.
- <host>: MQTT broker domain name or IP, maximum length: 64 bytes.
- <port>: MQTT broker port, maximum port number: 65535.
- <reconnect>:

	- 0: MQTT does not automatically reconnect.
	- 1: MQTT automatically reconnection

- <state>: MQTT status:

	- connected: connection successful
	- disconnect: connection failed

- <scheme>:

	- 1: MQTT over TCP
	- 2: MQTT over TLS (PSK encryption)

**Example**
::

	AT+MQTTCONN=0,192.168.0.102,8883,1


----------------------------------
AT+MQTTPUB: publish MQTT message
----------------------------------

**Execution command**

Command:
::

	AT+MQTTPUB=<linkID>,<"topic">,<"data">,<qos>,<retain>

Return:
::

	OK

**Parameters**

- <LinkID>: Currently only LinkID 0 is supported.
- <topic>: MQTT topic, maximum length: 128 bytes.
- <data>: MQTT string message.
- <qos>: QoS for publishing messages. The parameter can be 0, 1, or 2.
- <retain>: publish retain (0 or 1).

**Example**
::

	//Publish the string "987654321" message through the topic test/abc
	AT+MQTTPUB=0,test/abc,987654321,1,0


----------------------------------------------------
AT+ MQTTPUBRAW: publish MQTT messages (binary)
----------------------------------------------------

**Execution command**

Command:
::

	AT+MQTTPUBRAW=<linkid>,<"topic">,<length>,<qos>,<retain>

Return:
::

	OK
	>

The symbol > indicates that AT is ready to receive serial port data. At this time, you can input data. When the data length reaches the parameter <length> value, data transfer begins. If the transfer is successful, AT returns:
::

	+MQTTPUBRAW:OK

**Parameters**

- <LinkID>: Currently only link ID 0 is supported.
- <topic>: MQTT topic, maximum length: 128 bytes.
- <length>: MQTT message length, maximum length: 512 bytes.
- <qos>: QoS for publishing messages. The parameter can be 0, 1, or 2. Default value: 0.
- <retain>: publish retain.


----------------------------------------------------
AT+ MQTTSUB: subscribe to MQTT topic
----------------------------------------------------

**Query command**

Command:
::

	AT+MQTTSUB?

Return:
::

	+MQTTSUB:<linkid>,<status>,<topic1>,<“qos”>
	+MQTTSUB: <linkid>,<status>,<topic2>,<“qos”>
	OK

**Execution command**

Command:
::

	AT+MQTTSUB=<linkid>,<"topic">,<qos>

Return:
::

	OK

**Parameters**

- <LinkID>: Currently only link ID 0 is supported.
- <topic>: MQTT topic
- <qos>: MQTT QoS

**Notes**

When AT receives the MQTT message of the subscribed topic, it returns:
::

	+MQTTSUBRECV:<LinkID>,<"topic">,<data_length>,data

If the topic has been subscribed, return:
::

	ALREADY SUBSCRIBE


----------------------------------------------------
AT+ MQTTUNSUB: unsubscribe from MQTT topic
----------------------------------------------------

**Execution command**

Command:
::

	AT+MQTTUNSUB=<linkID>,<”topic”>

Return:
::

	OK

If the topic has not been subscribed or an MQTT connection has not been created, then return:
::

	NO UNSUBSCRIBE
	OK

**Parameters**

- <LinkID>: Currently only link ID 0 is supported.
- <topic>: MQTT topic, maximum length: 128 bytes.


--------------------------------------------------------------------
AT+ MQTTCLEAN: Disconnect MQTT connection
--------------------------------------------------------------------

**Execution command**

Command:
::

	AT+MQTTCLEAN=<LinkID>

Return:
::

	OK

**Parameters**

• <LinkID>: Currently only link ID 0 is supported.
