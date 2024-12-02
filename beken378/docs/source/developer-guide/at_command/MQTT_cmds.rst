MQTT指令集
=================


--------------------------------------------
AT+MQTTUSERCFG：设置 MQTT 用户属性
--------------------------------------------

**执行命令**

命令：
::

	AT+MQTTUSERCFG=<LinkID>,<scheme>,<"client_id">,<"username">,<"password">,<cert_key_ID>,<CA_ID>,<"path">

返回：
::

	OK

**参数**

- <LinkID>：当前仅支持 link ID 0。
- <scheme>：

	- 1: MQTT over TCP
	- 2: MQTT over TLS（PSK加密）

- <client_id>：MQTT 客户端 ID，最大长度：256 字节。
- <username>：用户名，用于登陆 MQTT broker，最大长度：64 字节。
- <password>：密码，用于登陆 MQTT broker，最大长度：64 字节。
- <cert_key_ID>：证书 ID，目前 暂不支持cert 证书，参数为 0。
- <CA_ID>：CA ID，目前暂不支持CA 证书，参数为 0。
- <path>：资源路径，最大长度：32 字节。

**说明**

- 每条 AT 命令的总长度不能超过 256 字节。


-----------------------------------------------
AT+ MQTTCONNCFG：设置 MQTT连接属性
-----------------------------------------------

**执行命令**

命令：
::

	AT+MQTTCONNCFG=<LinkID>,<keepalive>,<disable_clean_session>,<"lwt_topic">,<"lwt_msg">,<lwt_qos>,<lwt_retain>

返回：
::

	OK

**参数**

- <LinkID>：当前仅支持 link ID 0。
- <keepalive>：MQTT ping 超时时间，单位：秒。范围：[0,7200]。默认值：0，会被强制改为 120 秒。
- <disable_clean_session>：设置 MQTT 清理会话标志。

	- 0: 使能清理会话
	- 1: 禁用清理会话

- <lwt_topic>：遗嘱 topic，最大长度：128 字节。
- <lwt_msg>：遗嘱 message，最大长度：64 字节。
- <lwt_qos>：遗嘱 QoS，参数可选 0、1、2，默认值：0。
- <lwt_retain>：遗嘱 retain，参数可选 0 或 1，默认值：0。

**示例**
::

	AT+MQTTCONNCFG=0,180,0,"test/aaa","good bye",1,0


------------------------------------------
AT+MQTTCONN：设置 MQTT连接属性
------------------------------------------

**查询命令**

命令：
::

	AT+MQTTCONN?

返回：
::

	+MQTTCONN:<LinkID>,<state>,<scheme>,<"host">,<port>,<reconnect>
	OK

**执行命令**

命令：
::

	AT+MQTTCONN=<LinkID>,<host>,<port>,<reconnect>

返回：
::

	OK

**参数**

- <LinkID>：当前仅支持 link ID 0。
- <host>：MQTT broker 域名或IP，最大长度：64 字节。
- <port>：MQTT broker 端口，最大端口：65535。
- <reconnect>：

	- 0: MQTT 不自动重连
	- 1: MQTT 自动重连

- <state>：MQTT 状态：

	- connected：连接成功
	- disconnect：连接失败

- <scheme>：

	- 1: MQTT over TCP
	- 2: MQTT over TLS（PSK加密）

**示例**
::

	AT+MQTTCONN=0,192.168.0.102,8883,1


----------------------------
AT+MQTTPUB：发布 MQTT 消息
----------------------------

**执行命令**

命令：
::

	AT+MQTTPUB=<linkID>,<"topic">,<"data">,<qos>,<retain>

返回：
::

	OK

**参数**

- <LinkID>：当前仅支持 LinkID 0。
- <topic>：MQTT topic，最大长度：128字节。
- <data>：MQTT字符串消息。
- <qos>：发布消息的 QoS，参数可选0、1、或2。
- <retain>：发布 retain（0或1）。

**示例**
::

	//通过主题test/abc发布字串“987654321”消息
	AT+MQTTPUB=0,test/abc,987654321,1,0


----------------------------------------------------
AT+ MQTTPUBRAW：发布 MQTT 消息（二进制）
----------------------------------------------------

**执行命令**

命令：
::

	AT+MQTTPUBRAW=<linkid>,<"topic">,<length>,<qos>,<retain>

返回：
::

	OK
	>

符号 > 表示 AT 准备好接收串口数据，此时您可以输入数据，当数据长度达到参数 <length> 的值时，数据传输开始。
若传输成功，则 AT 返回：
::

	+MQTTPUBRAW:OK

**参数**

- <LinkID>：当前仅支持 link ID 0。
- <topic>：MQTT topic，最大长度：128 字节。
- <length>：MQTT 消息长度，最大长度为512字节。
- <qos>：发布消息的 QoS，参数可选 0、1、或 2，默认值：0。
- <retain>：发布 retain。


----------------------------------------------------
AT+ MQTTSUB：订阅 MQTT Topic
----------------------------------------------------

**查询命令**

命令：
::

	AT+MQTTSUB?

返回：
::

	+MQTTSUB:<linkid>,<status>,<topic1>,<“qos”>
	+MQTTSUB: <linkid>,<status>,<topic2>,<“qos”>
	OK

**执行命令**

命令：
::

	AT+MQTTSUB=<linkid>,<"topic">,<qos>

返回：
::

	OK

**参数**

- <LinkID>：当前仅支持 link ID 0。
- <topic>：订阅的 topic。
- <qos>：订阅的 QoS。

**说明**

当 AT 接收到已订阅的 topic 的 MQTT 消息时，返回：
::

	+MQTTSUBRECV:<LinkID>,<"topic">,<data_length>,data

若已订阅过该 topic，则返回：
::

	ALREADY SUBSCRIBE


----------------------------------------------------
AT+ MQTTUNSUB：取消订阅 MQTT Topic
----------------------------------------------------

**执行命令**

命令：
::

	AT+MQTTUNSUB=<linkID>,<”topic”>

返回：
::

	OK

若未订阅过该 topic，或未创建MQTT连接则返回：
::

	NO UNSUBSCRIBE
	OK

**参数**

- <LinkID>：当前仅支持 link ID 0。
- <topic>：MQTT topic，最大长度：128 字节。


--------------------------------------------------------------------
AT+ MQTTCLEAN：断开 MQTT 连接
--------------------------------------------------------------------

**执行命令**

命令：
::

	AT+MQTTCLEAN=<LinkID>

返回：
::

	OK

**参数**

• <LinkID>：当前仅支持 link ID 0。
